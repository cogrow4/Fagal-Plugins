/*
 * delay.c — CASC plugin: "Fagal Echo" (Fagal Plugins)
 *
 * Pro-grade stereo delay. Sections:
 *   - Stereo / Ping-Pong routing
 *   - Tone filtering in the feedback loop: Low-Cut (one-pole HP) + High-Cut (one-pole LP)
 *   - Modulation: chorus-y wow/flutter on the delay read position (fractional, interpolated)
 *   - Dual-tap: a second tap at a musical ratio of the main time, with its own level
 *   - Saturation in the feedback path (soft tanh-ish clip) for analog-style buildup
 *   - Freeze/Hold toggle: locks the buffer (input muted into the line, feedback = 1.0)
 *   - Stereo Width on the wet signal
 *
 * Real-time safe: no libc, no allocation in process, all state in a per-instance
 * struct with static delay lines.
 *
 * Build:
 *   clang --target=wasm32 -O3 -nostdlib -Wl,--no-entry -Wl,--export-dynamic \
 *         delay.c -o dsp.wasm
 *
 * Params (normalised 0..1, ids contiguous):
 *   0  Time        -> 1ms .. 1.5s
 *   1  Feedback    -> 0 .. 0.98
 *   2  Mix         -> dry/wet
 *   3  LowCut      -> feedback high-pass cutoff (0=off/low, 1=aggressive)
 *   4  HighCut     -> feedback low-pass cutoff (0=dark, 1=bright/open)
 *   5  ModRate     -> 0.05 .. 8 Hz LFO
 *   6  ModDepth    -> 0 .. ~6ms of time modulation
 *   7  Saturation  -> feedback-path soft drive
 *   8  Width       -> stereo width of wet
 *   9  PingPong    -> toggle (>=0.5 on)
 *   10 Freeze      -> toggle (>=0.5 on): hold buffer, mute input, fb=1
 *   11 Tap2Level   -> level of a second tap @ 0.6667 of main time
 */
#include <stdint.h>

/* ----- minimal math (no libc) ----- */
static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

/* Polynomial sine approximation, valid for any x. Reduces to [-PI,PI]. */
#define PI_F  3.14159265358979f
#define TWOPI_F 6.28318530717959f
static inline float fast_sin(float x) {
    /* range reduce to [-PI, PI] */
    x = x - TWOPI_F * (float)((int)(x * (1.0f / TWOPI_F) + (x >= 0.0f ? 0.5f : -0.5f)));
    /* Bhaskara-ish / parabola approximation */
    const float B = 1.27323954f;   /* 4/PI */
    const float C = -0.405284735f; /* -4/PI^2 */
    float y = B * x + C * x * (x < 0.0f ? -x : x);
    /* extra precision pass */
    const float P = 0.225f;
    y = P * (y * (y < 0.0f ? -y : y) - y) + y;
    return y;
}

/* soft saturation (cheap tanh approximation) */
static inline float soft_sat(float x) {
    /* rational tanh approx: keeps slope ~1 near 0, clamps gracefully */
    if (x < -3.0f) return -1.0f;
    if (x >  3.0f) return  1.0f;
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* Max delay buffer: ~2.0s at 48kHz (96000 floats) per channel.
 * At higher sample rates the max usable delay time scales down accordingly;
 * delaySamples is clamped to the buffer length. */
#define DELAY_MAX 96000

typedef struct {
    float bufL[DELAY_MAX];
    float bufR[DELAY_MAX];
    int   write;          /* shared write index */

    /* feedback tone filter states */
    float hpL, hpR;       /* high-pass (low-cut) states */
    float lpL, lpR;       /* low-pass  (high-cut) states */

    /* modulation LFO phase */
    float lfoPhase;

    /* smoothed control values to avoid zipper noise */
    float sTime, sFb, sMix, sTap2;

    /* normalised params */
    float p_time, p_fb, p_mix, p_lowcut, p_highcut;
    float p_modrate, p_moddepth, p_sat, p_width, p_ping, p_freeze, p_tap2;

    double sample_rate;
    int    max_block;
    int    active;
    int    primed; /* whether smoothers have been seeded */
} Delay;

#define MAX_INSTANCES 4
static Delay g_inst[MAX_INSTANCES];
static int g_next = 0;

#define NUM_PARAMS 12

/* ----- helpers ----- */
static void buf_clear(float* b, int n) { for (int i=0;i<n;i++) b[i]=0.0f; }

static void delay_setup(Delay* d, double sr) {
    d->sample_rate = sr;
    d->write = 0;
    d->hpL = d->hpR = 0.0f;
    d->lpL = d->lpR = 0.0f;
    d->lfoPhase = 0.0f;
    d->primed = 0;
    buf_clear(d->bufL, DELAY_MAX);
    buf_clear(d->bufR, DELAY_MAX);
}

static void seed_smoothers(Delay* d) {
    d->sTime = d->p_time;
    d->sFb   = d->p_fb;
    d->sMix  = d->p_mix;
    d->sTap2 = d->p_tap2;
    d->primed = 1;
}

/* read a fractional sample from a circular buffer (linear interp) */
static inline float read_frac(const float* buf, int write, float delaySamples) {
    if (delaySamples < 1.0f) delaySamples = 1.0f;
    if (delaySamples > (float)(DELAY_MAX - 2)) delaySamples = (float)(DELAY_MAX - 2);
    float readPos = (float)write - delaySamples;
    while (readPos < 0.0f) readPos += (float)DELAY_MAX;
    int i0 = (int)readPos;
    float frac = readPos - (float)i0;
    int i1 = i0 + 1; if (i1 >= DELAY_MAX) i1 -= DELAY_MAX;
    return buf[i0] + (buf[i1] - buf[i0]) * frac;
}

/* ----- exports ----- */
__attribute__((export_name("dsp_create")))
int32_t dsp_create(double sample_rate, int32_t max_block_size) {
    if (g_next >= MAX_INSTANCES) return -1;
    int h = g_next++;
    Delay* d = &g_inst[h];
    d->p_time     = 0.25f;
    d->p_fb       = 0.35f;
    d->p_mix      = 0.35f;
    d->p_lowcut   = 0.10f;
    d->p_highcut  = 0.70f;
    d->p_modrate  = 0.25f;
    d->p_moddepth = 0.0f;
    d->p_sat      = 0.0f;
    d->p_width    = 1.0f;
    d->p_ping     = 0.0f;
    d->p_freeze   = 0.0f;
    d->p_tap2     = 0.0f;
    d->max_block  = max_block_size;
    d->active     = 1;
    delay_setup(d, sample_rate);
    seed_smoothers(d);
    return h;
}

__attribute__((export_name("dsp_destroy")))
void dsp_destroy(int32_t handle) { (void)handle; }

__attribute__((export_name("dsp_reset")))
void dsp_reset(int32_t handle, double sample_rate, int32_t max_block_size) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    g_inst[handle].max_block = max_block_size;
    delay_setup(&g_inst[handle], sample_rate);
    seed_smoothers(&g_inst[handle]);
}

__attribute__((export_name("dsp_set_param")))
void dsp_set_param(int32_t handle, int32_t id, double value) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Delay* d = &g_inst[handle];
    float v = clampf((float)value, 0.0f, 1.0f);
    switch (id) {
        case 0:  d->p_time     = v; break;
        case 1:  d->p_fb       = v; break;
        case 2:  d->p_mix      = v; break;
        case 3:  d->p_lowcut   = v; break;
        case 4:  d->p_highcut  = v; break;
        case 5:  d->p_modrate  = v; break;
        case 6:  d->p_moddepth = v; break;
        case 7:  d->p_sat      = v; break;
        case 8:  d->p_width    = v; break;
        case 9:  d->p_ping     = v; break;
        case 10: d->p_freeze   = v; break;
        case 11: d->p_tap2     = v; break;
        default: break;
    }
}

__attribute__((export_name("dsp_get_param")))
double dsp_get_param(int32_t handle, int32_t id) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0.0;
    Delay* d = &g_inst[handle];
    switch (id) {
        case 0:  return d->p_time;
        case 1:  return d->p_fb;
        case 2:  return d->p_mix;
        case 3:  return d->p_lowcut;
        case 4:  return d->p_highcut;
        case 5:  return d->p_modrate;
        case 6:  return d->p_moddepth;
        case 7:  return d->p_sat;
        case 8:  return d->p_width;
        case 9:  return d->p_ping;
        case 10: return d->p_freeze;
        case 11: return d->p_tap2;
        default: return 0.0;
    }
}

__attribute__((export_name("dsp_process")))
void dsp_process(int32_t handle, int32_t in_ptr, int32_t out_ptr,
                 int32_t frames, int32_t in_ch, int32_t out_ch) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Delay* d = &g_inst[handle];
    float* in  = (float*)(uintptr_t)in_ptr;
    float* out = (float*)(uintptr_t)out_ptr;

    if (!d->primed) seed_smoothers(d);

    float sr = (float)d->sample_rate;
    float invSr = 1.0f / sr;

    /* ---- control mapping ---- */
    int freeze = d->p_freeze >= 0.5f;
    int ping   = d->p_ping   >= 0.5f;

    /* Feedback: in freeze mode lock to 1.0 (lossless hold). */
    float fbTarget = freeze ? 1.0f : (d->p_fb * 0.98f);

    /* Tone filters. One-pole coefficients.
     * High-cut (low-pass): highcut=1 -> bright (coef ~1), 0 -> dark. */
    float lpCoef = 0.015f + d->p_highcut * 0.985f;        /* 0.015 .. 1.0 */
    /* Low-cut (high-pass): lowcut=0 -> no cut (coef ~0), 1 -> strong cut. */
    float hpCoef = d->p_lowcut * 0.35f;                   /* 0 .. 0.35 */

    /* Saturation drive. */
    float satAmt = d->p_sat;                              /* 0 .. 1 */
    float drive  = 1.0f + satAmt * 3.0f;                  /* 1 .. 4 */
    float makeup = 1.0f / (1.0f + satAmt * 0.6f);

    /* Modulation. */
    float modRateHz = 0.05f + d->p_modrate * 7.95f;       /* 0.05 .. 8 Hz */
    float lfoInc    = TWOPI_F * modRateHz * invSr;
    float modSamps  = d->p_moddepth * (0.006f * sr);      /* up to ~6 ms */

    /* Width on wet. */
    float width = d->p_width;
    float wMid  = 0.5f * (1.0f + 0.0f);

    /* Tap2 ratio of main time (dotted-ish). */
    float tap2Ratio = 0.6667f;

    /* per-sample smoothing coefficient (~5ms) */
    float sc = 1.0f - 0.999f; /* placeholder, recomputed */
    sc = 1.0f / (0.005f * sr + 1.0f);

    for (int n = 0; n < frames; n++) {
        float inL, inR;
        if (in_ch >= 2) { inL = in[n*in_ch + 0]; inR = in[n*in_ch + 1]; }
        else            { inL = in[n*in_ch + 0]; inR = inL; }

        /* smooth controls */
        d->sTime += (d->p_time - d->sTime) * sc;
        d->sFb   += (fbTarget  - d->sFb)   * sc;
        d->sMix  += (d->p_mix  - d->sMix)  * sc;
        d->sTap2 += (d->p_tap2 - d->sTap2) * sc;

        float timeSec = 0.001f + d->sTime * 1.499f;        /* 1ms .. 1.5s */
        float baseSamps = timeSec * sr;

        /* LFO -> time modulation (fractional) */
        float lfo = fast_sin(d->lfoPhase);
        d->lfoPhase += lfoInc;
        if (d->lfoPhase >= TWOPI_F) d->lfoPhase -= TWOPI_F;
        float modSampsNow = lfo * modSamps;

        /* main delay samples (L/R get slight opposite mod for stereo motion) */
        float dSampL = baseSamps + modSampsNow;
        float dSampR = baseSamps - modSampsNow;

        float dL = read_frac(d->bufL, d->write, dSampL);
        float dR = read_frac(d->bufR, d->write, dSampR);

        /* second tap (no mod, fixed musical ratio) */
        float t2L = 0.0f, t2R = 0.0f;
        if (d->sTap2 > 0.0001f) {
            float ts = baseSamps * tap2Ratio;
            t2L = read_frac(d->bufL, d->write, ts);
            t2R = read_frac(d->bufR, d->write, ts);
        }

        /* ---- feedback tone shaping (on the delayed signal) ---- */
        /* low-pass (high-cut) */
        d->lpL += lpCoef * (dL - d->lpL);
        d->lpR += lpCoef * (dR - d->lpR);
        float fL = d->lpL;
        float fR = d->lpR;
        /* high-pass (low-cut): subtract a tracked low-band */
        d->hpL += hpCoef * (fL - d->hpL);
        d->hpR += hpCoef * (fR - d->hpR);
        fL = fL - d->hpL;
        fR = fR - d->hpR;

        /* saturation in feedback path */
        if (satAmt > 0.0001f) {
            fL = soft_sat(fL * drive) * makeup;
            fR = soft_sat(fR * drive) * makeup;
        }

        float fb = d->sFb;

        /* ---- write into delay lines ---- */
        float wL, wR;
        if (freeze) {
            /* hold: recirculate buffer losslessly, ignore input */
            wL = fL;
            wR = fR;
        } else if (ping) {
            /* ping-pong: input sums to mono in, feedback crosses channels */
            float monoIn = (inL + inR) * 0.5f;
            wL = monoIn + fR * fb;
            wR =          fL * fb;
        } else {
            wL = inL + fL * fb;
            wR = inR + fR * fb;
        }
        d->bufL[d->write] = wL;
        d->bufR[d->write] = wR;
        if (++d->write >= DELAY_MAX) d->write = 0;

        /* ---- build wet signal (main tap + tap2) ---- */
        float wetL = dL + t2L * d->sTap2;
        float wetR = dR + t2R * d->sTap2;

        /* stereo width (mid/side) on wet */
        float mid  = (wetL + wetR) * 0.5f;
        float side = (wetL - wetR) * 0.5f * width;
        wetL = mid + side;
        wetR = mid - side;
        (void)wMid;

        float mix = d->sMix;
        float dryG = 1.0f - mix;
        float yL = inL * dryG + wetL * mix;
        float yR = inR * dryG + wetR * mix;

        if (out_ch >= 2) { out[n*out_ch + 0] = yL; out[n*out_ch + 1] = yR; }
        else             { out[n*out_ch + 0] = (yL + yR) * 0.5f; }
    }
}

__attribute__((export_name("dsp_get_latency")))
int32_t dsp_get_latency(int32_t handle) { (void)handle; return 0; }

__attribute__((export_name("dsp_get_tail")))
int32_t dsp_get_tail(int32_t handle) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    /* tail = ~3s of delay decay */
    return (int32_t)(g_inst[handle].sample_rate * 3.0);
}

/* ----- state: store all NUM_PARAMS normalised params ----- */
__attribute__((export_name("dsp_save_state")))
int32_t dsp_save_state(int32_t handle, int32_t ptr, int32_t max_bytes) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    if (max_bytes < (int32_t)(NUM_PARAMS*sizeof(float))) return 0;
    float* o = (float*)(uintptr_t)ptr;
    Delay* d = &g_inst[handle];
    o[0]=d->p_time;     o[1]=d->p_fb;      o[2]=d->p_mix;
    o[3]=d->p_lowcut;   o[4]=d->p_highcut; o[5]=d->p_modrate;
    o[6]=d->p_moddepth; o[7]=d->p_sat;     o[8]=d->p_width;
    o[9]=d->p_ping;     o[10]=d->p_freeze; o[11]=d->p_tap2;
    return (int32_t)(NUM_PARAMS*sizeof(float));
}

__attribute__((export_name("dsp_load_state")))
void dsp_load_state(int32_t handle, int32_t ptr, int32_t byte_count) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    if (byte_count < (int32_t)(NUM_PARAMS*sizeof(float))) return;
    float* s = (float*)(uintptr_t)ptr;
    Delay* d = &g_inst[handle];
    d->p_time=s[0];     d->p_fb=s[1];      d->p_mix=s[2];
    d->p_lowcut=s[3];   d->p_highcut=s[4]; d->p_modrate=s[5];
    d->p_moddepth=s[6]; d->p_sat=s[7];     d->p_width=s[8];
    d->p_ping=s[9];     d->p_freeze=s[10]; d->p_tap2=s[11];
    d->primed = 0; /* re-seed smoothers next process */
}

/* ----- bump allocator ----- */
static uint8_t heap[1 << 20];
static int heap_top = 0;
__attribute__((export_name("casc_alloc")))
int32_t casc_alloc(int32_t n) {
    int a = (n + 7) & ~7;
    if (heap_top + a > (int)sizeof(heap)) return 0;
    int p = heap_top; heap_top += a;
    return (int32_t)((uintptr_t)heap + p);
}
__attribute__((export_name("casc_free")))
void casc_free(int32_t ptr) { (void)ptr; }
