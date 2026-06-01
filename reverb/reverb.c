/*
 * reverb.c — CASC plugin: Fagal Space (multi-section reverb)
 *
 * An expansive, Spectral-Audio-"Space"-style reverb with four independently
 * toggleable sections, all running into a shared wet bus:
 *
 *   Algorithmic   — classic Freeverb topology: 8 damped parallel combs + 4
 *                   series allpass per channel, with stereo spread.
 *   Diffusion/Grain — extra series allpass diffusion stages on the wet path,
 *                   driven by the Diffusion param (denser, smoother tail).
 *   Modulation/Shimmer — slow LFOs that modulate fractional comb delay reads,
 *                   producing chorused/shimmering motion (ModRate/ModDepth).
 *   Early Reflections — a small tapped multi-delay bank (convolution-style),
 *                   summed in before the late tail (EarlyLevel).
 *
 * Wet path is shaped by a one-pole LowCut (high-pass) and HighCut (low-pass).
 *
 * Build:
 *   clang --target=wasm32 -O3 -nostdlib -Wl,--no-entry -Wl,--export-dynamic \
 *         reverb.c -o dsp.wasm
 *
 * Params (normalised 0..1):
 *   0  Size        room size / comb feedback
 *   1  Decay       extra tail feedback (long ringout)
 *   2  Damping     high-frequency damping in the combs
 *   3  Width       stereo width of the wet signal
 *   4  Diffusion   amount of extra allpass diffusion (grain density)
 *   5  ModRate     modulation LFO rate
 *   6  ModDepth    modulation depth (delay-line vibrato / shimmer)
 *   7  EarlyLevel  level of the early-reflection tap bank
 *   8  LowCut      wet-path high-pass cutoff (removes low rumble)
 *   9  HighCut     wet-path low-pass cutoff (tames highs)
 *   10 AlgoOn      toggle: late algorithmic tail on/off
 *   11 GrainOn     toggle: diffusion/grain stages on/off
 *   12 ModOn       toggle: modulation/shimmer on/off
 *   13 EarlyOn     toggle: early reflections on/off
 *   14 Mix         dry/wet
 */
#include <stdint.h>
#include "../_shared/meter_ring.h"

#define NUM_PARAMS 15

/* ----- minimal math (no libc) ----- */
static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

/* Fast sine approximation (Bhaskara-style), input in radians, range-reduced. */
static const float TWO_PI = 6.28318530718f;
static const float PI_F   = 3.14159265359f;
static inline float fast_sin(float x) {
    /* reduce to [-pi, pi] */
    while (x >  PI_F) x -= TWO_PI;
    while (x < -PI_F) x += TWO_PI;
    /* Bhaskara I approximation */
    float ax = x < 0 ? -x : x;
    float num = 16.0f * x * (PI_F - ax);
    float den = 5.0f * PI_F * PI_F - 4.0f * ax * (PI_F - ax);
    return num / den;
}

/* Hard brick-wall ceiling. The previous Pade-based "soft_ceiling" was
 * a rational approximation of tanh(x) that DIVERGES for large x, so it
 * let peaks well past the intended ceiling — the symptom was the reverb
 * sounding like heavy machinery and clipping the speakers on silence.
 * A plain hard clip with a 0.95 headroom is the only reliable way to
 * guarantee a strict output bound; the knee is only felt on transients
 * already deep into the red, which is exactly when you want a guard. */
static inline float hard_ceiling(float x, float ceiling) {
    if (x >  ceiling) return  ceiling;
    if (x < -ceiling) return -ceiling;
    return x;
}

/* Freeverb tuning constants (samples @ 44.1kHz). Scaled by SR at reset. */
#define NUM_COMBS    8
#define NUM_ALLPASS  4
#define NUM_DIFFUSE  4   /* extra diffusion/grain allpass stages */
#define NUM_EARLY    8   /* early-reflection taps */
#define STEREO_SPREAD 23

static const int comb_tuning[NUM_COMBS] = {
    1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617
};
static const int allpass_tuning[NUM_ALLPASS] = { 556, 441, 341, 225 };
static const int diffuse_tuning[NUM_DIFFUSE] = { 167, 191, 127, 97 };

/* Early reflection taps (samples @44.1k) and gains. */
static const int   early_tap[NUM_EARLY] = { 113, 277, 421, 587, 743, 919, 1117, 1361 };
static const float early_gain[NUM_EARLY] = {
    0.90f, 0.78f, 0.67f, 0.58f, 0.49f, 0.41f, 0.34f, 0.27f
};

/* Max buffer sizes (largest tuning + spread + mod headroom for high SR). */
#define COMB_MAX     3600
#define ALLPASS_MAX  1400
#define DIFFUSE_MAX   600
#define EARLY_MAX    3200

typedef struct {
    float buf[COMB_MAX];
    int   size;
    int   idx;
    float filterstore; /* lowpass state for damping */
} Comb;

typedef struct {
    float buf[ALLPASS_MAX];
    int   size;
    int   idx;
} Allpass;

typedef struct {
    float buf[DIFFUSE_MAX];
    int   size;
    int   idx;
} Diffuse;

typedef struct {
    float buf[EARLY_MAX];
    int   size;       /* ring length (>= max tap) */
    int   idx;
    int   tap[NUM_EARLY];
} Early;

typedef struct {
    Comb     combL[NUM_COMBS],   combR[NUM_COMBS];
    Allpass  apL[NUM_ALLPASS],   apR[NUM_ALLPASS];
    Diffuse  dfL[NUM_DIFFUSE],   dfR[NUM_DIFFUSE];
    Early    earL,               earR;

    /* wet-path filter states (per channel) */
    float lpL, lpR;   /* high-cut (low-pass) one-pole state */
    float hpL, hpR;   /* low-cut  (high-pass) one-pole state */

    /* modulation LFO phases */
    float lfo1, lfo2;

    /* output ring buffer for UI meters */
    float meter_buf[METER_BUF];
    int   meter_idx;
    int   meter_filled;

    /* normalised params (id order) */
    float p[NUM_PARAMS];

    double sample_rate;
    int    max_block;
    int    active;
} Reverb;

#define MAX_INSTANCES 8
static Reverb g_inst[MAX_INSTANCES];
static int g_next = 0;

/* ----- helpers ----- */
static void buf_clear(float* b, int n) { for (int i=0;i<n;i++) b[i]=0.0f; }

static int scale_len(int base, double sr) {
    int n = (int)((double)base * sr / 44100.0 + 0.5);
    return n < 1 ? 1 : n;
}

static void reverb_setup(Reverb* r, double sr) {
    r->sample_rate = sr;
    for (int i=0;i<NUM_COMBS;i++) {
        int sL = scale_len(comb_tuning[i], sr);
        int sR = scale_len(comb_tuning[i] + STEREO_SPREAD, sr);
        if (sL > COMB_MAX) sL = COMB_MAX;
        if (sR > COMB_MAX) sR = COMB_MAX;
        r->combL[i].size = sL; r->combL[i].idx = 0; r->combL[i].filterstore = 0.0f;
        r->combR[i].size = sR; r->combR[i].idx = 0; r->combR[i].filterstore = 0.0f;
        buf_clear(r->combL[i].buf, sL);
        buf_clear(r->combR[i].buf, sR);
    }
    for (int i=0;i<NUM_ALLPASS;i++) {
        int sL = scale_len(allpass_tuning[i], sr);
        int sR = scale_len(allpass_tuning[i] + STEREO_SPREAD, sr);
        if (sL > ALLPASS_MAX) sL = ALLPASS_MAX;
        if (sR > ALLPASS_MAX) sR = ALLPASS_MAX;
        r->apL[i].size = sL; r->apL[i].idx = 0;
        r->apR[i].size = sR; r->apR[i].idx = 0;
        buf_clear(r->apL[i].buf, sL);
        buf_clear(r->apR[i].buf, sR);
    }
    for (int i=0;i<NUM_DIFFUSE;i++) {
        int sL = scale_len(diffuse_tuning[i], sr);
        int sR = scale_len(diffuse_tuning[i] + 13, sr);
        if (sL > DIFFUSE_MAX) sL = DIFFUSE_MAX;
        if (sR > DIFFUSE_MAX) sR = DIFFUSE_MAX;
        r->dfL[i].size = sL; r->dfL[i].idx = 0;
        r->dfR[i].size = sR; r->dfR[i].idx = 0;
        buf_clear(r->dfL[i].buf, sL);
        buf_clear(r->dfR[i].buf, sR);
    }
    /* early reflection ring + taps */
    int er_len = scale_len(early_tap[NUM_EARLY-1] + 64, sr);
    if (er_len > EARLY_MAX) er_len = EARLY_MAX;
    r->earL.size = er_len; r->earL.idx = 0; buf_clear(r->earL.buf, er_len);
    r->earR.size = er_len; r->earR.idx = 0; buf_clear(r->earR.buf, er_len);
    for (int i=0;i<NUM_EARLY;i++) {
        int tL = scale_len(early_tap[i], sr);
        int tR = scale_len(early_tap[i] + STEREO_SPREAD, sr);
        if (tL >= er_len) tL = er_len-1;
        if (tR >= er_len) tR = er_len-1;
        r->earL.tap[i] = tL;
        r->earR.tap[i] = tR;
    }
    r->lpL = r->lpR = 0.0f;
    r->hpL = r->hpR = 0.0f;
    r->lfo1 = 0.0f;
    r->lfo2 = 1.7f; /* offset second LFO */
    meter_ring_clear(r->meter_buf);
    r->meter_idx = 0;
    r->meter_filled = 0;
}

/* comb with fractional (modulated) read tap. mod = fractional sample offset. */
static inline float comb_process_mod(Comb* c, float in, float feedback,
                                     float damp, float mod) {
    int size = c->size;
    /* read position with negative modulation offset, fractional */
    float rp = (float)c->idx - mod;
    while (rp < 0.0f) rp += (float)size;
    int i0 = (int)rp;
    if (i0 >= size) i0 -= size;
    int i1 = i0 + 1; if (i1 >= size) i1 = 0;
    float frac = rp - (float)i0;
    float out = c->buf[i0] + (c->buf[i1] - c->buf[i0]) * frac;

    c->filterstore = out * (1.0f - damp) + c->filterstore * damp;
    c->buf[c->idx] = in + c->filterstore * feedback;
    if (++c->idx >= size) c->idx = 0;
    return out;
}

static inline float allpass_process(Allpass* a, float in) {
    float bufout = a->buf[a->idx];
    float out = -in + bufout;
    a->buf[a->idx] = in + bufout * 0.5f;
    if (++a->idx >= a->size) a->idx = 0;
    return out;
}

static inline float diffuse_process(Diffuse* d, float in, float g) {
    float bufout = d->buf[d->idx];
    float out = -in + bufout;
    d->buf[d->idx] = in + bufout * g;
    if (++d->idx >= d->size) d->idx = 0;
    return out;
}

/* tap an early-reflection ring */
static inline float early_tap_read(Early* e, int tap) {
    int p = e->idx - tap;
    if (p < 0) p += e->size;
    return e->buf[p];
}
static inline void early_write(Early* e, float in) {
    e->buf[e->idx] = in;
    if (++e->idx >= e->size) e->idx = 0;
}

/* ----- exports ----- */
__attribute__((export_name("dsp_create")))
int32_t dsp_create(double sample_rate, int32_t max_block_size) {
    if (g_next >= MAX_INSTANCES) return -1;
    int h = g_next++;
    Reverb* r = &g_inst[h];
    /* defaults (id order) */
    r->p[0]=0.5f;  /* Size      */
    r->p[1]=0.5f;  /* Decay     */
    r->p[2]=0.5f;  /* Damping   */
    r->p[3]=1.0f;  /* Width     */
    r->p[4]=0.4f;  /* Diffusion */
    r->p[5]=0.3f;  /* ModRate   */
    r->p[6]=0.25f; /* ModDepth  */
    r->p[7]=0.5f;  /* EarlyLevel*/
    r->p[8]=0.1f;  /* LowCut    */
    r->p[9]=0.8f;  /* HighCut   */
    r->p[10]=1.0f; /* AlgoOn    */
    r->p[11]=1.0f; /* GrainOn   */
    r->p[12]=1.0f; /* ModOn     */
    r->p[13]=1.0f; /* EarlyOn   */
    r->p[14]=0.33f;/* Mix       */
    r->max_block = max_block_size;
    r->active = 1;
    reverb_setup(r, sample_rate);
    return h;
}

__attribute__((export_name("dsp_destroy")))
void dsp_destroy(int32_t handle) { (void)handle; }

__attribute__((export_name("dsp_reset")))
void dsp_reset(int32_t handle, double sample_rate, int32_t max_block_size) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    g_inst[handle].max_block = max_block_size;
    reverb_setup(&g_inst[handle], sample_rate);
}

__attribute__((export_name("dsp_set_param")))
void dsp_set_param(int32_t handle, int32_t id, double value) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    if (id < 0 || id >= NUM_PARAMS) return;
    g_inst[handle].p[id] = clampf((float)value, 0.0f, 1.0f);
}

__attribute__((export_name("dsp_get_param")))
double dsp_get_param(int32_t handle, int32_t id) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0.0;
    if (id < 0 || id >= NUM_PARAMS) return 0.0;
    return g_inst[handle].p[id];
}

__attribute__((export_name("dsp_process")))
void dsp_process(int32_t handle, int32_t in_ptr, int32_t out_ptr,
                 int32_t frames, int32_t in_ch, int32_t out_ch) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Reverb* r = &g_inst[handle];
    float* in  = (float*)(uintptr_t)in_ptr;
    float* out = (float*)(uintptr_t)out_ptr;

    /* ----- map params to coefficients ----- */
    float size   = r->p[0];
    float decay  = r->p[1];
    float damp_n = r->p[2];
    float width  = r->p[3];
    float diff_n = r->p[4];
    float mrate  = r->p[5];
    float mdepth = r->p[6];
    float elev   = r->p[7];
    float lowcut = r->p[8];
    float highc  = r->p[9];
    float mix    = r->p[14];

    int algoOn  = r->p[10] >= 0.5f;
    int grainOn = r->p[11] >= 0.5f;
    int modOn   = r->p[12] >= 0.5f;
    int earlyOn = r->p[13] >= 0.5f;

    /* feedback: Freeverb-standard 0.84 ceiling. Going higher makes the
     * summed tail ring well past the output ceiling under all 4 sections. */
    float feedback = 0.70f + size * 0.12f + decay * 0.03f;   /* 0.70 .. ~0.85 */
    if (feedback > 0.84f) feedback = 0.84f;
    float damp     = damp_n * 0.4f;             /* 0 .. 0.4 */
    float diff_g   = 0.45f + diff_n * 0.30f;    /* allpass coef 0.45 .. 0.75 */
    float wet      = mix;
    float dry      = 1.0f - mix;

    /* stereo wet gains */
    float wet1 = wet * (width * 0.5f + 0.5f);
    float wet2 = wet * ((1.0f - width) * 0.5f);

    const float gain = 0.015f; /* Freeverb input scaling */

    /* modulation: LFO increment per sample (0.1 .. ~6 Hz) */
    float modHz  = 0.1f + mrate * mrate * 5.9f;
    float lfoInc = TWO_PI * modHz / (float)r->sample_rate;
    /* mod depth in samples (fractional) */
    float depthSamp = modOn ? (mdepth * 18.0f) : 0.0f;

    /* one-pole filter coefficients (cheap, stable) */
    /* HighCut: low-pass; coef 0=open .. higher cut as highc decreases */
    float lp_coef = 0.05f + highc * highc * 0.93f;      /* nearer 1 = more open */
    /* LowCut: high-pass; amount of low removed scales with lowcut */
    float hp_coef = lowcut * 0.4f;                      /* 0 .. 0.4 */

    float er_gain = earlyOn ? (elev * 0.9f) : 0.0f;

    for (int n = 0; n < frames; n++) {
        float inL, inR;
        if (in_ch >= 2) { inL = in[n*in_ch + 0]; inR = in[n*in_ch + 1]; }
        else            { inL = in[n*in_ch + 0]; inR = inL; }

        float input = (inL + inR) * gain;

        /* ---- modulation LFOs ---- */
        float m1 = 0.0f, m2 = 0.0f;
        if (modOn) {
            m1 = fast_sin(r->lfo1) * depthSamp;
            m2 = fast_sin(r->lfo2) * depthSamp;
            r->lfo1 += lfoInc; if (r->lfo1 > TWO_PI) r->lfo1 -= TWO_PI;
            r->lfo2 += lfoInc * 0.93f; if (r->lfo2 > TWO_PI) r->lfo2 -= TWO_PI;
        }

        /* ---- late algorithmic tail (combs + allpass) ---- */
        float lateL = 0.0f, lateR = 0.0f;
        if (algoOn) {
            for (int i = 0; i < NUM_COMBS; i++) {
                /* alternate combs use the two LFOs for richer motion */
                float mm = (i & 1) ? m2 : m1;
                lateL += comb_process_mod(&r->combL[i], input, feedback, damp, mm);
                lateR += comb_process_mod(&r->combR[i], input, feedback, damp, mm * 0.87f);
            }
            for (int i = 0; i < NUM_ALLPASS; i++) {
                lateL = allpass_process(&r->apL[i], lateL);
                lateR = allpass_process(&r->apR[i], lateR);
            }
        } else {
            /* keep comb buffers fed so toggling back on is smooth-ish but
               decayed; still advance allpass write heads with silence */
            for (int i = 0; i < NUM_COMBS; i++) {
                comb_process_mod(&r->combL[i], 0.0f, feedback, damp, m1);
                comb_process_mod(&r->combR[i], 0.0f, feedback, damp, m2);
            }
        }

        /* ---- diffusion / grain stages ---- */
        if (grainOn) {
            for (int i = 0; i < NUM_DIFFUSE; i++) {
                lateL = diffuse_process(&r->dfL[i], lateL, diff_g);
                lateR = diffuse_process(&r->dfR[i], lateR, diff_g);
            }
        }

        /* ---- early reflections (tapped delay bank) ---- */
        float erL = 0.0f, erR = 0.0f;
        /* always advance the ER ring so taps stay time-aligned */
        early_write(&r->earL, input);
        early_write(&r->earR, input);
        if (earlyOn) {
            for (int i = 0; i < NUM_EARLY; i++) {
                erL += early_tap_read(&r->earL, r->earL.tap[i]) * early_gain[i];
                erR += early_tap_read(&r->earR, r->earR.tap[i]) * early_gain[i];
            }
            erL *= er_gain;
            erR *= er_gain;
        }

        /* combine late tail + early reflections into wet bus */
        float wL = lateL + erL;
        float wR = lateR + erR;

        /* ---- wet-path filtering: LowCut (HP) then HighCut (LP) ---- */
        /* high-pass: y = x - lp(x) using a low-pass tracker */
        r->hpL += hp_coef * (wL - r->hpL);
        r->hpR += hp_coef * (wR - r->hpR);
        wL = wL - r->hpL;
        wR = wR - r->hpR;
        /* low-pass (high-cut) */
        r->lpL += lp_coef * (wL - r->lpL);
        r->lpR += lp_coef * (wR - r->lpR);
        wL = r->lpL;
        wR = r->lpR;

        /* stereo width matrix */
        float wetL = wL * wet1 + wR * wet2;
        float wetR = wR * wet1 + wL * wet2;

        float yL = inL * dry + wetL;
        float yR = inR * dry + wetR;

        /* hard brick-wall ceiling: protects against runaway peaks. */
        yL = hard_ceiling(yL, 0.95f);
        yR = hard_ceiling(yR, 0.95f);

        if (out_ch >= 2) { out[n*out_ch + 0] = yL; out[n*out_ch + 1] = yR; }
        else             { out[n*out_ch + 0] = (yL + yR) * 0.5f; }
        meter_ring_write(r->meter_buf, &r->meter_idx, &r->meter_filled, yL, yR);
    }
}

__attribute__((export_name("dsp_get_latency")))
int32_t dsp_get_latency(int32_t handle) { (void)handle; return 0; }

__attribute__((export_name("dsp_get_meter")))
int32_t dsp_get_meter(int32_t handle, int32_t ptr, int32_t max_samples) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    Reverb* r = &g_inst[handle];
    if (max_samples > METER_FRAMES) max_samples = METER_FRAMES;
    if (max_samples < 1) return 0;
    float* out = (float*)(uintptr_t)ptr;
    return meter_ring_read(r->meter_buf, r->meter_filled, r->meter_idx,
                           max_samples, out);
}

__attribute__((export_name("dsp_get_tail")))
int32_t dsp_get_tail(int32_t handle) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    Reverb* r = &g_inst[handle];
    /* tail grows with size + decay; report up to ~8s worth */
    float t = 1.0f + r->p[0] * 3.0f + r->p[1] * 4.0f;
    return (int32_t)(r->sample_rate * (double)t);
}

/* ----- state: store all NUM_PARAMS normalised params in id order ----- */
__attribute__((export_name("dsp_save_state")))
int32_t dsp_save_state(int32_t handle, int32_t ptr, int32_t max_bytes) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    if (max_bytes < (int32_t)(NUM_PARAMS*sizeof(float))) return 0;
    float* d = (float*)(uintptr_t)ptr;
    Reverb* r = &g_inst[handle];
    for (int i = 0; i < NUM_PARAMS; i++) d[i] = r->p[i];
    return (int32_t)(NUM_PARAMS*sizeof(float));
}

__attribute__((export_name("dsp_load_state")))
void dsp_load_state(int32_t handle, int32_t ptr, int32_t byte_count) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    if (byte_count < (int32_t)(NUM_PARAMS*sizeof(float))) return;
    float* s = (float*)(uintptr_t)ptr;
    Reverb* r = &g_inst[handle];
    for (int i = 0; i < NUM_PARAMS; i++) r->p[i] = clampf(s[i], 0.0f, 1.0f);
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
