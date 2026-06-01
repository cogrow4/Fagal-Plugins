/*
 * chorus.c — CASC plugin: FAGAL ENSEMBLE
 *
 * Expansive multi-voice modulation engine: a bank of independently-modulated
 * fractional delay-line voices fed by phase-offset sine LFOs. Three modes:
 *
 *   Chorus    — short delays, gentle sweep, no feedback (classic stereo chorus)
 *   Ensemble  — more voices, wider spread, richer detune cloud
 *   Flanger   — short center delay + feedback for the swept comb / jet sound
 *
 * Each voice has its own LFO phase offset (Spread) so the modulated taps fan
 * out across the stereo field. A one-pole tone control shapes the wet signal,
 * stereo Width controls the L/R cross-mix, and Feedback re-injects the summed
 * wet output (used heavily in Flanger mode).
 *
 * Self-contained: no math.h / no libc. Sine via parabolic approximation.
 *
 * Build:
 *   clang --target=wasm32 -O3 -nostdlib -Wl,--no-entry -Wl,--export-dynamic \
 *         chorus.c -o dsp.wasm
 *
 * Params (normalised 0..1):
 *   0 Rate     LFO rate, mapped 0.02 .. 8 Hz
 *   1 Depth    modulation depth, scales sweep around the center delay
 *   2 Mix      dry/wet
 *   3 Voices   active voice count, 1..MAX_VOICES (continuous -> rounded)
 *   4 Width    stereo width / spread of voices across L/R
 *   5 Tone     wet one-pole tone (dark .. bright)
 *   6 Feedback wet feedback (mostly for flanger), -0.95 .. +0.95
 *   7 Mode     0=Chorus 1=Ensemble 2=Flanger (steps:3)
 *   8 Center   base delay time, ~0.5 .. 25 ms
 */
#include <stdint.h>

/* ----- minimal math (no libc) ----- */
static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
static inline float my_fabsf(float x) { return x < 0.0f ? -x : x; }

#define PI     3.14159265358979323846f
#define TWO_PI (2.0f * PI)

/* Parabolic sine approximation. Input phase in -PI..PI. */
static inline float fast_sin(float x) {
    while (x >  PI) x -= TWO_PI;
    while (x < -PI) x += TWO_PI;
    float s = 1.27323954f * x - 0.405284735f * x * my_fabsf(x);
    s = 0.225f * (s * my_fabsf(s) - s) + s;
    return s;
}

#define DELAY_LEN  4096          /* ~42ms @ 96k, power of two */
#define DELAY_MASK (DELAY_LEN - 1)
#define MAX_VOICES 6

typedef struct {
    /* one stereo delay line shared by all voices (taps read at offsets) */
    float bufL[DELAY_LEN];
    float bufR[DELAY_LEN];
    int   widx;

    /* one LFO phase shared; per-voice offsets derived from index + spread */
    float lfo_phase;

    /* feedback state (summed wet of previous sample) */
    float fbL, fbR;

    /* tone one-pole state */
    float toneL, toneR;

    /* normalised params */
    float p_rate, p_depth, p_mix, p_voices, p_width, p_tone, p_fb, p_mode, p_center;

    double sample_rate;
    int    max_block;
    int    active;
} Ensemble;

#define NUM_PARAMS 9
#define MAX_INSTANCES 8
static Ensemble g_inst[MAX_INSTANCES];
static int g_next = 0;

/* ----- helpers ----- */
static void buf_clear(float* b, int n) { for (int i=0;i<n;i++) b[i]=0.0f; }

static void ensemble_setup(Ensemble* c, double sr) {
    c->sample_rate = sr;
    c->widx = 0;
    c->lfo_phase = 0.0f;
    c->fbL = c->fbR = 0.0f;
    c->toneL = c->toneR = 0.0f;
    buf_clear(c->bufL, DELAY_LEN);
    buf_clear(c->bufR, DELAY_LEN);
}

/* read delay buffer with linear interpolation; delay_samps is fractional,
 * measured backwards from the current write index. */
static inline float read_frac(const float* buf, int widx, float delay_samps) {
    float rpos = (float)widx - delay_samps;
    while (rpos < 0.0f)              rpos += (float)DELAY_LEN;
    while (rpos >= (float)DELAY_LEN) rpos -= (float)DELAY_LEN;
    int i0 = (int)rpos;
    float frac = rpos - (float)i0;
    int i1 = (i0 + 1) & DELAY_MASK;
    return buf[i0] + (buf[i1] - buf[i0]) * frac;
}

/* map p_voices (0..1) -> 1..MAX_VOICES */
static inline int voice_count(float p) {
    int v = (int)(p * (float)(MAX_VOICES - 1) + 0.5f) + 1;
    if (v < 1) v = 1;
    if (v > MAX_VOICES) v = MAX_VOICES;
    return v;
}

/* mode: 0..1 -> 0,1,2 */
static inline int mode_index(float p) {
    int m = (int)(p * 2.0f + 0.5f);
    if (m < 0) m = 0;
    if (m > 2) m = 2;
    return m;
}

/* ----- exports ----- */
__attribute__((export_name("dsp_create")))
int32_t dsp_create(double sample_rate, int32_t max_block_size) {
    if (g_next >= MAX_INSTANCES) return -1;
    int h = g_next++;
    Ensemble* c = &g_inst[h];
    c->p_rate   = 0.28f;
    c->p_depth  = 0.45f;
    c->p_mix    = 0.5f;
    c->p_voices = 0.6f;   /* ~4 voices */
    c->p_width  = 0.8f;
    c->p_tone   = 0.6f;
    c->p_fb     = 0.0f;   /* center = no feedback */
    c->p_mode   = 0.0f;   /* Chorus */
    c->p_center = 0.45f;
    c->max_block = max_block_size;
    c->active = 1;
    ensemble_setup(c, sample_rate);
    return h;
}

__attribute__((export_name("dsp_destroy")))
void dsp_destroy(int32_t handle) { (void)handle; }

__attribute__((export_name("dsp_reset")))
void dsp_reset(int32_t handle, double sample_rate, int32_t max_block_size) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    g_inst[handle].max_block = max_block_size;
    ensemble_setup(&g_inst[handle], sample_rate);
}

__attribute__((export_name("dsp_set_param")))
void dsp_set_param(int32_t handle, int32_t id, double value) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Ensemble* c = &g_inst[handle];
    float v = (float)value;
    switch (id) {
        case 0: c->p_rate   = clampf(v,0,1); break;
        case 1: c->p_depth  = clampf(v,0,1); break;
        case 2: c->p_mix    = clampf(v,0,1); break;
        case 3: c->p_voices = clampf(v,0,1); break;
        case 4: c->p_width  = clampf(v,0,1); break;
        case 5: c->p_tone   = clampf(v,0,1); break;
        case 6: c->p_fb     = clampf(v,0,1); break;
        case 7: c->p_mode   = clampf(v,0,1); break;
        case 8: c->p_center = clampf(v,0,1); break;
        default: break;
    }
}

__attribute__((export_name("dsp_get_param")))
double dsp_get_param(int32_t handle, int32_t id) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0.0;
    Ensemble* c = &g_inst[handle];
    switch (id) {
        case 0: return c->p_rate;
        case 1: return c->p_depth;
        case 2: return c->p_mix;
        case 3: return c->p_voices;
        case 4: return c->p_width;
        case 5: return c->p_tone;
        case 6: return c->p_fb;
        case 7: return c->p_mode;
        case 8: return c->p_center;
        default: return 0.0;
    }
}

__attribute__((export_name("dsp_process")))
void dsp_process(int32_t handle, int32_t in_ptr, int32_t out_ptr,
                 int32_t frames, int32_t in_ch, int32_t out_ch) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Ensemble* c = &g_inst[handle];
    float* in  = (float*)(uintptr_t)in_ptr;
    float* out = (float*)(uintptr_t)out_ptr;

    float sr = (float)c->sample_rate;

    int   mode   = mode_index(c->p_mode);
    int   nv     = voice_count(c->p_voices);

    /* Rate: 0.02 .. 8 Hz */
    float rateHz    = 0.02f + c->p_rate * 7.98f;
    float phase_inc = TWO_PI * rateHz / sr;

    /* Center delay: 0.5 .. 25 ms (flanger likes it short) */
    float center_ms = 0.5f + c->p_center * 24.5f;
    if (mode == 2) center_ms = 0.3f + c->p_center * 6.0f; /* flanger: short */

    /* Sweep depth in ms; ensemble pushes a touch wider */
    float depthScale = (mode == 1) ? 9.0f : (mode == 2 ? 3.0f : 7.0f);
    float sweep_ms   = 0.2f + c->p_depth * depthScale;

    float base_samps  = center_ms * 0.001f * sr;
    float sweep_samps = sweep_ms  * 0.001f * sr;

    /* Voice spread: how far each voice's LFO phase is offset.
     * Ensemble fans wider. Multiplied into per-voice phase + pan. */
    float spread = (mode == 1) ? 1.0f : (mode == 2 ? 0.25f : 0.7f);

    /* Width: 0 = mono-ish, 1 = full stereo fan */
    float width = c->p_width;

    /* Tone: one-pole lowpass coefficient. 0 = dark, 1 = bright (bypass). */
    float toneCoef = c->p_tone * c->p_tone; /* perceptual */

    /* Feedback: bipolar -0.92..+0.92, only meaningful for flanger but
     * available everywhere. */
    float fb = (c->p_fb * 2.0f - 1.0f) * 0.92f;
    if (mode != 2) fb *= 0.6f; /* tame feedback outside flanger */

    float wet = c->p_mix;
    float dry = 1.0f - c->p_mix;

    /* normalise voice sum so more voices don't blow up level */
    float vgain = 1.0f / (float)nv;

    float phase = c->lfo_phase;
    int   widx  = c->widx;
    float fbL = c->fbL, fbR = c->fbR;
    float toneL = c->toneL, toneR = c->toneR;

    for (int n = 0; n < frames; n++) {
        float inL, inR;
        if (in_ch >= 2) { inL = in[n*in_ch + 0]; inR = in[n*in_ch + 1]; }
        else            { inL = in[n*in_ch + 0]; inR = inL; }

        /* write input + feedback into the delay lines */
        c->bufL[widx] = inL + fbL * fb;
        c->bufR[widx] = inR + fbR * fb;

        float wetL = 0.0f, wetR = 0.0f;

        for (int v = 0; v < nv; v++) {
            /* per-voice phase offset spread across the LFO cycle */
            float voff = (float)v / (float)nv;            /* 0..1 */
            float vphase = phase + voff * TWO_PI * spread;

            float modA = fast_sin(vphase - PI);
            /* quadrature partner for the other channel */
            float modB = fast_sin(vphase - PI + (PI * 0.5f));

            float dA = base_samps + sweep_samps * modA;
            float dB = base_samps + sweep_samps * modB;

            if (dA < 1.0f) dA = 1.0f;
            if (dA > (float)(DELAY_LEN-2)) dA = (float)(DELAY_LEN-2);
            if (dB < 1.0f) dB = 1.0f;
            if (dB > (float)(DELAY_LEN-2)) dB = (float)(DELAY_LEN-2);

            float tA = read_frac(c->bufL, widx, dA);
            float tB = read_frac(c->bufR, widx, dB);

            /* per-voice pan: alternate voices lean L/R, scaled by width */
            float pan = ((v & 1) ? 1.0f : -1.0f) * voff * width;
            float gL = 0.5f * (1.0f - pan);
            float gR = 0.5f * (1.0f + pan);

            wetL += (tA * gL + tB * (1.0f - gL));
            wetR += (tB * gR + tA * (1.0f - gR));
        }

        wetL *= vgain;
        wetR *= vgain;

        /* tone: one-pole LP, blend toward bright as tone->1 */
        toneL += toneCoef * (wetL - toneL);
        toneR += toneCoef * (wetR - toneR);
        wetL = toneL + (wetL - toneL) * toneCoef; /* mild brighten near 1 */
        wetR = toneR + (wetR - toneR) * toneCoef;

        /* stereo width on the wet bus (mid/side-ish) */
        float mid  = (wetL + wetR) * 0.5f;
        float side = (wetL - wetR) * 0.5f * (0.5f + width);
        wetL = mid + side;
        wetR = mid - side;

        /* update feedback taps */
        fbL = wetL;
        fbR = wetR;

        float yL = inL * dry + wetL * wet;
        float yR = inR * dry + wetR * wet;

        if (out_ch >= 2) { out[n*out_ch + 0] = yL; out[n*out_ch + 1] = yR; }
        else             { out[n*out_ch + 0] = (yL + yR) * 0.5f; }

        widx = (widx + 1) & DELAY_MASK;
        phase += phase_inc;
        if (phase >= TWO_PI) phase -= TWO_PI;
    }

    c->lfo_phase = phase;
    c->widx = widx;
    c->fbL = fbL; c->fbR = fbR;
    c->toneL = toneL; c->toneR = toneR;
}

__attribute__((export_name("dsp_get_latency")))
int32_t dsp_get_latency(int32_t handle) { (void)handle; return 0; }

__attribute__((export_name("dsp_get_tail")))
int32_t dsp_get_tail(int32_t handle) { (void)handle; return 0; }

/* ----- state: store all NUM_PARAMS normalised params ----- */
__attribute__((export_name("dsp_save_state")))
int32_t dsp_save_state(int32_t handle, int32_t ptr, int32_t max_bytes) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    if (max_bytes < (int32_t)(NUM_PARAMS*sizeof(float))) return 0;
    float* d = (float*)(uintptr_t)ptr;
    Ensemble* c = &g_inst[handle];
    d[0]=c->p_rate;   d[1]=c->p_depth; d[2]=c->p_mix;
    d[3]=c->p_voices; d[4]=c->p_width; d[5]=c->p_tone;
    d[6]=c->p_fb;     d[7]=c->p_mode;  d[8]=c->p_center;
    return (int32_t)(NUM_PARAMS*sizeof(float));
}

__attribute__((export_name("dsp_load_state")))
void dsp_load_state(int32_t handle, int32_t ptr, int32_t byte_count) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    if (byte_count < (int32_t)(NUM_PARAMS*sizeof(float))) return;
    float* s = (float*)(uintptr_t)ptr;
    Ensemble* c = &g_inst[handle];
    c->p_rate=s[0];   c->p_depth=s[1]; c->p_mix=s[2];
    c->p_voices=s[3]; c->p_width=s[4]; c->p_tone=s[5];
    c->p_fb=s[6];     c->p_mode=s[7];  c->p_center=s[8];
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
