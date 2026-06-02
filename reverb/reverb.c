/*
 * reverb.c — CASC plugin: Fagal Space (Algorithmic Reverb)
 *
 * The DSP is the original Freeverb topology: 8 parallel damped feedback
 * comb filters per channel feeding 4 series allpass filters, with a
 * stereo spread on the comb tunings. This is the same algorithm as the
 * very first version of this plugin — simple, stable, no extra layers.
 *
 * Build:
 *   clang --target=wasm32 -O3 -nostdlib -Wl,--no-entry -Wl,--export-dynamic \
 *         reverb.c -o dsp.wasm
 *
 * Params (normalised 0..1):
 *   0 Size     room size / comb feedback
 *   1 Damping  high-frequency damping in the combs
 *   2 Width    stereo width of the wet signal
 *   3 Mix      dry/wet
 */
#include <stdint.h>
#include "../_shared/meter_ring.h"

/* ----- minimal math (no libc) ----- */
static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

/* Freeverb tuning constants (samples @ 44.1kHz). Scaled by SR at reset. */
#define NUM_COMBS    8
#define NUM_ALLPASS  4
#define STEREO_SPREAD 23

static const int comb_tuning[NUM_COMBS] = {
    1116, 1188, 1277, 1356, 1422, 1491, 1557, 1617
};
static const int allpass_tuning[NUM_ALLPASS] = { 556, 441, 341, 225 };

/* Max buffer sizes (largest tuning + spread, with headroom for high SR). */
#define COMB_MAX    3200
#define ALLPASS_MAX 1200

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
    Comb    combL[NUM_COMBS],   combR[NUM_COMBS];
    Allpass apL[NUM_ALLPASS],   apR[NUM_ALLPASS];

    /* output ring buffer for UI meters */
    float meter_buf[METER_BUF];
    int   meter_idx;
    int   meter_filled;

    /* normalised params */
    float p_size, p_damp, p_width, p_mix;

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
    /* tunings are for 44100; scale to current sr */
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
    meter_ring_clear(r->meter_buf);
    r->meter_idx = 0;
    r->meter_filled = 0;
}

static inline float comb_process(Comb* c, float in, float feedback, float damp) {
    float out = c->buf[c->idx];
    c->filterstore = out * (1.0f - damp) + c->filterstore * damp;
    c->buf[c->idx] = in + c->filterstore * feedback;
    if (++c->idx >= c->size) c->idx = 0;
    return out;
}

static inline float allpass_process(Allpass* a, float in) {
    float bufout = a->buf[a->idx];
    float out = -in + bufout;
    a->buf[a->idx] = in + bufout * 0.5f;
    if (++a->idx >= a->size) a->idx = 0;
    return out;
}

/* ----- exports ----- */
__attribute__((export_name("dsp_create")))
int32_t dsp_create(double sample_rate, int32_t max_block_size) {
    if (g_next >= MAX_INSTANCES) return -1;
    int h = g_next++;
    Reverb* r = &g_inst[h];
    r->p_size = 0.5f; r->p_damp = 0.5f; r->p_width = 1.0f; r->p_mix = 0.33f;
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
    Reverb* r = &g_inst[handle];
    float v = (float)value;
    if      (id == 0) r->p_size  = clampf(v,0,1);
    else if (id == 1) r->p_damp  = clampf(v,0,1);
    else if (id == 2) r->p_width = clampf(v,0,1);
    else if (id == 3) r->p_mix   = clampf(v,0,1);
}

__attribute__((export_name("dsp_get_param")))
double dsp_get_param(int32_t handle, int32_t id) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0.0;
    Reverb* r = &g_inst[handle];
    if      (id == 0) return r->p_size;
    else if (id == 1) return r->p_damp;
    else if (id == 2) return r->p_width;
    else if (id == 3) return r->p_mix;
    return 0.0;
}

__attribute__((export_name("dsp_process")))
void dsp_process(int32_t handle, int32_t in_ptr, int32_t out_ptr,
                 int32_t frames, int32_t in_ch, int32_t out_ch) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Reverb* r = &g_inst[handle];
    float* in  = (float*)(uintptr_t)in_ptr;
    float* out = (float*)(uintptr_t)out_ptr;

    /* Map params to algorithm coefficients. */
    float feedback = 0.7f + r->p_size * 0.28f;     /* 0.70 .. 0.98 */
    float damp     = r->p_damp * 0.4f;             /* 0 .. 0.4 */
    float wet      = r->p_mix;
    float dry      = 1.0f - r->p_mix;
    float width    = r->p_width;
    /* stereo wet gains */
    float wet1 = wet * (width * 0.5f + 0.5f);
    float wet2 = wet * ((1.0f - width) * 0.5f);
    const float gain = 0.015f; /* input scaling (Freeverb) */

    for (int n = 0; n < frames; n++) {
        float inL, inR;
        if (in_ch >= 2) { inL = in[n*in_ch + 0]; inR = in[n*in_ch + 1]; }
        else            { inL = in[n*in_ch + 0]; inR = inL; }

        float input = (inL + inR) * gain;
        float outL = 0.0f, outR = 0.0f;

        for (int i = 0; i < NUM_COMBS; i++) {
            outL += comb_process(&r->combL[i], input, feedback, damp);
            outR += comb_process(&r->combR[i], input, feedback, damp);
        }
        for (int i = 0; i < NUM_ALLPASS; i++) {
            outL = allpass_process(&r->apL[i], outL);
            outR = allpass_process(&r->apR[i], outR);
        }

        float wetL = outL * wet1 + outR * wet2;
        float wetR = outR * wet1 + outL * wet2;

        float yL = inL * dry + wetL;
        float yR = inR * dry + wetR;

        if (out_ch >= 2) { out[n*out_ch + 0] = yL; out[n*out_ch + 1] = yR; }
        else             { out[n*out_ch + 0] = (yL + yR) * 0.5f; }
        meter_ring_write(r->meter_buf, &r->meter_idx, &r->meter_filled, yL, yR);
    }
}

__attribute__((export_name("dsp_get_latency")))
int32_t dsp_get_latency(int32_t handle) { (void)handle; return 0; }

__attribute__((export_name("dsp_get_tail")))
int32_t dsp_get_tail(int32_t handle) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    /* tail grows with size; report up to ~4s worth */
    return (int32_t)(g_inst[handle].sample_rate * (1.0 + g_inst[handle].p_size * 3.0));
}

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

/* ----- state: store the 4 normalised params ----- */
__attribute__((export_name("dsp_save_state")))
int32_t dsp_save_state(int32_t handle, int32_t ptr, int32_t max_bytes) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    if (max_bytes < (int32_t)(4*sizeof(float))) return 0;
    float* d = (float*)(uintptr_t)ptr;
    Reverb* r = &g_inst[handle];
    d[0]=r->p_size; d[1]=r->p_damp; d[2]=r->p_width; d[3]=r->p_mix;
    return (int32_t)(4*sizeof(float));
}

__attribute__((export_name("dsp_load_state")))
void dsp_load_state(int32_t handle, int32_t ptr, int32_t byte_count) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    if (byte_count < (int32_t)(4*sizeof(float))) return;
    float* s = (float*)(uintptr_t)ptr;
    Reverb* r = &g_inst[handle];
    r->p_size=s[0]; r->p_damp=s[1]; r->p_width=s[2]; r->p_mix=s[3];
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
