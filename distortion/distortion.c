/*
 * distortion.c — CASC plugin: FAGAL DRIVE
 *
 * A multi-mode waveshaper with:
 *   - Five shaper curves: Tube, Tape, Soft, Hard, Fold
 *   - Pre- and post-tone one-pole filters (independent)
 *   - DC bias (asymmetric drive for grit / character)
 *   - Dynamics: an envelope follower that pulls drive back on loud peaks
 *     and lifts it on quiet material (auto-gain-ish).
 *   - A soft brick-wall ceiling (tanh on |y|>1) so you can push drive hard
 *     without digital overs or harsh clipping downstream.
 *   - Per-channel state; full dry/wet mix.
 *
 * Self-contained: no math.h / no libc.
 *
 * Build:
 *   clang --target=wasm32 -O3 -nostdlib -Wl,--no-entry -Wl,--export-dynamic \
 *         distortion.c -o dsp.wasm
 *
 * Params (normalised 0..1):
 *   0 Drive     1 .. 60 pre-shaper gain
 *   1 Bias      DC offset added pre-shaper, mapped -0.5 .. +0.5
 *   2 PreTone   one-pole lowpass pre-shaper (0 dark .. 1 open)
 *   3 PostTone  one-pole lowpass post-shaper (0 dark .. 1 open)
 *   4 Dynamics  amount of envelope follower attenuation (0 = none, 1 = strong)
 *   5 Ceiling   soft brick-wall ceiling, mapped 0.7 .. 1.0 in |x|
 *   6 Mode      0..4 -> Tube, Tape, Soft, Hard, Fold
 *   7 Output    linear output level 0 .. 1
 *   8 Mix       dry/wet
 */
#include <stdint.h>
#include "../_shared/meter_ring.h"

/* ----- minimal math (no libc) ----- */
static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
static inline float my_fabsf(float x) { return x < 0.0f ? -x : x; }

/* Cheap smooth soft-clip. Bounded in (-1, 1). */
static inline float fast_tanh(float x) {
    x = clampf(x, -8.0f, 8.0f);
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* Soft brick-wall: linear below c, asymptote to c above. y = c * tanh(x/c). */
static inline float soft_ceiling(float x, float c) {
    if (c < 0.001f) c = 0.001f;
    return c * fast_tanh(x / c);
}

#define NUM_PARAMS 9
#define MAX_INSTANCES 8

typedef struct {
    /* normalised params */
    /* output ring buffer for UI meters */
    float meter_buf[METER_BUF];
    int   meter_idx;
    int   meter_filled;

    float p_drive, p_bias, p_pretone, p_posttone, p_dyn;
    float p_ceil, p_mode, p_output, p_mix;

    /* tone filter states (per channel) */
    float preL, preR;   /* pre-shaper LP */
    float postL, postR; /* post-shaper LP */

    /* envelope follower (per channel) for dynamics */
    float envL, envR;

    double sample_rate;
    int    max_block;
    int    active;
} Drive;

static Drive g_inst[MAX_INSTANCES];
static int g_next = 0;

static void drive_setup(Drive* d, double sr) {
    d->sample_rate = sr;
    d->preL = d->preR = 0.0f;
    d->postL = d->postR = 0.0f;
    d->envL = d->envR = 0.0f;

    meter_ring_clear(d->meter_buf);
    d->meter_idx = 0;
    d->meter_filled = 0;}

/* ----- shaper curves (all in -> out, both in (-inf,inf) but bounded near +/-1) ----- */

/* Tube: warm asymmetric tanh with a touch of even harmonic via squaring. */
static inline float sh_tube(float x) {
    float y = x + 0.12f * x * x * x;
    return fast_tanh(y * 1.05f);
}

/* Tape: medium-saturated tanh with mild asymmetry (bias hint). */
static inline float sh_tape(float x) {
    return fast_tanh(x * 0.9f) + 0.05f * (x * x) * (x < 0 ? -1.0f : 1.0f);
}

/* Soft: classic tanh. */
static inline float sh_soft(float x) {
    return fast_tanh(x);
}

/* Hard: a hard-clipper with soft knee. */
static inline float sh_hard(float x) {
    if (x >  1.0f) return  1.0f;
    if (x < -1.0f) return -1.0f;
    if (x >  0.3333f) return  1.0f;
    if (x < -0.3333f) return -1.0f;
    return 3.0f * x;
}

/* Fold: wavefolder. Reflect signal into +/-1. */
static inline float sh_fold(float x) {
    float y = x;
    while (y >  1.0f) y = 2.0f - y;
    while (y < -1.0f) y = -2.0f - y;
    return fast_tanh(y * 1.2f);
}

static inline int mode_index(float p) {
    int m = (int)(p * 4.999f);
    if (m < 0) m = 0;
    if (m > 4) m = 4;
    return m;
}

static inline float apply_shape(int mode, float x) {
    switch (mode) {
        case 0:  return sh_tube(x);
        case 1:  return sh_tape(x);
        case 2:  return sh_soft(x);
        case 3:  return sh_hard(x);
        default: return sh_fold(x);
    }
}

/* ----- exports ----- */
__attribute__((export_name("dsp_create")))
int32_t dsp_create(double sample_rate, int32_t max_block_size) {
    if (g_next >= MAX_INSTANCES) return -1;
    int h = g_next++;
    Drive* d = &g_inst[h];
    d->p_drive    = 0.45f;
    d->p_bias     = 0.50f;   /* centered -> 0 */
    d->p_pretone  = 0.65f;
    d->p_posttone = 0.60f;
    d->p_dyn      = 0.0f;
    d->p_ceil     = 0.95f;
    d->p_mode     = 0.0f;    /* Tube */
    d->p_output   = 0.80f;
    d->p_mix      = 1.0f;
    d->max_block  = max_block_size;
    d->active     = 1;
    drive_setup(d, sample_rate);
    return h;
}

__attribute__((export_name("dsp_destroy")))
void dsp_destroy(int32_t handle) { (void)handle; }

__attribute__((export_name("dsp_reset")))
void dsp_reset(int32_t handle, double sample_rate, int32_t max_block_size) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    g_inst[handle].max_block = max_block_size;
    drive_setup(&g_inst[handle], sample_rate);
}

__attribute__((export_name("dsp_set_param")))
void dsp_set_param(int32_t handle, int32_t id, double value) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Drive* d = &g_inst[handle];
    float v = clampf((float)value, 0.0f, 1.0f);
    switch (id) {
        case 0: d->p_drive    = v; break;
        case 1: d->p_bias     = v; break;
        case 2: d->p_pretone  = v; break;
        case 3: d->p_posttone = v; break;
        case 4: d->p_dyn      = v; break;
        case 5: d->p_ceil     = v; break;
        case 6: d->p_mode     = v; break;
        case 7: d->p_output   = v; break;
        case 8: d->p_mix      = v; break;
        default: break;
    }
}

__attribute__((export_name("dsp_get_param")))
double dsp_get_param(int32_t handle, int32_t id) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0.0;
    Drive* d = &g_inst[handle];
    switch (id) {
        case 0: return d->p_drive;
        case 1: return d->p_bias;
        case 2: return d->p_pretone;
        case 3: return d->p_posttone;
        case 4: return d->p_dyn;
        case 5: return d->p_ceil;
        case 6: return d->p_mode;
        case 7: return d->p_output;
        case 8: return d->p_mix;
        default: return 0.0;
    }
}

__attribute__((export_name("dsp_process")))
void dsp_process(int32_t handle, int32_t in_ptr, int32_t out_ptr,
                 int32_t frames, int32_t in_ch, int32_t out_ch) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Drive* d = &g_inst[handle];
    float* in  = (float*)(uintptr_t)in_ptr;
    float* out = (float*)(uintptr_t)out_ptr;

    int   mode    = mode_index(d->p_mode);
    float drive   = 1.0f + d->p_drive * 59.0f;          /* 1 .. 60 */
    float bias    = (d->p_bias - 0.5f);                 /* -0.5 .. +0.5 */
    /* tone: one-pole lowpass coefficients. 0 dark, 1 bright (bypass-ish). */
    float preA    = 0.04f + d->p_pretone  * d->p_pretone  * 0.94f;
    float postA   = 0.05f + d->p_posttone * d->p_posttone * 0.93f;
    /* ceiling: tanh threshold magnitude */
    float ceilMag = 0.7f + d->p_ceil * 0.3f;            /* 0.7 .. 1.0 */
    /* dynamics: envelope follower release per sample; amount of attenuation */
    float dynAmt  = d->p_dyn;
    float envRel  = 0.0006f;                            /* follower release */
    float envAtk  = 0.05f;                              /* follower attack */
    float outg    = d->p_output;
    float wet     = d->p_mix;
    float dry     = 1.0f - wet;

    float preL = d->preL, preR = d->preR;
    float postL = d->postL, postR = d->postR;
    float envL = d->envL, envR = d->envR;

    for (int n = 0; n < frames; n++) {
        float inL, inR;
        if (in_ch >= 2) { inL = in[n*in_ch + 0]; inR = in[n*in_ch + 1]; }
        else            { inL = in[n*in_ch + 0]; inR = inL; }

        /* pre-tone one-pole lowpass */
        preL += preA * (inL - preL);
        preR += preA * (inR - preR);
        float preSL = preL + bias;
        float preSR = preR + bias;

        /* envelope follower (per-channel) on pre-tone signal */
        float aL = my_fabsf(preSL);
        float aR = my_fabsf(preSR);
        envL = (aL > envL) ? (envL + envAtk * (aL - envL)) : (envL + envRel * (aL - envL));
        envR = (aR > envR) ? (envR + envAtk * (aR - envR)) : (envR + envRel * (aR - envR));
        /* gain reduction: when env > ref, attenuate by 1 / (1 + dynAmt * (env/ref - 1)) */
        float ref = 0.15f;
        float gL = (envL > ref) ? (1.0f / (1.0f + dynAmt * ((envL / ref) - 1.0f))) : 1.0f;
        float gR = (envR > ref) ? (1.0f / (1.0f + dynAmt * ((envR / ref) - 1.0f))) : 1.0f;

        /* shaper */
        float sL = apply_shape(mode, preSL * drive * gL);
        float sR = apply_shape(mode, preSR * drive * gR);

        /* post-tone one-pole lowpass */
        postL += postA * (sL - postL);
        postR += postA * (sR - postR);

        /* soft brick-wall ceiling */
        float yL = soft_ceiling(postL, ceilMag);
        float yR = soft_ceiling(postR, ceilMag);

        /* output gain */
        yL *= outg;
        yR *= outg;

        /* dry/wet */
        float oL = inL * dry + yL * wet;
        float oR = inR * dry + yR * wet;

        if (out_ch >= 2) { out[n*out_ch + 0] = oL; out[n*out_ch + 1] = oR; }
        else             { out[n*out_ch + 0] = (oL + oR) * 0.5f; }

        if (out_ch >= 2) { out[n*out_ch + 0] = oL; out[n*out_ch + 1] = oR; }
        else             { out[n*out_ch + 0] = (oL + oR) * 0.5f; }
        meter_ring_write(d->meter_buf, &d->meter_idx, &d->meter_filled, yL, yR);
    }

    d->preL  = preL;  d->preR  = preR;
    d->postL = postL; d->postR = postR;
    d->envL  = envL;  d->envR  = envR;
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
    float* o = (float*)(uintptr_t)ptr;
    Drive* d = &g_inst[handle];
    o[0]=d->p_drive;    o[1]=d->p_bias;     o[2]=d->p_pretone;
    o[3]=d->p_posttone; o[4]=d->p_dyn;      o[5]=d->p_ceil;
    o[6]=d->p_mode;     o[7]=d->p_output;   o[8]=d->p_mix;
    return (int32_t)(NUM_PARAMS*sizeof(float));
}

__attribute__((export_name("dsp_load_state")))
void dsp_load_state(int32_t handle, int32_t ptr, int32_t byte_count) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    if (byte_count < (int32_t)(NUM_PARAMS*sizeof(float))) return;
    float* s = (float*)(uintptr_t)ptr;
    Drive* d = &g_inst[handle];
    d->p_drive=s[0];    d->p_bias=s[1];     d->p_pretone=s[2];
    d->p_posttone=s[3]; d->p_dyn=s[4];      d->p_ceil=s[5];
    d->p_mode=s[6];     d->p_output=s[7];   d->p_mix=s[8];
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

__attribute__((export_name("dsp_get_meter")))
int32_t dsp_get_meter(int32_t handle, int32_t ptr, int32_t max_samples) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    Drive* s = &g_inst[handle];
    if (max_samples > METER_FRAMES) max_samples = METER_FRAMES;
    if (max_samples < 1) return 0;
    float* out = (float*)(uintptr_t)ptr;
    return meter_ring_read(s->meter_buf, s->meter_filled, s->meter_idx,
                           max_samples, out);
}
