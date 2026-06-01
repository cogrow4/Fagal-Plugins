/*
 * filter.c — CASC plugin: FAGAL SCULPT
 *
 * Pro-grade state-variable filter with:
 *   - Four response shapes: Lowpass, Highpass, Bandpass, Notch
 *   - Independent pre-filter drive (one-pole soft clip on the input)
 *   - Envelope follower with adjustable amount and sensitivity,
 *     opening/closing the cutoff relative to the input level
 *   - LFO that modulates cutoff (rate + amount)
 *   - Keytrack (assumes host reports pitch via modulation, but here we model
 *     it as a static "cutoff offset" controlled by the user — p_keytrack
 *     re-biases the perceived cutoff around its nominal position; the host
 *     can drive p_cut directly for true keytracking)
 *   - Dry/Wet mix
 *
 * Self-contained: no libc / no math.h.
 *
 * Build:
 *   clang --target=wasm32 -O3 -nostdlib -Wl,--no-entry -Wl,--export-dynamic \
 *         filter.c -o dsp.wasm
 *
 * Params (normalised 0..1):
 *   0 Cutoff    ~20 Hz .. 20 kHz exponential
 *   1 Resonance 0 .. ~0.97
 *   2 Drive     pre-filter soft-clip drive (1..10)
 *   3 EnvAmt    envelope -> cutoff modulation depth (semitones * 0.1 .. ~3 oct)
 *   4 EnvSens   envelope follower sensitivity
 *   5 LFORate   LFO rate 0.05 .. 12 Hz
 *   6 LFOAmt    LFO -> cutoff modulation depth
 *   7 Keytrack  0 = off, 1 = strong (modeled as static bias re-centering)
 *   8 Type      0=LP, 1=HP, 2=BP, 3=Notch
 *   9 Mix       dry/wet
 */
#include <stdint.h>
#include "../_shared/meter_ring.h"

static inline float clampf(float x, float lo, float hi){
    return x<lo?lo:(x>hi?hi:x);
}
static inline float my_fabsf(float x){ return x<0.0f?-x:x; }

#define PI_F 3.14159265358979f
#define TWO_PI_F (2.0f * PI_F)

/* Parabolic sine, [-PI,PI]. */
static float fast_sin(float x){
    while (x >  PI_F) x -= TWO_PI_F;
    while (x < -PI_F) x += TWO_PI_F;
    float y = 1.27323954f*x - 0.405284735f*x*my_fabsf(x);
    y = 0.225f*(y*my_fabsf(y) - y) + y;
    return y;
}

/* 2^x for x>=0, used for cutoff mapping. */
static float pow2f(float x){
    if (x < 0.0f) x = 0.0f;
    int i = (int)x;
    float f = x - (float)i;
    float p = 1.0f + f*(0.6931472f + f*(0.2402265f + f*(0.0555041f + f*0.0096181f)));
    float r = p;
    for (int k=0;k<i;k++) r *= 2.0f;
    return r;
}
static float cutoff_hz(float v){
    /* 20 .. ~20000 */
    return 20.0f * pow2f(v * 9.96578f);
}

/* Cheap soft-clip for pre-drive. */
static inline float fast_tanh(float x) {
    x = clampf(x, -8.0f, 8.0f);
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

#define NUM_PARAMS 10
#define MAX_INSTANCES 8

typedef struct {
    /* SVF state, per channel */
    float lpL, bpL, lpR, bpR;

    /* envelope follower (per channel) */
    float envL, envR;

    /* LFO phase */
    float lfo;

    /* smoothed EnvAmt for click-free modulation */
    float sEnvAmt;

    /* normalised params */
    /* output ring buffer for UI meters */
    float meter_buf[METER_BUF];
    int   meter_idx;
    int   meter_filled;

    float p_cut, p_res, p_drive, p_env, p_sens, p_lforate, p_lfoamt, p_key, p_type, p_mix;

    double sample_rate;
    int    max_block;
    int    active;
} Sculpt;

static Sculpt g_inst[MAX_INSTANCES];
static int g_next = 0;

static void sculpt_setup(Sculpt* s, double sr) {
    s->sample_rate = sr;
    s->lpL = s->bpL = s->lpR = s->bpR = 0.0f;
    s->envL = s->envR = 0.0f;
    s->lfo = 0.0f;
    s->sEnvAmt = 0.0f;

    meter_ring_clear(s->meter_buf);
    s->meter_idx = 0;
    s->meter_filled = 0;}

__attribute__((export_name("dsp_create")))
int32_t dsp_create(double sample_rate, int32_t max_block_size) {
    if (g_next >= MAX_INSTANCES) return -1;
    int h = g_next++;
    Sculpt* s = &g_inst[h];
    s->p_cut    = 0.65f;
    s->p_res    = 0.25f;
    s->p_drive  = 0.0f;
    s->p_env    = 0.0f;
    s->p_sens   = 0.5f;
    s->p_lforate= 0.0f;
    s->p_lfoamt = 0.0f;
    s->p_key    = 0.0f;
    s->p_type   = 0.0f;  /* LP */
    s->p_mix    = 1.0f;
    s->max_block = max_block_size;
    s->active = 1;
    sculpt_setup(s, sample_rate);
    return h;
}

__attribute__((export_name("dsp_destroy")))
void dsp_destroy(int32_t handle) { (void)handle; }

__attribute__((export_name("dsp_reset")))
void dsp_reset(int32_t handle, double sample_rate, int32_t max_block_size) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    g_inst[handle].max_block = max_block_size;
    sculpt_setup(&g_inst[handle], sample_rate);
}

__attribute__((export_name("dsp_set_param")))
void dsp_set_param(int32_t handle, int32_t id, double value) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Sculpt* s = &g_inst[handle];
    float v = clampf((float)value, 0.0f, 1.0f);
    switch (id) {
        case 0:  s->p_cut     = v; break;
        case 1:  s->p_res     = v; break;
        case 2:  s->p_drive   = v; break;
        case 3:  s->p_env     = v; break;
        case 4:  s->p_sens    = v; break;
        case 5:  s->p_lforate = v; break;
        case 6:  s->p_lfoamt  = v; break;
        case 7:  s->p_key     = v; break;
        case 8:  s->p_type    = v; break;
        case 9:  s->p_mix     = v; break;
        default: break;
    }
}

__attribute__((export_name("dsp_get_param")))
double dsp_get_param(int32_t handle, int32_t id) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0.0;
    Sculpt* s = &g_inst[handle];
    switch (id) {
        case 0:  return s->p_cut;
        case 1:  return s->p_res;
        case 2:  return s->p_drive;
        case 3:  return s->p_env;
        case 4:  return s->p_sens;
        case 5:  return s->p_lforate;
        case 6:  return s->p_lfoamt;
        case 7:  return s->p_key;
        case 8:  return s->p_type;
        case 9:  return s->p_mix;
        default: return 0.0;
    }
}

__attribute__((export_name("dsp_process")))
void dsp_process(int32_t handle, int32_t in_ptr, int32_t out_ptr,
                 int32_t frames, int32_t in_ch, int32_t out_ch) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Sculpt* s = &g_inst[handle];
    float* in  = (float*)(uintptr_t)in_ptr;
    float* out = (float*)(uintptr_t)out_ptr;

    float sr = (float)s->sample_rate;
    float invSr = 1.0f / sr;

    /* base cutoff hz */
    float fcBase = cutoff_hz(s->p_cut);

    /* drive */
    float drive = 1.0f + s->p_drive * 9.0f;        /* 1..10 */

    /* resonance: higher p_res -> lower damping q */
    float q = 1.0f - s->p_res * 0.97f;
    if (q < 0.03f) q = 0.03f;

    /* LFO */
    float lfoHz   = 0.05f + s->p_lforate * 11.95f;
    float lfoInc  = TWO_PI_F * lfoHz * invSr;
    /* lfo -> cutoff: amount controls octaves of swing (0..3) */
    float lfoOct  = s->p_lfoamt * 3.0f;
    float lfo     = fast_sin(s->lfo);
    float lfoMul  = pow2f(lfo * lfoOct);            /* exp modulation */

    /* envelope follower */
    float envSens = s->p_sens;                     /* 0..1 */
    float envAtk  = 0.02f + envSens * 0.08f;       /* follower attack per sample */
    float envRel  = 0.0008f;
    /* env -> cutoff: amount in octaves, scaled by sensitivity */
    float envOctMax = s->p_env * 3.0f;              /* up to 3 oct swing */
    /* keytrack modeled as static re-centering (small bias on cutoff) */
    float keyBias = s->p_key * 0.15f;               /* 0..0.15 normalized */

    int   type = (int)(s->p_type * 3.999f);
    if (type < 0) type = 0; if (type > 3) type = 3;

    /* smoothed env amount */
    float sEnvInc = 1.0f / (0.005f * sr + 1.0f);

    float wet = s->p_mix, dry = 1.0f - wet;
    float lfoPhase = s->lfo;
    float envL = s->envL, envR = s->envR;
    float lpL = s->lpL, bpL = s->bpL, lpR = s->lpR, bpR = s->bpR;
    float sEnvAmt = s->sEnvAmt;

    for (int n = 0; n < frames; n++) {
        float inL, inR;
        if (in_ch >= 2) { inL = in[n*in_ch + 0]; inR = in[n*in_ch + 1]; }
        else            { inL = in[n*in_ch + 0]; inR = inL; }

        /* pre-filter drive (soft-clip) */
        float dL = fast_tanh(inL * drive);
        float dR = fast_tanh(inR * drive);

        /* envelope follower (per channel) on the dry input (post-drive too) */
        float aL = my_fabsf(dL);
        float aR = my_fabsf(dR);
        envL = (aL > envL) ? (envL + envAtk * (aL - envL)) : (envL + envRel * (aL - envL));
        envR = (aR > envR) ? (envR + envAtk * (aR - envR)) : (envR + envRel * (aR - envR));
        /* normalize envelope ~ to 0..1 using a soft squashing */
        float envNormL = 1.0f - 1.0f / (1.0f + envL * 4.0f);
        float envNormR = 1.0f - 1.0f / (1.0f + envR * 4.0f);
        float envOct = (envNormL + envNormR) * 0.5f * envOctMax;
        float envMul = pow2f(envOct - envOctMax * 0.5f);  /* center around 0 */

        /* smooth env amount a touch */
        sEnvAmt += sEnvInc * (s->p_env - sEnvAmt);

        /* combined cutoff multiplier: key bias (linear) + lfo + env */
        /* key bias shifts nominal cutoff; +0.15 doubles cutoff */
        float fcMul = pow2f(keyBias) * lfoMul * envMul;

        /* effective cutoff hz, clamped to algorithm-stable range */
        float fc = clampf(fcBase * fcMul, 15.0f, sr * 0.45f);
        float f = 2.0f * fast_sin(PI_F * fc / sr);
        if (f > 1.5f) f = 1.5f;

        /* left */
        lpL += f * bpL;
        float hpL = dL - lpL - q * bpL;
        bpL += f * hpL;
        float yL;
        switch (type) {
            case 0:  yL = lpL; break;       /* LP */
            case 1:  yL = hpL; break;       /* HP */
            case 2:  yL = bpL; break;       /* BP */
            default: yL = dL - hpL; break;  /* Notch = input - HP */
        }

        /* right */
        lpR += f * bpR;
        float hpR = dR - lpR - q * bpR;
        bpR += f * hpR;
        float yR;
        switch (type) {
            case 0:  yR = lpR; break;
            case 1:  yR = hpR; break;
            case 2:  yR = bpR; break;
            default: yR = dR - hpR; break;
        }

        float oL = inL * dry + yL * wet;
        float oR = inR * dry + yR * wet;

        if (out_ch >= 2) { out[n*out_ch + 0] = oL; out[n*out_ch + 1] = oR; }
        else             { out[n*out_ch + 0] = (oL + oR) * 0.5f; }

        if (out_ch >= 2) { out[n*out_ch + 0] = oL; out[n*out_ch + 1] = oR; }
        else             { out[n*out_ch + 0] = (oL + oR) * 0.5f; }
        meter_ring_write(s->meter_buf, &s->meter_idx, &s->meter_filled, oL, oR);

        /* advance lfo */
        lfoPhase += lfoInc;
        if (lfoPhase >= TWO_PI_F) lfoPhase -= TWO_PI_F;
    }

    s->lfo   = lfoPhase;
    s->envL  = envL;  s->envR  = envR;
    s->lpL   = lpL;   s->lpR   = lpR;
    s->bpL   = bpL;   s->bpR   = bpR;
    s->sEnvAmt = sEnvAmt;
}

__attribute__((export_name("dsp_get_latency")))
int32_t dsp_get_latency(int32_t handle) { (void)handle; return 0; }

__attribute__((export_name("dsp_get_tail")))
int32_t dsp_get_tail(int32_t handle) { (void)handle; return 0; }

__attribute__((export_name("dsp_save_state")))
int32_t dsp_save_state(int32_t handle, int32_t ptr, int32_t max_bytes) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    if (max_bytes < (int32_t)(NUM_PARAMS*sizeof(float))) return 0;
    float* d = (float*)(uintptr_t)ptr;
    Sculpt* s = &g_inst[handle];
    d[0]=s->p_cut; d[1]=s->p_res;   d[2]=s->p_drive;  d[3]=s->p_env;
    d[4]=s->p_sens;d[5]=s->p_lforate;d[6]=s->p_lfoamt; d[7]=s->p_key;
    d[8]=s->p_type;d[9]=s->p_mix;
    return (int32_t)(NUM_PARAMS*sizeof(float));
}

__attribute__((export_name("dsp_load_state")))
void dsp_load_state(int32_t handle, int32_t ptr, int32_t byte_count) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    if (byte_count < (int32_t)(NUM_PARAMS*sizeof(float))) return;
    float* s2 = (float*)(uintptr_t)ptr;
    Sculpt* s = &g_inst[handle];
    s->p_cut   =s2[0]; s->p_res   =s2[1]; s->p_drive =s2[2]; s->p_env   =s2[3];
    s->p_sens  =s2[4]; s->p_lforate=s2[5];s->p_lfoamt=s2[6]; s->p_key   =s2[7];
    s->p_type  =s2[8]; s->p_mix    =s2[9];
}

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
    Sculpt* s = &g_inst[handle];
    if (max_samples > METER_FRAMES) max_samples = METER_FRAMES;
    if (max_samples < 1) return 0;
    float* out = (float*)(uintptr_t)ptr;
    return meter_ring_read(s->meter_buf, s->meter_filled, s->meter_idx,
                           max_samples, out);
}
