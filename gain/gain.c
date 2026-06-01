/*
 * gain.c — CASC plugin: FAGAL BOOST (utility)
 *
 * Mastering-grade utility gain with:
 *   - Input trim (dB, -24 .. +24)
 *   - Gain (dB, -24 .. +24) with mid/side split (0 = mono, 0.5 = stereo,
 *     1 = wide stereo) — implemented as a cross-channel gain on S
 *   - Output trim (dB, -24 .. +24)
 *   - Soft brick-wall ceiling (tanh on |y|>c)
 *   - Polarity invert per channel
 *   - Channel link (when on, the L and R apply the same control values;
 *     here modeled by always reading the latest per-channel values but
 *     panning them as a stereo pair)
 *   - Hard bypass
 *
 * Self-contained: no libc.
 *
 * Build:
 *   clang --target=wasm32 -O3 -nostdlib -Wl,--no-entry -Wl,--export-dynamic \
 *         gain.c -o dsp.wasm
 *
 * Params (normalised 0..1):
 *   0 InTrim   -24 .. +24 dB
 *   1 Gain     -24 .. +24 dB
 *   2 OutTrim  -24 .. +24 dB
 *   3 MidSide  0 (mono) .. 1 (wide stereo)
 *   4 Ceiling  0.7 .. 1.0
 *   5 Link     0=independent, 1=linked
 *   6 PolL     0=normal, 1=invert
 *   7 PolR     0=normal, 1=invert
 *   8 Bypass   0=process, 1=bypass
 */
#include <stdint.h>

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
static inline float my_fabsf(float x) { return x < 0.0f ? -x : x; }

static float pow2p(float x) {
    /* x >= 0 */
    int i = (int)x;
    float f = x - (float)i;
    float p = 1.0f + f*(0.6931472f + f*(0.2402265f + f*(0.0555041f + f*0.0096181f)));
    float r = p;
    for (int k=0;k<i;k++) r *= 2.0f;
    return r;
}
static float pow10f(float x) {
    /* 10^x = 2^(x * log2(10)) = 2^(x * 3.321928) */
    float y = x * 3.32192809489f;
    if (y < 0.0f) return 1.0f / pow2p(-y);
    return pow2p(y);
}
static inline float db_to_linear(float db) {
    return pow10f(db / 20.0f);
}

static inline float fast_tanh(float x) {
    x = clampf(x, -8.0f, 8.0f);
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}
static inline float soft_ceiling(float x, float c) {
    if (c < 0.001f) c = 0.001f;
    return c * fast_tanh(x / c);
}

#define NUM_PARAMS 9
#define MAX_INSTANCES 8
#include "../_shared/meter_ring.h"

typedef struct {
    /* normalised params */
    float p_intrim, p_gain, p_outtrim, p_ms, p_ceil, p_link, p_poll, p_polr, p_bypass;

    /* smoothed per-sample gains (to avoid clicks) */
    float sInGain, sGain, sOutGain, sMS, sCeil;

    /* peak meters */
    float peakL, peakR;

    /* output ring buffer for UI meters */
    float meter_buf[METER_BUF];
    int   meter_idx;
    int   meter_filled;

    double sample_rate;
    int    max_block;
    int    active;
} Boost;

static Boost g_inst[MAX_INSTANCES];
static int g_next = 0;

static void boost_setup(Boost* b, double sr) {
    b->sample_rate = sr;
    b->sInGain = 1.0f;
    b->sGain = 1.0f;
    b->sOutGain = 1.0f;
    b->sMS = 0.0f;
    b->sCeil = 0.95f;
    b->peakL = 0.0f;
    b->peakR = 0.0f;
    meter_ring_clear(b->meter_buf);
    b->meter_idx = 0;
    b->meter_filled = 0;
}

__attribute__((export_name("dsp_create")))
int32_t dsp_create(double sample_rate, int32_t max_block_size) {
    if (g_next >= MAX_INSTANCES) return -1;
    int h = g_next++;
    Boost* b = &g_inst[h];
    b->p_intrim  = 0.75f;
    b->p_gain    = 0.62f;
    b->p_outtrim = 0.75f;
    b->p_ms      = 0.50f;
    b->p_ceil    = 0.95f;
    b->p_link    = 1.0f;
    b->p_poll    = 0.0f;
    b->p_polr    = 0.0f;
    b->p_bypass  = 0.0f;
    b->max_block = max_block_size;
    b->active    = 1;
    boost_setup(b, sample_rate);
    return h;
}

__attribute__((export_name("dsp_destroy")))
void dsp_destroy(int32_t handle) { (void)handle; }

__attribute__((export_name("dsp_reset")))
void dsp_reset(int32_t handle, double sample_rate, int32_t max_block_size) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    g_inst[handle].max_block = max_block_size;
    boost_setup(&g_inst[handle], sample_rate);
}

__attribute__((export_name("dsp_set_param")))
void dsp_set_param(int32_t handle, int32_t id, double value) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Boost* b = &g_inst[handle];
    float v = clampf((float)value, 0.0f, 1.0f);
    switch (id) {
        case 0: b->p_intrim  = v; break;
        case 1: b->p_gain    = v; break;
        case 2: b->p_outtrim = v; break;
        case 3: b->p_ms      = v; break;
        case 4: b->p_ceil    = v; break;
        case 5: b->p_link    = v; break;
        case 6: b->p_poll    = v; break;
        case 7: b->p_polr    = v; break;
        case 8: b->p_bypass  = v; break;
        default: break;
    }
}

__attribute__((export_name("dsp_get_param")))
double dsp_get_param(int32_t handle, int32_t id) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0.0;
    Boost* b = &g_inst[handle];
    switch (id) {
        case 0: return b->p_intrim;
        case 1: return b->p_gain;
        case 2: return b->p_outtrim;
        case 3: return b->p_ms;
        case 4: return b->p_ceil;
        case 5: return b->p_link;
        case 6: return b->p_poll;
        case 7: return b->p_polr;
        case 8: return b->p_bypass;
        default: return 0.0;
    }
}

__attribute__((export_name("dsp_process")))
void dsp_process(int32_t handle, int32_t in_ptr, int32_t out_ptr,
                 int32_t frames, int32_t in_ch, int32_t out_ch) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    Boost* b = &g_inst[handle];
    float* in  = (float*)(uintptr_t)in_ptr;
    float* out = (float*)(uintptr_t)out_ptr;

    int bypass = b->p_bypass >= 0.5f;
    int linked = b->p_link  >= 0.5f;

    /* dB params: 0..1 -> -24 .. +24 dB */
    float inDb  = (b->p_intrim  - 0.5f) * 48.0f;
    float gnDb  = (b->p_gain    - 0.5f) * 48.0f;
    float outDb = (b->p_outtrim - 0.5f) * 48.0f;
    float inG   = db_to_linear(inDb);
    float gnG   = db_to_linear(gnDb);
    float outG  = db_to_linear(outDb);
    float ms    = b->p_ms;   /* 0 = mono, 1 = wide */
    float ceilMag = 0.7f + b->p_ceil * 0.3f;

    float polL = (b->p_poll >= 0.5f) ? -1.0f : 1.0f;
    float polR = (b->p_polr >= 0.5f) ? -1.0f : 1.0f;
    if (linked) { /* link forces identical polarity */
        polR = polL;
    }

    /* per-sample smoothing (~5ms) */
    float sr = (float)b->sample_rate;
    float sc = 1.0f / (0.005f * sr + 1.0f);

    float peakL = b->peakL, peakR = b->peakR;
    float sIn = b->sInGain, sGn = b->sGain, sOut = b->sOutGain, sMS = b->sMS, sCeil = b->sCeil;

    /* peak meter envelope: slow release, instant attack */
    float peakRel = 0.00012f;

    for (int n = 0; n < frames; n++) {
        float inL, inR;
        if (in_ch >= 2) { inL = in[n*in_ch + 0]; inR = in[n*in_ch + 1]; }
        else            { inL = in[n*in_ch + 0]; inR = inL; }

        /* smooth controls */
        sIn   += sc * (inG  - sIn);
        sGn   += sc * (gnG  - sGn);
        sOut  += sc * (outG - sOut);
        sMS   += sc * (ms   - sMS);
        sCeil += sc * (ceilMag - sCeil);

        if (bypass) {
            /* hard bypass = pass-through with polarity invert still applied */
            if (out_ch >= 2) { out[n*out_ch + 0] = inL * polL; out[n*out_ch + 1] = inR * polR; }
            else             { out[n*out_ch + 0] = (inL * polL + inR * polR) * 0.5f; }
            continue;
        }

        /* input trim (with polarity) */
        float xL = inL * sIn * polL;
        float xR = inR * sIn * polR;

        /* mid/side cross-mix: at ms=0 we collapse to mono (side -> 0),
         * at ms=0.5 we keep the L/R (no cross), at ms=1 we widen */
        float sideMul = sMS * 2.0f;       /* 0..2 */
        float mid  = (xL + xR) * 0.5f;
        float side = (xL - xR) * 0.5f * sideMul;
        xL = mid + side;
        xR = mid - side;

        /* main gain */
        xL *= sGn;
        xR *= sGn;

        /* output trim + soft ceiling */
        xL = soft_ceiling(xL * sOut, sCeil);
        xR = soft_ceiling(xR * sOut, sCeil);

        /* peak meter update */
        float aL = my_fabsf(xL);
        float aR = my_fabsf(xR);
        peakL = (aL > peakL) ? aL : (peakL + peakRel * (aL - peakL));
        peakR = (aR > peakR) ? aR : (peakR + peakRel * (aR - peakR));

        if (out_ch >= 2) { out[n*out_ch + 0] = xL; out[n*out_ch + 1] = xR; }
        else             { out[n*out_ch + 0] = (xL + xR) * 0.5f; }
        meter_ring_write(b->meter_buf, &b->meter_idx, &b->meter_filled, xL, xR);
    }

    b->peakL  = peakL;  b->peakR  = peakR;
    b->sInGain= sIn;    b->sGain  = sGn;
    b->sOutGain=sOut;   b->sMS    = sMS;
    b->sCeil  = sCeil;
}

__attribute__((export_name("dsp_get_latency")))
int32_t dsp_get_latency(int32_t handle) { (void)handle; return 0; }

__attribute__((export_name("dsp_get_meter")))
int32_t dsp_get_meter(int32_t handle, int32_t ptr, int32_t max_samples) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    Boost* b = &g_inst[handle];
    if (max_samples > METER_FRAMES) max_samples = METER_FRAMES;
    if (max_samples < 1) return 0;
    float* out = (float*)(uintptr_t)ptr;
    int n = meter_ring_read(b->meter_buf, b->meter_filled, b->meter_idx,
                            max_samples, out);
    return n;
}

__attribute__((export_name("dsp_get_tail")))
int32_t dsp_get_tail(int32_t handle) { (void)handle; return 0; }

__attribute__((export_name("dsp_save_state")))
int32_t dsp_save_state(int32_t handle, int32_t ptr, int32_t max_bytes) {
    if (handle < 0 || handle >= MAX_INSTANCES) return 0;
    if (max_bytes < (int32_t)(NUM_PARAMS*sizeof(float))) return 0;
    float* o = (float*)(uintptr_t)ptr;
    Boost* b = &g_inst[handle];
    o[0]=b->p_intrim;  o[1]=b->p_gain;  o[2]=b->p_outtrim;
    o[3]=b->p_ms;      o[4]=b->p_ceil;  o[5]=b->p_link;
    o[6]=b->p_poll;    o[7]=b->p_polr;  o[8]=b->p_bypass;
    return (int32_t)(NUM_PARAMS*sizeof(float));
}

__attribute__((export_name("dsp_load_state")))
void dsp_load_state(int32_t handle, int32_t ptr, int32_t byte_count) {
    if (handle < 0 || handle >= MAX_INSTANCES) return;
    if (byte_count < (int32_t)(NUM_PARAMS*sizeof(float))) return;
    float* s = (float*)(uintptr_t)ptr;
    Boost* b = &g_inst[handle];
    b->p_intrim=s[0]; b->p_gain=s[1];   b->p_outtrim=s[2];
    b->p_ms=s[3];     b->p_ceil=s[4];   b->p_link=s[5];
    b->p_poll=s[6];   b->p_polr=s[7];   b->p_bypass=s[8];
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
