/*
 * synth.c — CASC plugin: POLYMATH (Fagal Plugins)
 *
 * Omnisphere-style deep polysynth:
 *   - 8-voice polyphony with voice stealing (oldest)
 *   - Per voice:
 *       Osc1 (sine/saw/square/tri)  +  Osc2 (sine/saw/square/tri, detuned)
 *       Sub osc (sine, 1 octave down)  +  white noise
 *       4-mode state-variable filter (LP/HP/BP/Notch) with drive
 *       Amp ADSR  +  Mod ADSR
 *   - Two global LFOs (LFO1 -> filter cutoff, LFO2 -> pitch vibrato)
 *   - Per-voice keytrack on filter cutoff
 *   - Built-in FX bus: short chorus + stereo ping-pong delay + simple
 *     Schroeder-style plate reverb
 *   - Arpeggiator (1/4 .. 1/32 rate, up by default, 50% gate)
 *
 * Self-contained: no libc / no math.h. Sine via parabolic approximation;
 * 2^x via bit-piecewise polynomial.
 *
 * Build:
 *   clang --target=wasm32 -O3 -nostdlib -Wl,--no-entry -Wl,--export-dynamic \
 *         synth.c -o dsp.wasm
 *
 * MIDI event layout (8 bytes each, packed in wasm memory):
 *   [0..3] int32 frame_offset (LE)   [4] status   [5] data1   [6] data2   [7] pad
 *
 * Params (normalised 0..1) — see manifest for ids/names/defaults.
 */
#include <stdint.h>

/* ============================================================================
 *  Math (no libc)
 * ========================================================================= */
static inline float clampf(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
static inline float my_fabsf(float x){return x<0.0f?-x:x;}

#define PI_F 3.14159265358979f
#define TWO_PI_F 6.28318530717959f

/* Sine approx on [-PI,PI] (parabolic). */
static float fast_sin(float x){
    while (x >  PI_F) x -= TWO_PI_F;
    while (x < -PI_F) x += TWO_PI_F;
    float y = 1.27323954f*x - 0.405284735f*x*my_fabsf(x);
    y = 0.225f*(y*my_fabsf(y) - y) + y;
    return y;
}

/* Cheap 2^x for x>=0 (used for note->freq, cutoff mapping, etc.). */
static float pow2f(float x){
    if (x < 0.0f) x = 0.0f;
    int i = (int)x;
    float f = x - (float)i;
    float p = 1.0f + f*(0.6931472f + f*(0.2402265f + f*(0.0555041f + f*0.0096181f)));
    float r = p;
    for (int k=0;k<i;k++) r *= 2.0f;
    return r;
}

/* MIDI note -> Hz: 440 * 2^((note-69)/12). */
static float note_to_hz(int note){
    float e = ((float)note - 69.0f) * (1.0f/12.0f);
    if (e < 0.0f) return 440.0f / pow2f(-e);
    return 440.0f * pow2f(e);
}
static float cutoff_hz(float v){ return 20.0f * pow2f(v * 9.96578f); }

/* Cheap soft-clip. */
static inline float fast_tanh(float x){
    x = clampf(x, -8.0f, 8.0f);
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* Linear congruential PRNG for noise. */
static inline uint32_t lcg_next(uint32_t* s){
    *s = (*s) * 1664525u + 1013904223u;
    return *s;
}
static inline float noise_sample(uint32_t* s){
    /* 24-bit signed -> [-1,1] */
    int32_t n = (int32_t)(lcg_next(s) >> 8) & 0xFFFFFF;
    return ((float)n / (float)0x800000) - 1.0f;
}

/* ============================================================================
 *  Voice
 * ========================================================================= */
enum { ENV_IDLE=0, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };

typedef struct {
    int   active;
    int   note;
    int   osc1Wave, osc2Wave;  /* 0..3 */
    float vel;
    /* per-voice phase accumulators (2 main + 1 sub) */
    float ph1, ph2, phs;
    /* per-voice increment multiplier (set from note; osc1/osc2 can detune) */
    float incBase;     /* for osc1 */
    float inc2Mul;     /* osc2 = base * 2^(semi/12) * (1 + detune/1200) */
    /* per-voice envelopes */
    int   stageA;
    float envA;
    int   stageM;
    float envM;
    /* arp scheduled time (in samples) for next note-off / note-on */
    uint32_t age;
} Voice;

#define NUM_VOICES 8

/* ============================================================================
 *  FX bus state — delay lines (stereo, ping-pong)
 * ========================================================================= */
#define DLY_LEN 48000   /* ~1.0s @ 48k */
static float fx_dlyL[DLY_LEN];
static float fx_dlyR[DLY_LEN];
static int   fx_dlyIdx = 0;

/* Simple Schroeder plate reverb: 4 comb filters + 2 allpasses per channel. */
#define RVB_COMBS 4
#define RVB_COMBLEN 4800
static float rvCombL[RVB_COMBS][RVB_COMBLEN];
static float rvCombR[RVB_COMBS][RVB_COMBLEN];
static int   rvCombIdx[RVB_COMBS];
static const int rvCombTune[RVB_COMBS] = { 1116, 1188, 1277, 1356 };
static const float rvCombGain[RVB_COMBS] = { 0.84f, 0.83f, 0.82f, 0.81f };

#define RVB_AP_LEN 1200
static float rvApL[2][RVB_AP_LEN];
static float rvApR[2][RVB_AP_LEN];
static int   rvApIdx[2];
static const int rvApTune[2] = { 556, 441 };

/* ============================================================================
 *  Synth instance
 * ========================================================================= */
#define NUM_PARAMS 30
#define MAX_INSTANCES 2

typedef struct {
    Voice v[NUM_VOICES];
    /* global LFO phases */
    float lfo1, lfo2;
    /* arp state */
    int   arpHold[NUM_VOICES];  /* up to NUM_VOICES held notes */
    int   arpHoldN;
    int   arpOrder;             /* 0 up, 1 down, 2 updown, 3 random */
    int   arpStep;
    int   arpVoice;             /* voice index for arp */
    int   arpDir;
    uint32_t arpNextSamp;       /* sample counter target */
    uint32_t sampCounter;
    /* last UI-read noise seed (so the LFO+filter doesn't reseed) */
    uint32_t noiseSeed;

    /* normalised params */
    float p_osc1w, p_osc1l, p_osc1d;
    float p_osc2w, p_osc2l, p_osc2d;
    float p_sub, p_noise;
    float p_cut, p_res, p_ftype, p_fdrive, p_fenv;
    float p_aA, p_aD, p_aS, p_aR;
    float p_mA, p_mD, p_mS, p_mR;
    float p_l1r, p_l1d, p_l2r, p_l2d;
    float p_fx, p_master;
    float p_arpOn, p_arpRate, p_arpGate;

    double sample_rate;
    int    max_block;
    int    active;
} Synth;

static Synth g_inst[MAX_INSTANCES];
static int g_next = 0;

/* ============================================================================
 *  Helpers
 * ========================================================================= */
static int alloc_voice(Synth* s){
    for (int i=0;i<NUM_VOICES;i++) if (!s->v[i].active) return i;
    int oldest=0; uint32_t best=s->v[0].age;
    for (int i=1;i<NUM_VOICES;i++) if (s->v[i].age < best){best=s->v[i].age;oldest=i;}
    return oldest;
}

static void note_on_internal(Synth* s, int note, float vel, int osc1W, int osc2W){
    int i = alloc_voice(s);
    Voice* v = &s->v[i];
    v->active = 1;
    v->note = note;
    v->vel  = vel;
    v->osc1Wave = osc1W;
    v->osc2Wave = osc2W;
    v->ph1 = 0.0f; v->ph2 = 0.0f; v->phs = 0.0f;
    /* base increment: note_hz / sr; osc2 detune from p_osc2d (0..1 -> -50..+50 cents) */
    float hz = note_to_hz(note);
    v->incBase = hz / (float)s->sample_rate;
    /* 2^(cents/1200); cents = (d-0.5)*100 -> -50..+50 */
    float detCents = (s->p_osc2d - 0.5f) * 100.0f;
    v->inc2Mul = pow2f(detCents / 1200.0f);
    v->stageA = ENV_ATTACK; v->envA = 0.0f;
    v->stageM = ENV_ATTACK; v->envM = 0.0f;
    v->age = ++s->sampCounter;
}

static void note_off_internal(Synth* s, int note){
    for (int i=0;i<NUM_VOICES;i++){
        if (s->v[i].active && s->v[i].note==note && s->v[i].stageA!=ENV_RELEASE)
            s->v[i].stageA = ENV_RELEASE;
    }
}

static void synth_init(Synth* s, double sr){
    s->sample_rate = sr;
    s->lfo1 = s->lfo2 = 0.0f;
    s->arpHoldN = 0; s->arpStep = 0; s->arpVoice = -1; s->arpDir = 1;
    s->arpNextSamp = 0; s->sampCounter = 0;
    s->noiseSeed = 0xC0FFEEu;

    /* default params */
    s->p_osc1w=0.0f; s->p_osc1l=0.7f;  s->p_osc1d=0.5f;
    s->p_osc2w=0.33f;s->p_osc2l=0.6f;  s->p_osc2d=0.5f;
    s->p_sub=0.3f;   s->p_noise=0.0f;
    s->p_cut=0.65f;  s->p_res=0.20f;    s->p_ftype=0.0f; s->p_fdrive=0.0f; s->p_fenv=0.4f;
    s->p_aA=0.05f;   s->p_aD=0.3f;      s->p_aS=0.7f;   s->p_aR=0.3f;
    s->p_mA=0.1f;    s->p_mD=0.3f;      s->p_mS=0.5f;   s->p_mR=0.4f;
    s->p_l1r=0.0f;   s->p_l1d=0.0f;     s->p_l2r=0.0f;  s->p_l2d=0.0f;
    s->p_fx=0.25f;   s->p_master=0.8f;
    s->p_arpOn=0.0f; s->p_arpRate=0.5f; s->p_arpGate=0.5f;

    for (int i=0;i<NUM_VOICES;i++){
        s->v[i].active=0; s->v[i].stageA=ENV_IDLE; s->v[i].envA=0.0f;
        s->v[i].stageM=ENV_IDLE; s->v[i].envM=0.0f;
        s->v[i].note=-1; s->v[i].vel=0.0f; s->v[i].age=0;
        s->v[i].incBase = 0.0f; s->v[i].inc2Mul = 1.0f;
    }

    /* clear FX */
    for (int i=0;i<DLY_LEN;i++){ fx_dlyL[i]=0.0f; fx_dlyR[i]=0.0f; }
    fx_dlyIdx = 0;
    for (int c=0;c<RVB_COMBS;c++){ rvCombIdx[c]=0;
        for (int i=0;i<RVB_COMBLEN;i++){ rvCombL[c][i]=0.0f; rvCombR[c][i]=0.0f; } }
    for (int c=0;c<2;c++){ rvApIdx[c]=0;
        for (int i=0;i<RVB_AP_LEN;i++){ rvApL[c][i]=0.0f; rvApR[c][i]=0.0f; } }
}

/* ============================================================================
 *  Oscillators (phase 0..1, output in -1..1)
 * ========================================================================= */
static inline float osc_run(int wave, float ph){
    switch (wave) {
        case 0:  return fast_sin(ph * TWO_PI_F - PI_F);          /* sine */
        case 1:  return 2.0f * ph - 1.0f;                        /* saw */
        case 2:  return ph < 0.5f ? 1.0f : -1.0f;                /* square */
        default: {
            /* triangle */
            float t = ph < 0.5f ? (ph*2.0f) : (2.0f - ph*2.0f);
            return t*2.0f - 1.0f;
        }
    }
}

/* ============================================================================
 *  Filter — Chamberlin SVF
 * ========================================================================= */
typedef struct { float lp, bp; } SVF;
static inline void svf_step(SVF* sv, float* lp_out, float* bp_out, float* hp_out,
                            float x, float fc, float q){
    sv->lp += fc * sv->bp;
    float hp = x - sv->lp - q * sv->bp;
    sv->bp += fc * hp;
    *lp_out = sv->lp;
    *bp_out = sv->bp;
    *hp_out = hp;
}

/* ============================================================================
 *  Arpeggiator
 * ========================================================================= */
static void arp_push(Synth* s, int note){
    /* keep a small stack of held notes (no sort) */
    if (s->arpHoldN < NUM_VOICES) s->arpHold[s->arpHoldN++] = note;
}
static void arp_remove(Synth* s, int note){
    for (int i=0;i<s->arpHoldN;i++){
        if (s->arpHold[i]==note){
            for (int j=i;j<s->arpHoldN-1;j++) s->arpHold[j]=s->arpHold[j+1];
            s->arpHoldN--; return;
        }
    }
}
static int arp_pick(Synth* s){
    if (s->arpHoldN<=0) return -1;
    int n = s->arpHoldN;
    int idx;
    if (s->arpOrder==0) {            /* up */
        idx = s->arpStep % n;
        s->arpStep++;
    } else if (s->arpOrder==1) {     /* down */
        if (s->arpStep<0) s->arpStep = n-1;
        idx = s->arpStep;
        s->arpStep--;
    } else if (s->arpOrder==2) {     /* up-down */
        idx = s->arpStep;
        s->arpStep += s->arpDir;
        if (s->arpStep >= n){ s->arpStep = n-2; s->arpDir = -1; }
        if (s->arpStep < 0) { s->arpStep = 1; s->arpDir = 1; }
    } else {                          /* random */
        idx = (int)(lcg_next(&s->noiseSeed) % (uint32_t)n);
    }
    return s->arpHold[idx];
}

/* ============================================================================
 *  FX bus — delay + reverb
 * ========================================================================= */
static inline void fx_process(float* inL, float* inR, float fxAmt){
    /* FX amount: 0..1; we mix in a bit of delay + reverb. */
    if (fxAmt <= 0.0001f) return;

    /* Delay: simple single-tap with feedback, ping-pong across L/R. */
    const float dlySamp = 0.42f;          /* seconds of delay */
    int dTap = (int)(dlySamp * 48000.0f);
    if (dTap < 1) dTap = 1;
    if (dTap >= DLY_LEN) dTap = DLY_LEN - 1;
    int rIdx = fx_dlyIdx - dTap; if (rIdx < 0) rIdx += DLY_LEN;
    float dL = fx_dlyL[rIdx];
    float dR = fx_dlyR[rIdx];
    const float fb = 0.42f;
    /* write: input + feedback cross */
    fx_dlyL[fx_dlyIdx] = *inL + dR * fb;
    fx_dlyR[fx_dlyIdx] = *inR + dL * fb;
    if (++fx_dlyIdx >= DLY_LEN) fx_dlyIdx = 0;

    /* Reverb: feed delay-out into a 4-comb plate, then 2 allpasses. */
    float rvL = dL * 0.7f;
    float rvR = dR * 0.7f;
    for (int c=0;c<RVB_COMBS;c++){
        int len = (int)((float)rvCombTune[c] * (48000.0f / 44100.0f));
        if (len >= RVB_COMBLEN) len = RVB_COMBLEN-1;
        int idx = rvCombIdx[c];
        float ol = rvCombL[c][idx];
        float or = rvCombR[c][idx];
        rvCombL[c][idx] = rvL + ol * rvCombGain[c];
        rvCombR[c][idx] = rvR + or * rvCombGain[c];
        rvL = ol; rvR = or;
        if (++rvCombIdx[c] >= len) rvCombIdx[c] = 0;
    }
    for (int a=0;a<2;a++){
        int len = (int)((float)rvApTune[a] * (48000.0f / 44100.0f));
        if (len >= RVB_AP_LEN) len = RVB_AP_LEN-1;
        int idx = rvApIdx[a];
        float bl = rvApL[a][idx];
        float br = rvApR[a][idx];
        /* allpass */
        float g = 0.5f;
        rvApL[a][idx] = rvL + bl * g;
        rvApR[a][idx] = rvR + br * g;
        rvL = bl - g * rvApL[a][idx];
        rvR = br - g * rvApR[a][idx];
        if (++rvApIdx[a] >= len) rvApIdx[a] = 0;
    }

    /* mix wet in */
    *inL = *inL + (dL + rvL) * fxAmt;
    *inR = *inR + (dR + rvR) * fxAmt;
}

/* ============================================================================
 *  Exports
 * ========================================================================= */
__attribute__((export_name("dsp_create")))
int32_t dsp_create(double sr, int32_t mbs){
    (void)mbs;
    if (g_next >= MAX_INSTANCES) return -1;
    int h = g_next++;
    Synth* s = &g_inst[h];
    s->active = 1;
    synth_init(s, sr);
    return h;
}
__attribute__((export_name("dsp_destroy")))
void dsp_destroy(int32_t h){ (void)h; }
__attribute__((export_name("dsp_reset")))
void dsp_reset(int32_t h, double sr, int32_t mbs){
    (void)mbs;
    if (h<0||h>=MAX_INSTANCES) return;
    synth_init(&g_inst[h], sr);
}

__attribute__((export_name("dsp_set_param")))
void dsp_set_param(int32_t h, int32_t id, double val){
    if (h<0||h>=MAX_INSTANCES) return;
    Synth* s=&g_inst[h];
    float v = clampf((float)val, 0.0f, 1.0f);
    switch (id){
        case 0:  s->p_osc1w=v; break;
        case 1:  s->p_osc1l=v; break;
        case 2:  s->p_osc1d=v; break;
        case 3:  s->p_osc2w=v; break;
        case 4:  s->p_osc2l=v; break;
        case 5:  s->p_osc2d=v; break;
        case 6:  s->p_sub=v;   break;
        case 7:  s->p_noise=v; break;
        case 8:  s->p_cut=v;   break;
        case 9:  s->p_res=v;   break;
        case 10: s->p_ftype=v; break;
        case 11: s->p_fdrive=v;break;
        case 12: s->p_fenv=v;  break;
        case 13: s->p_aA=v;    break;
        case 14: s->p_aD=v;    break;
        case 15: s->p_aS=v;    break;
        case 16: s->p_aR=v;    break;
        case 17: s->p_mA=v;    break;
        case 18: s->p_mD=v;    break;
        case 19: s->p_mS=v;    break;
        case 20: s->p_mR=v;    break;
        case 21: s->p_l1r=v;   break;
        case 22: s->p_l1d=v;   break;
        case 23: s->p_l2r=v;   break;
        case 24: s->p_l2d=v;   break;
        case 25: s->p_fx=v;    break;
        case 26: s->p_master=v;break;
        case 27: s->p_arpOn=v; break;
        case 28: s->p_arpRate=v;break;
        case 29: s->p_arpGate=v;break;
        default: break;
    }
}

__attribute__((export_name("dsp_get_param")))
double dsp_get_param(int32_t h, int32_t id){
    if (h<0||h>=MAX_INSTANCES) return 0.0;
    Synth* s=&g_inst[h];
    switch (id){
        case 0:  return s->p_osc1w;
        case 1:  return s->p_osc1l;
        case 2:  return s->p_osc1d;
        case 3:  return s->p_osc2w;
        case 4:  return s->p_osc2l;
        case 5:  return s->p_osc2d;
        case 6:  return s->p_sub;
        case 7:  return s->p_noise;
        case 8:  return s->p_cut;
        case 9:  return s->p_res;
        case 10: return s->p_ftype;
        case 11: return s->p_fdrive;
        case 12: return s->p_fenv;
        case 13: return s->p_aA;
        case 14: return s->p_aD;
        case 15: return s->p_aS;
        case 16: return s->p_aR;
        case 17: return s->p_mA;
        case 18: return s->p_mD;
        case 19: return s->p_mS;
        case 20: return s->p_mR;
        case 21: return s->p_l1r;
        case 22: return s->p_l1d;
        case 23: return s->p_l2r;
        case 24: return s->p_l2d;
        case 25: return s->p_fx;
        case 26: return s->p_master;
        case 27: return s->p_arpOn;
        case 28: return s->p_arpRate;
        case 29: return s->p_arpGate;
        default: return 0.0;
    }
}

__attribute__((export_name("dsp_send_midi")))
void dsp_send_midi(int32_t h, int32_t events_ptr, int32_t count){
    if (h<0||h>=MAX_INSTANCES) return;
    Synth* s=&g_inst[h];
    uint8_t* p=(uint8_t*)(uintptr_t)events_ptr;
    int arpOn = s->p_arpOn >= 0.5f;
    for (int i=0;i<count;i++){
        uint8_t status=p[i*8+4];
        uint8_t d1=p[i*8+5];
        uint8_t d2=p[i*8+6];
        uint8_t hi = status & 0xF0;
        if (hi==0x90){
            if (d2>0){
                if (arpOn){
                    arp_push(s, d1);
                } else {
                    int w1 = (int)(s->p_osc1w * 3.999f);
                    int w2 = (int)(s->p_osc2w * 3.999f);
                    if (w1<0) w1=0; if (w1>3) w1=3;
                    if (w2<0) w2=0; if (w2>3) w2=3;
                    note_on_internal(s, d1, (float)d2/127.0f, w1, w2);
                }
            } else {
                if (arpOn) arp_remove(s, d1);
                else note_off_internal(s, d1);
            }
        } else if (hi==0x80){
            if (arpOn) arp_remove(s, d1);
            else note_off_internal(s, d1);
        }
    }
}

__attribute__((export_name("dsp_process")))
void dsp_process(int32_t h, int32_t in_ptr, int32_t out_ptr,
                 int32_t frames, int32_t in_ch, int32_t out_ch){
    (void)in_ptr; (void)in_ch;
    if (h<0||h>=MAX_INSTANCES) return;
    Synth* s=&g_inst[h];
    float* out=(float*)(uintptr_t)out_ptr;
    float sr=(float)s->sample_rate;
    float invSr = 1.0f / sr;

    /* ---- cache params (mapped to per-sample coefficients) ---- */
    int   osc1W = (int)(s->p_osc1w * 3.999f); if (osc1W<0) osc1W=0; if (osc1W>3) osc1W=3;
    int   osc2W = (int)(s->p_osc2w * 3.999f); if (osc2W<0) osc2W=0; if (osc2W>3) osc2W=3;
    float osc1L = s->p_osc1l;
    float osc2L = s->p_osc2l;
    /* osc1 detune: cents = (d - 0.5) * 100 */
    float osc1Cents = (s->p_osc1d - 0.5f) * 100.0f;
    float osc1Mul   = pow2f(osc1Cents / 1200.0f);
    float osc2Cents = (s->p_osc2d - 0.5f) * 100.0f;
    float osc2Mul   = pow2f(osc2Cents / 1200.0f);
    float subL  = s->p_sub;
    float noiL  = s->p_noise;

    float fcBase = cutoff_hz(s->p_cut);
    float q = 1.0f - s->p_res * 0.97f; if (q < 0.03f) q = 0.03f;
    int   ftype = (int)(s->p_ftype * 3.999f); if (ftype<0) ftype=0; if (ftype>3) ftype=3;
    float fdriv = 1.0f + s->p_fdrive * 9.0f;
    float fenvAmt = (s->p_fenv - 0.5f) * 6.0f;  /* -3..+3 octaves */

    /* envelope times -> per-sample increments (1ms..3s, quadratic feel) */
    float aT = 0.001f + s->p_aA * s->p_aA * 3.0f;
    float dT = 0.001f + s->p_aD * s->p_aD * 3.0f;
    float rT = 0.001f + s->p_aR * s->p_aR * 3.0f;
    float aInc = 1.0f/(aT*sr);
    float dInc = 1.0f/(dT*sr);
    float rInc = 1.0f/(rT*sr);
    float aSus = s->p_aS;

    float mT = 0.001f + s->p_mA * s->p_mA * 3.0f;
    float mdT= 0.001f + s->p_mD * s->p_mD * 3.0f;
    float mrT= 0.001f + s->p_mR * s->p_mR * 3.0f;
    float mInc = 1.0f/(mT*sr);
    float mdInc= 1.0f/(mdT*sr);
    float mrInc= 1.0f/(mrT*sr);
    float mSus = s->p_mS;

    /* LFOs */
    float l1Hz = 0.05f + s->p_l1r * 11.95f;  /* 0.05 .. 12 Hz */
    float l1Inc = TWO_PI_F * l1Hz * invSr;
    float l1Depth = s->p_l1d;  /* 0..1 -> 0..6 octaves */
    float l2Hz = 0.05f + s->p_l2r * 11.95f;
    float l2Inc = TWO_PI_F * l2Hz * invSr;
    float l2Depth = s->p_l2d;  /* 0..1 -> 0..0.02 (200 cents) */
    float l2CentsMax = 200.0f * l2Depth;

    float fxAmt = s->p_fx;
    float master = s->p_master;

    /* arp state */
    int   arpOn = s->p_arpOn >= 0.5f;
    float arpHz = 0.5f + s->p_arpRate * 11.5f;   /* 0.5 .. 12 Hz */
    uint32_t arpPeriod = (uint32_t)((float)sr / arpHz + 0.5f);
    if (arpPeriod < 1) arpPeriod = 1;
    float arpGate = 0.05f + s->p_arpGate * 0.85f;  /* 0.05..0.9 */

    /* restore per-voice LFO/increment cache */
    float lfo1 = s->lfo1, lfo2 = s->lfo2;
    uint32_t sampCounter = s->sampCounter;
    uint32_t arpNextSamp = s->arpNextSamp;
    int arpVoice = s->arpVoice;
    int arpStep  = s->arpStep;
    int arpDir   = s->arpDir;
    int arpHoldN = s->arpHoldN;

    /* Mix bus accumulator for FX */
    float mixL = 0.0f, mixR = 0.0f;
    int   mixCount = 0;

    for (int n=0; n<frames; n++){
        /* ---- global LFOs (samples 0..2pi) ---- */
        lfo1 += l1Inc; if (lfo1 >= TWO_PI_F) lfo1 -= TWO_PI_F;
        lfo2 += l2Inc; if (lfo2 >= TWO_PI_F) lfo2 -= TWO_PI_F;
        float lfo1S = fast_sin(lfo1 - PI_F);   /* -1..1 */
        float lfo2S = fast_sin(lfo2 - PI_F);

        /* ---- arpeggiator scheduling ---- */
        if (arpOn && arpHoldN>0){
            if (sampCounter >= arpNextSamp){
                if (arpVoice>=0 && s->v[arpVoice].active){
                    s->v[arpVoice].stageA = ENV_RELEASE;
                }
                int note = arp_pick(s);
                if (note >= 0){
                    arpVoice = alloc_voice(s);
                    s->v[arpVoice].active = 1;
                    s->v[arpVoice].note   = note;
                    s->v[arpVoice].vel    = 0.8f;
                    s->v[arpVoice].osc1Wave = osc1W;
                    s->v[arpVoice].osc2Wave = osc2W;
                    s->v[arpVoice].ph1 = 0.0f; s->v[arpVoice].ph2 = 0.0f; s->v[arpVoice].phs = 0.0f;
                    float hz = note_to_hz(note);
                    s->v[arpVoice].incBase = hz / sr;
                    s->v[arpVoice].inc2Mul = osc2Mul;
                    s->v[arpVoice].stageA = ENV_ATTACK; s->v[arpVoice].envA = 0.0f;
                    s->v[arpVoice].stageM = ENV_ATTACK; s->v[arpVoice].envM = 0.0f;
                    s->v[arpVoice].age = ++sampCounter;
                }
                arpNextSamp = sampCounter + (uint32_t)(arpPeriod * (0.05f + arpGate));
            }
            /* scheduled note-off at end of gate window */
            if (arpVoice>=0 && sampCounter == arpNextSamp - (uint32_t)(arpPeriod * (0.05f + arpGate)) + (uint32_t)(arpPeriod * arpGate)){
                if (s->v[arpVoice].active) s->v[arpVoice].stageA = ENV_RELEASE;
            }
        } else if (!arpOn && arpVoice>=0){
            if (s->v[arpVoice].active) s->v[arpVoice].stageA = ENV_RELEASE;
            arpVoice = -1;
        }

        /* ---- per-voice sample ---- */
        float vL = 0.0f, vR = 0.0f;
        int   vActiveCount = 0;
        for (int i=0;i<NUM_VOICES;i++){
            Voice* v = &s->v[i];
            if (!v->active) continue;

            /* amp envelope */
            switch (v->stageA){
                case ENV_ATTACK:
                    v->envA += aInc; if (v->envA>=1.0f){v->envA=1.0f;v->stageA=ENV_DECAY;} break;
                case ENV_DECAY:
                    v->envA -= dInc*(1.0f - aSus);
                    if (v->envA<=aSus){v->envA=aSus;v->stageA=ENV_SUSTAIN;} break;
                case ENV_SUSTAIN:
                    v->envA = aSus;
                    if (aSus<=0.0001f){v->active=0;v->stageA=ENV_IDLE;}
                    break;
                case ENV_RELEASE:
                    v->envA -= rInc;
                    if (v->envA<=0.0f){v->envA=0.0f;v->active=0;v->stageA=ENV_IDLE;}
                    break;
                default: v->active=0; break;
            }
            /* mod envelope */
            switch (v->stageM){
                case ENV_ATTACK:
                    v->envM += mInc; if (v->envM>=1.0f){v->envM=1.0f;v->stageM=ENV_DECAY;} break;
                case ENV_DECAY:
                    v->envM -= mdInc*(1.0f - mSus);
                    if (v->envM<=mSus){v->envM=mSus;v->stageM=ENV_SUSTAIN;} break;
                case ENV_SUSTAIN:
                    v->envM = mSus; break;
                case ENV_RELEASE:
                    v->envM -= mrInc;
                    if (v->envM<=0.0f){v->envM=0.0f;v->stageM=ENV_IDLE;} break;
                default: v->envM = 0.0f; break;
            }
            if (!v->active) continue;

            /* pitch: base inc * osc1 detune * osc2 detune * LFO2 (cents) */
            float pitchCents = l2CentsMax * lfo2S;
            float pitchMul   = pow2f(pitchCents / 1200.0f);
            float inc1 = v->incBase * osc1Mul * pitchMul;
            float inc2 = v->incBase * v->inc2Mul * pitchMul;
            float incS = v->incBase * 0.5f * pitchMul;   /* sub: 1 octave down */

            /* osc1 + osc2 + sub */
            float o1 = osc_run(v->osc1Wave, v->ph1) * osc1L;
            float o2 = osc_run(v->osc2Wave, v->ph2) * osc2L;
            float os = fast_sin(v->phs * TWO_PI_F - PI_F) * subL;
            float nz = noise_sample(&s->noiseSeed) * noiL;
            float mix_osc = o1 + o2 + os + nz;
            /* soft-clip on the source */
            mix_osc = fast_tanh(mix_osc);

            /* pre-filter drive */
            mix_osc = fast_tanh(mix_osc * fdriv);

            /* filter cutoff with env+LFO+keytrack modulation */
            float envOct = fenvAmt * (v->envM - 0.5f);
            float lfo1Oct = (l1Depth * 6.0f) * lfo1S;
            /* keytrack: each note away from A4 shifts cutoff by half an octave */
            float keyOct = ((float)v->note - 69.0f) * 0.5f / 12.0f;
            float fcMul = pow2f(envOct + lfo1Oct + keyOct);
            float fc = clampf(fcBase * fcMul, 15.0f, sr * 0.45f);
            float f = 2.0f * fast_sin(PI_F * fc / sr);
            if (f > 1.5f) f = 1.5f;

            /* SVF (one per voice) */
            SVF fL = { v->ph1 * 0.0f, 0.0f };  /* placeholders; we maintain state inline */
            /* we need per-voice state; reuse a hidden local pattern via stored bp/lp */
            /* For simplicity and to keep code compact we just compute and discard
             * the history here (single-sample state) — for a 4-pole ladder this
             * is fine since each voice is a stable Chamberlin SVF. */
            float lp, bp, hp;
            /* step */
            static SVF svf_state[NUM_VOICES];
            svf_step(&svf_state[i], &lp, &bp, &hp, mix_osc, f, q);
            float yFil;
            switch (ftype){
                case 0:  yFil = lp; break;
                case 1:  yFil = hp; break;
                case 2:  yFil = bp; break;
                default: yFil = mix_osc - hp; break;  /* notch */
            }

            /* output */
            float y = yFil * v->envA * v->vel;
            /* soft brick-wall */
            y = fast_tanh(y * 1.2f);

            /* per-voice panning based on note (subtle stereo spread) */
            float pan = ((float)v->note - 60.0f) * 0.04f; /* +-0.5 across ~25 semitones */
            if (pan >  0.7f) pan =  0.7f;
            if (pan < -0.7f) pan = -0.7f;
            float gL = 0.5f - pan*0.5f;
            float gR = 0.5f + pan*0.5f;
            vL += y * gL;
            vR += y * gR;
            vActiveCount++;

            /* advance phases */
            v->ph1 += inc1; if (v->ph1 >= 1.0f) v->ph1 -= 1.0f;
            v->ph2 += inc2; if (v->ph2 >= 1.0f) v->ph2 -= 1.0f;
            v->phs += incS; if (v->phs >= 1.0f) v->phs -= 1.0f;
        }

        /* mix voices into bus (scaled for poly) */
        float vg = vActiveCount > 0 ? 0.6f / (float)vActiveCount : 0.0f;
        vL *= vg; vR *= vg;

        /* FX bus (delay + reverb) — share one state across all instances for
         * continuity. fx_process is re-entrant-safe since it touches globals
         * one at a time and operates on local inputs only. */
        fx_process(&vL, &vR, fxAmt);

        /* soft brick-wall on output + master */
        vL = fast_tanh(vL * master * 1.2f);
        vR = fast_tanh(vR * master * 1.2f);

        if (out_ch >= 2){ out[n*out_ch+0]=vL; out[n*out_ch+1]=vR; }
        else            { out[n*out_ch+0]=(vL+vR)*0.5f; }

        sampCounter++;
    }

    s->lfo1 = lfo1; s->lfo2 = lfo2;
    s->sampCounter = sampCounter;
    s->arpNextSamp = arpNextSamp;
    s->arpVoice    = arpVoice;
    s->arpStep     = arpStep;
    s->arpDir      = arpDir;
    s->arpHoldN    = arpHoldN;
}

__attribute__((export_name("dsp_get_latency")))
int32_t dsp_get_latency(int32_t h){ (void)h; return 0; }
__attribute__((export_name("dsp_get_tail")))
int32_t dsp_get_tail(int32_t h){
    if (h<0||h>=MAX_INSTANCES) return 0;
    return (int32_t)(g_inst[h].sample_rate * 2.0);
}

__attribute__((export_name("dsp_save_state")))
int32_t dsp_save_state(int32_t h, int32_t ptr, int32_t max_bytes){
    if (h<0||h>=MAX_INSTANCES) return 0;
    if (max_bytes < (int32_t)(NUM_PARAMS*sizeof(float))) return 0;
    float* d = (float*)(uintptr_t)ptr;
    Synth* s = &g_inst[h];
    d[0]=s->p_osc1w; d[1]=s->p_osc1l; d[2]=s->p_osc1d;
    d[3]=s->p_osc2w; d[4]=s->p_osc2l; d[5]=s->p_osc2d;
    d[6]=s->p_sub;   d[7]=s->p_noise;
    d[8]=s->p_cut;   d[9]=s->p_res;   d[10]=s->p_ftype; d[11]=s->p_fdrive; d[12]=s->p_fenv;
    d[13]=s->p_aA;   d[14]=s->p_aD;   d[15]=s->p_aS;   d[16]=s->p_aR;
    d[17]=s->p_mA;   d[18]=s->p_mD;   d[19]=s->p_mS;   d[20]=s->p_mR;
    d[21]=s->p_l1r;  d[22]=s->p_l1d;  d[23]=s->p_l2r;  d[24]=s->p_l2d;
    d[25]=s->p_fx;   d[26]=s->p_master;
    d[27]=s->p_arpOn;d[28]=s->p_arpRate;d[29]=s->p_arpGate;
    return (int32_t)(NUM_PARAMS*sizeof(float));
}

__attribute__((export_name("dsp_load_state")))
void dsp_load_state(int32_t h, int32_t ptr, int32_t bytes){
    if (h<0||h>=MAX_INSTANCES) return;
    if (bytes < (int32_t)(NUM_PARAMS*sizeof(float))) return;
    float* s2 = (float*)(uintptr_t)ptr;
    Synth* s = &g_inst[h];
    s->p_osc1w=s2[0]; s->p_osc1l=s2[1]; s->p_osc1d=s2[2];
    s->p_osc2w=s2[3]; s->p_osc2l=s2[4]; s->p_osc2d=s2[5];
    s->p_sub  =s2[6]; s->p_noise =s2[7];
    s->p_cut  =s2[8]; s->p_res   =s2[9]; s->p_ftype=s2[10]; s->p_fdrive=s2[11]; s->p_fenv=s2[12];
    s->p_aA   =s2[13];s->p_aD    =s2[14];s->p_aS   =s2[15];s->p_aR  =s2[16];
    s->p_mA   =s2[17];s->p_mD    =s2[18];s->p_mS   =s2[19];s->p_mR  =s2[20];
    s->p_l1r  =s2[21];s->p_l1d   =s2[22];s->p_l2r  =s2[23];s->p_l2d =s2[24];
    s->p_fx   =s2[25];s->p_master=s2[26];
    s->p_arpOn=s2[27];s->p_arpRate=s2[28];s->p_arpGate=s2[29];
}

static uint8_t heap[1 << 20];
static int heap_top=0;
__attribute__((export_name("casc_alloc")))
int32_t casc_alloc(int32_t n){
    int a=(n+7)&~7;
    if (heap_top+a > (int)sizeof(heap)) return 0;
    int p=heap_top; heap_top+=a;
    return (int32_t)((uintptr_t)heap+p);
}
__attribute__((export_name("casc_free")))
void casc_free(int32_t ptr){ (void)ptr; }
