/* meter_ring.h — common ring-buffer snippet for plugin DSPs that want to
 * expose dsp_get_meter for UI visualisation. #include after the plugin's
 * instance struct, then call meter_ring_write() inside dsp_process and
 * implement dsp_get_meter using meter_ring_read(). */
#ifndef CASC_METER_RING_H
#define CASC_METER_RING_H

#define METER_FRAMES 1024          /* frames kept in the ring (per channel) */
#define METER_CHANNELS 2           /* stereo; we only ever write 0..METER_CHANNELS */
#define METER_BUF (METER_FRAMES * METER_CHANNELS)

/* Per-instance ring state. Embed inside the plugin's instance struct:
 *     float meter_buf[METER_BUF];
 *     int   meter_idx;       // next write position (in frames)
 *     int   meter_filled;    // number of valid frames so far (<= METER_FRAMES)
 */
static inline void meter_ring_clear(float* buf) {
    for (int i = 0; i < METER_BUF; i++) buf[i] = 0.0f;
}

/* Append one frame (L, R). Wraps at METER_FRAMES. */
static inline void meter_ring_write(float* buf, int* idx, int* filled, float l, float r) {
    int i = *idx;
    if (i >= METER_FRAMES) i = 0;
    buf[i * 2 + 0] = l;
    buf[i * 2 + 1] = r;
    i++;
    if (i >= METER_FRAMES) i = 0;
    *idx = i;
    if (*filled < METER_FRAMES) (*filled)++;
}

/* Copy the most recent `out_count` frames into `out` as interleaved L,R,L,R...
 * in chronological order (oldest first). If fewer than out_count frames have
 * been written, the leading portion is zeroed. Returns the number of frames
 * actually copied (= out_count unless the ring isn't full yet). */
static inline int meter_ring_read(const float* buf, int filled, int idx,
                                  int out_count, float* out) {
    int avail = filled < METER_FRAMES ? filled : METER_FRAMES;
    int to_copy = out_count < avail ? out_count : avail;
    int start = (idx - to_copy + METER_FRAMES) % METER_FRAMES;
    /* Zero the leading (out_count - to_copy) frames if ring not full yet. */
    int leading = out_count - to_copy;
    for (int i = 0; i < leading; i++) { out[i*2] = 0.0f; out[i*2+1] = 0.0f; }
    for (int i = 0; i < to_copy; i++) {
        int p = (start + i) % METER_FRAMES;
        out[(leading + i)*2 + 0] = buf[p*2 + 0];
        out[(leading + i)*2 + 1] = buf[p*2 + 1];
    }
    return out_count;
}

#endif /* CASC_METER_RING_H */
