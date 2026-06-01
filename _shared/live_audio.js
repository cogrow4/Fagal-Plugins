/* live_audio.js — helper for plugin UIs: draw real audio waveform
 * (from window.casc.getMeter) on a canvas. Keeps the existing visual
 * underneath; the live waveform is added as a thin, semi-transparent line.
 *
 * Usage:
 *   const la = liveAudio(canvas);   // returns { update(arr), draw() }
 *   function tick(){
 *     la.update();                   // poll meter
 *     la.draw('#5fffb0', 0.85);      // color, alpha
 *     requestAnimationFrame(tick);
 *   }
 */
function liveAudio(canvas){
  const ctx = canvas.getContext('2d');
  let buf = new Float32Array(512 * 2);  // 512 frames * stereo
  let lastReq = 0, drawing = false;
  if (window.casc && window.casc.onMeter) {
    window.casc.onMeter((req, arr) => {
      if (req < lastReq) return;            // ignore stale
      lastReq = req;
      const n = arr.length >> 1;            // frames
      if (n * 2 > buf.length) buf = new Float32Array(n * 2);
      for (let i = 0; i < n * 2; i++) buf[i] = arr[i];
    });
  }
  function poll(){
    if (drawing || !window.casc || !window.casc.getMeter) return;
    drawing = true;
    window.casc.getMeter(512);
    // small delay before allowing next poll
    setTimeout(() => { drawing = false; }, 8);
  }
  function draw(color, alpha){
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    // buf has up to 512 frames; resample to canvas width
    const frames = buf.length >> 1;
    if (frames < 2) return;
    ctx.save();
    ctx.globalAlpha = alpha != null ? alpha : 0.9;
    ctx.strokeStyle = color || '#5fffb0';
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    for (let x = 0; x < w; x++) {
      const f = Math.floor(x / w * frames);
      const i = (f * 2) | 0;
      let v = (buf[i] + buf[i+1]) * 0.5;
      if (v > 1) v = 1; if (v < -1) v = -1;
      const y = h * 0.5 - v * h * 0.45;
      if (x === 0) ctx.moveTo(x + 0.5, y); else ctx.lineTo(x + 0.5, y);
    }
    ctx.stroke();
    ctx.restore();
  }
  return { poll, draw, buf: () => buf };
}
