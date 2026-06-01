# Fagal Plugins

A curated set of audio effects and instruments packaged as [CASC](https://github.com/cogrow4/CASC) plugins.

Every plugin ships as a single-folder CASC bundle: `manifest.json` + `*.c` (DSP source) + `ui.html` (browser-rendered interface) + `presets/`. Drop the folder into a CASC host and it loads. No native dependencies, no pre-built binaries, no DAW lock-in.

## The collection

| Plugin | Category | DSP character |
| --- | --- | --- |
| **Fagal Space** (reverb) | Effect | 4-section reverb (algorithmic + granular diffusion + modulation/shimmer + early reflections) inspired by Spectral Audio's *Space* |
| **Fagal Echo** (delay) | Effect | Ping-pong delay with tone-filtered feedback, dual tap, modulation, saturation, and freeze |
| **Fagal Ensemble** (chorus) | Effect | 6-voice chorus / ensemble / flanger with multi-stage modulation |
| **Fagal Drive** (distortion) | Effect | 5-shape waveshaper (Tube / Tape / Soft / Hard / Fold) with pre+post tone, env follower, soft ceiling |
| **Fagal Sculpt** (filter) | Effect | State-variable filter (LP / HP / BP / Notch) with drive, env, LFO, keytrack, dry/wet |
| **Fagal Boost** (gain) | Utility | Mastering-grade trim + gain + polarity + mid/side + soft ceiling with VU/peak meter |
| **Polymath** (synth) | Instrument | 8-voice polysynth with dual oscillators, sub, noise, 4-mode filter, dual ADSR, 2 LFOs, FX bus, and arpeggiator — Omnisphere-class architecture |

## Layout

```
Fagal-Plugins/
├── README.md
├── LICENSE
├── .gitignore
├── reverb/         # Fagal Space
├── delay/          # Fagal Echo
├── chorus/         # Fagal Ensemble
├── distortion/     # Fagal Drive
├── filter/         # Fagal Sculpt
├── gain/           # Fagal Boost
└── synth/          # Polymath
```

Each plugin folder is self-contained:

```
<plugin>/
├── manifest.json   # CASC contract: id, params, audio I/O, GUI size
├── <plugin>.c      # DSP source (self-contained wasm-C, no libc, no math.h)
├── ui.html         # Browser UI (HTML + Canvas + WebAudio)
├── dsp.wasm        # Pre-built wasm binary (rebuild from source if needed)
└── presets/        # User-loadable state snapshots
    ├── default.json
    └── *.json
```

## Building

Each plugin's DSP is a single C file that compiles to a self-contained wasm module with no dependencies:

```sh
clang --target=wasm32 -O3 -nostdlib \
      -Wl,--no-entry -Wl,--export-dynamic \
      reverb/reverb.c -o reverb/dsp.wasm
```

The `manifest.json` is consumed directly by the CASC host; no further packaging step is required. The shipped `dsp.wasm` files are byte-for-byte identical to this build.

To re-pack as a single-file `.casc` bundle for the CASC desktop host, use the upstream tooling in the [CASC](https://github.com/cogrow4/CASC) repository.

## Adding a new preset

Create a JSON file in `<plugin>/presets/`:

```json
{
  "casc_preset_version": "0.1",
  "plugin_id": "org.fagal.<plugin>",
  "plugin_version": "2.0.0",
  "name": "My Preset",
  "author": "Your Name",
  "params": {
    "0": 0.5,
    "1": 0.7
  }
}
```

The keys under `params` are integer param IDs and the values are normalized 0..1 — the same numbers you'll see when dragging a knob in the UI.

## License

MIT. See [LICENSE](LICENSE).

## Credits

DSP, UI, and packaging by Fagal Plugins. Built on the [CASC](https://github.com/cogrow4/CASC) format.
