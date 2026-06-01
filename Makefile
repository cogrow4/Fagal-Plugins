# Fagal Plugins — one-shot rebuild of all DSP modules
#
# Requires: clang (with --target=wasm32), lld
# On macOS:  brew install llvm lld
# On Linux:  apt install clang lld
#
# Each plugin is a self-contained wasm module with no libc, no math.h.
# The shipped dsp.wasm in each folder matches this build.

CLANG ?= /opt/homebrew/opt/llvm/bin/clang
WASM_TARGET ?= wasm32

CC = $(CLANG) --target=$(WASM_TARGET) -O3 -nostdlib \
      -Wl,--no-entry -Wl,--export-dynamic

PLUGINS := reverb delay chorus distortion filter gain synth

.PHONY: all clean $(PLUGINS)

all: $(PLUGINS)

reverb:        reverb/reverb.c
	$(CC) $< -o reverb/dsp.wasm

delay:         delay/delay.c
	$(CC) $< -o delay/dsp.wasm

chorus:        chorus/chorus.c
	$(CC) $< -o chorus/dsp.wasm

distortion:    distortion/distortion.c
	$(CC) $< -o distortion/dsp.wasm

filter:        filter/filter.c
	$(CC) $< -o filter/dsp.wasm

gain:          gain/gain.c
	$(CC) $< -o gain/dsp.wasm

synth:         synth/synth.c
	$(CC) $< -o synth/dsp.wasm

clean:
	rm -f reverb/dsp.wasm delay/dsp.wasm chorus/dsp.wasm \
	      distortion/dsp.wasm filter/dsp.wasm gain/dsp.wasm \
	      synth/dsp.wasm
