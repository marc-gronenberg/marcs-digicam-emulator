#!/bin/sh
# Builds Marcs Digicam Emulator for PhotoCraft and runs the checks in harness/.
#
#   ./build.sh          build the plug-in from the data in lut/, then test it
#   ./build.sh --data   re-run the camera model and the tables first (python3 with numpy, scipy)
#
# Needs Rust with the wasm32-unknown-unknown target: rustup target add wasm32-unknown-unknown
set -eu
cd "$(dirname "$0")"

if [ "${1:-}" = "--data" ]; then
  python3 model/digicam_model.py         # camera model -> lut/<camera>.lut, plugin/src/cameras.rs
  python3 model/make_tables.py           # decode tables, Adobe RGB tables, sensor noise
fi

(cd plugin && cargo build --release --target wasm32-unknown-unknown)
mkdir -p dist
cp plugin/target/wasm32-unknown-unknown/release/marcs_digicam_emulator.wasm dist/marcs-digicam-emulator.wasm
ls -l dist/*.wasm

(cd common && cargo test --release --quiet)
(cd harness && cargo test --release --quiet)
