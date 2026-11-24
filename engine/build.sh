#!/bin/sh
# Build the C cores (system clang only, no third-party code). -ffp-contract=off: no fused multiply-add, so the
# double reference rounds step by step as the Python one does (arm64 clang fused the lookup a*b+c)
cd "$(dirname "$0")" && clang -O2 -std=c99 -ffp-contract=off -Wall -Wextra -o fxchain_core fxchain_core.c -lm \
  && clang -O2 -std=c99 -ffp-contract=off -Wall -Wextra -Wno-unused-function -o fxgraph_core fxgraph_core.c -lm \
  && echo "build ok: engine/fxchain_core, engine/fxgraph_core" \
  || { echo "build FAILED (needs the system clang, C99)" >&2; exit 1; }
