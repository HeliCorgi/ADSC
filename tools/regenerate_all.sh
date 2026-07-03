#!/usr/bin/env bash
# Regenerates EVERY committed artifact under generated/ and evidence/ in
# dependency order (R6/R12/D10). CI runs exactly this script and then asserts
# `git diff --exit-code -- generated/ evidence/` -- so this file is the single
# source of truth for reproduction: a clean machine runs
#
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
#   bash tools/regenerate_all.sh build
#
# and every number quoted anywhere in the repository regenerates byte-for-byte.
# Wall-clock timing is printed to stdout only (never embedded in artifacts).
set -euo pipefail
cd "$(dirname "$0")/.."          # repo root
BUILD="${1:-build}"

echo "[1/8] WP5 campaign Monte Carlo (N=500 x 2 catalogs)"
"$BUILD/adsc_campaign" 500 generated
echo "[2/8] WP6 cost model + FoM (kit sweep, consumes WP5)"
"$BUILD/adsc_cost" 500 generated
echo "[3/8] WP3 decay-trade CSV"
"$BUILD/decay_trade" generated
echo "[4/8] T6 small-debris flux sweep"
"$BUILD/flux_sweep" generated
echo "[5/8] WP7 reference metrics (WP1/F1/WP2/WP3/WP4 pinned numbers)"
"$BUILD/sim_metrics" generated
echo "[6/8] WP7a visualization pack"
python3 tools/viz/make_viz.py . generated/viz
echo "[7/8] WP8 compliance precheck + matrix (not legal advice)"
python3 tools/compliance/check_compliance.py
python3 tools/compliance/generate_matrix.py
echo "[8/8] WP7 evidence pack"
python3 tools/evidence/make_evidence.py
echo "regenerate_all: complete"
