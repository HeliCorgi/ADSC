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

echo "[1/12] WP5 campaign Monte Carlo (N=500 x 2 catalogs)"
"$BUILD/adsc_campaign" 500 generated
echo "[2/12] WP12 fidelity ladder (L0/L1/L2 re-verification of the WP5 abort events)"
"$BUILD/adsc_ladder" 500 generated
echo "[3/12] WP6 cost model + FoM (kit sweep, consumes WP5)"
"$BUILD/adsc_cost" 500 generated
echo "[4/12] WP3 decay-trade CSV"
"$BUILD/decay_trade" generated
echo "[5/12] WP13 kit-class trade + class-C controlled-reentry comparison"
"$BUILD/kit_trade" generated
echo "[6/12] WP14 prioritization table (joined view of WP6 cost + WP13 kit trade)"
python3 tools/prioritization/make_prioritization.py . generated
echo "[7/12] T6 small-debris flux sweep"
"$BUILD/flux_sweep" generated
echo "[8/12] WP7 reference metrics (WP1/F1/WP2/WP3/WP4 pinned numbers)"
"$BUILD/sim_metrics" generated
echo "[9/12] WP10c keep-out violation forensics (read-only replay of WP5)"
python3 tools/forensics/make_forensics.py
echo "[10/12] WP7a visualization pack"
python3 tools/viz/make_viz.py . generated/viz
echo "[11/12] WP8 compliance precheck + matrix (not legal advice)"
python3 tools/compliance/check_compliance.py
python3 tools/compliance/generate_matrix.py
echo "[12/12] WP7 evidence pack"
python3 tools/evidence/make_evidence.py
echo "regenerate_all: complete"
