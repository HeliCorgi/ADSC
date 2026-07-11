#!/usr/bin/env python3
"""docs/ marker-block filler test (registered with ctest as `docs_numbers`).

  test_docs_numbers.py <repo_root>

Mirrors tools/readme/test_readme_numbers.py's discipline for the docs/
DOCS-*-START/END marker blocks filled by fill_docs_numbers.py. Checks
(non-zero exit on any failure, R4):
  1. --check mode passes against the real, committed docs/*.md files -- i.e.
     every DOCS-*-START/END marker block already reproduces byte-for-byte
     from the committed CSVs it cites (R16: docs/ must not drift from the
     generated artifacts it summarizes without this test catching it).
  2. --check mode FAILS when a number inside a marker block is corrupted in
     a scratch copy -- proves the check reads block content, not just
     marker-comment presence.
  3. Determinism: default (rewrite) mode is idempotent on an already-correct
     scratch copy (content-equal, ignoring the CRLF/LF checkout-dependent
     line-ending representation the same way test_evidence.py does), and two
     independent regenerations from the same inputs are byte-identical.
"""
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

# The docs/*.md files fill_docs_numbers.py rewrites, and the generated/ CSVs
# it reads -- kept in sync with fill_docs_numbers.py's own REGISTRY / Data
# loaders. Listed explicitly (rather than introspected) so this test fails
# loudly if the two files drift apart instead of silently under-testing.
DOC_FILES = [
    "docs/safety.md",
    "docs/cost_model.md",
    "docs/gnc.md",
    "docs/target_selection.md",
    "docs/technical_summary_5p.md",
]
CSV_FILES = [
    "wp5_campaign_summary.csv",
    "wp6_cost_summary.csv",
    "reference_metrics.csv",
    "wp3_decay_trade.csv",
    "wp13_kit_trade.csv",
    "wp13_classC.csv",
    "wp12_ladder.csv",
]

fails = []
def check(cond, msg):
    if not cond:
        fails.append(msg)


def sha_all(root):
    h = hashlib.sha256()
    for relpath in DOC_FILES:
        with open(os.path.join(root, *relpath.split("/")), "rb") as f:
            h.update(f.read())
    return h.hexdigest()


def make_scratch_repo(repo):
    """Copy just the inputs fill_docs_numbers.py needs (the docs/*.md files
    plus the committed CSVs they cite) into a throwaway directory, so the
    corruption test can mutate a copy without touching the real working
    tree."""
    tmp = tempfile.mkdtemp(prefix="wp15_docs_")
    os.makedirs(os.path.join(tmp, "docs"))
    os.makedirs(os.path.join(tmp, "generated"))
    for relpath in DOC_FILES:
        shutil.copy(os.path.join(repo, *relpath.split("/")),
                    os.path.join(tmp, *relpath.split("/")))
    for name in CSV_FILES:
        shutil.copy(os.path.join(repo, "generated", name),
                    os.path.join(tmp, "generated", name))
    return tmp


def run_tool(root, *args):
    return subprocess.run(
        [sys.executable, os.path.join(HERE, "fill_docs_numbers.py"), root]
        + list(args),
        capture_output=True, text=True)


def main():
    if len(sys.argv) < 2:
        print("usage: test_docs_numbers.py <repo_root>", file=sys.stderr)
        return 2
    repo = os.path.abspath(sys.argv[1])

    # 1. --check against the real, committed docs/*.md files must pass.
    r = run_tool(repo, "--check")
    check(r.returncode == 0,
          "fill_docs_numbers.py --check failed against the committed docs/: "
          "%s" % (r.stdout + r.stderr).strip())

    # 2. Corrupt a pinned number inside a marker block in a scratch copy;
    #    --check must now fail. Uses the WP6-FULL block's amortization-
    #    minimum figure (docs/cost_model.md), the docs/ home of the same
    #    "44.80" pin README's own digest carries.
    tmp1 = make_scratch_repo(repo)
    try:
        cost_path = os.path.join(tmp1, "docs", "cost_model.md")
        with open(cost_path, encoding="utf-8") as f:
            text = f.read()
        check("44.80" in text,
              "fixture assumption stale: pinned number '44.80' not found "
              "in docs/cost_model.md")
        corrupted = text.replace("44.80", "99.99", 1)
        check(corrupted != text, "corruption was a no-op -- fixture stale")
        with open(cost_path, "w", encoding="utf-8", newline="\n") as f:
            f.write(corrupted)
        r2 = run_tool(tmp1, "--check")
        check(r2.returncode != 0,
              "--check did not catch a corrupted number in a marker block")
    finally:
        shutil.rmtree(tmp1, ignore_errors=True)

    # 3. Determinism: rewrite-mode is idempotent on an already-correct copy
    #    (content-equal; CRLF/LF normalized via text-mode read, exactly like
    #    test_evidence.py's committed-vs-regenerated comparison), and two
    #    independent regenerations of the same inputs are byte-identical.
    tmp2 = make_scratch_repo(repo)
    try:
        before_texts = {}
        for relpath in DOC_FILES:
            with open(os.path.join(tmp2, *relpath.split("/")), encoding="utf-8") as f:
                before_texts[relpath] = f.read()
        r3 = run_tool(tmp2)
        check(r3.returncode == 0,
              "default-mode run failed: %s" % (r3.stdout + r3.stderr).strip())
        for relpath in DOC_FILES:
            with open(os.path.join(tmp2, *relpath.split("/")), encoding="utf-8") as f:
                after_text = f.read()
            check(before_texts[relpath] == after_text,
                  "rewriting an already-correct %s changed its content "
                  "(not idempotent)" % relpath)
        after_sha = sha_all(tmp2)
        r4 = run_tool(tmp2)
        check(r4.returncode == 0, "second default-mode run failed: %s"
              % (r4.stdout + r4.stderr).strip())
        after2_sha = sha_all(tmp2)
        check(after_sha == after2_sha,
              "two regenerations from the same inputs are not "
              "byte-identical (nondeterministic)")
    finally:
        shutil.rmtree(tmp2, ignore_errors=True)

    if fails:
        for m in fails:
            print("FAIL:", m)
        print("docs_numbers: %d failure(s)" % len(fails))
        return 1
    print("docs_numbers: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
