#!/usr/bin/env python3
"""README marker-block filler test (registered with ctest as `readme_numbers`).

  test_readme_numbers.py <repo_root>

Checks (non-zero exit on any failure, R4):
  1. --check mode passes against the real, committed README.md -- i.e. the
     WP5-NUMBERS/WP6-NUMBERS marker blocks already reproduce byte-for-byte
     from the committed CSVs (closes review finding C4: README cannot
     silently drift from generated/wp5_campaign_summary.csv /
     generated/wp6_cost_summary.csv without this test catching it).
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

fails = []
def check(cond, msg):
    if not cond:
        fails.append(msg)


def sha(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def make_scratch_repo(repo):
    """Copy just the inputs fill_readme_numbers.py needs (README.md plus the
    two committed CSVs it reads) into a throwaway directory, so the
    corruption test can mutate a copy without touching the real working
    tree."""
    tmp = tempfile.mkdtemp(prefix="wp14_readme_")
    os.makedirs(os.path.join(tmp, "generated"))
    shutil.copy(os.path.join(repo, "README.md"), os.path.join(tmp, "README.md"))
    for name in ("wp5_campaign_summary.csv", "wp6_cost_summary.csv"):
        shutil.copy(os.path.join(repo, "generated", name),
                    os.path.join(tmp, "generated", name))
    return tmp


def run_tool(root, *args):
    return subprocess.run(
        [sys.executable, os.path.join(HERE, "fill_readme_numbers.py"), root]
        + list(args),
        capture_output=True, text=True)


def main():
    if len(sys.argv) < 2:
        print("usage: test_readme_numbers.py <repo_root>", file=sys.stderr)
        return 2
    repo = os.path.abspath(sys.argv[1])

    # 1. --check against the real, committed README.md must pass.
    r = run_tool(repo, "--check")
    check(r.returncode == 0,
          "fill_readme_numbers.py --check failed against the committed "
          "README: %s" % (r.stdout + r.stderr).strip())

    # 2. Corrupt a pinned number inside the WP6 marker block in a scratch
    #    copy; --check must now fail.
    tmp1 = make_scratch_repo(repo)
    try:
        readme_path = os.path.join(tmp1, "README.md")
        with open(readme_path, encoding="utf-8") as f:
            text = f.read()
        check("44.80" in text,
              "fixture assumption stale: pinned number '44.80' not found "
              "in README.md")
        corrupted = text.replace("44.80", "99.99", 1)
        check(corrupted != text, "corruption was a no-op -- fixture stale")
        with open(readme_path, "w", encoding="utf-8", newline="\n") as f:
            f.write(corrupted)
        r2 = run_tool(tmp1, "--check")
        check(r2.returncode != 0,
              "--check did not catch a corrupted number in a marker block")
    finally:
        shutil.rmtree(tmp1, ignore_errors=True)

    # 3. Determinism: rewrite-mode is idempotent on an already-correct copy
    #    (content-equal; the local checkout's CRLF vs the tool's canonical
    #    LF output are not a real difference -- read both in text mode,
    #    which normalizes line endings, exactly like test_evidence.py's
    #    committed-vs-regenerated comparison), and two independent
    #    regenerations of the same inputs are byte-identical.
    tmp2 = make_scratch_repo(repo)
    try:
        readme_path = os.path.join(tmp2, "README.md")
        with open(readme_path, encoding="utf-8") as f:
            before_text = f.read()
        r3 = run_tool(tmp2)
        check(r3.returncode == 0,
              "default-mode run failed: %s" % (r3.stdout + r3.stderr).strip())
        with open(readme_path, encoding="utf-8") as f:
            after_text = f.read()
        check(before_text == after_text,
              "rewriting an already-correct README changed its content "
              "(not idempotent)")
        after_sha = sha(readme_path)
        r4 = run_tool(tmp2)
        check(r4.returncode == 0, "second default-mode run failed: %s"
              % (r4.stdout + r4.stderr).strip())
        after2_sha = sha(readme_path)
        check(after_sha == after2_sha,
              "two regenerations from the same inputs are not "
              "byte-identical (nondeterministic)")
    finally:
        shutil.rmtree(tmp2, ignore_errors=True)

    if fails:
        for m in fails:
            print("FAIL:", m)
        print("readme_numbers: %d failure(s)" % len(fails))
        return 1
    print("readme_numbers: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
