# Release engineering

Scope: how ADSC is version-tagged, when a tag is cut, and the exact,
step-by-step procedure for an archived, DOI-carrying release (Zenodo) —
plus the explicit deferral note (spec:288-291) covering why that step has
not been executed yet.

## Versioning scheme

ADSC is **not** on strict SemVer with a stable public API — it is an
evidence package, pre-1.0, and a version number does not promise API
compatibility to anything. The scheme actually in use is:

```
v0.<phase-number>-<short-descriptor>
```

- `<phase-number>` tracks the work-package/phase the repository had just
  completed at tag time — not a release cadence, and not applied to every
  merged PR. A tag marks a **phase boundary**: either the point a phase's
  headline numbers need archiving as an R15 "before" reference, or a point
  a major deliverable completed.
- `<short-descriptor>` is a short human-readable label for what that phase
  delivered (`phase0-baseline`, `fidelity`), not a restatement of the WP
  number.
- Tags are lightweight/annotated git tags on the merge commit; `CMakeLists.txt`'s
  own `project(ADSC VERSION 4.0 ...)` field is a separate, legacy
  CMake-internal version number and is not currently kept in lockstep with
  the git tag scheme (noted here as a known inconsistency, not fixed by
  this PR — flag to the owner if it should be reconciled).

## Precedents (both already in the repository)

| tag | commit | what it marks |
|---|---|---|
| `v0.10-phase0-baseline` | `1f8c0aa` | End of Phase 0 (WP10: spec adoption, citation-fill, keep-out-violation forensics — no behavior-changing code). Cut **before** WP11 landed specifically so the pre-WP11 keep-out-violation rate (0.014 [0.007, 0.029]) had an archived, citable reference commit for the R15 BEFORE/AFTER table once WP11 closed it to 0/500 (spec §1). |
| `v0.12-fidelity` | `a0deb9d` (merge of PR #26, `fixpack-pre-wp13`) | End of the WP12 fidelity-ladder phase plus the pre-WP13 fix-pack, cut immediately before WP13 (kit-class trade + EDT physics) began — a clean boundary between "safety/fidelity" work and "kit/cost" work. |

**Known gap, stated plainly:** WP13 (`4b02ac0`) and WP14 (`f41cb85`) merged
without their own phase tags — the tag-at-phase-boundary practice was not
applied mechanically to every WP in this chain. This is not fixed
retroactively by this PR (no git tags are created here); it is flagged so
the owner can decide whether to backfill `v0.13-kit-trade` /
`v0.14-cost-fom` tags on their existing merge commits, or accept the gap
and simply resume the practice from WP15 forward.

## Practice going forward

1. Tag at the merge commit of a PR that closes a WP or a WP group, when
   either (a) a headline number in that PR needs to become an R15 "before"
   reference for a later change, or (b) the PR completes a deliverable
   worth pointing an external reader at directly (a released package
   snapshot, not mid-refactor state).
2. Tag name: `v0.<phase-number>-<short-descriptor>`, matching the two
   precedents above. For this PR, the natural next tag at WP15's merge
   commit is **`v0.15-proposal-package`**.
3. Creating the tag and any GitHub Release is an owner/Fable5-authorized
   action (this PR does not create it): `git tag -a v0.15-proposal-package
   -m "<one-line summary>" <merge-commit>` then `git push origin
   v0.15-proposal-package`, authored with the repository's established
   commit identity (`HeliCorgi <269209176+HeliCorgi@users.noreply.github.com>`).
4. A **git tag alone does not trigger Zenodo** (see below) — an actual
   GitHub *Release* object must be published against that tag, e.g.
   `gh release create v0.15-proposal-package --title "ADSC v0.15 — proposal
   package" --notes-file <release-notes>` (the `gh` CLI is available at
   `C:\Program Files\GitHub CLI\gh.exe` on this machine as of 2026-07-05).

## The Zenodo DOI procedure — exact steps

Review improvement #11 (spec, WP15 task 5): earlier is better; the DOI
should not be gated on pitch-perfection. The steps below are the complete,
literal procedure; only step 1 has an external dependency this PR cannot
satisfy.

1. **Owner logs into Zenodo** at <https://zenodo.org>, e.g. via "Log in
   with GitHub" OAuth, so the Zenodo account is linked to the `HeliCorgi`
   GitHub identity. **This is the blocking step** — see deferral note
   below.
2. **Enable GitHub sync for this repository.** In Zenodo, go to
   account **Settings → GitHub** (`https://zenodo.org/account/settings/github/`).
   If `HeliCorgi/ADSC` is not listed, click **"Sync now"** to refresh the
   repository list from GitHub (this requires the Zenodo GitHub
   integration to have been granted access to the account's repositories).
   Toggle the switch **ON** next to `HeliCorgi/ADSC`. This registers a
   webhook on the repository that fires on new GitHub Releases; it does
   **not** archive anything by itself yet.
3. **(Recommended, not yet done in this repo) Add citation metadata.**
   Add a `CITATION.cff` file (and/or a `.zenodo.json`) at the repository
   root with title, author/creator (`HeliCorgi`), license (MIT), and a
   short description. Without this, Zenodo auto-generates a deposition
   from bare GitHub metadata (repo name/description only), which is
   functional but generic. This file does not exist in the repository
   today — flagged here as a recommended follow-up, not created by this
   PR (out of this PR's exact file list).
4. **Cut and publish a GitHub Release** against the tag to be archived
   (e.g. `v0.15-proposal-package`) via the repository's **Releases** page,
   or `gh release create v0.15-proposal-package --notes-file <notes>`.
   Zenodo's DOI minting triggers on a published **Release**, not on a bare
   tag push — a tag with no corresponding Release will never reach Zenodo.
5. **Zenodo mints the DOI automatically.** Within moments of the Release
   being published, the registered webhook fires; Zenodo downloads a zip
   snapshot of the repository at that tag, creates a new deposition, and
   mints a version-specific DOI. A **concept DOI** (stable across all
   versions of the archived repo) is also minted on the *first* archived
   release and continues to resolve to the latest version thereafter.
6. **Owner reviews/completes the deposition metadata** on the Zenodo
   record page (title, author list, keywords, related identifiers such as
   the GitHub repository URL and the specification document) — the files
   themselves become immutable once published, but metadata fields remain
   editable.
7. **Add the DOI badge back to the repository.** Copy the Markdown badge
   snippet Zenodo generates for the record (a `https://zenodo.org/badge/DOI/...svg`
   image linking to the DOI) and add it near the top of `README.md` and
   this file, pointing at the **concept DOI** (so the badge does not go
   stale on the next tagged release).
8. **Repeat only steps 4–7 for every future archived release** — the
   GitHub sync (steps 1–2) and citation metadata (step 3) are one-time
   setup; every subsequent published Release automatically becomes a new
   version under the same Zenodo concept DOI, preserving a citable version
   history consistent with the tag-at-phase-boundary practice above.

## Explicit deferral note

**The DOI-carrying archived release is deferred, not skipped**, per
spec:288-291 ("archived, DOI-carrying release (e.g., Zenodo) considered
and documented") and WP15 task 5 (`04-wp15-instructions.md`): *"execute if
the owner provides/authorizes a Zenodo account, else commit
`docs/release_engineering.md` documenting the exact procedure and defer."*
No Zenodo account has been provided or authorized as of this PR. This
document is the "considered and documented" deliverable; steps 1–3 above
remain outstanding and gate every step after them. No git tag, GitHub
Release, or Zenodo deposition is created by this PR — creating them is an
owner + Fable5 action per the task's role split, not a mechanical-worker
action.
