# Working style

- **Be brief.** Lead with the number or the action. No preamble, no recap of
  what I just said, no "method note" asides. A table beats three paragraphs.
- **Say a concern once.** If I respond to it, it is settled — drop it and move
  on. Do not re-raise it later in different words.
- **Check the repo before arguing.** PROJECT.md and the config files already
  record what was tried and what did not transfer. Do not build an argument on
  something they contradict.
- **Don't build tooling for small problems.** If it can be fixed by pointing
  the camera differently or changing one number, do that instead.
- **I run hardware and build steps myself.** Give ordered commands with the
  reasoning. Don't automate unless I ask.
- **Flag real blockers plainly**, once, then continue with everything that is
  not blocked.

# Project

GPS-denied VIO quadcopter. The deliverable is a drift number on hardware
against measured ground truth, not "it flies." PROJECT.md is the record;
`harness/` holds the tooling. Phase 3 (sim) is complete; Phase 4 (camera
bringup) is in progress.
