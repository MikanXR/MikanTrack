# AI Token Usage

<!-- AI_USAGE_BADGES:BEGIN -->
![AI tokens](https://img.shields.io/badge/AI_tokens-21M_out_%2F_5.5B_read-blueviolet) ![API-equiv. cost](https://img.shields.io/badge/API--equiv._cost-~$4.2k-green)
(cost is the API-equivalent at list prices, see [TOKEN_STATS.md](TOKEN_STATS.md))
<!-- AI_USAGE_BADGES:END -->

This project was developed almost entirely in Claude Code sessions, across whichever Claude models were current at the time. This document tracks the exact API token usage of every MikanTrack development session and what those tokens would cost at list prices. It also derives an order-of-magnitude estimate of the electricity and water that inference consumed. See `LEARNINGS.md` for what all those tokens actually bought.

## How the numbers are collected

Claude Code writes one JSONL transcript per session under `~/.claude/projects/`, plus a transcript per subagent, and every assistant record carries the exact token counts the API metered. Those transcripts are local to one machine and are pruned on a retention timer, so they are a collection source rather than a source of truth.

The source of truth is `docs/ai-usage.json`, a committed ledger holding per-session, per-model counters. `tools/token_stats.py` scans the transcripts, folds them into that ledger, and renders the badges above and the generated block below from the ledger alone.

```bash
python tools/token_stats.py
```

Three properties make that safe to run at any cadence.

- Counters only ever grow. A session is merged with `max(stored, scanned)` per class, so a pruned or partially rotated transcript cannot erase recorded history, and a resumed session grows correctly.

- Attribution is recorded, not recomputed. A session counts toward MikanTrack when the working directory recorded in its transcript is this repository or one of its worktrees. Much of this project was developed in sessions started from a sibling checkout, which that rule cannot see, so the script also scans that checkout's transcript folder and counts a session from it only when the ledger already lists it. Such a session is claimed by hand: add its id to the ledger's `sessions` map with a `label`, take it out of `excluded` if a scan put it there, and run the script. Everything else lands in the `excluded` map, which is authoritative and hand-editable. Adding an id there with `"reason": "manual"` and running `python tools/token_stats.py --render` deletes its counters for good, since later scans never re-add an excluded id.

- Rendering is a pure function of the ledger. Running the script twice with no new sessions produces no diff, and `python tools/token_stats.py --check` re-renders from the ledger and fails if the published files have drifted.

The ledger stores no absolute paths. Session ids are opaque, and the script refuses to write any string that looks like a filesystem location, so the file carries no drive letter, home directory, or user name.

The token counts are exact (they are what the API metered). The cost is those counts at list prices. Everything under "Energy and water estimate" is an estimate.

## Cost estimate

The cost column prices each model's counters at Anthropic's first-party list prices per token class: input, cache writes at each of the two cache lifetimes, cache reads, and output. The price table sits at the top of `tools/token_stats.py` with the date it was read, and a model missing from it stops the render, so a new model cannot go unpriced.

It is an API-equivalent figure, a way to read the token counts in human terms. It is not what the work was billed, since most of it ran under a subscription. Fast mode and batch pricing are ignored, and Claude Haiku 5.5 is priced at its rate for prompts up to 100,000 tokens because the ledger holds no per-request prompt size.

## Energy and water methodology

Anthropic publishes no per-token energy figures, so the estimate chains public reference points with stated assumptions. Treat the result as order-of-magnitude.

**Step 1: reduce the token classes to one.** The API meters classes that cost very different amounts of compute: fresh input, cache writes at each of the two cache lifetimes, cache reads, and output. We fold them into "weighted output-equivalent tokens" using Anthropic's standard price ratios as the compute proxy, since pricing is the closest public signal of relative serving cost. The table is fixed and applies to every model. Some newer models discount cache reads further, which the cost above follows and this table deliberately does not, so the energy estimate does not move when pricing does.

| Token class | Standard price relative to base input | Weight in output-equivalent tokens |
|---|---|---|
| Output | 5 | 1 |
| Input | 1 | 1/5 |
| Cache write (1 hour) | 2 | 2/5 |
| Cache write (5 minute) | 1.25 | 1/4 |
| Cache read | 0.1 | 1/50 |

Note the consequence: cache reads dominate this project's weighted total even at 2% weight, because agentic coding re-reads the growing conversation context on every call. That is real work (attention over the cached context) and should not be dropped.

**Step 2: energy per weighted token.** Public anchors: Google reported the median Gemini Apps text prompt at 0.24 Wh (Aug 2025 technical report), Epoch AI estimated ~0.3 Wh for a typical GPT-4o query, and Sam Altman quoted 0.34 Wh for an average ChatGPT query. A Microsoft study of inference at production scale ([Oviedo et al.](https://arxiv.org/abs/2509.20241), published in Joule in 2026) puts the median query to a frontier-scale model at 0.31 Wh, with an interquartile range of 0.16 to 0.60 Wh. A median chat prompt is a few hundred output-equivalent tokens, which lands those figures near 1 Wh per 1000 weighted tokens. Fable/Opus-class models are larger than what serves a median consumer prompt, so we take 2 Wh per 1000 weighted tokens as the central assumption, with 0.5 and 6 as low/high bounds. These constants sit at the top of `tools/token_stats.py`; correct them as better data appears.

**Step 3: water per kWh.** Onsite datacenter cooling: ~1.1 L/kWh (consistent with Google's reported fleet WUE and their 0.26 mL per 0.24 Wh prompt figure). Including the water footprint of electricity generation raises it to roughly 3 L/kWh on a typical US grid mix.

**Where the estimate is weakest.** Three simplifications carry most of the uncertainty.

- Cache reads are the largest share of the weighted total, and their weight comes from a price ratio. Public measurement work has not yet established what a cached prefix costs in energy.

- One energy rate covers every model. Measurement work models energy per token as a function of model size and treats input and output tokens separately, as in [Greenpixie's AI token methodology](https://arxiv.org/abs/2609.33965). That paper does not publish the per-model figures a size-aware rate would need.

- Measured energy per token varies several-fold with hardware and serving configuration, none of which is known here.

**What the estimate excludes:** model training (amortized per query it is generally estimated to be small relative to inference at scale, but it is not zero), datacenter construction, the dev machine running Claude Code locally, and the RTX 3090 that ran the actual hand-tracking workloads this project exists for. It also cannot know Anthropic's actual serving efficiency, batching, or datacenter locations, which is why the range spans an order of magnitude. A "for scale" line under the estimate table below is generated with the table, so it follows the totals.

<!-- TOKEN_STATS:BEGIN -->
Generated by `tools/token_stats.py` from `docs/ai-usage.json`, covering 49 sessions through 2026-11-21.

## Per-session usage

| Session | Dates | Models | API calls | Output | Cache read |
|---|---|---|---|---|---|
| Initial build: scaffold to working tracker, fusion, parametric model | 2026-07-30 to 2026-08-01 | claude-fable-5, claude-opus-5 | 963 | 1,349,064 | 301,783,597 |
| Triangulation overhaul, RTMPose A/B, RealSense, IMU, charuco extrinsics | 2026-08-02 to 2026-08-10 | claude-fable-5, claude-opus-5 | 1,388 | 1,441,674 | 660,237,686 |
| Image quality metrics, record/replay, hitch diagnosis | 2026-08-10 to 2026-08-11 | claude-fable-5, claude-opus-5 | 426 | 600,774 | 172,444,531 |
| Depth A/B, marker-scale discovery, bone calibration, seeding fixes | 2026-08-11 to 2026-08-12 | claude-opus-5 | 373 | 406,564 | 143,251,867 |
| LEARNINGS.md and token stats | 2026-08-12 | claude-fable-5 | 21 | 42,320 | 2,036,500 |
| 3-camera right-hand dropout: SQPnP fix, minCameraConfidence gate | 2026-08-13 | claude-fable-5 | 163 | 150,814 | 31,096,087 |
| Dedicated linux/macos tracking machine consideration | 2026-08-13 | claude-fable-5 | 14 | 13,324 | 1,013,749 |
| BlazePose revival: opt-in per-camera body pose, shoulders + head OSC | 2026-08-13 to 2026-08-15 | claude-fable-5, claude-opus-5 | 936 | 1,059,107 | 408,058,552 |
| VMC OSC output mode for VMC4UE and VSeeFace comparison | 2026-08-16 | claude-opus-5 | 181 | 169,468 | 47,773,144 |
| Hand state estimator + biomechanical priors: joint fit, limits, fitted angle prior | 2026-08-16 | claude-fable-5, claude-opus-5 | 524 | 543,403 | 298,635,336 |
| Left elbow stuck in a non-anatomic pose: recorded-replay diagnosis | 2026-08-16 | claude-opus-5 | 146 | 113,124 | 33,808,344 |
| OSC forearm position, wrist rotation removal, UE plugin update | 2026-08-16 to 2026-08-17 | claude-opus-5 | 92 | 52,147 | 16,392,624 |
| Open-sourcing docs pass: CLAUDE.md, docs/reference, rename to MikanTrack | 2026-08-17 | claude-fable-5, claude-opus-5, claude-sonnet-5 | 271 | 284,743 | 41,260,238 |
| Project files, guided setup flow, tracking configurations | 2026-08-17 to 2026-08-18 | claude-fable-5, claude-opus-5, claude-sonnet-5 | 746 | 863,084 | 129,151,726 |
| Single-camera removal and SystemAnimatorOnline transfer consideration | 2026-08-18 to 2026-08-19 | claude-opus-5 | 56 | 77,596 | 8,637,133 |
| testing.md reference doc | 2026-08-19 | claude-opus-5 | 38 | 24,631 | 3,748,073 |
| Cleanup pass: flicker tracker, ImuService split, CameraContext, capture timing, MainWindow split, loop stages | 2026-09-30 | claude-fable-5-1 | 106 | 243,969 | 36,487,268 |
| VRM avatar loading, the scene avatar, retargeting the pose onto it, and the VMC bone stream | 2026-10-01 to 2026-10-02 | claude-fable-5-1, claude-opus-5-5 | 205 | 467,505 | 60,912,053 |
| Avatar rig settings: bone remapping, rest trims, elbow hints, hand tweaks | 2026-10-02 to 2026-10-03 | claude-opus-5-5, claude-sonnet-5-5 | 104 | 116,956 | 22,073,886 |
| iFacialMocap face service, VMC face blendshapes, CPU face morph, Mikan OSC schema removal, shared with link-engine work | 2026-10-04 to 2026-10-16 | claude-fable-5-1, claude-opus-5-5, claude-sonnet-5-5 | 3,388 | 1,428,962 | 858,438,016 |
| Single-camera direct retarget: RTMW3D backend, DirectRetarget solver, guided setup, finger angles, rest easing | 2026-10-31 to 2026-11-03 | claude-fable-5-1, claude-opus-5-5, claude-sonnet-5-5 | 1,265 | 1,866,294 | 253,106,826 |
| Microphone mouth stage: MFCC visemes, uLipSync templates, Oculus LipSync, the Audio panel | 2026-11-06 to 2026-11-07 | claude-fable-5-1, claude-opus-5-5 | 652 | 1,095,761 | 132,210,993 |
| Microphone mouth stage, the forked first pass | 2026-11-06 | claude-fable-5-1 | 137 | 202,174 | 26,671,925 |
| Face expression proposal | 2026-11-07 | claude-opus-5-5 | 48 | 34,768 | 5,122,476 |
| Face expression proposal review, implementation review and fixes, Face tab adjustments, landing by concern | 2026-11-07 to 2026-11-09 | claude-fable-5-1, claude-opus-5-5 | 319 | 323,349 | 43,099,214 |
| Face expression system implementation | 2026-11-08 to 2026-11-09 | claude-opus-5-5 | 159 | 203,086 | 44,555,615 |
| Expression editor UI for a VRM face expression formula | 2026-11-09 to 2026-11-10 | claude-fable-5-1, claude-fable-5-2, claude-opus-5-5 | 383 | 461,494 | 72,434,117 |
| How the tracked pose is retargeted to the avatar skeleton | 2026-11-10 to 2026-11-11 | claude-fable-5-2, claude-opus-5-5, claude-sonnet-5-5 | 2,654 | 2,971,836 | 658,103,511 |
| Audio panel viseme meter layout | 2026-11-10 | claude-opus-5-5 | 21 | 5,226 | 1,283,006 |
| Camera frame synchronization on a multi-camera rig, research | 2026-11-10 | claude-opus-5-5 | 8 | 17,707 | 496,790 |
| Hand inference time spikes: single-warp landmark crop, --replay-hands | 2026-11-12 | claude-opus-5-5 | 100 | 70,121 | 16,656,215 |
| Which body model the single-camera setup runs | 2026-11-12 | claude-opus-5-5 | 51 | 34,953 | 6,075,021 |
| Whole-body landmarks proposal review | 2026-11-12 to 2026-11-13 | claude-fable-5-2 | 451 | 577,628 | 179,646,142 |
| Dimmed low-visibility landmark overlays | 2026-11-12 | claude-opus-5-5 | 43 | 26,341 | 4,296,846 |
| Follow-on to the whole-body landmarks work | 2026-11-12 to 2026-11-13 | claude-fable-5-2 | 395 | 410,869 | 90,762,226 |
| Camera intrinsics from a human body in view | 2026-11-13 to 2026-11-15 | claude-fable-5-2, claude-opus-5-5 | 1,068 | 1,097,398 | 245,318,187 |
| Face tracking comparison on the single-camera body model | 2026-11-13 | claude-opus-5-5 | 224 | 152,789 | 37,503,538 |
| Hands stuck in frame after the real hand has left | 2026-11-14 | claude-fable-5-2 | 101 | 85,492 | 25,669,433 |
| How MotionBERT lifts body landmarks into 3D, research | 2026-11-14 | claude-fable-5-2, claude-opus-5-5 | 92 | 90,263 | 11,336,033 |
| Make the mounting wizard's hand-direction check less marginal | 2026-11-14 | claude-fable-5-2 | 17 | 38,023 | 1,861,270 |
| btw: I was just a Pearson correlation is for determining the cor... | 2026-11-14 | claude-fable-5-2 | 11 | 26,502 | 1,002,356 |
| Getting the Kalidokit 3d sample to run | 2026-11-14 | claude-opus-5-5 | 20 | 5,842 | 1,298,728 |
| Switch controllers as arm trackers on the PC | 2026-11-15 | claude-opus-5-5 | 6 | 3,969 | 384,489 |
| Two profiling hot paths on the main and vision threads | 2026-11-15 | claude-opus-5-5 | 229 | 162,201 | 47,737,282 |
| Body model on every frame: divider default, held challenge frame, camera delivery grid | 2026-11-15 to 2026-11-16 | claude-fable-5-2 | 127 | 111,450 | 28,753,748 |
| Character pose tracking quality issues | 2026-11-16 | claude-fable-5-2 | 126 | 176,012 | 38,303,324 |
| Implement the Direct-mode UI regroup | 2026-11-20 | claude-fable-5-2, claude-opus-5-5 | 438 | 226,502 | 104,799,756 |
| Scene view cube and chain gizmo, IK chain modes, custom pose library | 2026-11-20 to 2026-11-21 | claude-fable-5-2, claude-opus-5-5 | 414 | 416,357 | 108,984,338 |
| Webcam blink readout history plots, held winks and the second-frame rule | 2026-11-21 | claude-fable-5-2 | 136 | 191,009 | 40,808,414 |

## Totals per model

| Model | API calls | Input | Cache write | Cache read | Output | Weighted | API-equiv. cost |
|---|---|---|---|---|---|---|---|
| claude-fable-5 | 2,591 | 21,074 | 12,397,951 | 942,019,064 | 3,193,585 | 26,698,430 | $1,335 |
| claude-fable-5-1 | 2,924 | 82,954 | 21,787,117 | 912,715,240 | 4,777,718 | 31,714,543 | $901 |
| claude-fable-5-2 | 3,264 | 7,184 | 11,526,016 | 1,013,431,271 | 3,978,612 | 28,821,438 | $545 |
| claude-opus-5 | 3,302 | 45,258 | 15,571,474 | 1,312,417,756 | 3,471,033 | 35,733,336 | $893 |
| claude-opus-5-5 | 7,176 | 14,504 | 28,669,148 | 1,270,287,986 | 4,498,997 | 37,808,429 | $502 |
| claude-sonnet-5 | 445 | 13,920 | 1,324,975 | 44,892,367 | 527,219 | 1,759,094 | $18 |
| claude-sonnet-5-5 | 134 | 268 | 988,477 | 9,758,545 | 87,481 | 529,824 | $4 |
| **total** | 19,836 | 185,162 | 92,265,158 | 5,505,522,229 | 20,534,645 | **163,065,098** | **$4,198** |

## Energy and water estimate

| Scenario | Wh per 1k weighted tokens | Energy | Water (onsite cooling) | Water (incl. generation) |
|---|---|---|---|---|
| low | 0.5 | 82 kWh | 90 L | 245 L |
| central | 2.0 | 326 kWh | 359 L | 978 L |
| high | 6.0 | 978 kWh | 1,076 L | 2,935 L |

For scale: the central estimate (326 kWh) is about 11 days of average US household electricity, or roughly 652 hours of the RTX 3090 dev machine at full load. The central water estimate (978 L) is about 5 bathtubs.
<!-- TOKEN_STATS:END -->
