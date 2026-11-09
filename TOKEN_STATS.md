# AI Token Usage

<!-- AI_USAGE_BADGES:BEGIN -->
![AI tokens](https://img.shields.io/badge/AI_tokens-13M_out_%2F_3.8B_read-blueviolet) ![est. energy](https://img.shields.io/badge/est._energy-~211_kWh-yellow) ![est. water](https://img.shields.io/badge/est._water-~633_L-blue)
(estimates, see [TOKEN_STATS.md](TOKEN_STATS.md))
<!-- AI_USAGE_BADGES:END -->

This project was developed almost entirely in Claude Code sessions (Claude Fable 5 and Claude Opus 5). This document tracks the exact API token usage of every MikanTrack development session, and derives an order-of-magnitude estimate of the electricity and water that inference consumed. See `LEARNINGS.md` for what all those tokens actually bought.

## How the numbers are collected

Claude Code stores a JSONL transcript per session (including subagent transcripts) under `~/.claude/projects/`, and every assistant message in it records the exact billed token counts per model. `tools/token_stats.py` scans those transcripts, deduplicates streamed message records, sums the sessions listed as MikanTrack work, and rewrites the generated block below plus the badges here and in `README.md`. To update after new sessions: add the new session id to the script's `SESSIONS` table and run

```bash
python tools/token_stats.py
```

The token counts are exact (they are what the API metered). Everything under "Energy and water estimate" is not.

## Estimation methodology

Anthropic publishes no per-token energy figures, so the estimate chains public reference points with stated assumptions. Treat the result as order-of-magnitude.

**Step 1: reduce four token classes to one.** The API meters four classes that cost very different amounts of compute: fresh input, cache writes, cache reads, and output. We fold them into "weighted output-equivalent tokens" using Anthropic's own price ratios as the compute proxy (input = 1/5 of output, cache write = 1/4, cache read = 1/50). Pricing is the closest public signal of relative serving cost. Note the consequence: cache reads dominate this project's weighted total even at 2% weight, because agentic coding re-reads the growing conversation context on every call. That is real work (attention over the cached context) and should not be dropped.

**Step 2: energy per weighted token.** Public anchors: Google reported the median Gemini Apps text prompt at 0.24 Wh (Aug 2025 technical report), Epoch AI estimated ~0.3 Wh for a typical GPT-4o query, and Sam Altman quoted 0.34 Wh for an average ChatGPT query. A median chat prompt is a few hundred output-equivalent tokens, which lands those figures near 1 Wh per 1000 weighted tokens. Fable/Opus-class models are larger than what serves a median consumer prompt, so we take 2 Wh per 1000 weighted tokens as the central assumption, with 0.5 and 6 as low/high bounds. These constants sit at the top of `tools/token_stats.py`; correct them as better data appears.

**Step 3: water per kWh.** Onsite datacenter cooling: ~1.1 L/kWh (consistent with Google's reported fleet WUE and their 0.26 mL per 0.24 Wh prompt figure). Including the water footprint of electricity generation raises it to roughly 3 L/kWh on a typical US grid mix.

**What the estimate excludes:** model training (amortized per query it is generally estimated to be small relative to inference at scale, but it is not zero), datacenter construction, the dev machine running Claude Code locally, and the RTX 3090 that ran the actual hand-tracking workloads this project exists for. It also cannot know Anthropic's actual serving efficiency, batching, or datacenter locations, which is why the range spans an order of magnitude.

**For scale:** the central estimate (~70 kWh) is about two and a half days of average US household electricity, or roughly 140 hours of the RTX 3090 dev machine at full load. The central water estimate (~200 L) is about one bathtub.

<!-- TOKEN_STATS:BEGIN -->
Last regenerated 2026-11-09 06:03 UTC by `tools/token_stats.py`.

## Per-session usage

| Session | Model | API calls | Output | Cache read | Cache write |
|---|---|---|---|---|---|
| Initial build: scaffold to working tracker, fusion, parametric model (Jul 30 - Aug 1) | claude-fable-5 | 745 | 1,109,801 | 245,527,743 | 5,178,551 |
| Initial build: scaffold to working tracker, fusion, parametric model (Jul 30 - Aug 1) | claude-opus-5 | 218 | 239,263 | 56,255,854 | 876,017 |
| Triangulation overhaul, RTMPose A/B, RealSense, IMU, charuco extrinsics (Aug 2 - Aug 10) | claude-fable-5 | 441 | 428,792 | 207,103,745 | 1,080,665 |
| Triangulation overhaul, RTMPose A/B, RealSense, IMU, charuco extrinsics (Aug 2 - Aug 10) | claude-opus-5 | 947 | 1,012,882 | 453,133,941 | 5,483,413 |
| Image quality metrics, record/replay, hitch diagnosis (Aug 10 - Aug 11) | claude-fable-5 | 227 | 355,415 | 73,796,202 | 1,877,293 |
| Image quality metrics, record/replay, hitch diagnosis (Aug 10 - Aug 11) | claude-opus-5 | 199 | 245,359 | 98,648,329 | 1,551,818 |
| Depth A/B, marker-scale discovery, bone calibration, seeding fixes (Aug 11 - Aug 12) | claude-opus-5 | 373 | 406,564 | 143,251,867 | 1,024,437 |
| LEARNINGS.md and token stats (Aug 12) | claude-fable-5 | 21 | 42,320 | 2,036,500 | 116,150 |
| 3-camera right-hand dropout: SQPnP fix, minCameraConfidence gate (Aug 12) | claude-fable-5 | 163 | 150,814 | 31,096,087 | 303,891 |
| Dedicated linux/macos tracking machine consideration (Aug 12) | claude-fable-5 | 14 | 13,324 | 1,013,749 | 70,154 |
| BlazePose revival: opt-in per-camera body pose, shoulders + head OSC (Aug 12) | claude-fable-5 | 151 | 176,162 | 34,416,298 | 379,915 |
| BlazePose revival: opt-in per-camera body pose, shoulders + head OSC (Aug 12) | claude-opus-5 | 785 | 882,945 | 373,642,254 | 3,352,535 |
| Hand state estimator + biomechanical priors: joint fit, limits, fitted angle prior (Aug 15 - Aug 16) | claude-fable-5 | 429 | 443,619 | 245,581,671 | 1,549,360 |
| Hand state estimator + biomechanical priors: joint fit, limits, fitted angle prior (Aug 15 - Aug 16) | claude-opus-5 | 95 | 99,784 | 53,053,665 | 707,108 |
| VMC OSC output mode for VMC4UE and VSeeFace comparison (Aug 16) | claude-opus-5 | 181 | 169,468 | 47,773,144 | 508,165 |
| Left elbow stuck in a non-anatomic pose: recorded-replay diagnosis (Aug 16) | claude-opus-5 | 146 | 113,124 | 33,808,344 | 322,776 |
| OSC forearm position, wrist rotation removal, UE plugin update (Aug 16 - Aug 17) | claude-opus-5 | 92 | 52,147 | 16,392,624 | 217,274 |
| Open-sourcing docs pass: CLAUDE.md, docs/reference, rename to MikanTrack (Aug 17) | claude-fable-5 | 127 | 159,017 | 17,666,658 | 770,851 |
| Open-sourcing docs pass: CLAUDE.md, docs/reference, rename to MikanTrack (Aug 17) | claude-opus-5 | 59 | 53,593 | 15,022,036 | 473,308 |
| Open-sourcing docs pass: CLAUDE.md, docs/reference, rename to MikanTrack (Aug 17) | claude-sonnet-5 | 85 | 72,133 | 8,571,544 | 386,751 |
| Project files, guided setup flow, tracking configurations (Aug 17 - Aug 18) | claude-fable-5 | 273 | 314,321 | 83,780,411 | 1,071,121 |
| Project files, guided setup flow, tracking configurations (Aug 17 - Aug 18) | claude-opus-5 | 113 | 93,677 | 9,050,492 | 601,205 |
| Project files, guided setup flow, tracking configurations (Aug 17 - Aug 18) | claude-sonnet-5 | 360 | 455,086 | 36,320,823 | 938,224 |
| Single-camera removal and SystemAnimatorOnline transfer consideration (Aug 18 - Aug 19) | claude-opus-5 | 56 | 77,596 | 8,637,133 | 342,360 |
| testing.md reference doc (Aug 19) | claude-opus-5 | 38 | 24,631 | 3,748,073 | 111,058 |
| Cleanup pass: flicker tracker, ImuService split, CameraContext, capture timing, MainWindow split, loop stages (Sep 29) | claude-fable-5-1 | 106 | 243,969 | 36,487,268 | 508,718 |
| VRM avatar loading, the scene avatar, retargeting the pose onto it, and the VMC bone stream (Oct 1) | claude-fable-5-1 | 127 | 429,960 | 53,922,752 | 2,246,586 |
| VRM avatar loading, the scene avatar, retargeting the pose onto it, and the VMC bone stream (Oct 1) | claude-opus-5-5 | 78 | 37,545 | 6,989,301 | 539,930 |
| Avatar rig settings: bone remapping, rest trims, elbow hints, hand tweaks (Oct 2 - Oct 3) | claude-opus-5-5 | 99 | 116,320 | 21,893,883 | 892,215 |
| Avatar rig settings: bone remapping, rest trims, elbow hints, hand tweaks (Oct 2 - Oct 3) | claude-sonnet-5-5 | 5 | 636 | 180,003 | 56,286 |
| iFacialMocap face service, VMC face blendshapes, CPU face morph, Mikan OSC schema removal, shared with link-engine work (Oct 4 - Oct 5) | claude-fable-5-1 | 797 | 884,628 | 402,530,506 | 6,472,703 |
| iFacialMocap face service, VMC face blendshapes, CPU face morph, Mikan OSC schema removal, shared with link-engine work (Oct 4 - Oct 5) | claude-opus-5-5 | 2,545 | 543,292 | 451,658,770 | 10,404,194 |
| iFacialMocap face service, VMC face blendshapes, CPU face morph, Mikan OSC schema removal, shared with link-engine work (Oct 4 - Oct 5) | claude-sonnet-5-5 | 46 | 1,042 | 4,248,740 | 241,602 |
| Single-camera direct retarget: RTMW3D backend, DirectRetarget solver, guided setup, finger angles, rest easing (Oct 30 - Nov 2) | claude-fable-5-1 | 983 | 1,642,551 | 225,568,659 | 5,849,887 |
| Single-camera direct retarget: RTMW3D backend, DirectRetarget solver, guided setup, finger angles, rest easing (Oct 30 - Nov 2) | claude-opus-5-5 | 220 | 166,406 | 23,571,623 | 1,340,990 |
| Single-camera direct retarget: RTMW3D backend, DirectRetarget solver, guided setup, finger angles, rest easing (Oct 30 - Nov 2) | claude-sonnet-5-5 | 62 | 57,337 | 3,966,544 | 402,765 |
| Microphone mouth stage: MFCC visemes, uLipSync templates, Oculus LipSync, the Audio panel (Nov 6 - Nov 7) | claude-fable-5-1 | 532 | 990,235 | 120,114,216 | 4,232,195 |
| Microphone mouth stage: MFCC visemes, uLipSync templates, Oculus LipSync, the Audio panel (Nov 6 - Nov 7) | claude-opus-5-5 | 120 | 105,526 | 12,096,777 | 544,508 |
| Microphone mouth stage, the forked first pass (Nov 6) | claude-fable-5-1 | 137 | 202,174 | 26,671,925 | 805,247 |
| Face expression proposal (Nov 7) | claude-opus-5-5 | 48 | 34,768 | 5,122,476 | 289,624 |
| Face expression proposal review, implementation review and fixes, Face tab adjustments, landing by concern (Nov 7 - Nov 8) | claude-fable-5-1 | 96 | 130,285 | 18,177,745 | 858,573 |
| Face expression proposal review, implementation review and fixes, Face tab adjustments, landing by concern (Nov 7 - Nov 8) | claude-opus-5-5 | 214 | 188,798 | 23,835,496 | 722,459 |
| Face expression system implementation (Nov 8) | claude-opus-5-5 | 159 | 203,086 | 44,555,615 | 871,072 |

## Totals per model

| Model | API calls | Input | Cache write | Cache read | Output | Weighted |
|---|---|---|---|---|---|---|
| claude-fable-5 | 2,591 | 21,074 | 12,397,951 | 942,019,064 | 3,193,585 | 25,137,668 |
| claude-fable-5-1 | 2,778 | 78,784 | 20,973,909 | 883,473,071 | 4,523,802 | 27,452,497 |
| claude-opus-5 | 3,302 | 45,258 | 15,571,474 | 1,312,417,756 | 3,471,033 | 33,621,308 |
| claude-opus-5-5 | 3,483 | 6,986 | 15,604,992 | 589,723,941 | 1,395,741 | 17,092,865 |
| claude-sonnet-5 | 445 | 13,920 | 1,324,975 | 44,892,367 | 527,219 | 1,759,094 |
| claude-sonnet-5-5 | 113 | 226 | 700,653 | 8,395,287 | 59,015 | 402,129 |
| **total** | 12,712 | 166,248 | 66,573,954 | 3,780,921,486 | 13,170,395 | **105,465,562** |

## Energy and water estimate

| Scenario | Wh per 1k weighted tokens | Energy | Water (onsite cooling) | Water (incl. generation) |
|---|---|---|---|---|
| low | 0.5 | 53 kWh | 58 L | 158 L |
| central | 2.0 | 211 kWh | 232 L | 633 L |
| high | 6.0 | 633 kWh | 696 L | 1,898 L |
<!-- TOKEN_STATS:END -->
