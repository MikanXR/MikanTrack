"""Shoulder-to-wrist reach against the stereo truth for the direct chain, the Kalidokit-style baseline,
MediaPipe Holistic's world landmarks, and Kalidokit's rig rotations posed on a T-pose arm as its 3d sample applies them.

  python score_ab.py <recording.jsonl> <gate-direct frames.csv> <holistic.jsonl> <rig.jsonl> <cameraIndex>

The recording is the one the gate ran on. It also prints the reach with the wrist on the direct chain's ray at a
fixed distance ahead of the shoulder, and each candidate's step between model frames.
"""
import csv, json, sys, itertools
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

take_jsonl, gate_csv, holistic_path, rig_path, cam = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], int(sys.argv[5])
UPPER, FORE = 0.30, 0.26

def v(r, p): return np.array([float(r[p + a]) for a in "XYZ"])
def ang(a, b): return np.degrees(np.arccos(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9), -1, 1)))

# record index -> the camera's frame index
frame_of = {}
with open(take_jsonl, encoding="utf-8") as f:
    f.readline(); n = -1
    for line in f:
        d = json.loads(line)
        if d["type"] != "frame": continue
        n += 1
        for c in d["fresh"]:
            if c["cam"] == cam: frame_of[n] = c["frameIndex"]

holistic = {}
for line in open(holistic_path):
    d = json.loads(line); holistic[d["frameIndex"]] = d
order = sorted(holistic)

# Kalidokit: the sample slerps each bone 0.3 of the way to its target every frame
def euler(e, k=1.0): return R.from_euler("XYZ", [e["x"] * k, e["y"] * k, e["z"] * k])   # three.js Euler order XYZ is intrinsic
state = {}
kalido = {}
rigs = {}
for line in open(rig_path):
    d = json.loads(line); rigs[d["frameIndex"]] = d["rig"]
def step(name, target):
    cur = state.get(name, R.identity())
    nxt = Slerp([0, 1], R.concatenate([cur, target]))(0.3)
    state[name] = nxt
    return nxt
scene = R.from_euler("y", np.pi)
for fi in order:
    rig = rigs.get(fi)
    if rig is None: continue
    hips = step("Hips", euler(rig["Hips"], 0.7)); spine = step("Spine", euler(rig["Spine"], 0.45)); chest = step("Chest", euler(rig["Spine"], 0.25))
    torso = scene * hips * spine * chest
    out = {}
    for arm, rest in (("Right", np.array([1.0, 0, 0])), ("Left", np.array([-1.0, 0, 0]))):
        ua = step(arm + "UpperArm", euler(rig[arm + "UpperArm"])); la = step(arm + "LowerArm", euler(rig[arm + "LowerArm"]))
        out[arm] = (torso * ua).apply(rest * UPPER + la.apply(rest * FORE))
        out[arm + "Rest"] = all(abs(rig[arm + "LowerArm"][k]) < 1e-9 for k in "xyz")
    kalido[fi] = out

rows = []
for r in csv.DictReader(open(gate_csv, newline="")):
    if float(r["truthWristZ"]) <= 0 or r["directChain"] != "1" or r["directPosed"] != "1" or r["baselinePosed"] != "1": continue
    fi = frame_of.get(int(r["frame"])); h = holistic.get(fi)
    side = int(r["side"])
    tS, tW = v(r, "truthShoulder"), v(r, "truthWrist")
    row = dict(side=side, frame=int(r["frame"]), truth=tW - tS, direct=v(r, "directChainReach"), baseline=v(r, "baselineReach"), shoulder=tS,
               world=None, kal=None, kalRest=False)
    if h and h["world"]:
        w = np.array(h["world"]); s, wr = (11, 15) if side == 0 else (12, 16)
        row["world"] = w[wr, :3] - w[s, :3]; row["worldVis"] = w[wr, 3]
    if fi in kalido:
        arm = "Right" if side == 0 else "Left"      # Kalidokit drives the avatar's right arm from the user's left
        row["kal"] = -kalido[fi][arm]; row["kalRaw"] = kalido[fi]; row["kalRest"] = kalido[fi][arm + "Rest"]
    rows.append(row)
print("arm frames with truth and both MikanTrack candidates:", len(rows), "| with Holistic pose:", sum(r["world"] is not None for r in rows), "| with a Kalidokit rig:", sum(r["kal"] is not None for r in rows))
shared = [r for r in rows if r["world"] is not None and r["kal"] is not None]
print("shared by all four:", len(shared), "| Kalidokit put the arm at rest on", round(np.mean([r["kalRest"] for r in shared]), 2))

# check the avatar-to-camera convention: every sign flip and both arm pairings
best = []
for signs in itertools.product([1, -1], repeat=3):
    for swap in (False, True):
        a = []
        for r in shared:
            arm = ("Right" if r["side"] == 0 else "Left") if not swap else ("Left" if r["side"] == 0 else "Right")
            a.append(ang(r["truth"], np.array(signs) * r["kalRaw"][arm]))
        best.append((np.median(a), signs, swap))
best.sort(key=lambda t: t[0])
print("Kalidokit avatar-to-camera conventions by p50 (derived one is signs (-1,-1,-1), no swap):", [(round(b[0], 1), b[1], b[2]) for b in best[:4]])

def line(name, sel, key):
    a = np.array([ang(r["truth"], r[key]) for r in sel])
    print(f"   {name:34s} p50 {np.median(a):5.1f}  p90 {np.percentile(a, 90):5.1f}  over 60 deg {np.mean(a > 60):.2f}")
def plane(r, d):
    ray = r["shoulder"] + r["direct"]; ray = ray / np.linalg.norm(ray)
    return ray * ((r["shoulder"][2] - d) / ray[2]) - r["shoulder"]
for label, sel in (("all shared frames", shared), ("reach at the camera (truth wrist 10 cm or more nearer)", [r for r in shared if r["truth"][2] < -0.1]),
                   ("the rest", [r for r in shared if r["truth"][2] >= -0.1]), ("frames Kalidokit posed (not at rest)", [r for r in shared if not r["kalRest"]])):
    print(f"{label}: n={len(sel)}")
    line("MikanTrack direct chain", sel, "direct")
    line("Kalidokit-style baseline (our model)", sel, "baseline")
    line("Holistic world landmarks, raw", sel, "world")
    line("Kalidokit rig on the avatar", sel, "kal")
    for d in (0.15, 0.20, 0.25):
        for r in sel: r["plane"] = plane(r, d)
        line(f"wrist on a plane {d:.2f} m ahead", sel, "plane")

# frame-to-frame change of each reach direction, against the truth's own
print("step between consecutive model frames, degrees (p50, p90, share over 20):")
for key, name in (("truth", "stereo truth"), ("direct", "MikanTrack direct chain"), ("baseline", "Kalidokit-style baseline"), ("world", "Holistic world landmarks"), ("kal", "Kalidokit rig on the avatar")):
    steps = []
    for side in (0, 1):
        seq = [r for r in shared if r["side"] == side]
        for a, b in zip(seq, seq[1:]):
            if b["frame"] - a["frame"] <= 4: steps.append(ang(a[key], b[key]))
    steps = np.array(steps)
    print(f"   {name:30s} {np.median(steps):5.1f} {np.percentile(steps, 90):5.1f} {np.mean(steps > 20):.3f}")
