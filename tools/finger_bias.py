import json, math, pathlib, statistics, sys

# The hand model's bend on a straight finger, from a diagnostic dump taken
# with both hands held up open, palms to the camera, fingers straight and
# relaxed: the per-joint median of the raw per-camera finger angles over the
# dump's history, per hand and pooled, in degrees. The pooled four-finger and
# thumb medians are the open-hand bias the direct retarget subtracts
# (DirectRetarget::kFingerOpenBias, kThumbOpenBias); re-measure them from a
# fresh dump when the hand model changes.
#
#   python tools/finger_bias.py <dump.json> [cameraIndex]

if len(sys.argv) < 2:
    print("usage: finger_bias.py <dump.json> [cameraIndex]")
    sys.exit(1)

DUMP = pathlib.Path(sys.argv[1])
CAMERA = int(sys.argv[2]) if len(sys.argv) > 2 else 0
FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
JOINTS = ["lateral", "proximal", "intermediate", "distal"]

dump = json.loads(DUMP.read_text(encoding="utf-8"))
history = dump["history"]

pooled = {"thumb": [[], [], [], []], "fingers": [[], [], [], []]}
for side in ("left", "right"):
    rows = [f["cameras"][CAMERA][side]["fingers"] for f in history if f["cameras"][CAMERA][side].get("tracked")]
    print(f"{side}: {len(rows)} tracked frames")
    for fingerIndex, finger in enumerate(FINGERS):
        medians = []
        for joint in range(4):
            values = [math.degrees(r[fingerIndex][joint]) for r in rows]
            medians.append(statistics.median(values) if values else float("nan"))
            group = "thumb" if finger == "thumb" else "fingers"
            pooled[group][joint].extend(values)
        print("  {:6s} ".format(finger) + "  ".join(f"{name} {value:6.1f}" for name, value in zip(JOINTS, medians)))

print("pooled medians (the bias, proximal / intermediate / distal):")
for group in ("fingers", "thumb"):
    bends = [statistics.median(pooled[group][joint]) if pooled[group][joint] else float("nan") for joint in (1, 2, 3)]
    print("  {:8s} {:5.1f} / {:5.1f} / {:5.1f} deg".format(group, *bends))
