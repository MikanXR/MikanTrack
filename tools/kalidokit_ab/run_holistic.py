"""MediaPipe Holistic over one camera's raw frames of a take, in frame order, as the
Kalidokit 3d sample configures it.

  python run_holistic.py <recording_frames_dir> <cameraIndex> <holistic.jsonl>
"""
import glob, json, os, re, sys
import cv2
import mediapipe as mp

frames_dir, cam, out_path = sys.argv[1], int(sys.argv[2]), sys.argv[3]
files = sorted(glob.glob(os.path.join(frames_dir, f"cam{cam}_*.jpg")), key=lambda p: int(re.search(r"_(\d+)\.jpg$", p).group(1)))
holistic = mp.solutions.holistic.Holistic(static_image_mode=False, model_complexity=1, smooth_landmarks=True,
                                          refine_face_landmarks=True, min_detection_confidence=0.7,
                                          min_tracking_confidence=0.7)
def pack(lms, vis=False):
    if lms is None: return None
    return [[p.x, p.y, p.z] + ([p.visibility] if vis else []) for p in lms.landmark]
with open(out_path, "w") as out:
    for i, path in enumerate(files):
        image = cv2.imread(path)
        result = holistic.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        out.write(json.dumps({"frameIndex": int(re.search(r"_(\d+)\.jpg$", path).group(1)), "w": image.shape[1], "h": image.shape[0],
                              "pose": pack(result.pose_landmarks, True), "world": pack(result.pose_world_landmarks, True),
                              "leftHand": pack(result.left_hand_landmarks), "rightHand": pack(result.right_hand_landmarks),
                              "face": result.face_landmarks is not None}) + "\n")
        if i % 300 == 0: print(i, len(files), flush=True)
print("done", len(files))
