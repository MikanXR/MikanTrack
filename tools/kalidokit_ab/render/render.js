// Kalidokit's sample/3d/script.js with the webcam replaced by a take's raw frames in order, and each
// posed frame posted back to the server. The rigging below is the sample's, unchanged.
const remap = Kalidokit.Utils.remap;
const clamp = Kalidokit.Utils.clamp;
const lerp = Kalidokit.Vector.lerp;

const WIDTH = 1280;
const HEIGHT = 720;
const FRAME_SECONDS = 1 / 30;
const statusElement = document.getElementById("status");

let currentVrm;

const renderer = new THREE.WebGLRenderer({ alpha: false, preserveDrawingBuffer: true });
renderer.setSize(WIDTH, HEIGHT);
renderer.setPixelRatio(1);
renderer.setClearColor(0x0d0d12);
document.body.appendChild(renderer.domElement);

// The sample's lens, pulled back to hold the arms of a seated figure. Kalidokit moves the hips toward
// and away from the viewer, so the camera rides with them and the upper body stays framed.
const camera = new THREE.PerspectiveCamera(35, WIDTH / HEIGHT, 0.1, 1000);
const CAMERA_ABOVE_HIPS = 0.35;
const CAMERA_DISTANCE = 1.4;
const hipsPosition = new THREE.Vector3();
const followHips = () => {
  currentVrm.humanoid.getBoneNode(THREE.VRMSchema.HumanoidBoneName.Hips).getWorldPosition(hipsPosition);
  camera.position.set(hipsPosition.x, hipsPosition.y + CAMERA_ABOVE_HIPS, hipsPosition.z + CAMERA_DISTANCE);
  camera.lookAt(hipsPosition.x, hipsPosition.y + CAMERA_ABOVE_HIPS, hipsPosition.z);
};

const scene = new THREE.Scene();
const light = new THREE.DirectionalLight(0xffffff);
light.position.set(1.0, 1.0, 1.0).normalize();
scene.add(light);

const loadVrm = () =>
  new Promise((resolve, reject) => {
    const loader = new THREE.GLTFLoader();
    loader.load(
      "/vrm",
      (gltf) => {
        THREE.VRMUtils.removeUnnecessaryJoints(gltf.scene);
        THREE.VRM.from(gltf).then((vrm) => {
          scene.add(vrm.scene);
          currentVrm = vrm;
          currentVrm.scene.rotation.y = Math.PI; // Rotate model 180deg to face camera
          resolve();
        });
      },
      undefined,
      reject
    );
  });

const rigRotation = (name, rotation = { x: 0, y: 0, z: 0 }, dampener = 1, lerpAmount = 0.3) => {
  if (!currentVrm) {
    return;
  }
  const Part = currentVrm.humanoid.getBoneNode(THREE.VRMSchema.HumanoidBoneName[name]);
  if (!Part) {
    return;
  }
  let euler = new THREE.Euler(rotation.x * dampener, rotation.y * dampener, rotation.z * dampener);
  let quaternion = new THREE.Quaternion().setFromEuler(euler);
  Part.quaternion.slerp(quaternion, lerpAmount);
};

const rigPosition = (name, position = { x: 0, y: 0, z: 0 }, dampener = 1, lerpAmount = 0.3) => {
  if (!currentVrm) {
    return;
  }
  const Part = currentVrm.humanoid.getBoneNode(THREE.VRMSchema.HumanoidBoneName[name]);
  if (!Part) {
    return;
  }
  let vector = new THREE.Vector3(position.x * dampener, position.y * dampener, position.z * dampener);
  Part.position.lerp(vector, lerpAmount);
};

let oldLookTarget = new THREE.Euler();
const rigFace = (riggedFace) => {
  if (!currentVrm) {
    return;
  }
  rigRotation("Neck", riggedFace.head, 0.7);

  const Blendshape = currentVrm.blendShapeProxy;
  const PresetName = THREE.VRMSchema.BlendShapePresetName;

  riggedFace.eye.l = lerp(clamp(1 - riggedFace.eye.l, 0, 1), Blendshape.getValue(PresetName.Blink), 0.5);
  riggedFace.eye.r = lerp(clamp(1 - riggedFace.eye.r, 0, 1), Blendshape.getValue(PresetName.Blink), 0.5);
  riggedFace.eye = Kalidokit.Face.stabilizeBlink(riggedFace.eye, riggedFace.head.y);
  Blendshape.setValue(PresetName.Blink, riggedFace.eye.l);

  Blendshape.setValue(PresetName.I, lerp(riggedFace.mouth.shape.I, Blendshape.getValue(PresetName.I), 0.5));
  Blendshape.setValue(PresetName.A, lerp(riggedFace.mouth.shape.A, Blendshape.getValue(PresetName.A), 0.5));
  Blendshape.setValue(PresetName.E, lerp(riggedFace.mouth.shape.E, Blendshape.getValue(PresetName.E), 0.5));
  Blendshape.setValue(PresetName.O, lerp(riggedFace.mouth.shape.O, Blendshape.getValue(PresetName.O), 0.5));
  Blendshape.setValue(PresetName.U, lerp(riggedFace.mouth.shape.U, Blendshape.getValue(PresetName.U), 0.5));

  let lookTarget = new THREE.Euler(
    lerp(oldLookTarget.x, riggedFace.pupil.y, 0.4),
    lerp(oldLookTarget.y, riggedFace.pupil.x, 0.4),
    0,
    "XYZ"
  );
  oldLookTarget.copy(lookTarget);
  currentVrm.lookAt.applyer.lookAt(lookTarget);
};

const FINGERS = ["Ring", "Index", "Middle", "Thumb", "Little"];
const SEGMENTS = ["Proximal", "Intermediate", "Distal"];

const animateVRM = (vrm, results, imageSize) => {
  if (!vrm) {
    return;
  }
  let riggedPose, riggedLeftHand, riggedRightHand, riggedFace;

  const faceLandmarks = results.faceLandmarks;
  const pose3DLandmarks = results.ea;
  const pose2DLandmarks = results.poseLandmarks;
  // Be careful, hand landmarks may be reversed
  const leftHandLandmarks = results.rightHandLandmarks;
  const rightHandLandmarks = results.leftHandLandmarks;

  if (faceLandmarks) {
    riggedFace = Kalidokit.Face.solve(faceLandmarks, { runtime: "mediapipe", imageSize: imageSize });
    rigFace(riggedFace);
  }

  if (pose2DLandmarks && pose3DLandmarks) {
    riggedPose = Kalidokit.Pose.solve(pose3DLandmarks, pose2DLandmarks, { runtime: "mediapipe", imageSize: imageSize });
    rigRotation("Hips", riggedPose.Hips.rotation, 0.7);
    rigPosition(
      "Hips",
      {
        x: -riggedPose.Hips.position.x,
        y: riggedPose.Hips.position.y + 1,
        z: -riggedPose.Hips.position.z,
      },
      1,
      0.07
    );

    rigRotation("Chest", riggedPose.Spine, 0.25, 0.3);
    rigRotation("Spine", riggedPose.Spine, 0.45, 0.3);

    rigRotation("RightUpperArm", riggedPose.RightUpperArm, 1, 0.3);
    rigRotation("RightLowerArm", riggedPose.RightLowerArm, 1, 0.3);
    rigRotation("LeftUpperArm", riggedPose.LeftUpperArm, 1, 0.3);
    rigRotation("LeftLowerArm", riggedPose.LeftLowerArm, 1, 0.3);

    rigRotation("LeftUpperLeg", riggedPose.LeftUpperLeg, 1, 0.3);
    rigRotation("LeftLowerLeg", riggedPose.LeftLowerLeg, 1, 0.3);
    rigRotation("RightUpperLeg", riggedPose.RightUpperLeg, 1, 0.3);
    rigRotation("RightLowerLeg", riggedPose.RightLowerLeg, 1, 0.3);
  }

  // The sample reads riggedPose here without a guard, so the hands are posed only with a pose
  if (leftHandLandmarks && riggedPose) {
    riggedLeftHand = Kalidokit.Hand.solve(leftHandLandmarks, "Left");
    rigRotation("LeftHand", {
      z: riggedPose.LeftHand.z,
      y: riggedLeftHand.LeftWrist.y,
      x: riggedLeftHand.LeftWrist.x,
    });
    for (const finger of FINGERS) {
      for (const segment of SEGMENTS) {
        rigRotation("Left" + finger + segment, riggedLeftHand["Left" + finger + segment]);
      }
    }
  }
  if (rightHandLandmarks && riggedPose) {
    riggedRightHand = Kalidokit.Hand.solve(rightHandLandmarks, "Right");
    rigRotation("RightHand", {
      z: riggedPose.RightHand.z,
      y: riggedRightHand.RightWrist.y,
      x: riggedRightHand.RightWrist.x,
    });
    for (const finger of FINGERS) {
      for (const segment of SEGMENTS) {
        rigRotation("Right" + finger + segment, riggedRightHand["Right" + finger + segment]);
      }
    }
  }
};

const holistic = new Holistic({
  locateFile: (file) => `https://cdn.jsdelivr.net/npm/@mediapipe/holistic@0.5.1635989137/${file}`,
});
holistic.setOptions({
  modelComplexity: 1,
  smoothLandmarks: true,
  minDetectionConfidence: 0.7,
  minTrackingConfidence: 0.7,
  refineFaceLandmarks: true,
});
let pendingResults = null;
holistic.onResults((results) => {
  if (pendingResults) {
    pendingResults(results);
  }
});

const loadImage = (url) =>
  new Promise((resolve, reject) => {
    const image = new Image();
    image.onload = () => resolve(image);
    image.onerror = reject;
    image.src = url;
  });

const run = async () => {
  await loadVrm();
  const frames = await (await fetch("/frames.json")).json();
  const start = parseInt(new URLSearchParams(location.search).get("start") || "0");
  window.renderProgress = { done: 0, total: frames.length, finished: false, error: null };
  for (let i = start; i < frames.length; i++) {
    const image = await loadImage(`/frame/${frames[i]}.jpg`);
    const results = await new Promise((resolve) => {
      pendingResults = resolve;
      holistic.send({ image: image });
    });
    animateVRM(currentVrm, results, { width: image.naturalWidth, height: image.naturalHeight });
    currentVrm.update(FRAME_SECONDS);
    followHips();
    renderer.render(scene, camera);
    const blob = await new Promise((resolve) => renderer.domElement.toBlob(resolve, "image/jpeg", 0.92));
    await fetch(`/save/${frames[i]}`, { method: "POST", body: blob });
    window.renderProgress.done = i + 1;
    if (i % 30 === 0) {
      statusElement.textContent = `${i + 1} of ${frames.length}`;
    }
  }
  window.renderProgress.finished = true;
  statusElement.textContent = `done, ${frames.length} frames`;
};
run().catch((error) => {
  window.renderProgress = Object.assign(window.renderProgress || {}, { error: String(error) });
  statusElement.textContent = "failed: " + error;
});
