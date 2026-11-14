// A Kalidokit build over recorded Holistic landmarks, as its 3d sample calls it:
//   node solve_kalidokit.js <kalidokit.umd.js> <holistic.jsonl> <rig.jsonl>
const fs = require("fs");
const path = require("path");
const [kalidokitPath, inPath, outPath] = process.argv.slice(2);
const Kalidokit = require(path.resolve(kalidokitPath));
const toObj = (p) => ({ x: p[0], y: p[1], z: p[2], visibility: p[3] });
const out = fs.createWriteStream(outPath);
let n = 0, solved = 0;
for (const line of fs.readFileSync(inPath, "utf8").split("\n")) {
    if (!line.trim()) continue;
    const f = JSON.parse(line);
    n++;
    let rig = null;
    if (f.pose && f.world) {
        const r = Kalidokit.Pose.solve(f.world.map(toObj), f.pose.map(toObj), { runtime: "mediapipe", imageSize: { width: f.w, height: f.h } });
        if (r) {
            solved++;
            rig = { Hips: r.Hips.rotation, Spine: r.Spine, RightUpperArm: r.RightUpperArm, RightLowerArm: r.RightLowerArm, LeftUpperArm: r.LeftUpperArm, LeftLowerArm: r.LeftLowerArm };
        }
    }
    out.write(JSON.stringify({ frameIndex: f.frameIndex, rig }) + "\n");
}
out.end(() => console.log("frames", n, "solved", solved));
