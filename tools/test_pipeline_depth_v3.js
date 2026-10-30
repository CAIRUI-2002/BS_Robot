const fs = require('fs');
const path = require('path');
global.window = global;
require('../src/main/resources/static/js/pipeline_depth_v3.js');

function assert(cond, msg) {
    if (!cond) throw new Error('ASSERT FAILED: ' + msg);
}
function near(a,b,tol=1e-9){ return Math.abs(a-b) <= tol; }

const singleJson = JSON.parse(fs.readFileSync(path.join(__dirname,'../src/main/resources/static/config/pipeline_depth_v3_single_model.json'),'utf8'));
const q25Json = JSON.parse(fs.readFileSync(path.join(__dirname,'../src/main/resources/static/config/pipeline_depth_v3_q25_model.json'),'utf8'));
const models = global.PIPELINE_DEPTH_V3_MODELS;
assert(models.single.knotsCm[0] === singleJson.knots_cm[0] && models.single.knotsCm[1] === singleJson.knots_cm[1], 'single knots mismatch');
assert(near(models.single.c, singleJson.coefficients.value_at_first_knot_m), 'single c mismatch');
assert(near(models.single.s1, singleJson.coefficients.slope_shallow), 'single s1 mismatch');
assert(near(models.single.s2, singleJson.coefficients.slope_middle), 'single s2 mismatch');
assert(near(models.single.s3, singleJson.coefficients.slope_deep), 'single s3 mismatch');
assert(models.q25.knotsCm[0] === q25Json.knots_cm[0] && models.q25.knotsCm[1] === q25Json.knots_cm[1], 'q25 knots mismatch');
assert(near(models.q25.c, q25Json.coefficients.value_at_first_knot_m), 'q25 c mismatch');
assert(near(models.q25.s1, q25Json.coefficients.slope_shallow), 'q25 s1 mismatch');
assert(near(models.q25.s2, q25Json.coefficients.slope_middle), 'q25 s2 mismatch');
assert(near(models.q25.s3, q25Json.coefficients.slope_deep), 'q25 s3 mismatch');

const e1 = new global.PipelineDepthEstimatorV3();
const t = Date.now();
const path1 = [0,1,2,3,4].map((x,i) => ({x, y:0, userData:{timestampMs:t-4000+i*1000}}));
e1.addSignal(0, t);
const vals = [];
for (const rawDepthCm of [98,99,100,102,101]) {
    const obs = e1.ingestDepth({rawDepthCm, serverReceiveTimeMs:t, captureTimeMs:t, backendReceiveTimeMs:t}, path1);
    vals.push(obs.calibratedDepthM);
}
assert(vals.every(v => v > 0.60 && v < 0.66), '98-102cm calibration should be around 0.625m, not raw/100');

const e2 = new global.PipelineDepthEstimatorV3();
const path2 = [0,1,2,3,4].map((x,i) => ({x, y:0, userData:{timestampMs:t-4000+i*1000}}));
e2.addPose(3.0,0.2,t-2100); e2.addPose(3.4,0.2,t-1900);
e2.addSignal(0,t-2000);
const obs2 = e2.ingestDepth({rawDepthCm:100, serverReceiveTimeMs:t, captureTimeMs:t-2000, backendReceiveTimeMs:t},path2);
assert(obs2.mappingMode === 'pose_projection', 'expected pose projection mapping');
assert(obs2.targetIndex === 3, 'pose projection should bind to node 3');
assert(near(path2[3].userData.depthSigmaM, 0.25, 1e-9), 'first measurement must not be double-counted; sigma should remain 0.25m');

const e3 = new global.PipelineDepthEstimatorV3();
const path3 = [0,1,2,3,4,5].map((x,i) => ({x, y:0, userData:{timestampMs:t-5000+i*1000}}));
for (let i=0;i<6;i++) e3.addPose(i,0,t-5000+i*1000);
e3.addSignal(0,t-4000);
e3.ingestDepth({rawDepthCm:100, serverReceiveTimeMs:t-4000, captureTimeMs:t-4000, backendReceiveTimeMs:t-4000}, path3);
e3.addSignal(0,t-1000);
e3.ingestDepth({rawDepthCm:500, serverReceiveTimeMs:t-1000, captureTimeMs:t-1000, backendReceiveTimeMs:t-1000}, path3);
const onlineBefore = path3.map(p => p.userData.estimatedDepthM);
const finalSummary = e3.finalizePath(path3);
assert(finalSummary && finalSummary.finalized === true, 'RTS finalization should return finalized summary');
assert(path3.every(p => Number.isFinite(p.userData.finalDepthM)), 'all nodes should have finalDepthM after finalize');
assert(path3.every(p => Number.isFinite(p.userData.estimatedDepthM)), 'estimatedDepthM should remain finite after finalize');
assert(path3.some((p,i) => Number.isFinite(p.userData.onlineDepthM) && near(p.userData.onlineDepthM, onlineBefore[i], 1e-9)), 'online depth should be retained for audit');
assert(path3.every(p => p.userData.estimatedDepthM >= 0.05 && p.userData.estimatedDepthM <= 5.0), 'final depth must obey physical bounds');

console.log('PASS: V3 constants, calibration, pose projection, first-observation covariance and RTS finalization checks.');
