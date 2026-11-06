// Run with: node --test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { generatePose, checkPose, restPose, blendPoses, shapeHand, P } from '../js/hand-pose.js';
import { mulberry32 } from '../js/math.js';

const DEG = Math.PI / 180;

test('the resting hands satisfy every constraint', () => {
  assert.deepEqual(checkPose(restPose()), []);
});

test('generated poses respect range of motion, the desk and the other arm', () => {
  let crossed = 0;
  for (let seed = 1; seed <= 120; seed++) {
    const pose = generatePose(mulberry32(seed));
    assert.ok(!pose.rest, `seed ${seed} fell back to the resting pose`);
    assert.deepEqual(checkPose(pose), [], `seed ${seed}`);
    for (const s of ['L', 'R']) {
      // the forearm swings inward or points straight ahead, never outward
      assert.ok(pose[s][P.yaw] >= 0, `seed ${seed} ${s} yaw`);
    }
    if (Math.max(pose.L[P.lift], pose.R[P.lift]) > 3 * DEG) crossed++;
  }
  // arms sometimes cross, but mostly lie side by side
  assert.ok(crossed > 5 && crossed < 50, `crossed ${crossed}`);
});

test('finger shapes stay doable', () => {
  for (let seed = 1; seed <= 120; seed++) {
    const pose = generatePose(mulberry32(seed));
    for (const s of ['L', 'R']) {
      const straight = [0, 1, 2, 3].map((i) => pose[s][P.finger(i, 2)] < 20 * DEG);
      // no middle finger standing up alone
      assert.ok(!(straight[1] && !straight[0] && !straight[2]), `seed ${seed} ${s}`);
    }
  }
});

test('the thumb reaches the fingertips for a pinch', () => {
  for (const pinching of [[0], [1], [1, 2]]) {
    const p = restPose().R;
    const fingers = ['straight', 'straight', 'straight', 'straight'].map((f, i) => (pinching.includes(i) ? 'curled' : f));
    assert.ok(shapeHand(p, { fingers, thumb: 'pinch', pinching }), `pinch ${pinching}`);
  }
});

test('blending keeps the hands out of the desk', () => {
  const rng = mulberry32(7);
  const a = generatePose(rng), b = generatePose(rng);
  for (const [x, y] of [[restPose(), a], [a, b], [b, restPose()]]) {
    for (const t of [0.15, 0.3, 0.5, 0.7, 0.85]) {
      const problems = checkPose(blendPoses(x, y, t));
      assert.ok(!problems.some((p) => p.startsWith('into the desk')), `t=${t}: ${problems}`);
    }
  }
});
