// Run with: node --test
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { generatePose, checkPose, restPose, blendPoses, mulberry32 } from '../js/pose.js';

test('the rest pose satisfies every constraint', () => {
  assert.deepEqual(checkPose(restPose()), []);
});

test('generated poses respect range of motion, foot contact, balance and collisions', () => {
  const supports = new Set();
  for (let seed = 1; seed <= 150; seed++) {
    const pose = generatePose(mulberry32(seed));
    assert.ok(pose.q.some((v) => v !== 0), `seed ${seed} fell back to the rest pose`);
    assert.deepEqual(checkPose(pose), [], `seed ${seed}`);
    supports.add(pose.support);
  }
  // single-leg poses on either foot and two-footed poses all occur
  assert.deepEqual([...supports].sort(), ['L', 'R', 'both']);
});

test('poses are centered on the turntable axis', () => {
  const pose = generatePose(mulberry32(3));
  assert.ok(Math.hypot(pose.rootP[0], pose.rootP[2]) < 0.5);
});

test('blending keeps the body on the ground', () => {
  const a = generatePose(mulberry32(10)), b = generatePose(mulberry32(11));
  for (const t of [0.25, 0.5, 0.75]) {
    const problems = checkPose(blendPoses(a, b, t));
    assert.ok(!problems.includes('below the ground'), `t=${t}`);
  }
});
