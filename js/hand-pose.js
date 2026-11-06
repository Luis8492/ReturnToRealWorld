// Forearms and hands on a desk: skeleton, kinematics, constraints and random
// pose generation. Pure math with no three.js dependency, so it can be tested
// in Node.
//
// Coordinates: y is up with the desk top at y = 0. The person sits at +z and
// faces -z, so their right hand side is +x. Units are meters and radians.
//
// Both arms are described as right arms; the left arm is the mirror image
// (x -> -x) of a right arm with the same parameters. Every bone points along
// its local -z, the palm side is local -y and the thumb side is local -x.
//
// Only what is in front of the elbows moves, and the elbows stay on the desk:
//   - the forearm swings inward about the elbow, from straight ahead (as when
//     standing at attention with the elbows bent) to across the body; never
//     outward
//   - the forearm turns the palm down, to the side or up
//   - the wrist bends the hand up and down (and a little sideways)
//   - the fingers and the thumb take various shapes
// A forearm only leaves the desk to lie across the other arm.
//
// A generated pose satisfies:
//   - every joint stays within its range of motion
//   - nothing goes into the desk and the two arms do not pass through each other
//   - a forearm that is off the desk rests on the other arm
//   - finger shapes are ones a hand can make: the ring finger moves with the
//     middle or the little finger, and the middle finger never stands alone

import { DEG, mul, apply, transpose, add, sub, scale, cross, clamp, lerp, rotX, rotY, rotZ, rotVec, segmentDistance } from './math.js';

/* ---------- skeleton ---------- */

export const ELBOW_X = 0.19; // elbows rest this far left and right of the middle
export const ELBOW_HEIGHT = 0.036; // forearm radius at the elbow
export const WRIST_HEIGHT = 0.028; // forearm radius at the wrist
export const FOREARM = 0.25;
const BASE_PITCH = -Math.asin((ELBOW_HEIGHT - WRIST_HEIGHT) / FOREARM); // lying flat on the desk

// prettier-ignore
export const FINGERS = [
  // knuckle in the hand frame, phalanx lengths, radius, spread (deg) [together, apart]
  { name: 'index',  knuckle: [-0.026, 0, -0.093], lengths: [0.046, 0.026, 0.021], radius: 0.0088, spread: [1.5, 11] },
  { name: 'middle', knuckle: [-0.008, 0, -0.097], lengths: [0.050, 0.030, 0.022], radius: 0.0092, spread: [0, 1] },
  { name: 'ring',   knuckle: [0.0095, 0, -0.092], lengths: [0.047, 0.029, 0.021], radius: 0.0087, spread: [-2, -9] },
  { name: 'pinky',  knuckle: [0.0255, 0, -0.082], lengths: [0.037, 0.022, 0.019], radius: 0.0077, spread: [-5, -20] },
];
export const THUMB = { base: [-0.021, -0.009, -0.024], lengths: [0.046, 0.032, 0.027], radius: [0.0128, 0.011, 0.0098] };
// Turns the thumb about its own axis so it bends across the palm.
const THUMB_TWIST = rotZ(60 * DEG);

// Parameters of one arm.
export const P = {
  yaw: 0, // inward swing about the elbow
  lift: 1, // forearm raised off the desk (only to lie across the other arm)
  roll: 2, // 0 palm down, 90 deg thumb up, 180 deg palm up
  wx: 3, // wrist: rotation vector in the forearm frame,
  wy: 4, //        x > 0 bends the hand back, y > 0 toward the thumb
  finger: (f, k) => 5 + 4 * f + k, // k: 0 spread, 1..3 bend of the three joints
  thumb: (k) => 21 + k, // k: 0 away from the index finger, 1 toward the palm side, 2..3 bend
};
export const ARM_PARAMS = 25;

// prettier-ignore
const RANGES = [
  [0, 95], [0, 35], [0, 180], null, null,
  ...FINGERS.flatMap(() => [[-25, 15], [-5, 92], [0, 105], [0, 85]]),
  [-25, 90], [-5, 75], [-10, 65], [-20, 85],
].map((r) => r && r.map((v) => v * DEG));
// Wrist limits: extension, flexion, toward the thumb, toward the little finger.
const WRIST = { back: 65 * DEG, palm: 75 * DEG, thumb: 22 * DEG, little: 35 * DEG };

// Bones of a right arm: parent, offset in the parent frame and local rotation.
const bone = (name, parent, offset, rot) => ({ name, parent, offset, rot });
export const BONES = [
  bone('forearm', -1, [ELBOW_X, ELBOW_HEIGHT, 0], (p) => mul(rotY(p[P.yaw]), mul(rotX(BASE_PITCH + p[P.lift]), rotZ(-p[P.roll])))),
  bone('hand', 0, [0, 0, -FOREARM], (p) => rotVec(p[P.wx], p[P.wy], 0)),
  bone('thumb1', 1, THUMB.base, (p) => mul(rotY(p[P.thumb(0)]), mul(rotX(-p[P.thumb(1)]), THUMB_TWIST))),
  bone('thumb2', 2, [0, 0, -THUMB.lengths[0]], (p) => rotX(-p[P.thumb(2)])),
  bone('thumb3', 3, [0, 0, -THUMB.lengths[1]], (p) => rotX(-p[P.thumb(3)])),
];
FINGERS.forEach((f, i) => {
  const b = BONES.length;
  BONES.push(
    bone(f.name + '1', 1, f.knuckle, (p) => mul(rotY(p[P.finger(i, 0)]), rotX(-p[P.finger(i, 1)]))),
    bone(f.name + '2', b, [0, 0, -f.lengths[0]], (p) => rotX(-p[P.finger(i, 2)])),
    bone(f.name + '3', b + 1, [0, 0, -f.lengths[1]], (p) => rotX(-p[P.finger(i, 3)])),
  );
});
const BI = Object.fromEntries(BONES.map((b, i) => [b.name, i]));

// Collision capsules: [bone, a, b, radius] in bone coordinates.
const seg = (bone, len, r) => [BI[bone], [0, 0, 0], [0, 0, -len], r];
// prettier-ignore
const CAPSULES = [
  [0, [0, 0, 0], [0, 0, -0.12], 0.033],
  [0, [0, 0, -0.12], [0, 0, -FOREARM], 0.028],
  [1, [-0.022, 0, -0.016], [-0.024, 0, -0.084], 0.014],
  [1, [0, 0, -0.016], [-0.002, 0, -0.088], 0.014],
  [1, [0.02, 0, -0.016], [0.022, 0, -0.076], 0.014],
  seg('thumb1', THUMB.lengths[0], THUMB.radius[0]),
  seg('thumb2', THUMB.lengths[1], THUMB.radius[1]),
  seg('thumb3', THUMB.lengths[2], THUMB.radius[2]),
  ...FINGERS.flatMap((f) => f.lengths.map((len, k) => seg(f.name + (k + 1), len, f.radius * (1 - 0.06 * k)))),
].map(([bone, a, b, r]) => ({ bone, a, b, r }));
const FOREARM_CAPSULES = CAPSULES.filter((c) => c.bone === 0);
// What has to be on screen: the hand and the forearm's last stretch, which shows where it points.
const VIEWED = [{ bone: 0, a: [0, 0, -0.15], b: [0, 0, -FOREARM], r: 0.028 }, ...CAPSULES.filter((c) => c.bone !== 0)];
const HAND_CAPSULES = CAPSULES.filter((c) => c.bone !== 0);

/* ---------- kinematics ---------- */

// World transforms of the bones of an arm with parameters p, seen as a right arm.
function armFK(p) {
  const R = new Array(BONES.length), Pos = new Array(BONES.length);
  for (let i = 0; i < BONES.length; i++) {
    const b = BONES[i], local = b.rot(p);
    if (b.parent < 0) {
      R[i] = local;
      Pos[i] = b.offset.slice();
    } else {
      R[i] = mul(R[b.parent], local);
      Pos[i] = add(Pos[b.parent], apply(R[b.parent], b.offset));
    }
  }
  return { R, P: Pos };
}

export function localRotation(i, p) {
  return BONES[i].rot(p);
}

const mirrorPoint = (v) => [-v[0], v[1], v[2]];

// World capsules (segment ends and radius) of an arm on the given side.
function capsulesOf(fk, side, list = CAPSULES) {
  const m = side === 'L' ? mirrorPoint : (v) => v;
  return list.map((c) => {
    const a = m(add(fk.P[c.bone], apply(fk.R[c.bone], c.a)));
    const b = m(add(fk.P[c.bone], apply(fk.R[c.bone], c.b)));
    return { a, b, r: c.r, mid: scale(add(a, b), 0.5), half: Math.hypot(...sub(a, b)) / 2 };
  });
}

const lowest = (caps) => caps.reduce((low, c) => Math.min(low, c.a[1] - c.r, c.b[1] - c.r), Infinity);

const DESK_TOLERANCE = 0.0015;
const TOUCH = 0.92; // capsules may sink into each other this much (soft surfaces)

function hits(caps, others) {
  if (!others) return false;
  for (const c of caps) {
    for (const o of others) {
      const reach = c.half + o.half + c.r + o.r;
      const dx = c.mid[0] - o.mid[0], dy = c.mid[1] - o.mid[1], dz = c.mid[2] - o.mid[2];
      if (dx * dx + dy * dy + dz * dz > reach * reach) continue;
      if (segmentDistance(c.a, c.b, o.a, o.b) < TOUCH * (c.r + o.r)) return true;
    }
  }
  return false;
}

function gap(caps, others) {
  let d = Infinity;
  for (const c of caps) for (const o of others) d = Math.min(d, segmentDistance(c.a, c.b, o.a, o.b) - (c.r + o.r));
  return d;
}

/* ---------- wrist ---------- */

function wristWithinLimits(p) {
  const x = p[P.wx], y = p[P.wy];
  const rx = x > 0 ? WRIST.back : WRIST.palm, ry = y > 0 ? WRIST.thumb : WRIST.little;
  return (x / rx) ** 2 + (y / ry) ** 2 <= 1 + 1e-6;
}

// Axes in the forearm frame that tilt the hand up (toward the sky) and sideways,
// whichever way the palm faces.
function wristAxes(p) {
  const up = apply(transpose(BONES[0].rot(p)), [0, 1, 0]);
  let a = cross([0, 0, -1], up);
  const n = Math.hypot(...a);
  a = n < 1e-6 ? [1, 0, 0] : scale(a, 1 / n);
  return { up: a, side: cross(a, [0, 0, -1]) };
}

function setWrist(p, axes, tilt, side) {
  p[P.wx] = tilt * axes.up[0] + side * axes.side[0];
  p[P.wy] = tilt * axes.up[1] + side * axes.side[1];
}

/* ---------- placing an arm on the desk ---------- */

const handBlocked = (p, side, other) => {
  const caps = capsulesOf(armFK(p), side, HAND_CAPSULES);
  return lowest(caps) < -DESK_TOLERANCE || hits(caps, other);
};

// Sets the wrist so the hand rests on whatever is below it (the desk or the
// other arm), or is held up `raise` above that, with the hand turned `sway`
// sideways. Returns false when the hand cannot be placed without running into
// something.
function placeHand(p, side, other, { mode, raise, sway }) {
  const axes = wristAxes(p);
  const inRange = (tilt) => (setWrist(p, axes, tilt, sway), wristWithinLimits(p));
  const free = (tilt) => inRange(tilt) && !handBlocked(p, side, other);
  const step = 3 * DEG;
  let hi = 80 * DEG;
  while (hi > -80 * DEG && !inRange(hi)) hi -= step;
  if (!free(hi)) return false;
  let lo = hi;
  while (lo - step > -80 * DEG && inRange(lo - step) && free(lo - step)) lo -= step;
  if (inRange(lo - step)) {
    // something below: find where the hand touches it
    let blocked = lo - step;
    for (let i = 0; i < 7; i++) {
      const mid = (lo + blocked) / 2;
      if (free(mid)) lo = mid;
      else blocked = mid;
    }
  }
  let tilt = lo;
  if (mode === 'up') {
    if (hi - lo < 15 * DEG) return false;
    tilt = Math.min(hi, lo + raise);
  }
  return free(tilt);
}

const LIFT_STEP = 1 * DEG;

// Puts an arm down: the forearm lies on the desk, or on the other arm if that
// is in the way, and the hand rests or is held up.
function placeArm(p, side, other, plan) {
  const forearmFree = () => !hits(capsulesOf(armFK(p), side, FOREARM_CAPSULES), other);
  const ok = (lift) => {
    p[P.lift] = lift;
    return forearmFree() && placeHand(p, side, other, plan);
  };
  for (let lift = 0; lift <= RANGES[P.lift][1]; lift += LIFT_STEP) {
    if (!ok(lift)) continue;
    if (lift > 0) {
      // lower it until it just rests on the other arm
      let lo = lift - LIFT_STEP, hi = lift;
      for (let i = 0; i < 5; i++) {
        const mid = (lo + hi) / 2;
        if (ok(mid)) hi = mid;
        else lo = mid;
      }
      ok(hi);
    }
    return true;
  }
  return false;
}

/* ---------- hand shapes ---------- */

// Bends of the three finger joints (deg) for each finger state.
const FINGER_STATES = {
  straight: [2, 3, 2],
  curled: [88, 100, 62],
  hook: [6, 96, 74],
  relaxed: [10, 16, 8],
};

const THUMB_STATES = {
  out: [78, 6, 0, -8],
  along: [16, 10, 8, 6],
  relaxed: [22, 14, 10, 10],
};

function setFingers(p, states, spread) {
  states.forEach((s, i) => {
    const bend = FINGER_STATES[s];
    const apart = spread && s === 'straight';
    p[P.finger(i, 0)] = FINGERS[i].spread[apart ? 1 : 0] * DEG;
    for (let k = 0; k < 3; k++) p[P.finger(i, k + 1)] = bend[k] * DEG;
  });
}

function setThumb(p, values) {
  values.forEach((v, k) => (p[P.thumb(k)] = v * DEG));
}

// Hand-frame FK (only the hand and its children matter here).
function handFK(p) {
  const fk = armFK(p);
  const inv = transpose(fk.R[1]);
  const to = (v) => apply(inv, sub(v, fk.P[1]));
  return { fk, to, R: (i) => mul(inv, fk.R[i]) };
}

// Tip of a finger or the thumb (end of its last bone) in the hand frame.
function tip(h, name) {
  const i = BI[name];
  const len = name.startsWith('thumb') ? THUMB.lengths[2] : FINGERS.find((f) => name.startsWith(f.name)).lengths[2];
  return h.to(add(h.fk.P[i], apply(h.fk.R[i], [0, 0, -len])));
}

// Damped least squares on a few parameters, keeping them in range.
function solve(p, active, residual, tol, iterations = 40) {
  const lambda2 = 0.01 ** 2, eps = 1e-5;
  let e = residual(p);
  for (let it = 0; it < iterations; it++) {
    if (Math.hypot(...e) < tol) return true;
    const m = e.length, n = active.length;
    const J = active.map((k) => {
      const old = p[k];
      p[k] = old + eps;
      const e2 = residual(p);
      p[k] = old;
      return e2.map((v, i) => (v - e[i]) / eps);
    });
    // (J J^T + lambda^2 I) y = e, step = -J^T y   (J stored by column)
    const A = Array.from({ length: m }, (_, i) => Array.from({ length: m }, (_, k) => (i === k ? lambda2 : 0) + J.reduce((s, col) => s + col[i] * col[k], 0)));
    const y = solveSmall(A, e.slice());
    const step = J.map((col) => -col.reduce((s, v, i) => s + v * y[i], 0));
    const big = Math.max(...step.map(Math.abs));
    const f = big > 0.2 ? 0.2 / big : 1;
    active.forEach((k, j) => (p[k] = clamp(p[k] + step[j] * f, RANGES[k][0], RANGES[k][1])));
    e = residual(p);
  }
  return Math.hypot(...e) < tol;
}

function solveSmall(A, b) {
  const n = b.length;
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[piv][c])) piv = r;
    [A[c], A[piv]] = [A[piv], A[c]];
    [b[c], b[piv]] = [b[piv], b[c]];
    for (let r = c + 1; r < n; r++) {
      const f = A[r][c] / A[c][c];
      for (let k = c; k < n; k++) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  const x = new Array(n);
  for (let r = n - 1; r >= 0; r--) {
    let s = b[r];
    for (let k = r + 1; k < n; k++) s -= A[r][k] * x[k];
    x[r] = s / A[r][r];
  }
  return x;
}

const THUMB_PARAMS = [0, 1, 2, 3].map(P.thumb);

// Thumb laid over the bent fingers (as in a fist) or folded onto the palm.
function tuckThumb(p, states) {
  setThumb(p, [20, 45, 20, 20]);
  // over the bent index / middle fingers if there are any, or else the ring / little fingers
  const bentOf = (list) => list.filter((i) => states[i] !== 'straight');
  const bent = bentOf([0, 1]).length ? bentOf([0, 1]) : bentOf([2, 3]);
  let target;
  if (bent.length) {
    // over the middle bones of those fingers
    const h = handFK(p);
    const points = bent.map((i) => {
      const b = BI[FINGERS[i].name + '2'];
      const R = h.R(b);
      const center = h.to(add(h.fk.P[b], apply(h.fk.R[b], [0, 0, -FINGERS[i].lengths[1] / 2])));
      return add(center, scale([R[1], R[4], R[7]], FINGERS[i].radius + THUMB.radius[2]));
    });
    target = scale(points.reduce(add), 1 / points.length);
  } else {
    // onto the palm below the ring finger
    target = [0.008, -0.026, -0.07];
  }
  return solve(p, THUMB_PARAMS, (v) => sub(tip(handFK(v), 'thumb3'), target), 0.004);
}

// Thumb tip touching the tip of one or two fingers, which bend to meet it.
function pinch(p, fingers) {
  setThumb(p, [35, 40, 15, 15]);
  const curl = [0.6, 0.85, 0.6].map((k) => k * 90 * DEG);
  // The bend rides on the first finger's base joint and is shared by the others.
  const lead = P.finger(fingers[0], 1);
  const setCurl = (v, c) => {
    c = clamp(c, 0.15, 1.2);
    for (const i of fingers) curl.forEach((a, j) => (v[P.finger(i, j + 1)] = c * a));
  };
  setCurl(p, 0.6);
  const residual = (v) => {
    setCurl(v, v[lead] / curl[0]);
    const h = handFK(v);
    const tips = fingers.map((i) => tip(h, FINGERS[i].name + '3'));
    return sub(tip(h, 'thumb3'), scale(tips.reduce(add), 1 / tips.length));
  };
  const ok = solve(p, [...THUMB_PARAMS, lead], residual, 0.004, 60);
  residual(p);
  return ok;
}

// Finger states; `fixed` holds fingers already decided (pinching the thumb).
// Gives the hand of arm parameters p a shape:
//   fingers   state of each finger: 'straight', 'curled', 'hook' or 'relaxed'
//   thumb     'out', 'along', 'tucked' (over the bent fingers or onto the palm),
//             'pinch' (tip to tip with the `pinching` fingers) or 'relaxed'
//   spread    straight fingers apart instead of together
// Returns false if the thumb cannot get where it should.
export function shapeHand(p, { fingers, thumb, spread = false, pinching = [] }) {
  setFingers(p, fingers, spread);
  if (thumb === 'pinch') return pinch(p, pinching);
  if (thumb === 'tucked') return tuckThumb(p, fingers);
  setThumb(p, THUMB_STATES[thumb]);
  return true;
}

function sampleFingers(rng, wild, fixed = {}) {
  for (;;) {
    const states = FINGERS.map((_, i) => {
      if (fixed[i]) return fixed[i];
      const r = rng();
      if (wild && r < 0.18) return 'hook';
      return r < 0.58 ? 'straight' : 'curled';
    });
    // The ring finger cannot move on its own: it goes with the middle or the little finger.
    if (states[2] !== states[1] && states[2] !== states[3]) continue;
    // No middle finger standing up alone.
    if (states[1] === 'straight' && states[0] !== 'straight' && states[2] !== 'straight') continue;
    // Bending the index or middle finger alone, or the little finger next to a
    // straight ring finger, takes some effort: less often unless far out.
    const up = states.map((s) => s === 'straight');
    const effort = (!up[0] && up[1]) + (!up[1] && up[0] && up[2]) + (!up[3] && up[2]);
    if (!wild && rng() > 0.45 ** effort) continue;
    return states;
  }
}

/* ---------- poses ---------- */

const pick = (rng, table) => {
  let r = rng() * table.reduce((s, [, w]) => s + w, 0);
  for (const [v, w] of table) if ((r -= w) < 0) return v;
  return table[table.length - 1][0];
};

// Plan for one arm: everything except where it ends up, which placeArm decides.
function samplePlan(rng, wild, crossing) {
  const p = new Array(ARM_PARAMS).fill(0);
  const k = (moderate, far) => (wild ? far : moderate);

  if (crossing) p[P.yaw] = lerp(52, k(80, 92), rng()) * DEG;
  else p[P.yaw] = k(58, 85) * rng() ** k(1, 0.8) * DEG;

  if (wild) p[P.roll] = rng() * 180 * DEG;
  else p[P.roll] = clamp(pick(rng, [[0, 0.5], [90, 0.3], [180, 0.2]]) + lerp(-12, 12, rng()), 0, 180) * DEG;

  const thumb = pick(rng, [['out', 0.35], ['along', 0.3], ['tucked', 0.25], ['pinch', k(0.12, 0.3)]]);
  const pinching = thumb === 'pinch' ? pick(rng, [[[0], 0.5], [[1], 0.2], [[1, 2], 0.3]]) : [];
  const fingers = sampleFingers(rng, wild, Object.fromEntries(pinching.map((i) => [i, 'curled'])));
  const straight = fingers.filter((s) => s === 'straight').length;
  if (!shapeHand(p, { fingers, thumb, spread: straight >= 2 && rng() < 0.5, pinching })) return null;

  const mode = rng() < k(0.35, 0.5) ? 'up' : 'down';
  return {
    p,
    mode,
    raise: lerp(20, k(45, 70), rng()) * DEG,
    sway: lerp(-1, 1, rng()) * k(8, 24) * DEG,
  };
}

// One try at a pose, or null.
function attempt(rng, wild, crossing) {
  const first = rng() < 0.5 ? 'L' : 'R'; // placed first; lies underneath if they cross
  const second = first === 'L' ? 'R' : 'L';
  const plans = { [first]: samplePlan(rng, wild, crossing), [second]: samplePlan(rng, wild, crossing) };
  if (!plans.L || !plans.R) return null;

  if (!placeArm(plans[first].p, first, null, plans[first])) return null;
  const under = capsulesOf(armFK(plans[first].p), first);
  if (!placeArm(plans[second].p, second, under, plans[second])) return null;
  const pose = { L: plans.L.p, R: plans.R.p };
  // Crossed arms lie on each other; otherwise both stay on the desk.
  const lifted = Math.max(pose.L[P.lift], pose.R[P.lift]) > 3 * DEG;
  if (lifted !== crossing) return null;
  if (checkPose(pose).length) return null;
  if (hidden(pose, 'L') > 0.25 || hidden(pose, 'R') > 0.25) return null;
  return pose;
}

// How far the pose is from resting both hands flat: forearms swung in, palms
// turned, hands held up or bent, fingers in mixed shapes, arms crossed, and the
// two hands doing different things.
function weirdness(pose) {
  let score = 0;
  for (const s of ['L', 'R']) {
    const p = pose[s];
    score += 0.4 * (p[P.yaw] / (90 * DEG)) + 0.35 * Math.sin(p[P.roll] / 2) ** 2;
    score += 0.5 * Math.min(1, Math.hypot(p[P.wx], p[P.wy]) / (50 * DEG));
    const bends = FINGERS.map((_, i) => p[P.finger(i, 2)] / (100 * DEG));
    for (let i = 1; i < 4; i++) score += 0.04 * Math.abs(bends[i] - bends[i - 1]);
    score += 0.1 * Math.abs(p[P.thumb(0)] - THUMB_STATES.along[0] * DEG);
  }
  score += 0.5 * Math.min(1, (pose.L[P.lift] + pose.R[P.lift]) / (8 * DEG));
  score += 0.2 * Math.abs(Math.cos(pose.L[P.roll]) - Math.cos(pose.R[P.roll]));
  for (let i = 0; i < 4; i++) score += 0.06 * Math.abs(pose.L[P.finger(i, 2)] - pose.R[P.finger(i, 2)]);
  return score;
}

// Picks the strangest of a few valid candidates. Most poses are moderately
// strange; about one in five is far out.
export function generatePose(rng = Math.random) {
  const wild = rng() < 0.2;
  const crossing = rng() < (wild ? 0.4 : 0.15);
  const candidates = wild ? 6 : 3;
  let best = null, bestScore = -Infinity;
  for (let i = 0, found = 0; i < 400 && found < candidates; i++) {
    const pose = attempt(rng, wild, crossing);
    if (!pose) continue;
    found++;
    const score = weirdness(pose);
    if (score > bestScore) [best, bestScore] = [pose, score];
  }
  return best || restPose();
}

let REST = null;
// Both hands resting flat, a little turned in, fingers relaxed.
export function restPose() {
  if (!REST) {
    const make = (side, other) => {
      const p = new Array(ARM_PARAMS).fill(0);
      p[P.yaw] = 10 * DEG;
      setFingers(p, ['relaxed', 'relaxed', 'relaxed', 'relaxed'], false);
      setThumb(p, THUMB_STATES.relaxed);
      placeArm(p, side, other, { mode: 'down', raise: 0, sway: 0 });
      return p;
    };
    const R = make('R', null);
    REST = { L: make('L', capsulesOf(armFK(R), 'R')), R, rest: true };
  }
  return { L: REST.L.slice(), R: REST.R.slice(), rest: true };
}

/* ---------- what you see ---------- */

// Your eyes, where the camera looks from (see hands.js).
export const EYE = [0, 0.46, 0.12];

// Points along the bones of a hand, to see how much of it is in view.
const HAND_SAMPLES = HAND_CAPSULES.flatMap((c) => [0.2, 0.8].map((t) => ({ bone: c.bone, at: c.a.map((v, i) => lerp(v, c.b[i], t)) })));

// Share of a hand hidden behind the other arm, seen from EYE.
function hidden(pose, side) {
  const other = side === 'L' ? 'R' : 'L';
  const fk = armFK(pose[side]);
  const m = side === 'L' ? mirrorPoint : (v) => v;
  const blockers = capsulesOf(armFK(pose[other]), other);
  let count = 0;
  for (const s of HAND_SAMPLES) {
    const p = m(add(fk.P[s.bone], apply(fk.R[s.bone], s.at)));
    if (blockers.some((c) => segmentDistance(p, EYE, c.a, c.b) < c.r)) count++;
  }
  return count / HAND_SAMPLES.length;
}

/* ---------- validation ---------- */

// Returns a list of violated constraints (empty when the pose is valid).
export function checkPose(pose) {
  const problems = [];
  const caps = {};
  for (const s of ['L', 'R']) {
    const p = pose[s];
    RANGES.forEach((r, k) => {
      if (r && (p[k] < r[0] - 1e-6 || p[k] > r[1] + 1e-6)) problems.push(`range of motion: ${s} ${k}`);
    });
    if (!wristWithinLimits(p)) problems.push(`range of motion: ${s} wrist`);
    caps[s] = capsulesOf(armFK(p), s);
    if (lowest(caps[s]) < -DESK_TOLERANCE) problems.push(`into the desk: ${s}`);
  }
  if (hits(caps.L, caps.R)) problems.push('arms pass through each other');
  // An arm off the desk rests on the other arm or on its own hand.
  for (const s of ['L', 'R']) {
    const o = s === 'L' ? 'R' : 'L';
    const onHand = lowest(caps[s].slice(FOREARM_CAPSULES.length)) < 0.004;
    if (pose[s][P.lift] > 0.5 * DEG && !onHand && gap(caps[s], caps[o]) > 0.01) problems.push(`forearm in the air: ${s}`);
  }
  return problems;
}

/* ---------- helpers for display ---------- */

// Raises a hand that dips into the desk or the other arm, and lifts the arm
// on top until the two are apart. Keeps the in-between poses of a blend clean.
function settle(pose, top) {
  const under = top === 'L' ? 'R' : 'L';
  for (const s of [under, top]) {
    const p = pose[s];
    const other = s === top ? capsulesOf(armFK(pose[under]), under) : null;
    const axes = wristAxes(p);
    let raised = 0;
    while (raised < 40 && handBlocked(p, s, other)) {
      p[P.wx] += axes.up[0] * DEG;
      p[P.wy] += axes.up[1] * DEG;
      raised++;
    }
    if (s === top) {
      let lifted = 0;
      while (lifted < 30 && hits(capsulesOf(armFK(p), s), other)) {
        p[P.lift] += DEG;
        lifted++;
      }
    }
  }
  return pose;
}

const ease = (t) => t * t * (3 - 2 * t);

function blendDirect(a, b, t) {
  const pose = {};
  for (const s of ['L', 'R']) {
    pose[s] = a[s].map((v, k) => lerp(v, b[s][k], t));
    // lift early and put down late, so the arm on top clears the other one
    pose[s][P.lift] = Math.max(a[s][P.lift] * (1 - ease(clamp((t - 0.6) / 0.4, 0, 1))), b[s][P.lift] * ease(clamp(t / 0.4, 0, 1)));
  }
  const lift = (s) => Math.max(a[s][P.lift], b[s][P.lift]);
  return settle(pose, lift('L') > lift('R') ? 'L' : 'R');
}

// In-between pose. Going from one pose to another passes through the resting
// pose, so the arms never have to slide through each other.
export function blendPoses(a, b, t) {
  if (t <= 0) return a;
  if (t >= 1) return b;
  if (a.rest || b.rest) return blendDirect(a, b, t);
  const rest = restPose();
  return t < 0.5 ? blendDirect(a, rest, ease(t * 2)) : blendDirect(rest, b, ease(t * 2 - 1));
}

// Points that have to be on screen: along the hands and the front of the
// forearms (which shows where they point), each with its radius.
export function viewPoints(pose) {
  const points = [];
  for (const s of ['L', 'R']) {
    for (const c of capsulesOf(armFK(pose[s]), s, VIEWED)) points.push({ at: c.a, r: c.r }, { at: c.b, r: c.r });
  }
  return points;
}
