// Skeleton, forward kinematics and constrained random pose generation.
// Pure math with no three.js dependency, so it can be tested in Node.
//
// Coordinates: y is up, the figure faces +z and its left hand side is +x.
// Units are meters and radians.
//
// A generated pose satisfies:
//   - every joint (limbs, neck, back) stays within a human range of motion
//   - at least one foot is flat on the ground
//   - the center of mass is above the supporting foot, or between both feet
//   - nothing goes below the ground and limbs do not pass through the body

const DEG = Math.PI / 180;

/* ---------- small vector / 3x3 matrix helpers (row-major) ---------- */

const I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];

function mul(a, b) {
  return [
    a[0] * b[0] + a[1] * b[3] + a[2] * b[6],
    a[0] * b[1] + a[1] * b[4] + a[2] * b[7],
    a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
    a[3] * b[0] + a[4] * b[3] + a[5] * b[6],
    a[3] * b[1] + a[4] * b[4] + a[5] * b[7],
    a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
    a[6] * b[0] + a[7] * b[3] + a[8] * b[6],
    a[6] * b[1] + a[7] * b[4] + a[8] * b[7],
    a[6] * b[2] + a[7] * b[5] + a[8] * b[8],
  ];
}

function apply(m, v) {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ];
}

function transpose(m) {
  return [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
}

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);

function rotX(t) {
  const c = Math.cos(t), s = Math.sin(t);
  return [1, 0, 0, 0, c, -s, 0, s, c];
}

function rotY(t) {
  const c = Math.cos(t), s = Math.sin(t);
  return [c, 0, s, 0, 1, 0, -s, 0, c];
}

function rotZ(t) {
  const c = Math.cos(t), s = Math.sin(t);
  return [c, -s, 0, s, c, 0, 0, 0, 1];
}

const AXIS_ROT = { x: rotX, y: rotY, z: rotZ };

// Rotation by angle |(x, y, z)| about the axis (x, y, z).
function rotVec(x, y, z) {
  const t = Math.hypot(x, y, z);
  if (t < 1e-12) return I3.slice();
  const kx = x / t, ky = y / t, kz = z / t;
  const c = Math.cos(t), s = Math.sin(t), C = 1 - c;
  return [
    c + kx * kx * C, kx * ky * C - kz * s, kx * kz * C + ky * s,
    ky * kx * C + kz * s, c + ky * ky * C, ky * kz * C - kx * s,
    kz * kx * C - ky * s, kz * ky * C + kx * s, c + kz * kz * C,
  ];
}

/* ---------- quaternions ([x, y, z, w]) ---------- */

export function matToQuat(m) {
  const tr = m[0] + m[4] + m[8];
  let x, y, z, w;
  if (tr > 0) {
    const s = 0.5 / Math.sqrt(tr + 1);
    w = 0.25 / s; x = (m[7] - m[5]) * s; y = (m[2] - m[6]) * s; z = (m[3] - m[1]) * s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const s = 2 * Math.sqrt(1 + m[0] - m[4] - m[8]);
    w = (m[7] - m[5]) / s; x = 0.25 * s; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const s = 2 * Math.sqrt(1 + m[4] - m[0] - m[8]);
    w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = 0.25 * s; z = (m[5] + m[7]) / s;
  } else {
    const s = 2 * Math.sqrt(1 + m[8] - m[0] - m[4]);
    w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = 0.25 * s;
  }
  return [x, y, z, w];
}

function quatToMat([x, y, z, w]) {
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}

function slerp(a, b, t) {
  let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (d < 0) { b = b.map((v) => -v); d = -d; }
  if (d > 0.9995) {
    const r = a.map((v, i) => v + (b[i] - v) * t);
    const n = Math.hypot(...r);
    return r.map((v) => v / n);
  }
  const th = Math.acos(d), s = Math.sin(th);
  const wa = Math.sin((1 - t) * th) / s, wb = Math.sin(t * th) / s;
  return a.map((v, i) => v * wa + b[i] * wb);
}

/* ---------- skeleton ---------- */

// Ball joint: a swing (rotation vector in the parent's x/z plane) followed by a
// twist about the bone axis. The swing limits form a super-ellipse with
// exponent p, so combined motions are limited more than single-axis ones.
//   x: rotation about the left-right axis (flexion / extension)
//   z: rotation about the front-back axis (abduction / adduction, side bend)
//   y: twist about the bone itself (internal / external rotation)
const ball = (x, z, y, p = 2) => ({ type: 'ball', x, z, y, p });
// Euler joint: R = R_axes[0](a0) * R_axes[1](a1) * ...
const euler = (axes, ...limits) => ({ type: 'euler', axes, limits });

// Limits are in degrees, written for the left side (right side is mirrored).
// Signs: for a bone hanging down (-y), negative x swings it forward;
// for a bone pointing up (+y), positive x bends it forward.
// prettier-ignore
const LEFT_SPEC = [
  // name       parent       offset              joint                                         mass    center of mass
  ['pelvis',    null,        [0, 0, 0],          null,                                         0.142,  [0, 0, 0]],
  ['spine1',    'pelvis',    [0, 0.08, 0],       ball([-25, 45], [-25, 25], [-12, 12]),        0.139,  [0, 0.09, 0]],
  ['spine2',    'spine1',    [0, 0.18, 0],       ball([-20, 30], [-20, 20], [-30, 30]),        0.216,  [0, 0.10, 0]],
  ['neck',      'spine2',    [0, 0.21, 0],       ball([-55, 45], [-35, 35], [-65, 65]),        0.081,  [0, 0.12, 0.01]],
  ['shoulderL', 'spine2',    [0.19, 0.18, 0],    ball([-170, 50], [-30, 170], [-70, 80]),      0.028,  [0, -0.126, 0]],
  ['elbowL',    'shoulderL', [0, -0.29, 0],      euler('xy', [-145, 0], [-80, 80]),            0.016,  [0, -0.112, 0]],
  ['wristL',    'elbowL',    [0, -0.26, 0],      euler('zx', [-70, 60], [-20, 30]),            0.006,  [0, -0.05, 0]],
  ['hipL',      'pelvis',    [0.09, -0.06, 0],   ball([-120, 20], [-25, 45], [-40, 45], 3),    0.100,  [0, -0.186, 0]],
  ['kneeL',     'hipL',      [0, -0.43, 0],      euler('x', [0, 140]),                         0.0465, [0, -0.182, 0]],
  ['ankleL',    'kneeL',     [0, -0.42, 0],      euler('xz', [-20, 45], [-30, 20]),            0.0145, [0, -0.04, 0.06]],
];

// Collision / ground capsules: [bone, a, b, radius] in bone-local coordinates.
// prettier-ignore
const LEFT_CAPSULES = [
  ['pelvis',    [0.06, -0.02, 0], [-0.06, -0.02, 0], 0.09],
  ['spine1',    [0, 0.04, 0], [0, 0.15, 0], 0.1],
  ['spine2',    [0, 0.04, 0], [0, 0.13, 0], 0.13],
  ['spine2',    [0.15, 0.17, 0], [-0.15, 0.17, 0], 0.05],
  ['neck',      [0, 0.13, 0.008], [0, 0.13, 0.008], 0.105],
  ['shoulderL', [0, 0, 0], [0, -0.29, 0], 0.042],
  ['elbowL',    [0, 0, 0], [0, -0.26, 0], 0.035],
  ['wristL',    [0, -0.02, 0], [0, -0.1, 0], 0.028],
  ['hipL',      [0, 0, 0], [0, -0.43, 0], 0.06],
  ['kneeL',     [0, 0, 0], [0, -0.42, 0], 0.045],
  ['ankleL',    [0, -0.04, -0.025], [0, -0.04, 0.145], 0.035],
];

export const ANKLE_HEIGHT = 0.075;
// Sole rectangle in the foot (ankle) frame.
export const SOLE = { halfWidth: 0.04, heel: -0.06, toe: 0.18 };

const mirrorName = (n) => n.replace(/L$/, 'R');
const mirrorRange = ([a, b]) => [-b, -a];

function mirrorJoint(j) {
  if (!j) return j;
  if (j.type === 'ball') return { ...j, z: mirrorRange(j.z), y: mirrorRange(j.y) };
  return { ...j, limits: j.limits.map((l, i) => (j.axes[i] === 'x' ? l : mirrorRange(l))) };
}

function buildSkeleton() {
  const specs = [];
  for (const s of LEFT_SPEC) specs.push(s);
  for (const [name, parent, off, joint, mass, com] of LEFT_SPEC) {
    if (!name.endsWith('L')) continue;
    specs.push([mirrorName(name), parent && mirrorName(parent), [-off[0], off[1], off[2]], mirrorJoint(joint), mass, [-com[0], com[1], com[2]]]);
  }
  const bones = [];
  const index = {};
  let nParams = 0;
  for (const [name, parent, offset, joint, mass, com] of specs) {
    const toRad = (r) => r.map((v) => v * DEG);
    let j = null;
    if (joint && joint.type === 'ball') j = { type: 'ball', x: toRad(joint.x), z: toRad(joint.z), y: toRad(joint.y), p: joint.p, n: 3 };
    if (joint && joint.type === 'euler') j = { type: 'euler', axes: joint.axes, limits: joint.limits.map(toRad), n: joint.axes.length };
    const bone = { name, parent: parent == null ? -1 : index[parent], offset, joint: j, mass, com, param: nParams };
    nParams += j ? j.n : 0;
    index[name] = bones.length;
    bones.push(bone);
  }
  return { bones, index, nParams };
}

const SKELETON = buildSkeleton();
export const BONES = SKELETON.bones;
const BI = SKELETON.index;
export const PARAM_COUNT = SKELETON.nParams;

const CAPSULES = [];
for (const [bone, a, b, r] of LEFT_CAPSULES) {
  CAPSULES.push({ bone: BI[bone], a, b, r });
  if (bone.endsWith('L')) CAPSULES.push({ bone: BI[mirrorName(bone)], a: [-a[0], a[1], a[2]], b: [-b[0], b[1], b[2]], r });
}

// Capsule pairs that are checked for interpenetration: everything except
// pieces of the same bone and directly connected bones.
const COLLISION_PAIRS = [];
for (let i = 0; i < CAPSULES.length; i++) {
  for (let k = i + 1; k < CAPSULES.length; k++) {
    const a = CAPSULES[i].bone, b = CAPSULES[k].bone;
    if (a === b || BONES[a].parent === b || BONES[b].parent === a) continue;
    COLLISION_PAIRS.push([i, k]);
  }
}

// Pairs within the upper body (from the lower back up). Balancing only moves
// the legs and the lower back joint, so these can be checked before balancing.
const inUpperBody = (bone) => bone >= 0 && (bone === BI.spine1 || inUpperBody(BONES[bone].parent));
const UPPER_BODY_PAIRS = COLLISION_PAIRS.filter(([i, k]) => inUpperBody(CAPSULES[i].bone) && inUpperBody(CAPSULES[k].bone));

/* ---------- joints ---------- */

export function jointRotation(bone, q) {
  const j = bone.joint;
  if (!j) return I3.slice();
  const o = bone.param;
  if (j.type === 'ball') return mul(rotVec(q[o], 0, q[o + 1]), rotY(q[o + 2]));
  let m = AXIS_ROT[j.axes[0]](q[o]);
  for (let i = 1; i < j.n; i++) m = mul(m, AXIS_ROT[j.axes[i]](q[o + i]));
  return m;
}

const range = (r, v) => (v < 0 ? -r[0] : r[1]);

function swingNorm(j, sx, sz) {
  return Math.abs(sx / range(j.x, sx)) ** j.p + Math.abs(sz / range(j.z, sz)) ** j.p;
}

function clampJoint(bone, q) {
  const j = bone.joint;
  if (!j) return;
  const o = bone.param;
  if (j.type === 'ball') {
    const f = swingNorm(j, q[o], q[o + 1]);
    if (f > 1) {
      const k = f ** (-1 / j.p);
      q[o] *= k;
      q[o + 1] *= k;
    }
    q[o + 2] = clamp(q[o + 2], j.y[0], j.y[1]);
  } else {
    for (let i = 0; i < j.n; i++) q[o + i] = clamp(q[o + i], j.limits[i][0], j.limits[i][1]);
  }
}

function withinLimits(bone, q) {
  const j = bone.joint;
  if (!j) return true;
  const o = bone.param, e = 1e-6;
  if (j.type === 'ball') return swingNorm(j, q[o], q[o + 1]) <= 1 + 1e-4 && q[o + 2] >= j.y[0] - e && q[o + 2] <= j.y[1] + e;
  return j.limits.every((l, i) => q[o + i] >= l[0] - e && q[o + i] <= l[1] + e);
}

// Random joint values inside the range of motion, shrunk by `s` (0..1).
// A `bias` below 1 pushes the values out toward the ends of the range.
function sampleJoint(bone, q, rng, s = 1, bias = 1) {
  const j = bone.joint, o = bone.param;
  const reach = () => s * rng() ** bias;
  // Measured from 0, or from the end nearest to 0 for one-sided ranges.
  const pick = ([lo, hi]) => {
    if (lo >= 0) return lo + (hi - lo) * reach();
    if (hi <= 0) return hi + (lo - hi) * reach();
    return rng() < -lo / (hi - lo) ? lo * reach() : hi * reach();
  };
  if (j.type === 'ball') {
    const phi = rng() * 2 * Math.PI, r = s * rng() ** (bias / 2);
    const c = r * Math.cos(phi), d = r * Math.sin(phi);
    q[o] = c * range(j.x, c);
    q[o + 1] = d * range(j.z, d);
    q[o + 2] = pick(j.y);
  } else {
    for (let i = 0; i < j.n; i++) q[o + i] = pick(j.limits[i]);
  }
}

const lerp = (a, b, t) => a + (b - a) * t;

/* ---------- kinematics ---------- */

// World transforms of every bone for joint values q and a pelvis transform.
function forwardKinematics(q, rootR = I3, rootP = [0, 0, 0]) {
  const R = new Array(BONES.length), P = new Array(BONES.length);
  for (let i = 0; i < BONES.length; i++) {
    const b = BONES[i];
    const local = jointRotation(b, q);
    if (b.parent < 0) {
      R[i] = mul(rootR, local);
      P[i] = rootP.slice();
    } else {
      P[i] = add(P[b.parent], apply(R[b.parent], b.offset));
      R[i] = mul(R[b.parent], local);
    }
  }
  return { R, P };
}

// Pelvis transform that puts the given foot flat on the ground at the origin.
function worldOnFoot(q, side) {
  const fk = forwardKinematics(q);
  const a = BI['ankle' + side];
  const rootR = transpose(fk.R[a]);
  const rootP = sub([0, ANKLE_HEIGHT, 0], apply(rootR, fk.P[a]));
  return { rootR, rootP, R: fk.R.map((m) => mul(rootR, m)), P: fk.P.map((p) => add(apply(rootR, p), rootP)) };
}

const toWorld = (w, bone, p) => add(w.P[bone], apply(w.R[bone], p));

function centerOfMass(w) {
  let c = [0, 0, 0], m = 0;
  for (let i = 0; i < BONES.length; i++) {
    c = add(c, scale(toWorld(w, i, BONES[i].com), BONES[i].mass));
    m += BONES[i].mass;
  }
  return scale(c, 1 / m);
}

function soleCorners(w, side) {
  const a = BI['ankle' + side], { halfWidth: x, heel, toe } = SOLE;
  return [[x, -ANKLE_HEIGHT, heel], [-x, -ANKLE_HEIGHT, heel], [-x, -ANKLE_HEIGHT, toe], [x, -ANKLE_HEIGHT, toe]].map((p) => toWorld(w, a, p));
}

function soleCenter(w, side) {
  return toWorld(w, BI['ankle' + side], [0, -ANKLE_HEIGHT, (SOLE.heel + SOLE.toe) / 2]);
}

function isFlat(w, side) {
  return soleCorners(w, side).every((p) => Math.abs(p[1]) < 0.006);
}

function capsuleWorld(w, c) {
  return [toWorld(w, c.bone, c.a), toWorld(w, c.bone, c.b)];
}

function lowestPoint(w) {
  let low = Infinity;
  for (const c of CAPSULES) {
    const [a, b] = capsuleWorld(w, c);
    low = Math.min(low, a[1] - c.r, b[1] - c.r);
  }
  return low;
}

// Distance between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection).
function segmentDistance(p1, q1, p2, q2) {
  const d1 = sub(q1, p1), d2 = sub(q2, p2), r = sub(p1, p2);
  const a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  let s = 0, t = 0;
  if (a < 1e-12 && e < 1e-12) {
    s = t = 0;
  } else if (a < 1e-12) {
    t = clamp(f / e, 0, 1);
  } else {
    const c = dot(d1, r);
    if (e < 1e-12) {
      s = clamp(-c / a, 0, 1);
    } else {
      const b = dot(d1, d2), den = a * e - b * b;
      s = den > 1e-12 ? clamp((b * f - c * e) / den, 0, 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = clamp(-c / a, 0, 1); }
      else if (t > 1) { t = 1; s = clamp((b - c) / a, 0, 1); }
    }
  }
  const c1 = add(p1, scale(d1, s)), c2 = add(p2, scale(d2, t));
  return Math.hypot(...sub(c1, c2));
}

function selfCollision(w, pairs = COLLISION_PAIRS) {
  const seg = CAPSULES.map((c) => capsuleWorld(w, c));
  for (const [i, k] of pairs) {
    const d = segmentDistance(seg[i][0], seg[i][1], seg[k][0], seg[k][1]);
    if (d < 0.9 * (CAPSULES[i].r + CAPSULES[k].r)) return `${BONES[CAPSULES[i].bone].name}/${BONES[CAPSULES[k].bone].name}`;
  }
  return null;
}

// Convex hull of points on the ground plane (x, z), counter-clockwise.
function hull2(points) {
  const p = points.map((v) => [v[0], v[2]]).sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [], upper = [];
  for (const v of p) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], v) <= 0) lower.pop();
    lower.push(v);
  }
  for (const v of p.slice().reverse()) {
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], v) <= 0) upper.pop();
    upper.push(v);
  }
  return lower.slice(0, -1).concat(upper.slice(0, -1));
}

// Signed distance from (x, z) to the edges of a convex polygon (positive inside).
function insideDistance(poly, x, z) {
  let d = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const ex = b[0] - a[0], ez = b[1] - a[1], len = Math.hypot(ex, ez);
    d = Math.min(d, (ex * (z - a[1]) - ez * (x - a[0])) / len);
  }
  return d;
}

/* ---------- inverse kinematics ---------- */

// Damped least squares on the `active` parameters, keeping joints in range.
// A parameter that runs into its limit sits out the next few iterations so
// the others can take over instead of the solver stalling against the limit.
function solve(q, active, weights, residual, tol, iterations = 60) {
  const n = active.length, eps = 1e-5, lambda2 = 0.02 ** 2;
  const resting = new Uint8Array(n);
  let e = residual(q);
  for (let it = 0; it < iterations; it++) {
    const err = Math.hypot(...e);
    if (err < tol) return true;
    if (tol > 0 && it >= 30 && err > 0.02) return false; // not going to make it
    const m = e.length;
    const J = new Float64Array(m * n);
    for (let j = 0; j < n; j++) {
      if (resting[j]) continue;
      const k = active[j], old = q[k];
      q[k] = old + eps;
      const e2 = residual(q);
      q[k] = old;
      for (let i = 0; i < m; i++) J[i * n + j] = ((e2[i] - e[i]) / eps) * weights[j];
    }
    // (J J^T + lambda^2 I) y = e
    const A = new Float64Array(m * m);
    for (let i = 0; i < m; i++) {
      for (let k = 0; k < m; k++) {
        let s = i === k ? lambda2 : 0;
        for (let j = 0; j < n; j++) s += J[i * n + j] * J[k * n + j];
        A[i * m + k] = s;
      }
    }
    const y = solveLinear(A, e.slice(), m);
    const step = new Float64Array(n);
    let big = 0;
    for (let j = 0; j < n; j++) {
      let s = 0;
      for (let i = 0; i < m; i++) s += J[i * n + j] * y[i];
      step[j] = -s * weights[j];
      big = Math.max(big, Math.abs(step[j]));
    }
    const k = big > 0.15 ? 0.15 / big : 1;
    const wanted = active.map((a, j) => q[a] + step[j] * k);
    wanted.forEach((v, j) => (q[active[j]] = v));
    for (const b of BONES) clampJoint(b, q);
    for (let j = 0; j < n; j++) {
      if (resting[j]) resting[j]--;
      else if (Math.abs(q[active[j]] - wanted[j]) > 1e-6) resting[j] = 4;
    }
    e = residual(q);
  }
  return Math.hypot(...e) < tol;
}

function solveLinear(A, b, n) {
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r * n + c]) > Math.abs(A[p * n + c])) p = r;
    if (p !== c) {
      for (let k = 0; k < n; k++) [A[c * n + k], A[p * n + k]] = [A[p * n + k], A[c * n + k]];
      [b[c], b[p]] = [b[p], b[c]];
    }
    for (let r = c + 1; r < n; r++) {
      const f = A[r * n + c] / A[c * n + c];
      for (let k = c; k < n; k++) A[r * n + k] -= f * A[c * n + k];
      b[r] -= f * b[c];
    }
  }
  const x = new Array(n);
  for (let r = n - 1; r >= 0; r--) {
    let s = b[r];
    for (let k = r + 1; k < n; k++) s -= A[r * n + k] * x[k];
    x[r] = s / A[r * n + r];
  }
  return x;
}

const paramsOf = (...names) => names.flatMap((n) => {
  const b = BONES[BI[n]];
  return Array.from({ length: b.joint.n }, (_, i) => b.param + i);
});

/* ---------- validation ---------- */

// Returns a list of violated constraints (empty when the pose is valid).
export function checkPose(pose) {
  const { q } = pose;
  const w = { rootR: pose.rootR, rootP: pose.rootP, ...forwardKinematics(q, pose.rootR, pose.rootP) };
  const problems = [];
  for (const b of BONES) if (!withinLimits(b, q)) problems.push(`range of motion: ${b.name}`);
  const flat = ['L', 'R'].filter((s) => isFlat(w, s));
  if (!flat.length) problems.push('no foot flat on the ground');
  else {
    const c = centerOfMass(w);
    const support = hull2(flat.flatMap((s) => soleCorners(w, s)));
    if (insideDistance(support, c[0], c[2]) < 0.005) problems.push('center of mass outside the support');
  }
  if (lowestPoint(w) < -0.006) problems.push('below the ground');
  const hit = selfCollision(w);
  if (hit) problems.push(`self collision: ${hit}`);
  if (w.P[0][1] < 0.45) problems.push('pelvis too low');
  return problems;
}

/* ---------- generation ---------- */

// One try at a pose standing on the left foot, the right foot or both feet.
// `wild` (0..1) goes from moderately strange to far out.
function attempt(rng, support, wild) {
  const q = new Float64Array(PARAM_COUNT);
  const both = support === 'both';
  const S = both ? (rng() < 0.5 ? 'L' : 'R') : support; // the foot the body is rooted on
  const F = S === 'L' ? 'R' : 'L';
  const set = (name, s, bias) => sampleJoint(BONES[BI[name]], q, rng, s, bias);
  const knee = (side, lo, hi, bias = 1) => { q[BONES[BI['knee' + side]].param] = lerp(lo, hi, rng() ** bias) * DEG; };
  const k = (moderate, far) => lerp(moderate, far, wild);

  set('spine1', k(0.85, 1), k(1, 0.7));
  set('spine2', k(0.9, 1), k(1, 0.7));
  set('neck', k(0.9, 1), k(1, 0.6));
  for (const s of ['L', 'R']) {
    set('shoulder' + s, 1, k(0.85, 0.6));
    set('elbow' + s, 1, k(1, 0.8));
    set('wrist' + s, k(0.9, 1), k(1, 0.6));
  }

  let active, weights, goals, residual;
  if (!both) {
    set('hip' + S, k(0.55, 0.75), k(1, 0.8));
    knee(S, 0, k(45, 55));
    set('hip' + F, 1, k(0.8, 0.5));
    knee(F, 0, k(130, 140), k(1, 0.7));
    set('ankle' + F, 1, k(0.9, 0.6));
    // Keep the center of mass over the middle part of the standing foot.
    const tx = lerp(-0.015, 0.015, rng()), tz = lerp(0.0, 0.09, rng());
    active = paramsOf('ankle' + S, 'knee' + S, 'hip' + S, 'spine1');
    weights = [1, 1, 0.4, 0.7, 0.7, 0.3, 0.4, 0.4, 0.2];
    residual = (v) => {
      const c = centerOfMass(worldOnFoot(v, S));
      return [c[0] - tx, c[2] - tz];
    };
  } else {
    for (const s of ['L', 'R']) {
      set('hip' + s, k(0.65, 0.8), k(0.9, 0.7));
      knee(s, 10, k(75, 90));
    }
    // Second foot flat on the ground, center of mass between the feet.
    const t = lerp(0.3, 0.7, rng());
    // Loose goals for a wide, lunging or squatting stance.
    const fx = (S === 'L' ? -1 : 1) * lerp(k(0.2, 0.25), k(0.7, 0.85), rng()), fz = k(0.45, 0.55) * lerp(-1, 1, rng());
    const hipsY = lerp(k(0.64, 0.55), k(0.95, 0.92), rng());
    const aF = BI['ankle' + F];
    active = paramsOf('ankle' + S, 'knee' + S, 'hip' + S, 'ankle' + F, 'knee' + F, 'hip' + F, 'spine1');
    weights = [1, 1, 0.6, 0.6, 0.6, 0.3, 1, 1, 0.6, 0.6, 0.6, 0.3, 0.3, 0.3, 0.1];
    const evaluate = (v) => {
      const w = worldOnFoot(v, S);
      const c = centerOfMass(w);
      const R = w.R[aF];
      const target = add(soleCenter(w, S), scale(sub(soleCenter(w, F), soleCenter(w, S)), t));
      return { w, hard: [w.P[aF][1] - ANKLE_HEIGHT, R[1], R[7], c[0] - target[0], c[2] - target[2]] };
    };
    residual = (v) => evaluate(v).hard;
    goals = (v) => {
      const { w, hard } = evaluate(v);
      return [...hard, 0.5 * (w.P[aF][0] - fx), 0.5 * (w.P[aF][2] - fz), 0.5 * (w.P[0][1] - hipsY)];
    };
  }

  if (selfCollision(forwardKinematics(q), UPPER_BODY_PAIRS)) return null;
  if (goals) solve(q, active, weights, goals, 0, 30);
  if (!solve(q, active, weights, residual, 1.5e-3)) return null;

  const { rootR, rootP } = worldOnFoot(q, S);
  const pose = { q: Array.from(q), rootR, rootP, support };
  if (both) {
    const w = forwardKinematics(q, rootR, rootP);
    const gap = Math.hypot(...sub(w.P[BI.ankleL], w.P[BI.ankleR]));
    if (gap < 0.2 || gap > 1.0) return null;
  }
  if (checkPose(pose).length) return null;
  return facingFront(pose);
}

// How far the pose is from standing still: joints near their limits, a
// leaning trunk, a tilted head, a raised foot, limbs flung out, low hips.
function weirdness(pose) {
  const { q } = pose;
  let sum = 0, n = 0;
  for (const b of BONES) {
    const j = b.joint, o = b.param;
    if (!j) continue;
    if (j.type === 'ball') {
      sum += swingNorm(j, q[o], q[o + 1]) ** (1 / j.p) + Math.abs(q[o + 2]) / range(j.y, q[o + 2]);
      n += 2;
    } else {
      for (let i = 0; i < j.n; i++) sum += Math.abs(q[o + i]) / Math.max(Math.abs(j.limits[i][0]), Math.abs(j.limits[i][1]));
      n += j.n;
    }
  }
  const w = forwardKinematics(q, pose.rootR, pose.rootP);
  const tilt = (bone) => Math.acos(clamp(w.R[bone][4], -1, 1));
  const footUp = Math.max(w.P[BI.ankleL][1], w.P[BI.ankleR][1]) - ANKLE_HEIGHT;
  return (
    sum / n +
    0.35 * Math.min(1, tilt(BI.spine2) / (60 * DEG)) +
    0.3 * Math.min(1.5, tilt(BI.neck) / (70 * DEG)) +
    0.5 * Math.min(1, footUp / 0.6) +
    0.5 * poseExtent(pose).radius +
    2 * Math.max(0, 0.95 - w.P[0][1])
  );
}

// Turn the pose so the chest faces +z, with the center of mass above the origin.
function facingFront(pose) {
  const w = forwardKinematics(pose.q, pose.rootR, pose.rootP);
  const c = centerOfMass(w);
  const chest = w.R[BI.spine2];
  const yaw = Math.atan2(chest[2], chest[8]);
  const turn = rotY(-yaw);
  return {
    ...pose,
    rootR: mul(turn, pose.rootR),
    rootP: apply(turn, sub(pose.rootP, [c[0], 0, c[2]])),
  };
}

export function restPose() {
  return facingFront({ q: new Array(PARAM_COUNT).fill(0), rootR: I3.slice(), rootP: [0, 0.06 + 0.43 + 0.42 + ANKLE_HEIGHT, 0], support: 'both' });
}

// Picks the strangest of a few valid candidates. Most poses are moderately
// strange; about one in five is far out.
export function generatePose(rng = Math.random) {
  const r = rng();
  const support = r < 0.4 ? 'both' : r < 0.7 ? 'L' : 'R';
  const wild = rng() < 0.2 ? 1 : 0;
  const candidates = wild ? 6 : 3;
  let best = null, bestScore = -Infinity;
  for (let i = 0, found = 0; i < 1500 && found < candidates; i++) {
    const pose = attempt(rng, support, wild);
    if (!pose) continue;
    found++;
    const score = weirdness(pose);
    if (score > bestScore) [best, bestScore] = [pose, score];
  }
  return best || restPose();
}

/* ---------- helpers for display ---------- */

function turnPose(pose, yaw) {
  const turn = rotY(yaw);
  return { ...pose, rootR: mul(turn, pose.rootR), rootP: apply(turn, pose.rootP) };
}

// Turns the pose by `angle` to the left or to the right, whichever shows the
// body less foreshortened to a viewer looking from +z.
export function turnForViewer(pose, angle) {
  const shown = (p) => {
    const w = forwardKinematics(p.q, p.rootR, p.rootP);
    return CAPSULES.reduce((sum, c) => {
      const [a, b] = capsuleWorld(w, c);
      return sum + Math.hypot(a[0] - b[0], a[1] - b[1]);
    }, 0);
  };
  const left = turnPose(pose, -angle), right = turnPose(pose, angle);
  return shown(right) >= shown(left) ? right : left;
}

// Interpolates two poses, keeping the lowest point of the body on the ground.
export function blendPoses(a, b, t) {
  const q = a.q.map((v, i) => lerp(v, b.q[i], t));
  const rootR = quatToMat(slerp(matToQuat(a.rootR), matToQuat(b.rootR), t));
  const rootP = a.rootP.map((v, i) => lerp(v, b.rootP[i], t));
  rootP[1] -= lowestPoint(forwardKinematics(q, rootR, rootP));
  return { q, rootR, rootP, support: t < 0.5 ? a.support : b.support };
}

// Space the pose occupies: horizontal radius around the y axis, half width
// (x), how far it reaches toward the viewer (+z) and height.
export function poseExtent(pose) {
  const w = forwardKinematics(pose.q, pose.rootR, pose.rootP);
  let radius = 0, halfWidth = 0, front = 0, height = 0;
  for (const c of CAPSULES) {
    for (const p of capsuleWorld(w, c)) {
      radius = Math.max(radius, Math.hypot(p[0], p[2]) + c.r);
      halfWidth = Math.max(halfWidth, Math.abs(p[0]) + c.r);
      front = Math.max(front, p[2] + c.r);
      height = Math.max(height, p[1] + c.r);
    }
  }
  return { radius, halfWidth, front, height };
}

export function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
