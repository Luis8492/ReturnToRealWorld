import * as THREE from '../vendor/three/three.module.min.js';
import { generatePose, restPose, blendPoses, turnForViewer, poseExtent, mulberry32 } from './pose.js';
import { createFigure } from './figure.js';
import { createWorld } from './world.js';

/* ---------- timeline (seconds) ---------- */

const DAY = 20; // one pose, one sunrise to sunset
const INTRO = 3;
const NIGHT = 4;
const OUTRO = 3;
const T1 = INTRO; // first sunrise
const T2 = T1 + DAY; // first sunset
const T3 = T2 + NIGHT; // second sunrise
const T4 = T3 + DAY; // second sunset
const T_END = T4 + OUTRO;

const SUN_RATE = Math.PI / DAY;

// The figure stays still, turned a little from the viewer so that reaching
// forward or back reads as well as reaching sideways.
const VIEW_ANGLE = THREE.MathUtils.degToRad(30);

const lerp = (a, b, t) => a + (b - a) * t;
const clamp01 = (x) => Math.min(1, Math.max(0, x));
const smooth = (a, b, x) => {
  const t = clamp01((x - a) / (b - a));
  return t * t * (3 - 2 * t);
};
// Cubic Hermite from p0 to p1 with end slopes m0, m1 (per unit of u).
const hermite = (p0, p1, m0, m1, u) => {
  const u2 = u * u, u3 = u2 * u;
  return (2 * u3 - 3 * u2 + 1) * p0 + (u3 - 2 * u2 + u) * m0 + (-2 * u3 + 3 * u2) * p1 + (u3 - u2) * m1;
};

function sunAngle(t) {
  if (t < T1) return hermite(-0.6, 0, 0, SUN_RATE * INTRO, clamp01(t / INTRO));
  if (t < T2) return SUN_RATE * (t - T1);
  if (t < T3) return hermite(Math.PI, 2 * Math.PI, SUN_RATE * NIGHT, SUN_RATE * NIGHT, (t - T2) / NIGHT);
  if (t < T4) return 2 * Math.PI + SUN_RATE * (t - T3);
  return hermite(3 * Math.PI, 3 * Math.PI + 0.7, SUN_RATE * OUTRO, 0, clamp01((t - T4) / OUTRO));
}

/* ---------- scene ---------- */

const canvas = document.getElementById('scene');
const startScreen = document.getElementById('start');
const button = document.getElementById('go');
const leaves = document.getElementById('leaves');
const leafRight = leaves.querySelector('.right');
const leafLeft = leaves.querySelector('.left');
const bye = document.getElementById('bye');

const params = new URLSearchParams(location.search);

let renderer;
try {
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
} catch (err) {
  console.error(err);
  button.disabled = true;
  button.textContent = 'WebGL is not available';
}

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 5000);
let world, figure;

if (renderer) {
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  world = createWorld(scene, renderer);
  figure = createFigure();
  scene.add(figure.object);
}

/* ---------- session ---------- */

let session = null;
let extent = poseExtent(restPose());
const EYE_HEIGHT = 1.6; // the camera sits at the eye level of the standing figure

function frame() {
  const w = window.innerWidth, h = window.innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.fov = camera.aspect < 1 ? 52 : 40;
  const vHalf = THREE.MathUtils.degToRad(camera.fov / 2);
  const hHalf = Math.atan(Math.tan(vHalf) * camera.aspect);
  const lookY = extent.height * 0.5;
  const fitHeight = (extent.height * 0.5 + 0.45) / Math.tan(vHalf);
  const fitWidth = (extent.halfWidth + 0.15) / Math.tan(hHalf) + extent.front;
  const dist = Math.max(fitHeight, fitWidth, 3.5);
  camera.position.set(0, EYE_HEIGHT, dist);
  camera.userData.lookY = lookY;
  camera.lookAt(0, lookY, 0);
  camera.updateProjectionMatrix();
  camera.updateMatrixWorld();
  world.setFraming(camera);
  // "bye." appears in the sky just above the head of the figure standing at the end.
  const head = new THREE.Vector3(0, 1.75, 0).project(camera);
  const above = h - ((1 - head.y) / 2) * h + 12;
  bye.style.bottom = `${Math.min(above, h - bye.offsetHeight - 16)}px`;
}

function poseAt(s, t) {
  if (t < T1) return blendPoses(s.rest, s.pose1, smooth(0.5, 2.6, t));
  if (t < T2) return s.pose1;
  if (t < T3) return blendPoses(s.pose1, s.pose2, smooth(T2 + 0.8, T2 + 3.2, t));
  if (t < T4) return s.pose2;
  return blendPoses(s.pose2, s.rest, smooth(T4 + 0.6, T4 + 2.6, t));
}

function render(t) {
  world.update(sunAngle(t));
  figure.setPose(poseAt(session, t));
  renderer.render(scene, camera);
}

function tick(now) {
  const s = session;
  if (!s || s.done) return;
  const dt = Math.min(0.1, (now - s.last) / 1000); // pauses while the tab is hidden
  s.last = now;
  s.t += dt;
  const t = s.t;
  if (t >= T2 + 0.4) leafRight.classList.add('lit');
  if (t >= T4 + 0.4) leafLeft.classList.add('lit');
  if (t >= T4 + 1.4) bye.classList.add('shown');
  render(Math.min(t, T_END));
  if (t >= T_END) {
    s.done = true;
    releaseWakeLock();
    whenIdle(preparePoses);
    return;
  }
  requestAnimationFrame(tick);
}

// Poses are made ahead of time so the button responds at once.
let nextPoses = null;
function preparePoses() {
  const seed = params.has('seed') ? Number(params.get('seed')) : (Math.random() * 2 ** 32) >>> 0;
  const rng = mulberry32(seed);
  nextPoses = [generatePose(rng), generatePose(rng)];
}
const whenIdle = (fn) => (window.requestIdleCallback ? requestIdleCallback(fn, { timeout: 1000 }) : setTimeout(fn, 200));

function start() {
  if (session || !renderer) return;
  if (!nextPoses) preparePoses();
  const [pose1, pose2] = nextPoses.map((p) => turnForViewer(p, VIEW_ANGLE));
  nextPoses = null;
  const rest = restPose();
  const e1 = poseExtent(pose1), e2 = poseExtent(pose2);
  extent = {
    halfWidth: Math.max(e1.halfWidth, e2.halfWidth),
    front: Math.max(e1.front, e2.front),
    height: Math.max(e1.height, e2.height),
  };
  session = { rest, pose1, pose2, t: Math.max(0, Number(params.get('t')) || 0), last: performance.now(), done: false };
  frame();
  render(session.t);

  startScreen.classList.add('hidden');
  leaves.classList.add('shown');
  requestWakeLock();
  requestAnimationFrame(tick);
}

function reset() {
  session = null;
  leafLeft.classList.remove('lit');
  leafRight.classList.remove('lit');
  bye.classList.remove('shown');
  leaves.classList.remove('shown');
  startScreen.classList.remove('hidden');
  button.focus({ preventScroll: true });
}

if (renderer) whenIdle(preparePoses);
button.addEventListener('click', start);
// After "bye.", a tap anywhere brings the button back.
canvas.addEventListener('click', () => {
  if (session && session.done) reset();
});

window.addEventListener('resize', () => {
  if (!session) return;
  frame();
  if (session.done) render(T_END);
});

/* ---------- keep the screen on during the session ---------- */

let wakeLock = null;
async function requestWakeLock() {
  try {
    wakeLock = await navigator.wakeLock?.request('screen');
  } catch {
    wakeLock = null;
  }
}
function releaseWakeLock() {
  wakeLock?.release().catch(() => {});
  wakeLock = null;
}
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'visible' && session && !session.done) {
    session.last = performance.now();
    requestWakeLock();
  }
});
