// The session shared by both versions: the buttons, two sunrises and sunsets
// (one pose each), the leaves, "bye." and keeping the screen on.
//
// Each version is a mode: the button that starts it, options for its world
// (see world.js) and a stage, which puts its own figure into the scene and
// frames the camera:
//   rest                      pose at the start and the end
//   makePoses(rng)            two new poses (slow, done ahead of time)
//   begin([pose1, pose2])     poses about to be shown, returns them as shown
//   blend(a, b, t)            in-between pose
//   setPose(pose)
//   frame(camera, w, h)       places the camera, returns the world point
//                             "bye." appears above

import * as THREE from '../vendor/three/three.module.min.js';
import { mulberry32 } from './math.js';
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

export function startApp(modes) {
  const canvas = document.getElementById('scene');
  const startScreen = document.getElementById('start');
  const leaves = document.getElementById('leaves');
  const leafRight = leaves.querySelector('.right');
  const leafLeft = leaves.querySelector('.left');
  const bye = document.getElementById('bye');
  for (const mode of modes) mode.button = document.getElementById(mode.button);

  const params = new URLSearchParams(location.search);

  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  } catch (err) {
    console.error(err);
    for (const mode of modes) mode.button.disabled = true;
    modes[0].button.textContent = 'WebGL is not available';
    return;
  }

  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 5000);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  // Each mode has a scene of its own.
  for (const mode of modes) {
    mode.scene = new THREE.Scene();
    mode.world = createWorld(mode.scene, renderer, mode.worldOptions);
    mode.stage = mode.createStage(mode.scene, renderer);
    mode.nextPoses = null;
  }

  /* ---------- session ---------- */

  let session = null;

  function frame() {
    const { stage, world } = session.mode;
    const w = window.innerWidth, h = window.innerHeight;
    renderer.setSize(w, h, false);
    const anchor = stage.frame(camera, w, h);
    camera.updateProjectionMatrix();
    camera.updateMatrixWorld();
    world.setFraming(camera);
    // "bye." appears just above the anchor point.
    const p = anchor.clone().project(camera);
    const above = h - ((1 - p.y) / 2) * h + 12;
    bye.style.bottom = `${Math.min(above, h - bye.offsetHeight - 16)}px`;
  }

  function poseAt(s, t) {
    const { stage } = s.mode;
    if (t < T1) return stage.blend(stage.rest, s.pose1, smooth(0.5, 2.6, t));
    if (t < T2) return s.pose1;
    if (t < T3) return stage.blend(s.pose1, s.pose2, smooth(T2 + 0.8, T2 + 3.2, t));
    if (t < T4) return s.pose2;
    return stage.blend(s.pose2, stage.rest, smooth(T4 + 0.6, T4 + 2.6, t));
  }

  function render(t) {
    const { stage, world, scene } = session.mode;
    world.update(sunAngle(t));
    stage.setPose(poseAt(session, t));
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
      whenIdle(() => preparePoses(s.mode));
      return;
    }
    requestAnimationFrame(tick);
  }

  // Poses are made ahead of time so the buttons respond at once.
  function preparePoses(mode) {
    const seed = params.has('seed') ? Number(params.get('seed')) : (Math.random() * 2 ** 32) >>> 0;
    mode.nextPoses = mode.stage.makePoses(mulberry32(seed));
  }
  const whenIdle = (fn) => (window.requestIdleCallback ? requestIdleCallback(fn, { timeout: 1000 }) : setTimeout(fn, 200));

  function start(mode) {
    if (session) return;
    if (!mode.nextPoses) preparePoses(mode);
    const [pose1, pose2] = mode.stage.begin(mode.nextPoses);
    mode.nextPoses = null;
    session = { mode, pose1, pose2, t: Math.max(0, Number(params.get('t')) || 0), last: performance.now(), done: false };
    frame();
    render(session.t);

    startScreen.classList.add('hidden');
    leaves.classList.add('shown');
    requestWakeLock();
    requestAnimationFrame(tick);
  }

  function reset() {
    const { mode } = session;
    session = null;
    leafLeft.classList.remove('lit');
    leafRight.classList.remove('lit');
    bye.classList.remove('shown');
    leaves.classList.remove('shown');
    startScreen.classList.remove('hidden');
    mode.button.focus({ preventScroll: true });
  }

  for (const mode of modes) {
    whenIdle(() => {
      preparePoses(mode);
      // compile the shaders too, so the first frame after a click does not stall
      if (!session) renderer.compile(mode.scene, camera);
    });
    mode.button.addEventListener('click', () => start(mode));
  }
  // After "bye.", a tap anywhere brings the buttons back.
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
}
