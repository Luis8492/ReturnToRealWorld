// A desk with two wooden-mannequin forearms and hands, built on the skeleton
// from hand-pose.js.

import * as THREE from '../vendor/three/three.module.min.js';
import { BONES, FINGERS, THUMB, FOREARM, ELBOW_HEIGHT, WRIST_HEIGHT, localRotation } from './hand-pose.js';
import { matToQuat } from './math.js';

export const DESK_HEIGHT = 0.72;

// Same paint as the full-body figure: blue is your right, orange your left.
// Here you see the hands from behind, as your own, so they are not mirrored.
const LIMB = { R: 0x4f86c6, L: 0xe0883a };
const LIMB_JOINT = { R: 0x3d6da8, L: 0xc0702c };
const WOOD = 0xdcc19c; // the unpainted palm, so you can tell which way it faces
const DESK = 0xa49a8a;
const DESK_LEGS = 0x6f675b;

// Bone-local meshes point along -z.
function along(geometry, length) {
  geometry.rotateX(Math.PI / 2);
  geometry.translate(0, 0, -length / 2);
  return geometry;
}

const taper = (length, r1, r2, segments = 24) => along(new THREE.CylinderGeometry(r1, r2, length, segments, 1, true), length);
const capsule = (length, r) => along(new THREE.CapsuleGeometry(r, length, 6, 14), length);

// Outline of the palm seen from the back of the hand (x: toward the little
// finger, y: toward the fingers), extruded to its thickness.
function palmGeometry() {
  const s = new THREE.Shape();
  s.moveTo(-0.02, 0.0);
  s.lineTo(0.018, 0.0);
  s.quadraticCurveTo(0.03, 0.002, 0.032, 0.02);
  s.lineTo(0.034, 0.072);
  s.quadraticCurveTo(0.034, 0.088, 0.022, 0.091);
  s.quadraticCurveTo(0.006, 0.1, -0.012, 0.102);
  s.quadraticCurveTo(-0.032, 0.103, -0.035, 0.088);
  s.lineTo(-0.034, 0.03);
  s.quadraticCurveTo(-0.032, 0.002, -0.02, 0.0);
  const bevel = 0.006, half = 0.0135;
  const g = new THREE.ExtrudeGeometry(s, { depth: 2 * (half - bevel), bevelEnabled: true, bevelThickness: bevel, bevelSize: bevel * 0.8, bevelSegments: 3, curveSegments: 10 });
  g.translate(0, 0, -(half - bevel));
  // shape x -> hand x, shape y -> hand -z, extrusion z -> hand y
  g.rotateX(-Math.PI / 2);
  g.translate(0, 0, -0.004);
  g.computeVertexNormals();
  return g;
}

function deskObject() {
  const top = new THREE.MeshStandardMaterial({ color: DESK, roughness: 0.7 });
  const legs = new THREE.MeshStandardMaterial({ color: DESK_LEGS, roughness: 0.6 });
  const desk = new THREE.Group();
  const width = 2.2, near = 0.4, far = -1.2, thick = 0.04;
  const board = new THREE.Mesh(new THREE.BoxGeometry(width, thick, near - far), top);
  board.position.set(0, -thick / 2, (near + far) / 2);
  desk.add(board);
  for (const x of [-1, 1]) {
    for (const z of [near - 0.08, far + 0.08]) {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.05, DESK_HEIGHT - thick, 0.05), legs);
      leg.position.set(x * (width / 2 - 0.08), -thick - (DESK_HEIGHT - thick) / 2, z);
      desk.add(leg);
    }
  }
  desk.traverse((m) => {
    m.castShadow = m.isMesh;
    m.receiveShadow = m.isMesh;
  });
  return desk;
}

function armObject(side) {
  // Front faces cast the shadows: with the low sun, back faces would leave
  // holes in the shadows of the parts resting on the desk.
  const material = (color, roughness) => new THREE.MeshStandardMaterial({ color, roughness, shadowSide: THREE.FrontSide });
  const paint = material(LIMB[side], 0.55);
  const knob = material(LIMB_JOINT[side], 0.5);
  const wood = material(WOOD, 0.6);
  const sphere = new THREE.SphereGeometry(1, 24, 16);

  const root = new THREE.Group();
  const nodes = BONES.map((b) => {
    const g = new THREE.Group();
    g.position.fromArray(b.offset);
    return g;
  });
  BONES.forEach((b, i) => (b.parent < 0 ? root : nodes[b.parent]).add(nodes[i]));
  const node = Object.fromEntries(BONES.map((b, i) => [b.name, nodes[i]]));

  const part = (parent, geometry, material, position = [0, 0, 0], size = [1, 1, 1]) => {
    const m = new THREE.Mesh(geometry, material);
    m.position.fromArray(position);
    m.scale.fromArray(size);
    m.castShadow = true;
    m.receiveShadow = true;
    parent.add(m);
    return m;
  };
  const ball = (parent, r, position, material = knob) => part(parent, sphere, material, position, [r, r, r]);

  // forearm
  ball(node.forearm, ELBOW_HEIGHT + 0.002);
  part(node.forearm, taper(FOREARM, ELBOW_HEIGHT - 0.002, WRIST_HEIGHT - 0.002), paint, undefined, [1.08, 0.94, 1]);
  // hand
  ball(node.hand, 0.024);
  part(node.hand, palmGeometry(), paint);
  part(node.hand, sphere, wood, [-0.001, -0.0115, -0.054], [0.027, 0.0045, 0.036]);
  // thumb
  part(node.thumb1, capsule(THUMB.lengths[0], THUMB.radius[0]), paint);
  ball(node.thumb2, THUMB.radius[1] * 1.08);
  part(node.thumb2, capsule(THUMB.lengths[1], THUMB.radius[1]), paint);
  ball(node.thumb3, THUMB.radius[2] * 1.06);
  part(node.thumb3, capsule(THUMB.lengths[2], THUMB.radius[2]), paint);
  // fingers
  for (const f of FINGERS) {
    f.lengths.forEach((len, k) => {
      const n = node[f.name + (k + 1)], r = f.radius * (1 - 0.06 * k);
      ball(n, r * (k ? 1.06 : 1.14));
      part(n, capsule(len, r), paint);
    });
  }

  return { root, nodes };
}

export function createHands() {
  const object = new THREE.Group();
  object.add(deskObject());
  const arms = {};
  for (const s of ['L', 'R']) {
    arms[s] = armObject(s);
    // A left arm is a mirrored right arm.
    if (s === 'L') arms[s].root.scale.x = -1;
    object.add(arms[s].root);
  }
  object.position.y = DESK_HEIGHT;

  function setPose(pose) {
    for (const s of ['L', 'R']) {
      const p = pose[s], nodes = arms[s].nodes;
      for (let i = 0; i < BONES.length; i++) nodes[i].quaternion.fromArray(matToQuat(localRotation(i, p)));
    }
  }

  return { object, setPose };
}
