// A wooden-mannequin style figure built on the skeleton from pose.js.

import * as THREE from '../vendor/three/three.module.min.js';
import { BONES, ANKLE_HEIGHT, SOLE, jointRotation, matToQuat } from './pose.js';

const WOOD = 0xdcc19c;
const JOINT = 0xc4a27c;

// Tapered limb from the joint (y = 0) down to y = -length.
function limb(length, r1, r2) {
  const g = new THREE.CylinderGeometry(r1, r2, length, 24, 1, true);
  g.translate(0, -length / 2, 0);
  return g;
}

// Side profile of the foot extruded across its width, with a flat sole.
function footGeometry() {
  const b = 0.008; // bevel, the outline below is inset by this much
  const y0 = -ANKLE_HEIGHT + b;
  const heel = SOLE.heel + b, toe = SOLE.toe - b;
  const s = new THREE.Shape();
  s.moveTo(heel + 0.015, y0);
  s.lineTo(toe - 0.03, y0);
  s.quadraticCurveTo(toe, y0, toe, y0 + 0.016);
  s.quadraticCurveTo(toe - 0.004, y0 + 0.03, toe - 0.05, y0 + 0.036);
  s.lineTo(0.05, -0.016);
  s.quadraticCurveTo(0.01, 0.008, -0.03, 0.0);
  s.quadraticCurveTo(heel, -0.012, heel, y0 + 0.022);
  s.quadraticCurveTo(heel, y0, heel + 0.015, y0);
  const depth = SOLE.halfWidth * 2 - 2 * b;
  const g = new THREE.ExtrudeGeometry(s, { depth, bevelEnabled: true, bevelThickness: b, bevelSize: b, bevelSegments: 3, curveSegments: 12 });
  g.translate(0, 0, -depth / 2);
  g.rotateY(-Math.PI / 2); // profile x -> forward (z)
  g.computeVertexNormals();
  return g;
}

export function createFigure() {
  const wood = new THREE.MeshStandardMaterial({ color: WOOD, roughness: 0.58 });
  const joint = new THREE.MeshStandardMaterial({ color: JOINT, roughness: 0.5 });
  const sphere = new THREE.SphereGeometry(1, 32, 20);

  const root = new THREE.Group();
  const nodes = BONES.map((b) => {
    const g = new THREE.Group();
    g.position.fromArray(b.offset);
    return g;
  });
  BONES.forEach((b, i) => (b.parent < 0 ? root : nodes[b.parent]).add(nodes[i]));
  const bone = Object.fromEntries(BONES.map((b, i) => [b.name, nodes[i]]));

  const part = (parent, geometry, material, position = [0, 0, 0], size = [1, 1, 1]) => {
    const m = new THREE.Mesh(geometry, material);
    m.position.fromArray(position);
    m.scale.fromArray(size);
    m.castShadow = true;
    m.receiveShadow = true;
    parent.add(m);
    return m;
  };
  const ball = (parent, r, position) => part(parent, sphere, joint, position, [r, r, r]);

  // trunk
  part(bone.pelvis, sphere, wood, [0, -0.015, 0], [0.145, 0.1, 0.1]);
  ball(bone.spine1, 0.075);
  part(bone.spine1, sphere, wood, [0, 0.095, 0.005], [0.12, 0.1, 0.088]);
  part(bone.spine2, sphere, wood, [0, 0.085, 0], [0.16, 0.135, 0.105]);
  const shoulders = part(bone.spine2, new THREE.CapsuleGeometry(0.045, 0.3, 8, 20), wood, [0, 0.17, 0]);
  shoulders.rotation.z = Math.PI / 2;

  // neck and head (with a small nose so the head's direction is readable)
  part(bone.neck, new THREE.CapsuleGeometry(0.034, 0.07, 6, 16), joint, [0, 0.035, 0]);
  part(bone.neck, sphere, wood, [0, 0.13, 0.008], [0.085, 0.108, 0.095]);
  part(bone.neck, sphere, wood, [0, 0.115, 0.098], [0.016, 0.02, 0.016]);

  for (const s of ['L', 'R']) {
    const side = s === 'L' ? 1 : -1;
    // arm
    ball(bone['shoulder' + s], 0.05);
    part(bone['shoulder' + s], limb(0.29, 0.042, 0.034), wood);
    ball(bone['elbow' + s], 0.036);
    part(bone['elbow' + s], limb(0.26, 0.034, 0.027), wood);
    ball(bone['wrist' + s], 0.027);
    part(bone['wrist' + s], sphere, wood, [0, -0.065, 0], [0.02, 0.055, 0.04]);
    const thumb = part(bone['wrist' + s], new THREE.CapsuleGeometry(0.012, 0.035, 4, 10), wood, [-0.004 * side, -0.045, 0.04]);
    thumb.rotation.x = -0.6;
    // leg
    ball(bone['hip' + s], 0.065);
    part(bone['hip' + s], limb(0.43, 0.065, 0.047), wood);
    ball(bone['knee' + s], 0.047);
    part(bone['knee' + s], limb(0.42, 0.046, 0.034), wood);
    ball(bone['ankle' + s], 0.034);
    part(bone['ankle' + s], footGeometry(), wood);
  }

  function setPose(pose) {
    nodes[0].position.fromArray(pose.rootP);
    nodes[0].quaternion.fromArray(matToQuat(pose.rootR));
    for (let i = 1; i < BONES.length; i++) nodes[i].quaternion.fromArray(matToQuat(jointRotation(BONES[i], pose.q)));
  }

  return { object: root, setPose };
}
