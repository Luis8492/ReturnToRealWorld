// Full-body version: a standing wooden figure on the open ground.

import * as THREE from '../vendor/three/three.module.min.js';
import { generatePose, restPose, blendPoses, turnForViewer, poseExtent } from './pose.js';
import { createFigure } from './figure.js';

// The figure stays still, turned a little from the viewer so that reaching
// forward or back reads as well as reaching sideways.
const VIEW_ANGLE = THREE.MathUtils.degToRad(30);
const EYE_HEIGHT = 1.6; // the camera sits at the eye level of the standing figure

export const bodyMode = {
  button: 'go',
  worldOptions: {},
  createStage(scene) {
    const figure = createFigure();
    scene.add(figure.object);
    const rest = restPose();
    let extent = poseExtent(rest);

    return {
      rest,
      makePoses: (rng) => [generatePose(rng), generatePose(rng)],
      begin(poses) {
        const shown = poses.map((p) => turnForViewer(p, VIEW_ANGLE));
        const [e1, e2] = shown.map(poseExtent);
        extent = {
          halfWidth: Math.max(e1.halfWidth, e2.halfWidth),
          front: Math.max(e1.front, e2.front),
          height: Math.max(e1.height, e2.height),
        };
        return shown;
      },
      blend: blendPoses,
      setPose: figure.setPose,
      frame(camera, w, h) {
        camera.aspect = w / h;
        camera.fov = camera.aspect < 1 ? 52 : 40;
        const vHalf = THREE.MathUtils.degToRad(camera.fov / 2);
        const hHalf = Math.atan(Math.tan(vHalf) * camera.aspect);
        const lookY = extent.height * 0.5;
        const fitHeight = (extent.height * 0.5 + 0.45) / Math.tan(vHalf);
        const fitWidth = (extent.halfWidth + 0.15) / Math.tan(hHalf) + extent.front;
        const dist = Math.max(fitHeight, fitWidth, 3.5);
        camera.position.set(0, EYE_HEIGHT, dist);
        camera.lookAt(0, lookY, 0);
        // "bye." appears in the sky just above the head of the figure standing at the end.
        return new THREE.Vector3(0, 1.75, 0);
      },
    };
  },
};
