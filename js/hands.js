// Hands-only version: two forearms and hands on a desk, seen from your own
// eyes. The sun stays out of sight; its light and the shadows it casts on the
// desk tell the time of day.

import * as THREE from '../vendor/three/three.module.min.js';
import { generatePose, restPose, blendPoses, viewPoints, EYE } from './hand-pose.js';
import { createHands, DESK_HEIGHT } from './hand-figure.js';

const deg = THREE.MathUtils.degToRad;

// Looking down at the hands from the eyes of someone sitting at the desk.
const LOOK_DOWN = deg(52);

export const handsMode = {
  button: 'go-hands',
  worldOptions: {
    shadowCenter: [0, DESK_HEIGHT, -0.2],
    shadowSize: 1.3,
    shadowDepth: 5,
    shadowNormalBias: 0.002,
    // rises far to the left, high in front at noon, sets far to the right
    sunPath: [deg(75), deg(60)],
  },
  createStage(scene) {
    const hands = createHands();
    scene.add(hands.object);
    const rest = restPose();
    let points = viewPoints(rest);

    return {
      rest,
      makePoses: (rng) => [generatePose(rng), generatePose(rng)],
      begin(poses) {
        points = [rest, ...poses].flatMap(viewPoints).map(({ at, r }) => ({ at: new THREE.Vector3(at[0], DESK_HEIGHT + at[1], at[2]), r }));
        return poses;
      },
      blend: blendPoses,
      setPose: hands.setPose,
      frame(camera, w, h) {
        camera.aspect = w / h;
        camera.fov = camera.aspect < 1 ? 66 : 50;
        // From the eyes; on narrow screens step back until both hands fit.
        const forward = new THREE.Vector3(0, -Math.sin(LOOK_DOWN), -Math.cos(LOOK_DOWN));
        const eye = new THREE.Vector3(EYE[0], DESK_HEIGHT + EYE[1], EYE[2]);
        const v = new THREE.Vector3();
        const tanV = Math.tan(deg(camera.fov / 2));
        const inView = ({ at, r }) => {
          v.copy(at).applyMatrix4(camera.matrixWorldInverse);
          const m = r / (-v.z * tanV); // the radius on screen
          v.copy(at).project(camera);
          return Math.abs(v.x) + m / camera.aspect < 0.95 && Math.abs(v.y) + m < 0.95;
        };
        for (let back = 0; back < 1.5; back += 0.02) {
          camera.position.copy(eye).addScaledVector(forward, -back);
          camera.lookAt(v.copy(camera.position).add(forward));
          camera.updateMatrixWorld();
          camera.updateProjectionMatrix();
          if (points.every(inView)) break;
        }
        // "bye." appears in the upper part of the screen, over the far side of the desk.
        return new THREE.Vector3(0, 0.62, 0.5).unproject(camera);
      },
    };
  },
};
