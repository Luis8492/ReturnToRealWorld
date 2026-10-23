// Sky, sun, stars, ground and lights. update(theta) moves the sun:
// theta = 0 is sunrise on the left, PI/2 is noon, PI is sunset on the right,
// and PI..2PI is the night below the horizon.

import * as THREE from '../vendor/three/three.module.min.js';

// Palette keyed by the sun height s = sin(theta): [s, zenith, horizon, glow]
const SKY = [
  [-1.0, 0x03060f, 0x090f1f, 0x000000],
  [-0.35, 0x060b1c, 0x111a33, 0x05060f],
  [-0.12, 0x14204a, 0x433d63, 0x4a2440],
  [0.0, 0x2a4373, 0xd08068, 0xff6f35],
  [0.12, 0x4872b3, 0xeab48a, 0xff9e55],
  [0.35, 0x5a90d4, 0xc9dcec, 0xffd29a],
  [1.0, 0x3f82d8, 0xbcd7ef, 0xffeccc],
];

const smoothstep = (a, b, x) => {
  const t = Math.min(1, Math.max(0, (x - a) / (b - a)));
  return t * t * (3 - 2 * t);
};

const skyVertex = /* glsl */ `
  varying vec3 vDir;
  void main() {
    vDir = position;
    gl_Position = projectionMatrix * viewMatrix * vec4(position + cameraPosition, 1.0);
  }
`;

const skyFragment = /* glsl */ `
  uniform vec3 zenith;
  uniform vec3 horizon;
  uniform vec3 glow;
  uniform vec3 sunColor;
  uniform vec3 sunDir;
  varying vec3 vDir;
  void main() {
    vec3 d = normalize(vDir);
    float h = max(d.y, 0.0);
    vec3 col = mix(horizon, zenith, pow(h, 0.5));
    float c = max(dot(d, sunDir), 0.0);
    // Glow around the sun, spreading along the horizon when the sun is low.
    float halo = pow(c, 24.0) * 0.28 + pow(c, 700.0) * 0.7;
    float band = pow(c, 4.0) * exp(-h * 14.0) * 0.5;
    col += glow * (halo + band);
    float disc = smoothstep(0.99972, 0.9998, dot(d, sunDir));
    col = mix(col, sunColor, disc);
    gl_FragColor = vec4(col, 1.0);
    #include <tonemapping_fragment>
    #include <colorspace_fragment>
  }
`;

function groundTexture() {
  const size = 256;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const g = canvas.getContext('2d');
  g.fillStyle = '#6f8458';
  g.fillRect(0, 0, size, size);
  const tones = ['#7d935f', '#62774d', '#879b66', '#5a6d47', '#76875a'];
  for (let i = 0; i < 2600; i++) {
    g.fillStyle = tones[i % tones.length];
    g.globalAlpha = 0.25 + Math.random() * 0.4;
    const x = Math.random() * size, y = Math.random() * size;
    g.fillRect(x, y, 1 + Math.random() * 3, 1 + Math.random() * 2);
  }
  g.globalAlpha = 1;
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  return tex;
}

function createStars(count = 900) {
  const pos = new Float32Array(count * 3), col = new Float32Array(count * 3);
  for (let i = 0; i < count; i++) {
    const y = 0.03 + Math.random() * 0.97, a = Math.random() * Math.PI * 2, r = Math.sqrt(1 - y * y);
    pos.set([Math.cos(a) * r * 900, y * 900, Math.sin(a) * r * 900], i * 3);
    const b = 0.35 + Math.random() ** 3 * 0.65;
    col.set([b, b, b * 1.05], i * 3);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  const mat = new THREE.PointsMaterial({ size: 1.5, sizeAttenuation: false, vertexColors: true, transparent: true, opacity: 0, depthWrite: false, fog: false });
  const stars = new THREE.Points(geo, mat);
  stars.frustumCulled = false;
  return stars;
}

export function createWorld(scene, renderer) {
  const uniforms = {
    zenith: { value: new THREE.Color() },
    horizon: { value: new THREE.Color() },
    glow: { value: new THREE.Color() },
    sunColor: { value: new THREE.Color() },
    sunDir: { value: new THREE.Vector3(0, 1, 0) },
  };
  const sky = new THREE.Mesh(
    new THREE.SphereGeometry(1, 48, 32),
    new THREE.ShaderMaterial({ uniforms, vertexShader: skyVertex, fragmentShader: skyFragment, side: THREE.BackSide, depthTest: false, depthWrite: false }),
  );
  sky.frustumCulled = false;
  sky.renderOrder = -1;
  scene.add(sky);

  const stars = createStars();
  scene.add(stars);

  const tex = groundTexture();
  tex.repeat.set(600, 600);
  tex.anisotropy = renderer.capabilities.getMaxAnisotropy();
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(3000, 3000), new THREE.MeshStandardMaterial({ map: tex, roughness: 1 }));
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  scene.fog = new THREE.Fog(0x000000, 12, 450);

  const sun = new THREE.DirectionalLight(0xffffff, 0);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -2.2, right: 2.2, top: 2.2, bottom: -2.2, near: 1, far: 80 });
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 0.02;
  sun.target.position.set(0, 0.9, 0);
  scene.add(sun, sun.target);

  const hemi = new THREE.HemisphereLight(0xffffff, 0x000000, 1);
  scene.add(hemi);

  // Soft light from the viewer's side so the figure stays readable against the sun.
  const fill = new THREE.DirectionalLight(0xffffff, 0.3);
  scene.add(fill);

  // Sun path, set from the camera so the whole arc stays on screen.
  let arcWidth = 0.4, arcHeight = 0.35;
  function setFraming(camera) {
    const vHalf = THREE.MathUtils.degToRad(camera.fov / 2);
    const hHalf = Math.atan(Math.tan(vHalf) * camera.aspect);
    const pitch = Math.atan2(camera.userData.lookY - camera.position.y, camera.position.z);
    arcWidth = hHalf * 0.78;
    arcHeight = (vHalf + pitch) * 0.82;
    fill.position.set(camera.position.x + 2, camera.position.y + 4, camera.position.z);
    stars.material.size = 1.5 * renderer.getPixelRatio();
  }

  const c1 = new THREE.Color(), c2 = new THREE.Color();
  function palette(s, column, out) {
    let i = 0;
    while (i < SKY.length - 2 && s > SKY[i + 1][0]) i++;
    const [s0] = SKY[i], [s1] = SKY[i + 1];
    const t = Math.min(1, Math.max(0, (s - s0) / (s1 - s0)));
    return out.copy(c1.setHex(SKY[i][column])).lerp(c2.setHex(SKY[i + 1][column]), t);
  }

  const warm = new THREE.Color(0xff9a52), white = new THREE.Color(0xfff6e6);
  const nightBlue = new THREE.Color(0x4d5d8c), dayBlue = new THREE.Color(0xc4dcff);
  const nightGround = new THREE.Color(0x14141c), dayGround = new THREE.Color(0x6f6a4c);
  const moon = new THREE.Color(0x8fa3df), noon = new THREE.Color(0xfff1e0);

  function update(theta) {
    const s = Math.sin(theta);
    const az = -arcWidth * Math.cos(theta), el = arcHeight * s;
    const dir = uniforms.sunDir.value.set(Math.sin(az) * Math.cos(el), Math.sin(el), -Math.cos(az) * Math.cos(el));

    palette(s, 1, uniforms.zenith.value);
    palette(s, 2, uniforms.horizon.value);
    palette(s, 3, uniforms.glow.value);
    uniforms.sunColor.value.copy(warm).lerp(white, smoothstep(0, 0.35, s)).multiplyScalar(1.6);
    scene.fog.color.copy(uniforms.horizon.value);

    const day = smoothstep(-0.25, 0.45, s);
    sun.position.copy(sun.target.position).addScaledVector(dir, 30);
    sun.color.copy(warm).lerp(white, smoothstep(0, 0.45, s));
    sun.intensity = 2.6 * smoothstep(-0.02, 0.2, s);
    hemi.color.copy(nightBlue).lerp(dayBlue, day);
    hemi.groundColor.copy(nightGround).lerp(dayGround, day);
    hemi.intensity = 0.45 + 1.15 * day;
    fill.color.copy(moon).lerp(noon, day);
    fill.intensity = 0.45 + 0.75 * day;
    stars.material.opacity = smoothstep(0.02, -0.3, s);
  }

  return { setFraming, update };
}
