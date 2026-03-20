import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import URDFLoader from 'https://cdn.jsdelivr.net/npm/urdf-loader@0.12.4/src/URDFLoader.js';

const viewer = document.querySelector('#viewer');
const statusText = document.querySelector('#status');
const resetJointsBtn = document.querySelector('#resetJointsBtn');

const showIKBtn = document.querySelector('#showIKBtn');
const showJointsBtn = document.querySelector('#showJointsBtn');
const ikPanel = document.querySelector('#ikPanel');
const jointsPanel = document.querySelector('#jointsPanel');
const jointSlidersContainer = document.querySelector('#jointSliders');

const themeToggleBtn = document.querySelector('#themeToggleBtn');
const urdfFileInput = document.querySelector('#urdfFileInput');
const meshFilesInput = document.querySelector('#meshFilesInput');
const meshFolderInput = document.querySelector('#meshFolderInput');
const loadUploadBtn = document.querySelector('#loadUploadBtn');

const uploadCollapseBtn = document.querySelector('#uploadCollapseBtn');
const uploadPanelBody = document.querySelector('#uploadPanelBody');
const uploadPanelHeader = document.querySelector('#uploadPanelHeader');
const uploadFileList = document.querySelector('#uploadFileList');

const workspaceGenerateBtn = document.querySelector('#workspaceGenerateBtn');
const workspaceClearBtn = document.querySelector('#workspaceClearBtn');
const workspaceToggleBtn = document.querySelector('#workspaceToggleBtn');
const workspaceSampleCountInput = document.querySelector('#workspaceSampleCount');
const workspaceInfoEl = document.querySelector('#workspaceInfo');

const ikXInput = document.querySelector('#ikX');
const ikYInput = document.querySelector('#ikY');
const ikZInput = document.querySelector('#ikZ');
const ikRollInput = document.querySelector('#ikRoll');
const ikPitchInput = document.querySelector('#ikPitch');
const ikYawInput = document.querySelector('#ikYaw');
const poseSliderElements = [ikXInput, ikYInput, ikZInput, ikRollInput, ikPitchInput, ikYawInput].filter(Boolean);

const ikSolveBtn = document.querySelector('#ikSolveBtn');
const ikPickEEBtn = document.querySelector('#ikPickEEBtn');
const ikStatusEl = document.querySelector('#ikStatus');
const ikEEDisplayEl = document.querySelector('#ikEEPos');
const ikModePositionBtn = document.querySelector('#ikModePositionBtn');
const ikModePoseBtn = document.querySelector('#ikModePoseBtn');
const nullspaceEnableInput = document.querySelector('#nullspaceEnable');
const nullspaceBiasInput = document.querySelector('#nullspaceBias');
const nullspaceBiasOut = document.querySelector('#nullspaceBiasOut');
const nullspaceHint = document.querySelector('#nullspaceHint');
const nullspacePlayBtn = document.querySelector('#nullspacePlayBtn');

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.01, 100);
camera.position.set(1.4, 1.2, 1.8);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;
viewer.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0.35, 0);

const hemiLight = new THREE.HemisphereLight(0xffffff, 0x93a4b2, 1.5);
scene.add(hemiLight);

const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
keyLight.position.set(2.5, 4, 2);
scene.add(keyLight);

const fillLight = new THREE.DirectionalLight(0xffffff, 0.5);
fillLight.position.set(-2, 2, -2);
scene.add(fillLight);

const grid = new THREE.GridHelper(4, 24, 0x8aa0b4, 0xc9d4de);
grid.position.y = -0.14;
scene.add(grid);

const floor = new THREE.Mesh(
  new THREE.CircleGeometry(2.2, 48),
  new THREE.MeshStandardMaterial({ color: 0xf8fbfd, transparent: true, opacity: 0.8 })
);
floor.rotation.x = -Math.PI / 2;
floor.position.y = -0.141;
scene.add(floor);

const THEMES = {
  dark: {
    sceneBg: 0x0d1219,
    gridC1: 0x1e2d3d,
    gridC2: 0x253344,
    floorColor: 0x121a24,
  },
  light: {
    sceneBg: 0xe7edf2,
    gridC1: 0x8aa0b4,
    gridC2: 0xc9d4de,
    floorColor: 0xf8fbfd,
  },
};

function applyTheme(theme) {
  const t = THEMES[theme] || THEMES.dark;
  document.documentElement.dataset.theme = theme;
  scene.background = new THREE.Color(t.sceneBg);
  if (Array.isArray(grid.material)) {
    grid.material[0].color.setHex(t.gridC1);
    grid.material[1].color.setHex(t.gridC2);
  } else {
    grid.material.color.setHex(t.gridC1);
  }
  floor.material.color.setHex(t.floorColor);
  if (themeToggleBtn) themeToggleBtn.textContent = theme === 'dark' ? '☀️' : '🌙';
}

applyTheme('dark');

themeToggleBtn?.addEventListener('click', () => {
  const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
  applyTheme(next);
});

const ikMarker = new THREE.Mesh(
  new THREE.SphereGeometry(0.018, 16, 12),
  new THREE.MeshBasicMaterial({ color: 0xff4422, opacity: 0.82, transparent: true })
);
ikMarker.visible = false;
scene.add(ikMarker);

const eeTipMarker = new THREE.Mesh(
  new THREE.SphereGeometry(0.009, 14, 10),
  new THREE.MeshBasicMaterial({ color: 0x00c4bb })
);
eeTipMarker.visible = false;
scene.add(eeTipMarker);

let workspacePoints = null;
let workspaceVisible = true;
let workspacePointCount = 0;
const WORKSPACE_PICK_THRESHOLD = 0.008;

let robot = null;
let eeLink = null;
const eeLocalOffset = new THREE.Vector3();
const eeOrientationTarget = new THREE.Quaternion();

let sliderElements = [];
let jointNames = [];
const ikJointValues = {};

let eePickMode = false;
let isDragging = false;
let currentLoadId = 0;
let pendingMeshLoads = 0;
let robotStructureReady = false;
let ikPriorityMode = 'pose';
let nullspaceBiasValue = 0;
let nullspacePlayTimerId = null;
let nullspacePlayDirection = 1;
let nullspaceLockedPos = null;
let nullspaceLockedQuat = null;

const _dragPlane = new THREE.Plane();
const _dragRaycaster = new THREE.Raycaster();
const _dragIntersect = new THREE.Vector3();

const loadingManager = new THREE.LoadingManager();
const urdfLoader = new URDFLoader(loadingManager);
const stlLoader = new STLLoader(loadingManager);
const colladaLoader = new ColladaLoader(loadingManager);
const objLoader = new OBJLoader(loadingManager);

const uploadedMeshUrlMap = new Map();
const uploadedMeshObjectUrls = [];

function normalizePath(path) {
  return String(path || '').replace(/\\/g, '/').replace(/^\/+/, '').replace(/^\.\//, '');
}

function stripMeshUriPrefixes(path) {
  let value = String(path || '').trim();
  value = value.replace(/^(package|model|file):\/\//i, '');
  value = value.split('?')[0].split('#')[0];

  try {
    value = decodeURIComponent(value);
  } catch {
    // keep original if URI decoding fails
  }

  return normalizePath(value);
}

function setStatusText(message) {
  statusText.textContent = message;
}

function updateOutput(slider) {
  const output = slider.parentElement.querySelector('output');
  if (!output) return;
  output.value = `${slider.value}deg`;
  output.textContent = `${slider.value}deg`;
}

function updatePoseOutput(slider) {
  const output = slider.parentElement.querySelector('output');
  if (!output) return;

  const kind = slider.dataset.pose;
  const isPosition = kind === 'x' || kind === 'y' || kind === 'z';
  const text = isPosition ? `${Number(slider.value).toFixed(3)}m` : `${Math.round(Number(slider.value))}deg`;
  output.value = text;
  output.textContent = text;
}

function updateAllPoseOutputs() {
  poseSliderElements.forEach(updatePoseOutput);
}

function clearIKStatus() {
  if (!ikStatusEl) return;
  ikStatusEl.textContent = '';
  ikStatusEl.className = 'ik-status';
}

function updateNullspaceBiasOutput() {
  if (!nullspaceBiasOut || !nullspaceBiasInput) return;
  nullspaceBiasOut.textContent = `${Number(nullspaceBiasInput.value).toFixed(0)}%`;
}

function refreshNullspaceControls() {
  if (!nullspaceEnableInput || !nullspaceHint) return;

  const isRedundant = jointNames.length > 6;
  nullspaceEnableInput.disabled = !isRedundant;
  if (nullspaceBiasInput) nullspaceBiasInput.disabled = !isRedundant;
  if (nullspacePlayBtn) nullspacePlayBtn.disabled = !isRedundant;

  nullspaceHint.textContent = isRedundant
    ? 'Sweeps joint-space null-motion while holding the EE pose fixed.'
    : 'Load a redundant robot (7+ joints) to explore null-space.';

  if (!isRedundant) {
    nullspaceEnableInput.checked = false;
    setNullspacePlaying(false);
    nullspaceBiasValue = 0;
    nullspaceLockedPos = null;
    nullspaceLockedQuat = null;
    if (nullspaceBiasInput) nullspaceBiasInput.value = '0';
    updateNullspaceBiasOutput();
  }
}

function setNullspacePlaying(playing) {
  if (!nullspacePlayBtn) return;

  if (!playing) {
    if (nullspacePlayTimerId !== null) {
      clearInterval(nullspacePlayTimerId);
      nullspacePlayTimerId = null;
    }
    nullspacePlayBtn.classList.remove('active');
    nullspacePlayBtn.textContent = 'Play';
    return;
  }

  if (!robot || jointNames.length <= 6) return;
  if (!nullspaceEnableInput?.checked) {
    nullspaceEnableInput.checked = true;
  }

  lockNullspaceTarget();
  nullspacePlayBtn.classList.add('active');
  nullspacePlayBtn.textContent = 'Stop';

  if (nullspacePlayTimerId !== null) return;
  nullspacePlayTimerId = window.setInterval(() => {
    if (!robot || !nullspaceEnableInput?.checked || jointNames.length <= 6) {
      setNullspacePlaying(false);
      return;
    }

    const current = Number(nullspaceBiasInput?.value || 0);
    let next = current + nullspacePlayDirection * 2;

    if (next >= 100) {
      next = 100;
      nullspacePlayDirection = -1;
    } else if (next <= -100) {
      next = -100;
      nullspacePlayDirection = 1;
    }

    if (nullspaceBiasInput) nullspaceBiasInput.value = String(next);
    nullspaceBiasValue = next / 100;
    updateNullspaceBiasOutput();
    solveNullspaceExploration();
  }, 40);
}

function getNullspacePostureBias() {
  const bias = new Array(jointNames.length).fill(0);
  if (!nullspaceEnableInput?.checked || jointNames.length <= 6) return bias;

  jointNames.forEach((name, idx) => {
    const joint = robot?.joints?.[name];
    if (!joint) return;
    const lo = Number.isFinite(joint.limit?.lower) ? joint.limit.lower : -Math.PI;
    const hi = Number.isFinite(joint.limit?.upper) ? joint.limit.upper : Math.PI;
    const span = Math.max(hi - lo, 1e-3);
    const mid = 0.5 * (lo + hi);
    const target = mid + nullspaceBiasValue * 0.45 * span;
    const current = ikJointValues[name] ?? mid;
    bias[idx] = 0.25 * (target - current) / span;
  });
  return bias;
}

function lockNullspaceTarget() {
  if (!robot || !eeLink) return;
  robot.updateMatrixWorld(true);
  nullspaceLockedPos = getEEWorldPos();
  nullspaceLockedQuat = getEEWorldQuaternion();
}

function hasActiveNullspaceRequest() {
  return Boolean(nullspaceEnableInput?.checked && jointNames.length > 6);
}

function setIKPriorityMode(mode) {
  ikPriorityMode = mode === 'position' ? 'position' : 'pose';
  ikModePositionBtn?.classList.toggle('active', ikPriorityMode === 'position');
  ikModePoseBtn?.classList.toggle('active', ikPriorityMode === 'pose');
}

function updateWorkspaceInfo(message = null) {
  if (!workspaceInfoEl) return;
  if (message !== null) {
    workspaceInfoEl.textContent = message;
    return;
  }
  workspaceInfoEl.textContent = workspacePointCount > 0 ? `${workspacePointCount} points` : 'No samples';
}

function clearWorkspaceCloud() {
  workspacePointCount = 0;
  if (workspacePoints) {
    scene.remove(workspacePoints);
    workspacePoints.geometry.dispose();
    workspacePoints.material.dispose();
    workspacePoints = null;
  }
  updateWorkspaceInfo();
}

function setWorkspaceVisible(visible) {
  workspaceVisible = visible;
  if (workspacePoints) workspacePoints.visible = workspaceVisible;
  if (workspaceToggleBtn) workspaceToggleBtn.textContent = workspaceVisible ? 'Hide' : 'Show';
}

function getJointSampleLimitRad(joint) {
  if (!joint) return [-Math.PI, Math.PI];
  if (joint.jointType === 'continuous') return [-Math.PI, Math.PI];

  const lo = Number.isFinite(joint.limit?.lower) ? joint.limit.lower : -Math.PI;
  const hi = Number.isFinite(joint.limit?.upper) ? joint.limit.upper : Math.PI;
  if (!Number.isFinite(lo) || !Number.isFinite(hi) || lo === hi) return [-Math.PI, Math.PI];

  return [Math.min(lo, hi), Math.max(lo, hi)];
}

function randomBetween(min, max) {
  return min + Math.random() * (max - min);
}

function generateWorkspaceCloud() {
  if (!robot || !eeLink || jointNames.length === 0) {
    updateWorkspaceInfo('Load robot first');
    return;
  }

  const rawCount = Number.parseInt(workspaceSampleCountInput?.value || '3000', 10);
  const sampleCount = Number.isFinite(rawCount) ? Math.max(200, Math.min(12000, rawCount)) : 3000;
  if (workspaceSampleCountInput) workspaceSampleCountInput.value = String(sampleCount);

  updateWorkspaceInfo('Generating...');

  const originalValues = {};
  jointNames.forEach((name) => {
    originalValues[name] = ikJointValues[name] ?? robot.joints[name]?.angle ?? 0;
  });

  const positions = new Float32Array(sampleCount * 3);
  const colors = new Float32Array(sampleCount * 3);
  const sampleQuaternions = new Float32Array(sampleCount * 4);
  const sampleJointValues = new Float32Array(sampleCount * jointNames.length);
  const eePos = new THREE.Vector3();
  const eeQuat = new THREE.Quaternion();
  const color = new THREE.Color();

  for (let i = 0; i < sampleCount; i++) {
    jointNames.forEach((name) => {
      const joint = robot.joints[name];
      const [lo, hi] = getJointSampleLimitRad(joint);
      const value = randomBetween(lo, hi);
      ikJointValues[name] = value;
      robot.setJointValue(name, value);
      sampleJointValues[i * jointNames.length + jointNames.indexOf(name)] = value;
    });

    robot.updateMatrixWorld(true);
    const worldPos = getEEWorldPos();
    if (!worldPos) continue;

    eePos.copy(worldPos);
    positions[3 * i + 0] = eePos.x;
    positions[3 * i + 1] = eePos.y;
    positions[3 * i + 2] = eePos.z;

    const worldQuat = getEEWorldQuaternion();
    if (worldQuat) {
      eeQuat.copy(worldQuat);
      sampleQuaternions[4 * i + 0] = eeQuat.x;
      sampleQuaternions[4 * i + 1] = eeQuat.y;
      sampleQuaternions[4 * i + 2] = eeQuat.z;
      sampleQuaternions[4 * i + 3] = eeQuat.w;
    }

    const hue = THREE.MathUtils.clamp(0.66 - (eePos.y + 0.8) * 0.15, 0.02, 0.7);
    color.setHSL(hue, 0.8, 0.56);
    colors[3 * i + 0] = color.r;
    colors[3 * i + 1] = color.g;
    colors[3 * i + 2] = color.b;
  }

  jointNames.forEach((name) => {
    ikJointValues[name] = originalValues[name];
    robot.setJointValue(name, originalValues[name]);
  });
  robot.updateMatrixWorld(true);
  syncSlidersFromIKValues();

  clearWorkspaceCloud();

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

  const material = new THREE.PointsMaterial({
    size: 0.009,
    vertexColors: true,
    transparent: true,
    opacity: 0.86,
    depthWrite: false,
  });

  workspacePoints = new THREE.Points(geometry, material);
  workspacePoints.userData.sampleQuaternions = sampleQuaternions;
  workspacePoints.userData.sampleJointValues = sampleJointValues;
  workspacePoints.userData.jointCount = jointNames.length;
  workspacePoints.visible = workspaceVisible;
  scene.add(workspacePoints);

  workspacePointCount = sampleCount;
  updateWorkspaceInfo();
}

function setIKStatus(error) {
  if (error === null || !ikStatusEl) return;
  if (error < 0.01) {
    ikStatusEl.textContent = `✓ ${(error * 1000).toFixed(1)} mm`;
    ikStatusEl.className = 'ik-status ok';
  } else {
    ikStatusEl.textContent = `${(error * 1000).toFixed(0)} mm off`;
    ikStatusEl.className = 'ik-status warn';
  }
}

function styleRobotMeshes(object) {
  let meshCount = 0;

  object.traverse((child) => {
    if (!child.isMesh) return;

    meshCount += 1;
    child.frustumCulled = false;
    child.castShadow = true;
    child.receiveShadow = true;

    if (child.geometry && !child.geometry.attributes.normal) {
      child.geometry.computeVertexNormals();
    }

    child.material = new THREE.MeshStandardMaterial({
      color: 0x5f7f93,
      roughness: 0.58,
      metalness: 0.08,
      side: THREE.DoubleSide,
    });
  });

  return meshCount;
}

function frameRobot(object) {
  object.updateMatrixWorld(true);

  const bounds = new THREE.Box3().setFromObject(object);
  if (bounds.isEmpty()) {
    setStatusText('Robot loaded, but no visible mesh was found');
    return;
  }

  const center = bounds.getCenter(new THREE.Vector3());
  const size = bounds.getSize(new THREE.Vector3());
  const maxDimension = Math.max(size.x, size.y, size.z);

  if (maxDimension === 0) {
    setStatusText('Robot loaded, but mesh bounds are zero');
    return;
  }

  const fitHeightDistance = maxDimension / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov * 0.5)));
  const fitWidthDistance = fitHeightDistance / camera.aspect;
  const distance = 1.35 * Math.max(fitHeightDistance, fitWidthDistance, maxDimension);
  const direction = new THREE.Vector3(1, 0.7, 1).normalize();

  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(direction, distance);
  camera.near = Math.max(distance / 100, 0.001);
  camera.far = Math.max(distance * 20, 10);
  camera.updateProjectionMatrix();
  controls.update();
}

function activateMode(mode) {
  const showIK = mode === 'ik';
  ikPanel.classList.toggle('hidden', !showIK);
  jointsPanel.classList.toggle('hidden', showIK);
  showIKBtn.classList.toggle('active', showIK);
  showJointsBtn.classList.toggle('active', !showIK);
}

function degreesToQuaternion(rollDeg, pitchDeg, yawDeg) {
  const euler = new THREE.Euler(
    THREE.MathUtils.degToRad(rollDeg),
    THREE.MathUtils.degToRad(pitchDeg),
    THREE.MathUtils.degToRad(yawDeg),
    'XYZ'
  );
  return new THREE.Quaternion().setFromEuler(euler);
}

function getTargetPoseFromInputs() {
  const targetPos = new THREE.Vector3(
    parseFloat(ikXInput.value) || 0,
    parseFloat(ikYInput.value) || 0,
    parseFloat(ikZInput.value) || 0
  );

  const targetQuat = degreesToQuaternion(
    parseFloat(ikRollInput.value) || 0,
    parseFloat(ikPitchInput.value) || 0,
    parseFloat(ikYawInput.value) || 0
  );

  return { targetPos, targetQuat };
}

function getJointLimitsDeg(jointName) {
  const joint = robot?.joints?.[jointName];
  if (!joint) return { min: -180, max: 180 };

  const lower = Number.isFinite(joint.limit?.lower) ? THREE.MathUtils.radToDeg(joint.limit.lower) : -180;
  const upper = Number.isFinite(joint.limit?.upper) ? THREE.MathUtils.radToDeg(joint.limit.upper) : 180;

  if (lower === upper) return { min: lower - 180, max: upper + 180 };

  return {
    min: Math.max(-360, Math.min(lower, upper)),
    max: Math.min(360, Math.max(lower, upper)),
  };
}

function rebuildJointControls() {
  jointSlidersContainer.innerHTML = '';
  sliderElements = [];

  if (jointNames.length === 0) {
    const info = document.createElement('p');
    info.textContent = 'No actuated joints found in this URDF.';
    info.style.margin = '0';
    info.style.fontSize = '0.85rem';
    info.style.color = '#52606d';
    jointSlidersContainer.appendChild(info);
    return;
  }

  jointNames.forEach((name) => {
    const { min, max } = getJointLimitsDeg(name);

    const row = document.createElement('label');
    row.className = 'slider-row';
    row.htmlFor = `joint-${name}`;

    const head = document.createElement('div');
    head.className = 'slider-head';

    const title = document.createElement('span');
    title.textContent = name;

    const output = document.createElement('output');
    output.setAttribute('for', `joint-${name}`);
    output.textContent = '0deg';

    const input = document.createElement('input');
    input.id = `joint-${name}`;
    input.type = 'range';
    input.min = String(Math.round(min));
    input.max = String(Math.round(max));
    input.step = '1';
    input.value = '0';
    input.dataset.joint = name;

    head.appendChild(title);
    head.appendChild(output);
    row.appendChild(head);
    row.appendChild(input);
    jointSlidersContainer.appendChild(row);

    sliderElements.push(input);

    updateOutput(input);
    input.addEventListener('input', () => {
      updateOutput(input);
      setRobotJointFromSlider(input);
    });
  });
}

function discoverActuatedJoints() {
  const joints = Object.values(robot?.joints ?? {});

  jointNames = joints
    .filter((joint) => joint.jointType === 'revolute' || joint.jointType === 'continuous')
    .map((joint) => joint.name)
    .sort((a, b) => a.localeCompare(b, undefined, { numeric: true }));

  jointNames.forEach((name) => {
    const joint = robot.joints[name];
    ikJointValues[name] = Number.isFinite(joint.angle) ? joint.angle : 0;
  });

  refreshNullspaceControls();
}

function setRobotJointFromSlider(slider) {
  if (!robot) return;

  const name = slider.dataset.joint;
  const radians = THREE.MathUtils.degToRad(Number(slider.value));

  if (name in ikJointValues) ikJointValues[name] = radians;
  robot.setJointValue(name, radians);
}

function syncSlidersFromIKValues() {
  sliderElements.forEach((slider) => {
    const name = slider.dataset.joint;
    if (!(name in ikJointValues)) return;

    const deg = Math.round(THREE.MathUtils.radToDeg(ikJointValues[name]));
    slider.value = String(Math.max(Number(slider.min), Math.min(Number(slider.max), deg)));
    updateOutput(slider);
  });
}

function getEEWorldPos() {
  if (!eeLink) return null;
  return eeLink.localToWorld(eeLocalOffset.clone());
}

function getEEWorldQuaternion() {
  if (!eeLink) return null;
  return eeLink.getWorldQuaternion(new THREE.Quaternion());
}

function updateOrientationTargetFromCurrent() {
  const q = getEEWorldQuaternion();
  if (q) eeOrientationTarget.copy(q);
}

function findEndEffectorLink() {
  const links = Object.values(robot?.links ?? {});
  if (links.length === 0) return null;

  const eePriorityNames = [
    'end_effector',
    'end_effector_link',
    'endeffector',
    'end-effector',
    'ee_link',
    'tool0',
    'tool',
    'tcp',
    'flange',
  ];

  const scoreLinkName = (name) => {
    const n = String(name || '').toLowerCase();
    if (!n) return 0;

    if (eePriorityNames.includes(n)) return 100;

    const tokens = n.split(/[^a-z0-9]+/).filter(Boolean);
    const tokenSet = new Set(tokens);

    if (tokenSet.has('end') && (tokenSet.has('effector') || tokenSet.has('eef') || tokenSet.has('ee'))) return 95;
    if (tokenSet.has('tool') || tokenSet.has('tcp') || tokenSet.has('flange')) return 85;
    if (n.includes('end_effector') || n.includes('end-effector') || n.includes('tool0')) return 90;

    return 0;
  };

  let bestNamed = null;
  let bestScore = 0;
  links.forEach((link) => {
    const score = scoreLinkName(link.name || link.urdfName || '');
    if (score > bestScore) {
      bestScore = score;
      bestNamed = link;
    }
  });

  if (bestNamed && bestScore >= 85) {
    return { link: bestNamed, fromUrdf: true };
  }

  const basePosWorld = robot.getWorldPosition(new THREE.Vector3());
  let chosen = links[0];
  let maxDistance = -1;

  links.forEach((link) => {
    const p = link.getWorldPosition(new THREE.Vector3());
    const d = p.distanceTo(basePosWorld);
    if (d > maxDistance) {
      maxDistance = d;
      chosen = link;
    }
  });

  return { link: chosen, fromUrdf: false };
}

function computeEETipOffset() {
  const eeSelection = findEndEffectorLink();
  eeLink = eeSelection?.link || null;
  if (!eeLink) return;

  robot.updateMatrixWorld(true);
  const bounds = new THREE.Box3().setFromObject(eeLink);
  if (bounds.isEmpty()) {
    eeLocalOffset.set(0, 0, 0);
    eeTipMarker.visible = true;
    return;
  }

  const boundsCenter = bounds.getCenter(new THREE.Vector3());
  eeLocalOffset.copy(eeLink.worldToLocal(boundsCenter.clone()));
  eeTipMarker.visible = true;
}

function resetAllJoints() {
  sliderElements.forEach((slider) => {
    const zero = Math.max(Number(slider.min), Math.min(Number(slider.max), 0));
    slider.value = String(Math.round(zero));
    updateOutput(slider);
    setRobotJointFromSlider(slider);
  });

  if (eeLink) {
    robot.updateMatrixWorld(true);
    const eePos = getEEWorldPos();
    if (eePos) {
      ikMarker.position.copy(eePos);
      ikMarker.visible = true;
      ikXInput.value = eePos.x.toFixed(3);
      ikYInput.value = eePos.y.toFixed(3);
      ikZInput.value = eePos.z.toFixed(3);
    }
    const eeQuat = getEEWorldQuaternion();
    if (eeQuat) {
      const targetEuler = new THREE.Euler().setFromQuaternion(eeQuat, 'XYZ');
      ikRollInput.value = THREE.MathUtils.radToDeg(targetEuler.x).toFixed(1);
      ikPitchInput.value = THREE.MathUtils.radToDeg(targetEuler.y).toFixed(1);
      ikYawInput.value = THREE.MathUtils.radToDeg(targetEuler.z).toFixed(1);
    }
    updateAllPoseOutputs();
  }

  clearIKStatus();
}

function zeros(rows, cols) {
  return new Array(rows).fill(null).map(() => new Array(cols).fill(0));
}

function identity(size) {
  const matrix = zeros(size, size);
  for (let i = 0; i < size; i++) matrix[i][i] = 1;
  return matrix;
}

function transpose(matrix) {
  const rows = matrix.length;
  const cols = matrix[0].length;
  const out = zeros(cols, rows);
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) out[c][r] = matrix[r][c];
  }
  return out;
}

function matMul(a, b) {
  const rows = a.length;
  const cols = b[0].length;
  const inner = b.length;
  const out = zeros(rows, cols);

  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      let sum = 0;
      for (let k = 0; k < inner; k++) sum += a[r][k] * b[k][c];
      out[r][c] = sum;
    }
  }

  return out;
}

function matVecMul(matrix, vec) {
  const out = new Array(matrix.length).fill(0);
  for (let r = 0; r < matrix.length; r++) {
    let sum = 0;
    for (let c = 0; c < vec.length; c++) sum += matrix[r][c] * vec[c];
    out[r] = sum;
  }
  return out;
}

function invertMatrix(matrix) {
  const n = matrix.length;
  const aug = zeros(n, n * 2);

  for (let r = 0; r < n; r++) {
    for (let c = 0; c < n; c++) aug[r][c] = matrix[r][c];
    for (let c = 0; c < n; c++) aug[r][c + n] = r === c ? 1 : 0;
  }

  for (let pivot = 0; pivot < n; pivot++) {
    let maxRow = pivot;
    let maxVal = Math.abs(aug[pivot][pivot]);
    for (let r = pivot + 1; r < n; r++) {
      const val = Math.abs(aug[r][pivot]);
      if (val > maxVal) {
        maxVal = val;
        maxRow = r;
      }
    }

    if (maxVal < 1e-12) return null;
    if (maxRow !== pivot) {
      const tmp = aug[pivot];
      aug[pivot] = aug[maxRow];
      aug[maxRow] = tmp;
    }

    const diag = aug[pivot][pivot];
    for (let c = 0; c < n * 2; c++) aug[pivot][c] /= diag;

    for (let r = 0; r < n; r++) {
      if (r === pivot) continue;
      const factor = aug[r][pivot];
      for (let c = 0; c < n * 2; c++) {
        aug[r][c] -= factor * aug[pivot][c];
      }
    }
  }

  const inv = zeros(n, n);
  for (let r = 0; r < n; r++) {
    for (let c = 0; c < n; c++) inv[r][c] = aug[r][c + n];
  }
  return inv;
}

function getJointAxisWorld(joint) {
  const q = new THREE.Quaternion();
  const p = new THREE.Vector3();
  const s = new THREE.Vector3();
  joint.matrixWorld.decompose(p, q, s);
  return joint.axis.clone().applyQuaternion(q).normalize();
}

function computeJacobianAtEE(eePos, orientationWeight = 0.55) {
  const J = zeros(6, jointNames.length);
  const jointPos = new THREE.Vector3();

  jointNames.forEach((name, idx) => {
    const joint = robot.joints[name];
    if (!joint) return;

    joint.getWorldPosition(jointPos);
    const axisWorld = getJointAxisWorld(joint);
    const r = eePos.clone().sub(jointPos);
    const linear = new THREE.Vector3().crossVectors(axisWorld, r);

    J[0][idx] = linear.x;
    J[1][idx] = linear.y;
    J[2][idx] = linear.z;
    J[3][idx] = axisWorld.x * orientationWeight;
    J[4][idx] = axisWorld.y * orientationWeight;
    J[5][idx] = axisWorld.z * orientationWeight;
  });

  return J;
}

function computeJointLimitAvoidance() {
  const z = new Array(jointNames.length).fill(0);

  jointNames.forEach((name, idx) => {
    const joint = robot.joints[name];
    const q = ikJointValues[name];
    const lo = Number.isFinite(joint.limit?.lower) ? joint.limit.lower : -Math.PI;
    const hi = Number.isFinite(joint.limit?.upper) ? joint.limit.upper : Math.PI;

    const mid = 0.5 * (lo + hi);
    const half = Math.max(0.5 * (hi - lo), 1e-3);
    z[idx] = 0.03 * (mid - q) / (half * half);
  });

  return z;
}

function applyJointStep(dq, maxStep = 0.14) {
  jointNames.forEach((name, idx) => {
    const joint = robot.joints[name];
    const lo = Number.isFinite(joint.limit?.lower) ? joint.limit.lower : -Math.PI;
    const hi = Number.isFinite(joint.limit?.upper) ? joint.limit.upper : Math.PI;

    const boundedStep = Math.max(-maxStep, Math.min(maxStep, dq[idx]));
    const next = Math.max(lo, Math.min(hi, (ikJointValues[name] ?? 0) + boundedStep));

    ikJointValues[name] = next;
    robot.setJointValue(name, next);
  });
}

function quaternionToRotationVector(errorQuat) {
  const q = errorQuat.clone();
  if (q.w < 0) {
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;
    q.w = -q.w;
  }

  const sinHalf = Math.sqrt(Math.max(1 - q.w * q.w, 0));
  if (sinHalf < 1e-6) {
    return new THREE.Vector3(2 * q.x, 2 * q.y, 2 * q.z);
  }

  const angle = 2 * Math.atan2(sinHalf, q.w);
  return new THREE.Vector3((q.x / sinHalf) * angle, (q.y / sinHalf) * angle, (q.z / sinHalf) * angle);
}

function solveIKPose(targetWorld, targetOrientation, maxIter = 180, posThreshold = 0.003, rotThreshold = 0.03, priorityMode = 'pose') {
  if (!robot || !eeLink || jointNames.length === 0) return null;

  const eePos = new THREE.Vector3();
  const orientationWeight = priorityMode === 'position' ? 0.0 : 0.55;

  for (let iter = 0; iter < maxIter; iter++) {
    robot.updateMatrixWorld(true);
    eePos.copy(getEEWorldPos());

    const posError = targetWorld.clone().sub(eePos);
    const qCurrent = getEEWorldQuaternion();
    const qError = targetOrientation.clone().multiply(qCurrent.invert());
    const rotVec = quaternionToRotationVector(qError).multiplyScalar(orientationWeight);
    const rotMag = Math.hypot(rotVec.x, rotVec.y, rotVec.z);

    const taskSatisfied = posError.length() < posThreshold && (priorityMode === 'position' || rotMag < rotThreshold);
    const nullspaceRequested = hasActiveNullspaceRequest();

    if (taskSatisfied && !nullspaceRequested) {
      break;
    }

    const errorTask = [posError.x * 0.9, posError.y * 0.9, posError.z * 0.9, rotVec.x, rotVec.y, rotVec.z];
    const J = computeJacobianAtEE(eePos, orientationWeight);
    const JT = transpose(J);
    const JJT = matMul(J, JT);

    const lambda = 0.08;
    const A = JJT.map((row, r) => row.map((value, c) => value + (r === c ? lambda * lambda : 0)));
    const AInv = invertMatrix(A);
    if (!AInv) break;

    const jSharp = matMul(JT, AInv);
    const dqPrimary = matVecMul(jSharp, errorTask);

    const jSharpJ = matMul(jSharp, J);
    const projector = identity(jointNames.length);
    for (let r = 0; r < jointNames.length; r++) {
      for (let c = 0; c < jointNames.length; c++) {
        projector[r][c] -= jSharpJ[r][c];
      }
    }

    const zLimit = computeJointLimitAvoidance();
    const zPosture = getNullspacePostureBias();
    const z = zLimit.map((value, idx) => value + zPosture[idx]);
    const dqNull = matVecMul(projector, z);

    const dq = new Array(jointNames.length).fill(0);
    for (let i = 0; i < jointNames.length; i++) {
      dq[i] = dqPrimary[i] + (hasActiveNullspaceRequest() ? 0.8 : 0.25) * dqNull[i];
    }

    applyJointStep(dq);
  }

  robot.updateMatrixWorld(true);
  return getEEWorldPos().distanceTo(targetWorld);
}

function solveFromPoseInputs() {
  if (!robot) return;

  const { targetPos, targetQuat } = getTargetPoseFromInputs();
  ikMarker.position.copy(targetPos);
  ikMarker.visible = true;

  setIKStatus(solveIKPose(targetPos, targetQuat, 180, 0.003, 0.03, ikPriorityMode));
  syncSlidersFromIKValues();
}

function solveNullspaceExploration() {
  if (!robot || !eeLink || jointNames.length === 0) return;

  // Lock target on first call if not already locked
  if (!nullspaceLockedPos || !nullspaceLockedQuat) lockNullspaceTarget();
  if (!nullspaceLockedPos || !nullspaceLockedQuat) return;

  ikMarker.position.copy(nullspaceLockedPos);
  ikMarker.visible = true;

  ikXInput.value = nullspaceLockedPos.x.toFixed(3);
  ikYInput.value = nullspaceLockedPos.y.toFixed(3);
  ikZInput.value = nullspaceLockedPos.z.toFixed(3);

  const e = new THREE.Euler().setFromQuaternion(nullspaceLockedQuat, 'XYZ');
  ikRollInput.value = THREE.MathUtils.radToDeg(e.x).toFixed(1);
  ikPitchInput.value = THREE.MathUtils.radToDeg(e.y).toFixed(1);
  ikYawInput.value = THREE.MathUtils.radToDeg(e.z).toFixed(1);

  updateAllPoseOutputs();
  setIKStatus(solveIKPose(nullspaceLockedPos, nullspaceLockedQuat, 180, 0.0008, 0.015, 'pose'));
  syncSlidersFromIKValues();
}

function clearUploadedMeshUrls() {
  uploadedMeshObjectUrls.splice(0).forEach((url) => URL.revokeObjectURL(url));
  uploadedMeshUrlMap.clear();
}

function indexUploadedMeshFile(file) {
  const objectUrl = URL.createObjectURL(file);
  uploadedMeshObjectUrls.push(objectUrl);

  const rel = normalizePath(file.webkitRelativePath || file.name).toLowerCase();
  const parts = rel.split('/');

  for (let i = 0; i < parts.length; i++) {
    const suffix = parts.slice(i).join('/');
    if (suffix) uploadedMeshUrlMap.set(suffix, objectUrl);
  }

  uploadedMeshUrlMap.set(file.name.toLowerCase(), objectUrl);
}

function indexUploadedMeshes(fileList) {
  clearUploadedMeshUrls();
  Array.from(fileList || []).forEach(indexUploadedMeshFile);
}

function resolveMeshPath(path) {
  const normalized = stripMeshUriPrefixes(path).toLowerCase();
  const parts = normalized.split('/').filter(Boolean);

  const candidates = new Set();
  candidates.add(normalized);
  candidates.add(parts.join('/'));
  if (parts.length > 1) candidates.add(parts.slice(1).join('/'));
  if (parts.length > 0) candidates.add(parts[parts.length - 1]);

  for (const candidate of candidates) {
    const resolved = uploadedMeshUrlMap.get(candidate);
    if (resolved) return resolved;
  }

  return path;
}

function indexUploadedMeshSources(fileLists) {
  clearUploadedMeshUrls();
  (fileLists || []).forEach((list) => {
    Array.from(list || []).forEach(indexUploadedMeshFile);
  });
}

function clearCurrentRobot() {
  clearWorkspaceCloud();
  setNullspacePlaying(false);
  nullspaceLockedPos = null;
  nullspaceLockedQuat = null;
  if (!robot) return;
  scene.remove(robot);
  robot = null;
  eeLink = null;
  eeTipMarker.visible = false;
  ikMarker.visible = false;
  if (ikEEDisplayEl) ikEEDisplayEl.textContent = 'EE x: --  y: --  z: --';
  jointNames = [];
  sliderElements = [];
  jointSlidersContainer.innerHTML = '';
  refreshNullspaceControls();
}

function finalizeRobotLoad(loadId) {
  if (loadId !== currentLoadId || !robot || !robotStructureReady || pendingMeshLoads !== 0) return;

  frameRobot(robot);
  computeEETipOffset();
  updateOrientationTargetFromCurrent();

  const eePos = getEEWorldPos();
  if (eePos) {
    ikMarker.position.copy(eePos);
    ikMarker.visible = true;

    ikXInput.value = eePos.x.toFixed(3);
    ikYInput.value = eePos.y.toFixed(3);
    ikZInput.value = eePos.z.toFixed(3);
  }

  const targetEuler = new THREE.Euler().setFromQuaternion(eeOrientationTarget.clone(), 'XYZ');
  ikRollInput.value = THREE.MathUtils.radToDeg(targetEuler.x).toFixed(1);
  ikPitchInput.value = THREE.MathUtils.radToDeg(targetEuler.y).toFixed(1);
  ikYawInput.value = THREE.MathUtils.radToDeg(targetEuler.z).toFixed(1);

  updateAllPoseOutputs();
  clearIKStatus();
  activateMode('joints');
  setStatusText(`Robot loaded (${jointNames.length} joints)`);
  updateUploadFileList(true);
  setUploadPanelCollapsed(true);
}

function beginRobotLoad() {
  currentLoadId += 1;
  pendingMeshLoads = 0;
  robotStructureReady = false;
  return currentLoadId;
}

function onRobotLoaded(loadedRobot, loadId = currentLoadId) {
  if (loadId !== currentLoadId) return;

  clearCurrentRobot();

  robot = loadedRobot;
  robot.rotation.x = -Math.PI / 2;
  robot.position.y = 0;
  scene.add(robot);

  const meshCount = styleRobotMeshes(robot);

  discoverActuatedJoints();
  rebuildJointControls();
  resetAllJoints();

  setStatusText(meshCount > 0 ? `Loading robot meshes... (${meshCount} mesh nodes)` : 'Loading robot meshes...');
  robotStructureReady = true;
  finalizeRobotLoad(loadId);
}

function loadRobotFromUrl(url) {
  const loadId = beginRobotLoad();
  setStatusText('Loading robot...');

  urdfLoader.load(
    url,
    (loadedRobot) => {
      onRobotLoaded(loadedRobot, loadId);
    },
    undefined,
    (error) => {
      console.error(error);
      setStatusText('Failed to load robot');
    }
  );
}

function loadRobotFromText(urdfText) {
  try {
    const loadId = beginRobotLoad();
    setStatusText('Loading uploaded robot...');
    const loadedRobot = urdfLoader.parse(urdfText, '');
    onRobotLoaded(loadedRobot, loadId);
  } catch (error) {
    console.error(error);
    setStatusText('Failed to parse uploaded URDF');
  }
}

urdfLoader.loadMeshCb = (path, _manager, done) => {
  const loadId = currentLoadId;
  const requestedPath = stripMeshUriPrefixes(path).toLowerCase();
  const resolvedPath = resolveMeshPath(path);

  let meshType = null;
  if (requestedPath.endsWith('.stl')) meshType = 'stl';
  else if (requestedPath.endsWith('.dae')) meshType = 'dae';
  else if (requestedPath.endsWith('.obj')) meshType = 'obj';

  pendingMeshLoads += 1;

  const finish = (object) => {
    if (loadId === currentLoadId) {
      pendingMeshLoads = Math.max(0, pendingMeshLoads - 1);
      finalizeRobotLoad(loadId);
    }
    done(object || new THREE.Object3D());
  };

  if (!meshType) {
    urdfLoader.defaultMeshLoader(resolvedPath, loadingManager, (obj) => {
      finish(obj);
    });
    return;
  }

  if (meshType === 'stl') {
    stlLoader.load(
      resolvedPath,
      (geometry) => {
        finish(new THREE.Mesh(geometry, new THREE.MeshStandardMaterial()));
      },
      undefined,
      (err) => {
        console.error(`Failed to load STL mesh: ${resolvedPath}`, err);
        finish(new THREE.Object3D());
      }
    );
  } else if (meshType === 'dae') {
    colladaLoader.load(
      resolvedPath,
      (dae) => {
        finish(dae?.scene || new THREE.Object3D());
      },
      undefined,
      (err) => {
        console.error(`Failed to load DAE mesh: ${resolvedPath}`, err);
        finish(new THREE.Object3D());
      }
    );
  } else {
    objLoader.load(
      resolvedPath,
      (obj) => {
        finish(obj || new THREE.Object3D());
      },
      undefined,
      (err) => {
        console.error(`Failed to load OBJ mesh: ${resolvedPath}`, err);
        finish(new THREE.Object3D());
      }
    );
  }
};

loadingManager.onError = (url) => {
  console.error(`Failed to load asset: ${url}`);
  setStatusText('Failed to load one or more mesh assets');
};

function getMouseNDC(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  return new THREE.Vector2(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1
  );
}

renderer.domElement.addEventListener('pointerdown', (event) => {
  _dragRaycaster.setFromCamera(getMouseNDC(event), camera);

  if (eePickMode && robot) {
    const meshes = [];
    robot.traverse((c) => { if (c.isMesh) meshes.push(c); });

    const hits = _dragRaycaster.intersectObjects(meshes, false);
    if (hits.length > 0 && eeLink) {
      const hit = hits[0].point;
      eeLocalOffset.copy(eeLink.worldToLocal(hit.clone()));

      ikMarker.position.copy(hit);
      ikMarker.visible = true;
      ikXInput.value = hit.x.toFixed(3);
      ikYInput.value = hit.y.toFixed(3);
      ikZInput.value = hit.z.toFixed(3);

      updateAllPoseOutputs();
      clearIKStatus();
    }

    eePickMode = false;
    ikPickEEBtn.classList.remove('active');
    renderer.domElement.style.cursor = '';
    return;
  }

  const markerHit = ikMarker.visible && _dragRaycaster.intersectObject(ikMarker, false).length > 0;
  if (markerHit) {
    const camDir = new THREE.Vector3();
    camera.getWorldDirection(camDir);
    _dragPlane.setFromNormalAndCoplanarPoint(camDir, ikMarker.position);

    isDragging = true;
    controls.enabled = false;
    renderer.domElement.setPointerCapture?.(event.pointerId);
    renderer.domElement.style.cursor = 'grabbing';
    return;
  }

  if (workspacePoints && workspaceVisible) {
    _dragRaycaster.params.Points.threshold = WORKSPACE_PICK_THRESHOLD;
    const pointHits = _dragRaycaster.intersectObject(workspacePoints, false);
    if (pointHits.length > 0 && pointHits[0].distanceToRay <= WORKSPACE_PICK_THRESHOLD) {
      const hit = pointHits[0];
      const index = Number.isInteger(hit.index) ? hit.index : -1;
      if (index < 0) return;

      const positionAttr = workspacePoints.geometry.getAttribute('position');
      const hitPoint = new THREE.Vector3().fromBufferAttribute(positionAttr, index).applyMatrix4(workspacePoints.matrixWorld);

      const sampleJoint = workspacePoints.userData.sampleJointValues;
      const sampleJointCount = workspacePoints.userData.jointCount || 0;
      if (sampleJoint && sampleJointCount === jointNames.length) {
        jointNames.forEach((name, jointIdx) => {
          const value = sampleJoint[index * sampleJointCount + jointIdx];
          ikJointValues[name] = value;
          robot.setJointValue(name, value);
        });
        robot.updateMatrixWorld(true);
        syncSlidersFromIKValues();
      }

      const sampleQuat = workspacePoints.userData.sampleQuaternions;
      if (sampleQuat) {
        const q = new THREE.Quaternion(
          sampleQuat[index * 4 + 0],
          sampleQuat[index * 4 + 1],
          sampleQuat[index * 4 + 2],
          sampleQuat[index * 4 + 3]
        );
        const e = new THREE.Euler().setFromQuaternion(q, 'XYZ');
        ikRollInput.value = THREE.MathUtils.radToDeg(e.x).toFixed(1);
        ikPitchInput.value = THREE.MathUtils.radToDeg(e.y).toFixed(1);
        ikYawInput.value = THREE.MathUtils.radToDeg(e.z).toFixed(1);
      }

      ikXInput.value = hitPoint.x.toFixed(3);
      ikYInput.value = hitPoint.y.toFixed(3);
      ikZInput.value = hitPoint.z.toFixed(3);
      updateAllPoseOutputs();
      activateMode('ik');
      solveFromPoseInputs();
      return;
    }
  }
});

renderer.domElement.addEventListener('pointermove', (event) => {
  _dragRaycaster.setFromCamera(getMouseNDC(event), camera);

  if (isDragging) {
    if (_dragRaycaster.ray.intersectPlane(_dragPlane, _dragIntersect)) {
      ikMarker.position.copy(_dragIntersect);
      ikXInput.value = _dragIntersect.x.toFixed(3);
      ikYInput.value = _dragIntersect.y.toFixed(3);
      ikZInput.value = _dragIntersect.z.toFixed(3);
      updateAllPoseOutputs();
      solveFromPoseInputs();
    }
    return;
  }

  if (eePickMode && robot) {
    const meshes = [];
    robot.traverse((c) => { if (c.isMesh) meshes.push(c); });
    const hits = _dragRaycaster.intersectObjects(meshes, false);
    renderer.domElement.style.cursor = hits.length > 0 ? 'crosshair' : 'default';
    return;
  }

  const markerHits = ikMarker.visible ? _dragRaycaster.intersectObject(ikMarker, false) : [];
  if (markerHits.length > 0) {
    renderer.domElement.style.cursor = 'grab';
    return;
  }

  if (workspacePoints && workspaceVisible) {
    _dragRaycaster.params.Points.threshold = WORKSPACE_PICK_THRESHOLD;
    const pointHits = _dragRaycaster.intersectObject(workspacePoints, false);
    renderer.domElement.style.cursor = (pointHits.length > 0 && pointHits[0].distanceToRay <= WORKSPACE_PICK_THRESHOLD) ? 'pointer' : '';
    return;
  }

  renderer.domElement.style.cursor = '';
});

renderer.domElement.addEventListener('pointerup', (event) => {
  if (!isDragging) return;
  isDragging = false;
  controls.enabled = true;
  renderer.domElement.releasePointerCapture?.(event.pointerId);
  renderer.domElement.style.cursor = '';
});

renderer.domElement.addEventListener('pointercancel', (event) => {
  if (!isDragging) return;
  isDragging = false;
  controls.enabled = true;
  renderer.domElement.releasePointerCapture?.(event.pointerId);
  renderer.domElement.style.cursor = '';
});

ikPickEEBtn?.addEventListener('click', () => {
  eePickMode = !eePickMode;
  ikPickEEBtn.classList.toggle('active', eePickMode);
  if (!eePickMode) renderer.domElement.style.cursor = '';
});

ikModePositionBtn?.addEventListener('click', () => setIKPriorityMode('position'));
ikModePoseBtn?.addEventListener('click', () => setIKPriorityMode('pose'));

nullspaceEnableInput?.addEventListener('change', () => {
  if (!robot) return;
  if (nullspaceEnableInput.checked) {
    lockNullspaceTarget();
    solveNullspaceExploration();
  } else {
    setNullspacePlaying(false);
    nullspaceLockedPos = null;
    nullspaceLockedQuat = null;
    solveFromPoseInputs();
  }
});

nullspaceBiasInput?.addEventListener('input', () => {
  nullspaceBiasValue = Number(nullspaceBiasInput.value || 0) / 100;
  updateNullspaceBiasOutput();
  if (robot && nullspaceEnableInput?.checked) solveNullspaceExploration();
});

nullspacePlayBtn?.addEventListener('click', () => {
  const shouldPlay = nullspacePlayTimerId === null;
  setNullspacePlaying(shouldPlay);
});

ikSolveBtn?.addEventListener('click', () => {
  solveFromPoseInputs();
});

poseSliderElements.forEach((slider) => {
  updatePoseOutput(slider);
  slider.addEventListener('input', () => {
    updatePoseOutput(slider);
    solveFromPoseInputs();
  });
});

resetJointsBtn?.addEventListener('click', () => {
  resetAllJoints();
});

workspaceGenerateBtn?.addEventListener('click', () => {
  generateWorkspaceCloud();
});

workspaceClearBtn?.addEventListener('click', () => {
  clearWorkspaceCloud();
});

workspaceToggleBtn?.addEventListener('click', () => {
  setWorkspaceVisible(!workspaceVisible);
});

showIKBtn?.addEventListener('click', () => activateMode('ik'));
showJointsBtn?.addEventListener('click', () => activateMode('joints'));

function setUploadPanelCollapsed(collapsed) {
  uploadPanelBody?.classList.toggle('collapsed', collapsed);
  if (uploadCollapseBtn) uploadCollapseBtn.textContent = collapsed ? '▼' : '▲';
}

function updateUploadFileList(loaded) {
  if (!uploadFileList) return;
  uploadFileList.innerHTML = '';

  const urdfFile = urdfFileInput?.files?.[0];
  const meshFiles = [
    ...Array.from(meshFilesInput?.files || []),
    ...Array.from(meshFolderInput?.files || []),
  ];

  if (!urdfFile && meshFiles.length === 0) return;

  if (urdfFile) {
    const urdfRow = document.createElement('div');
    urdfRow.className = 'ufl-urdf-row';
    const nameEl = document.createElement('span');
    nameEl.className = 'ufl-filename';
    nameEl.title = urdfFile.name;
    nameEl.textContent = urdfFile.name;
    urdfRow.appendChild(nameEl);
    if (loaded) {
      const badge = document.createElement('span');
      badge.className = 'ufl-loaded-badge';
      badge.textContent = '✓ loaded';
      urdfRow.appendChild(badge);
    }
    uploadFileList.appendChild(urdfRow);
  }

  if (meshFiles.length > 0) {
    const meshTitle = document.createElement('div');
    meshTitle.className = 'ufl-section-title';
    meshTitle.textContent = `Meshes (${meshFiles.length} files)`;
    uploadFileList.appendChild(meshTitle);

    const meshNamesDiv = document.createElement('div');
    meshNamesDiv.className = 'ufl-mesh-names';

    const MAX_SHOWN = 20;
    meshFiles.slice(0, MAX_SHOWN).forEach((f) => {
      const row = document.createElement('div');
      row.className = 'ufl-filename';
      const relativePath = f.webkitRelativePath || f.name;
      row.title = relativePath;

      const nameDiv = document.createElement('div');
      nameDiv.textContent = f.name;
      row.appendChild(nameDiv);

      if (relativePath !== f.name) {
        const pathDiv = document.createElement('div');
        pathDiv.className = 'ufl-path';
        pathDiv.textContent = relativePath;
        row.appendChild(pathDiv);
      }

      meshNamesDiv.appendChild(row);
    });

    if (meshFiles.length > MAX_SHOWN) {
      const more = document.createElement('div');
      more.style.cssText = 'color:#8a99a6;font-style:italic';
      more.textContent = `… and ${meshFiles.length - MAX_SHOWN} more`;
      meshNamesDiv.appendChild(more);
    }

    uploadFileList.appendChild(meshNamesDiv);
  }
}

uploadPanelHeader?.addEventListener('click', () => {
  setUploadPanelCollapsed(!uploadPanelBody?.classList.contains('collapsed'));
});

urdfFileInput?.addEventListener('change', () => updateUploadFileList(false));
meshFilesInput?.addEventListener('change', () => updateUploadFileList(false));
meshFolderInput?.addEventListener('change', () => updateUploadFileList(false));

loadUploadBtn?.addEventListener('click', () => {
  const urdfFile = urdfFileInput?.files?.[0];
  if (!urdfFile) {
    setStatusText('Select a URDF file first');
    return;
  }

  indexUploadedMeshSources([
    meshFilesInput?.files,
    meshFolderInput?.files,
  ]);

  urdfFile.text()
    .then((text) => {
      loadRobotFromText(text);
    })
    .catch((error) => {
      console.error(error);
      setStatusText('Failed to read uploaded URDF');
    });
});

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

window.addEventListener('resize', onWindowResize);

function render() {
  requestAnimationFrame(render);
  controls.update();
  renderer.render(scene, camera);

  if (robot && eeLink && ikEEDisplayEl) {
    const eePos = getEEWorldPos();
    if (eePos) {
      eeTipMarker.position.copy(eePos);
      ikEEDisplayEl.textContent =
        `EE  x: ${eePos.x.toFixed(3)}  y: ${eePos.y.toFixed(3)}  z: ${eePos.z.toFixed(3)}`;
    }
  }
}

activateMode('joints');
setIKPriorityMode('pose');
updateNullspaceBiasOutput();
refreshNullspaceControls();
updateWorkspaceInfo();
setWorkspaceVisible(true);
updateAllPoseOutputs();
setStatusText('No robot loaded. Upload URDF and meshes to start.');
render();
