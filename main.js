/*
 * ViewURDF prismatic-joint compatibility entry point.
 *
 * To preserve the existing viewer logic exactly, this entry point loads the
 * original main.js from a pinned public commit, applies the local Arm Controls
 * patch, and imports the patched module. The main website already uses CDN
 * modules; this additionally requires raw.githubusercontent.com to be reachable.
 *
 * Once you validate the behavior, these changes can be merged directly into
 * the repository's main.js to remove this bootstrap fetch.
 */
const SOURCE_URL =
  'https://raw.githubusercontent.com/D-Sampath07/viewurdf/3644205a8b8cb74b471185532dfbf447689b457b/main.js';

const statusEl = document.querySelector('#status');

function setBootstrapStatus(message) {
  if (statusEl) statusEl.textContent = message;
}

function replaceOnce(source, needle, replacement, label) {
  const first = source.indexOf(needle);
  if (first === -1 || source.indexOf(needle, first + needle.length) !== -1) {
    throw new Error('Could not uniquely find source section: ' + label);
  }
  return source.slice(0, first) + replacement + source.slice(first + needle.length);
}

function replaceBetween(source, startMarker, endMarker, replacement, label) {
  const start = source.indexOf(startMarker);
  const end = source.indexOf(endMarker, start + startMarker.length);
  if (
    start === -1 ||
    end === -1 ||
    source.indexOf(startMarker, start + startMarker.length) !== -1 ||
    source.indexOf(endMarker, end + endMarker.length) !== -1 ||
    end <= start
  ) {
    throw new Error('Could not uniquely find source section: ' + label);
  }
  return source.slice(0, start) + replacement + source.slice(end);
}

function patchOriginalSource(source) {
  source = replaceOnce(
    source,
    'let sliderElements = [];\nlet jointNames = [];',
    'let sliderElements = [];\nlet jointNames = [];\nlet controlJointNames = [];',
    'joint list declarations'
  );

  source = replaceBetween(
    source,
    'function updateOutput(slider)',
    'function updatePoseOutput(slider)',
    `function updateOutput(slider) {
  const output = slider.parentElement.querySelector('output');
  if (!output) return;

  const joint = robot?.joints?.[slider.dataset.joint];
  const value = Number(slider.value);
  const text = joint?.jointType === 'prismatic'
    ? value.toFixed(4) + 'm'
    : Math.round(value) + 'deg';

  output.value = text;
  output.textContent = text;
}

`,
    'joint slider output'
  );

  source = replaceBetween(
    source,
    'function getJointLimitsDeg(jointName)',
    'function getEEWorldPos()',
    `function getJointSliderLimits(jointName) {
  const joint = robot?.joints?.[jointName];
  if (!joint) return { min: -180, max: 180, step: 1, unit: 'deg' };

  if (joint.jointType === 'prismatic') {
    // Prismatic URDF limits are linear distances in meters.
    const lower = Number.isFinite(joint.limit?.lower) ? joint.limit.lower : -0.1;
    const upper = Number.isFinite(joint.limit?.upper) ? joint.limit.upper : 0.1;
    const min = Math.min(lower, upper);
    const max = Math.max(lower, upper);
    const step = max > min ? Math.max((max - min) / 1000, 0.0001) : 0.001;
    return { min, max, step, unit: 'm' };
  }

  // Keep revolute and continuous controls degree-based.
  const lower = Number.isFinite(joint.limit?.lower) ? THREE.MathUtils.radToDeg(joint.limit.lower) : -180;
  const upper = Number.isFinite(joint.limit?.upper) ? THREE.MathUtils.radToDeg(joint.limit.upper) : 180;
  if (lower === upper) return { min: lower - 180, max: upper + 180, step: 1, unit: 'deg' };

  return {
    min: Math.max(-360, Math.min(lower, upper)),
    max: Math.min(360, Math.max(lower, upper)),
    step: 1,
    unit: 'deg',
  };
}

function rebuildJointControls() {
  jointSlidersContainer.innerHTML = '';
  sliderElements = [];

  if (controlJointNames.length === 0) {
    const info = document.createElement('p');
    info.textContent = 'No actuated joints found in this URDF.';
    info.style.margin = '0';
    info.style.fontSize = '0.85rem';
    info.style.color = '#52606d';
    jointSlidersContainer.appendChild(info);
    return;
  }

  controlJointNames.forEach((name) => {
    const { min, max, step, unit } = getJointSliderLimits(name);
    const joint = robot?.joints?.[name];
    const isPrismatic = joint?.jointType === 'prismatic';
    const currentValue = Number.isFinite(ikJointValues[name])
      ? ikJointValues[name]
      : (Number.isFinite(joint?.angle) ? joint.angle : 0);
    const sliderValue = isPrismatic ? currentValue : THREE.MathUtils.radToDeg(currentValue);

    const row = document.createElement('label');
    row.className = 'slider-row';
    row.htmlFor = 'joint-' + name;

    const head = document.createElement('div');
    head.className = 'slider-head';

    const title = document.createElement('span');
    title.textContent = name;

    const output = document.createElement('output');
    output.setAttribute('for', 'joint-' + name);
    output.textContent = isPrismatic
      ? currentValue.toFixed(4) + 'm'
      : Math.round(sliderValue) + 'deg';

    const input = document.createElement('input');
    input.id = 'joint-' + name;
    input.type = 'range';
    input.min = String(min);
    input.max = String(max);
    input.step = String(step);
    input.value = String(Math.max(min, Math.min(max, sliderValue)));
    input.dataset.joint = name;
    input.dataset.unit = unit;

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
  const sorted = (items) => items
    .map((joint) => joint.name)
    .sort((a, b) => a.localeCompare(b, undefined, { numeric: true }));

  // The existing approximate IK solver remains rotational-joint-only.
  jointNames = sorted(joints.filter(
    (joint) => joint.jointType === 'revolute' || joint.jointType === 'continuous'
  ));

  // Arm Controls adds prismatic joints, but avoids separate controls for
  // mimic followers so a master gripper joint can drive its coupled fingers.
  controlJointNames = sorted(joints.filter(
    (joint) => !joint.mimicJoint &&
      ['revolute', 'continuous', 'prismatic'].includes(joint.jointType)
  ));

  controlJointNames.forEach((name) => {
    const joint = robot.joints[name];
    ikJointValues[name] = Number.isFinite(joint.angle) ? joint.angle : 0;
  });

  refreshNullspaceControls();
}

function setRobotJointFromSlider(slider) {
  if (!robot) return;

  const name = slider.dataset.joint;
  const joint = robot.joints?.[name];
  if (!joint) return;

  const value = joint.jointType === 'prismatic'
    ? Number(slider.value)
    : THREE.MathUtils.degToRad(Number(slider.value));

  if (name in ikJointValues) ikJointValues[name] = value;
  robot.setJointValue(name, value);
  robot.updateMatrixWorld(true);
  clearIKStatus();
}

function syncSlidersFromIKValues() {
  sliderElements.forEach((slider) => {
    const name = slider.dataset.joint;
    if (!(name in ikJointValues)) return;

    const joint = robot?.joints?.[name];
    const rawValue = ikJointValues[name];
    const value = joint?.jointType === 'prismatic'
      ? rawValue
      : THREE.MathUtils.radToDeg(rawValue);

    slider.value = String(Math.max(Number(slider.min), Math.min(Number(slider.max), value)));
    updateOutput(slider);
  });
}

`,
    'joint control builder and value conversion'
  );

  source = replaceOnce(
    source,
    '  jointNames = [];\n  sliderElements = [];',
    '  jointNames = [];\n  controlJointNames = [];\n  sliderElements = [];',
    'robot reset state'
  );

  source = replaceOnce(
    source,
    '  setStatusText(`Robot loaded (${jointNames.length} joints)`);',
    "  setStatusText('Robot loaded (' + controlJointNames.length + ' controllable joints)');",
    'robot loaded status'
  );

  return source;
}

(async () => {
  try {
    setBootstrapStatus('Loading viewer…');
    const response = await fetch(SOURCE_URL, { mode: 'cors', cache: 'force-cache' });
    if (!response.ok) {
      throw new Error('Could not fetch original main.js (HTTP ' + response.status + ')');
    }

    const originalSource = await response.text();
    const patchedSource = patchOriginalSource(originalSource);

    const sourceUrl = URL.createObjectURL(
      new Blob([patchedSource], { type: 'text/javascript' })
    );

    try {
      await import(sourceUrl);
    } finally {
      URL.revokeObjectURL(sourceUrl);
    }
  } catch (error) {
    console.error('ViewURDF prismatic-joint patch failed:', error);
    setBootstrapStatus('Viewer startup failed: ' + (error?.message || String(error)));
  }
})();
