import RobotWindow from 'https://cyberbotics.com/wwi/R2025a/RobotWindow.js';

const elements = {
  connection: document.getElementById('connection-status'),
  message: document.getElementById('message-input'),
  byteCounter: document.getElementById('byte-counter'),
  actionDuration: document.getElementById('action-duration'),
  idleDuration: document.getElementById('idle-duration'),
  destinationX: document.getElementById('destination-x'),
  destinationY: document.getElementById('destination-y'),
  routePreview: document.getElementById('route-preview'),
  resetTiming: document.getElementById('reset-timing-button'),
  start: document.getElementById('start-button'),
  stop: document.getElementById('stop-button'),
  inputError: document.getElementById('input-error'),
  executionTitle: document.getElementById('execution-title'),
  currentAction: document.getElementById('current-action'),
  overallProgress: document.getElementById('overall-progress'),
  slotStatus: document.getElementById('slot-status'),
  timeStatus: document.getElementById('time-status'),
  metricBytes: document.getElementById('metric-bytes'),
  metricFrames: document.getElementById('metric-frames'),
  metricActions: document.getElementById('metric-actions'),
  metricDuration: document.getElementById('metric-duration'),
  metricRouteSpeed: document.getElementById('metric-route-speed'),
  metricCrossTrack: document.getElementById('metric-cross-track'),
  emptyState: document.getElementById('empty-state'),
  frameList: document.getElementById('frame-list'),
};

const state = {
  connected: false,
  controllerSupportsIdle: true,
  supportsRoute: false,
  running: false,
  maxBytes: 240,
  defaultActionMs: 500,
  minActionMs: 100,
  maxActionMs: 5000,
  defaultIdleMs: 500,
  minIdleMs: 0,
  maxIdleMs: 5000,
  summary: null,
  frames: [],
  encodeTimer: null,
};

const utf8Encoder = new TextEncoder();
const robotWindow = new RobotWindow();
window.robotWindow = robotWindow;
robotWindow.setTitle('UAV Motion Encoder');

function utf8Bytes() {
  return utf8Encoder.encode(elements.message.value);
}

function bytesToHex(bytes) {
  return Array.from(bytes, byte => byte.toString(16).padStart(2, '0')).join('').toUpperCase();
}

function setConnection(label, connectionState) {
  elements.connection.dataset.state = connectionState;
  elements.connection.lastElementChild.textContent = label;
}

function showError(message) {
  elements.inputError.textContent = message;
  elements.inputError.hidden = !message;
}

function validateInput() {
  const bytes = utf8Bytes();
  const actionMs = Number(elements.actionDuration.value);
  const idleMs = Number(elements.idleDuration.value);
  const destinationX = Number(elements.destinationX.value);
  const destinationY = Number(elements.destinationY.value);
  const overLimit = bytes.length > state.maxBytes;
  elements.byteCounter.textContent = `${bytes.length} / ${state.maxBytes} bytes`;
  elements.byteCounter.dataset.overLimit = String(overLimit);

  let error = '';
  if (bytes.length === 0)
    error = 'Enter at least one character.';
  else if (state.connected && !state.controllerSupportsIdle)
    error = 'Restart the Webots simulation to load the updated action/idle timing controller.';
  else if (overLimit)
    error = `The message is ${bytes.length} UTF-8 bytes and exceeds the ${state.maxBytes}-byte limit.`;
  else if (!Number.isInteger(actionMs) || actionMs < state.minActionMs || actionMs > state.maxActionMs)
    error = `The action duration must be between ${state.minActionMs} and ${state.maxActionMs} ms.`;
  else if (!Number.isInteger(idleMs) || idleMs < state.minIdleMs || idleMs > state.maxIdleMs)
    error = `The idle duration must be between ${state.minIdleMs} and ${state.maxIdleMs} ms.`;
  else if (state.connected && !state.supportsRoute)
    error = 'Restart the Webots simulation to load the A-to-B centerline controller.';
  else if (!Number.isFinite(destinationX) || !Number.isFinite(destinationY))
    error = 'Destination X and Y must be finite numbers.';

  showError(error);
  elements.start.disabled = Boolean(error) || !state.connected || state.running;
  return error ? null : {bytes, actionMs, idleMs, destinationX, destinationY};
}

function requestEncoding(command = 'ENCODE') {
  if (!state.connected || state.running)
    return;
  const input = validateInput();
  if (!input) {
    if (utf8Bytes().length === 0)
      clearEncoding();
    return;
  }
  const routeCommand = command === 'START' ? 'START_ROUTE' : 'ENCODE_ROUTE';
  robotWindow.send(`${routeCommand}|${input.actionMs}|${input.idleMs}|${input.destinationX}|${input.destinationY}|${bytesToHex(input.bytes)}`);
  if (command === 'START')
    elements.start.disabled = true;
}

function scheduleEncoding() {
  validateInput();
  window.clearTimeout(state.encodeTimer);
  state.encodeTimer = window.setTimeout(() => requestEncoding(), 220);
}

function clearEncoding() {
  state.summary = null;
  state.frames = [];
  elements.frameList.replaceChildren();
  elements.emptyState.hidden = false;
  elements.metricBytes.textContent = '—';
  elements.metricFrames.textContent = '—';
  elements.metricActions.textContent = '—';
  elements.metricDuration.textContent = '—';
  elements.metricRouteSpeed.textContent = '—';
  elements.metricCrossTrack.textContent = '—';
  elements.routePreview.textContent = 'A is captured when the sequence starts.';
}

function createCodeRow(label, bitCount, value) {
  const row = document.createElement('div');
  row.className = 'code-row';

  const heading = document.createElement('div');
  heading.className = 'code-label';
  const name = document.createElement('span');
  name.textContent = label;
  const count = document.createElement('span');
  count.textContent = bitCount;
  heading.append(name, count);

  const code = document.createElement('code');
  code.className = 'code-value';
  code.textContent = value;
  row.append(heading, code);
  return row;
}

function renderFrames() {
  elements.frameList.replaceChildren();
  elements.emptyState.hidden = state.frames.length > 0;

  state.frames.forEach((frame, frameIndex) => {
    const details = document.createElement('details');
    details.className = 'frame-card';
    details.dataset.frameIndex = String(frameIndex);
    details.open = frameIndex === 0;

    const summary = document.createElement('summary');
    const title = document.createElement('span');
    title.className = 'frame-name';
    title.textContent = `Frame ${frame.index + 1}`;
    const meta = document.createElement('span');
    meta.className = 'frame-summary-meta';
    meta.textContent = `SEQ ${frame.seq.toString(2).padStart(2, '0')} · LEN ${frame.length} · ${frame.payloadHex || '∅'}`;
    summary.append(title, meta);

    const content = document.createElement('div');
    content.className = 'frame-content';
    content.append(
      createCodeRow('Information block · LEN + SEQ + DATA', '45 bits', frame.informationBits),
      createCodeRow('BCH parity', '18 bits', frame.parityBits),
      createCodeRow('Systematic codeword', '63 bits', frame.codewordBits),
      createCodeRow('Transmitted bits · trailing zero pad', '64 bits', frame.paddedBits),
    );

    const actionRow = document.createElement('div');
    actionRow.className = 'code-row';
    const actionHeading = document.createElement('div');
    actionHeading.className = 'code-label';
    actionHeading.innerHTML = '<span>UAV actions</span><span>40 slots</span>';
    const actionSequence = document.createElement('div');
    actionSequence.className = 'action-sequence';
    Array.from(frame.actions).forEach((action, slotIndex) => {
      const chip = document.createElement('span');
      chip.className = `action-chip${slotIndex < 8 ? ' sync' : ''}`;
      chip.dataset.slotIndex = String(slotIndex);
      chip.textContent = action;
      chip.title = `Slot ${slotIndex} · ${slotIndex < 8 ? 'SYNC' : 'DATA'}`;
      actionSequence.append(chip);
    });
    actionRow.append(actionHeading, actionSequence);
    content.append(actionRow);

    details.append(summary, content);
    elements.frameList.append(details);
  });
}

function renderSummary() {
  if (!state.summary)
    return;
  elements.metricBytes.textContent = state.summary.byteLength;
  elements.metricFrames.textContent = state.summary.frameCount;
  elements.metricActions.textContent = state.summary.totalActions;
  elements.metricDuration.textContent = `${state.summary.durationSeconds.toFixed(1)} s`;
  elements.metricRouteSpeed.textContent = Number.isFinite(state.summary.nominalSpeedMetersPerSec)
    ? `${state.summary.nominalSpeedMetersPerSec.toFixed(3)} m/s` : '—';
  if (Number.isFinite(state.summary.routeStartX)) {
    elements.routePreview.textContent = `A (${state.summary.routeStartX.toFixed(2)}, ${state.summary.routeStartY.toFixed(2)}) → B (${state.summary.destinationX.toFixed(2)}, ${state.summary.destinationY.toFixed(2)}) · ${state.summary.routeDistanceMeters.toFixed(2)} m`;
  }
  elements.timeStatus.textContent = `0.0 / ${state.summary.durationSeconds.toFixed(1)} s`;
}

function setRunning(running) {
  state.running = running;
  elements.message.disabled = running;
  elements.actionDuration.disabled = running;
  elements.idleDuration.disabled = running;
  elements.destinationX.disabled = running;
  elements.destinationY.disabled = running;
  elements.resetTiming.disabled = running;
  elements.stop.disabled = !running;
  elements.start.disabled = running || !validateInput();
  setConnection(running ? 'Sequence running' : 'Controller connected', running ? 'running' : 'connected');
}

function resetActiveAction() {
  document.querySelectorAll('.action-chip.active').forEach(chip => chip.classList.remove('active'));
}

function updateStatus(message) {
  const completedSlots = message.globalSlot + message.slotProgress;
  const progress = message.totalSlots > 0 ? completedSlots / message.totalSlots : 0;
  const phaseLabel = message.phase === 'return' ? 'Return to centerline' :
    (message.phase === 'align' ? 'Aligning to route' : message.phase === 'settle' ? 'Settling at B' : 'Action');
  elements.executionTitle.textContent = `Frame ${message.frameIndex + 1} · Slot ${message.slotIndex + 1} · ${phaseLabel}`;
  elements.currentAction.dataset.action = message.action;
  elements.currentAction.textContent = `${message.action} · ${message.actionName}`;
  elements.overallProgress.style.width = `${Math.min(100, Math.max(0, progress * 100))}%`;
  elements.slotStatus.textContent = `Action ${message.globalSlot + 1} / ${message.totalSlots} · ${phaseLabel}`;
  elements.timeStatus.textContent = `${message.elapsedSeconds.toFixed(1)} / ${message.totalSeconds.toFixed(1)} s`;
  if (Number.isFinite(message.crossTrackMeters))
    elements.metricCrossTrack.textContent = `${message.crossTrackMeters.toFixed(2)} m`;
  if (Number.isFinite(message.nominalSpeedMetersPerSec))
    elements.metricRouteSpeed.textContent = `${message.nominalSpeedMetersPerSec.toFixed(3)} m/s`;

  resetActiveAction();
  const frameCard = elements.frameList.querySelector(`[data-frame-index="${message.frameIndex}"]`);
  const chip = frameCard?.querySelector(`[data-slot-index="${message.slotIndex}"]`);
  if (chip)
    chip.classList.add('active');
}

function finishExecution(stateName, reason) {
  setRunning(false);
  resetActiveAction();
  elements.currentAction.dataset.action = '-';
  elements.currentAction.textContent = '— · Position hold';

  if (stateName === 'complete') {
    elements.executionTitle.textContent = 'Sequence complete';
    elements.overallProgress.style.width = '100%';
    elements.slotStatus.textContent = 'All actions completed';
    if (state.summary)
      elements.timeStatus.textContent = `${state.summary.durationSeconds.toFixed(1)} / ${state.summary.durationSeconds.toFixed(1)} s`;
  } else {
    elements.executionTitle.textContent = reason === 'emergency' ? 'Emergency stop' : 'Stopped and holding position';
    elements.slotStatus.textContent = reason === 'emergency' ? 'Press P to resume flight control before restarting' : 'Sequence stopped by user';
  }
}

function receive(rawMessage) {
  let message;
  try {
    message = JSON.parse(rawMessage);
  } catch (error) {
    showError(`The controller returned an unreadable response: ${String(error)}`);
    return;
  }

  switch (message.type) {
    case 'ready':
      state.connected = true;
      state.controllerSupportsIdle = Number.isFinite(message.defaultIdleMs);
      state.supportsRoute = message.supportsRoute === true;
      state.maxBytes = message.maxBytes;
      state.defaultActionMs = message.defaultActionMs ?? message.defaultSlotMs ?? 500;
      state.minActionMs = message.minActionMs ?? message.minSlotMs ?? 100;
      state.maxActionMs = message.maxActionMs ?? message.maxSlotMs ?? 5000;
      state.defaultIdleMs = message.defaultIdleMs ?? 500;
      state.minIdleMs = message.minIdleMs ?? 0;
      state.maxIdleMs = message.maxIdleMs ?? 5000;
      setRunning(Boolean(message.running));
      validateInput();
      if (!message.running)
        requestEncoding();
      break;
    case 'encoding-start':
      state.summary = message;
      state.frames = [];
      renderSummary();
      break;
    case 'encoding-frame':
      state.frames.push(message);
      break;
    case 'encoding-end':
      renderFrames();
      break;
    case 'execution':
      if (message.state === 'running') {
        setRunning(true);
        elements.executionTitle.textContent = 'Starting sequence';
        elements.overallProgress.style.width = '0%';
      } else {
        finishExecution(message.state, message.reason);
      }
      break;
    case 'status':
      if (!state.running)
        setRunning(true);
      updateStatus(message);
      break;
    case 'error':
      if (message.code === 'flight-stopped')
        setRunning(false);
      else
        validateInput();
      showError(message.message === 'The route requires more than the 0.30 m/s nominal speed limit'
        ? 'The running controller is an older build with the removed 0.30 m/s limit. Stop and restart the Webots simulation after rebuilding mavic2pro.'
        : message.message);
      break;
    default:
      break;
  }
}

robotWindow.receive = receive;

elements.message.addEventListener('input', scheduleEncoding);
elements.actionDuration.addEventListener('input', scheduleEncoding);
elements.idleDuration.addEventListener('input', scheduleEncoding);
elements.destinationX.addEventListener('input', scheduleEncoding);
elements.destinationY.addEventListener('input', scheduleEncoding);
elements.resetTiming.addEventListener('click', () => {
  elements.actionDuration.value = String(state.defaultActionMs);
  elements.idleDuration.value = String(state.defaultIdleMs);
  scheduleEncoding();
});
elements.start.addEventListener('click', () => {
  showError('');
  requestEncoding('START');
});
elements.stop.addEventListener('click', () => robotWindow.send('STOP'));

window.addEventListener('load', () => {
  setConnection('Connecting to controller', 'waiting');
  validateInput();
  robotWindow.send('HELLO');
});
