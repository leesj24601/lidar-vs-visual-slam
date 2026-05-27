const posePresets = {
  mapping_start: {
    label: "mapping_start",
    yaw: "yaw 164.5",
    initialPose: "-0.171703 0.161575 0 0 0 2.870674372",
  },
  dock: {
    label: "dock",
    yaw: "yaw 0.0",
    initialPose: "0 0 0 0 0 0",
  },
  lab_entry: {
    label: "lab_entry",
    yaw: "yaw 90.0",
    initialPose: "0 0 0 0 0 1.570796327",
  },
};

const state = {
  sessionName: "odom_mapping",
  mapViz: true,
  mappingRunning: false,
  localizationRunning: false,
  correctionEnabled: true,
  mode: "align",
  activePose: "mapping_start",
  apiOnline: false,
};

const apiBase = typeof window.GO2_API_BASE === "string" ? window.GO2_API_BASE : "";
const apiEnabled = typeof window.GO2_API_BASE === "string"
  || new URLSearchParams(window.location.search).get("api") === "1";
const STATUS_TIMEOUT_MS = 2500;
const COMMAND_TIMEOUT_MS = 20000;

const elements = {
  shell: document.querySelector(".dashboard-shell"),
  rosStatus: document.querySelector("#rosStatus"),
  rtabStatus: document.querySelector("#rtabStatus"),
  lockStatus: document.querySelector("#lockStatus"),
  currentMode: document.querySelector("#currentMode"),
  modeDescription: document.querySelector("#modeDescription"),
  mappingBadge: document.querySelector("#mappingBadge"),
  localizationBadge: document.querySelector("#localizationBadge"),
  correctionBadge: document.querySelector("#correctionBadge"),
  correctionBadgeWrap: document.querySelector("#correctionBadgeWrap"),
  correctionState: document.querySelector("#correctionState"),
  sessionName: document.querySelector("#sessionName"),
  sessionPath: document.querySelector("#sessionPath"),
  mapVizToggle: document.querySelector("#mapVizToggle"),
  posePreset: document.querySelector("#posePreset"),
  selectedPoseYaw: document.querySelector("#selectedPoseYaw"),
  startMapping: document.querySelector("#startMapping"),
  stopMapping: document.querySelector("#stopMapping"),
  startLocalization: document.querySelector("#startLocalization"),
  sendPose: document.querySelector("#sendPose"),
  stopLocalization: document.querySelector("#stopLocalization"),
  alignMode: document.querySelector("#alignMode"),
  lockTracking: document.querySelector("#lockTracking"),
  killAll: document.querySelector("#killAll"),
  clearLog: document.querySelector("#clearLog"),
  operatorLog: document.querySelector("#operatorLog"),
  proximityId: document.querySelector("#proximityId"),
  loopClosureId: document.querySelector("#loopClosureId"),
  tfStatus: document.querySelector("#tfStatus"),
  tfAge: document.querySelector("#tfAge"),
  poseStream: document.querySelector("#poseStream"),
};

function timestamp() {
  return new Intl.DateTimeFormat("en-GB", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  }).format(new Date());
}

function addLog(message) {
  const item = document.createElement("li");
  item.textContent = `${timestamp()}  ${message}`;
  elements.operatorLog.prepend(item);

  while (elements.operatorLog.children.length > 30) {
    elements.operatorLog.lastElementChild.remove();
  }
}

async function apiRequest(path, options = {}) {
  if (!apiEnabled) {
    throw new Error("Backend API disabled for static preview.");
  }

  if (window.location.protocol === "file:") {
    throw new Error("No HTTP server is available for API calls.");
  }

  const timeoutMs = options.timeoutMs ?? STATUS_TIMEOUT_MS;
  const controller = new AbortController();
  let timedOut = false;
  const timeoutId = window.setTimeout(() => {
    timedOut = true;
    controller.abort();
  }, timeoutMs);

  try {
    const response = await fetch(`${apiBase}${path}`, {
      method: options.method || "GET",
      headers: {
        "Content-Type": "application/json",
        ...(options.headers || {}),
      },
      body: options.body ? JSON.stringify(options.body) : undefined,
      signal: controller.signal,
    });

    if (!response.ok) {
      throw new Error(`${response.status} ${response.statusText}`);
    }

    const text = await response.text();
    return text ? JSON.parse(text) : {};
  } catch (error) {
    if (timedOut && error.name === "AbortError") {
      throw new Error(`request timed out after ${(timeoutMs / 1000).toFixed(1)}s`);
    }
    throw error;
  } finally {
    window.clearTimeout(timeoutId);
  }
}

async function command(path, body, optimisticUpdate, label) {
  const useLocalFallback = !apiEnabled;
  if (useLocalFallback) {
    optimisticUpdate();
    render();
  }

  try {
    const payload = await apiRequest(path, {
      method: "POST",
      body,
      timeoutMs: COMMAND_TIMEOUT_MS,
    });
    state.apiOnline = true;
    addLog(`${label}: accepted by backend`);
    applyStatus(payload);
  } catch (error) {
    state.apiOnline = false;
    if (useLocalFallback) {
      addLog(`${label}: local UI fallback (${error.message})`);
    } else {
      addLog(`${label}: failed (${error.message})`);
      await pollStatus();
    }
  }

  render();
}

function applyStatus(payload = {}) {
  if (typeof payload.mappingRunning === "boolean") {
    state.mappingRunning = payload.mappingRunning;
  }
  if (typeof payload.localizationRunning === "boolean") {
    state.localizationRunning = payload.localizationRunning;
  }
  if (typeof payload.correctionEnabled === "boolean") {
    state.correctionEnabled = payload.correctionEnabled;
    state.mode = payload.correctionEnabled ? "align" : "tracking";
  }

  if (payload.rtabmap) {
    elements.proximityId.textContent = payload.rtabmap.proximityId ?? elements.proximityId.textContent;
    elements.loopClosureId.textContent = payload.rtabmap.loopClosureId ?? elements.loopClosureId.textContent;
  }
  if (payload.tf) {
    elements.tfStatus.textContent = payload.tf.status || elements.tfStatus.textContent;
    if (typeof payload.tf.lastSeenSec === "number") {
      const rateText = typeof payload.tf.rateHz === "number"
        ? ` · ${payload.tf.rateHz.toFixed(1)} Hz`
        : "";
      const ageText = payload.tf.lastSeenSec <= 0.1
        ? "receiving now"
        : `last ${payload.tf.lastSeenSec.toFixed(1)}s ago`;
      elements.tfAge.textContent = `${ageText}${rateText}`;
    } else if (payload.tf.detail) {
      elements.tfAge.textContent = payload.tf.detail;
    }
  } else if (payload.tfStatus) {
    elements.tfStatus.textContent = payload.tfStatus;
  }
  if (payload.poseStream) {
    elements.poseStream.innerHTML = payload.poseStream;
  }
}

function selectedPose() {
  return posePresets[state.activePose] || posePresets.mapping_start;
}

function modeCopy() {
  if (state.mode === "tracking") {
    return {
      title: "TRACKING",
      description: "Correction is locked after alignment.",
      lock: "TRACKING LOCK",
      correction: "CORRECTION LOCKED",
      correctionText: "locked",
    };
  }

  return {
    title: "ALIGNING",
    description: "Correction is enabled for initial alignment.",
    lock: "ALIGN MODE",
    correction: "CORRECTION ENABLED",
    correctionText: "enabled",
  };
}

function renderToggle(button, value) {
  button.classList.toggle("is-on", value);
  button.setAttribute("aria-pressed", String(value));
}

function sanitizedSessionName() {
  const normalized = state.sessionName.trim().replace(/[^A-Za-z0-9_-]+/g, "_");
  return normalized.replace(/^_+|_+$/g, "") || "unnamed_session";
}

function sessionDatabasePath() {
  return `maps/sessions/${sanitizedSessionName()}/rtabmap.db`;
}

function render() {
  const pose = selectedPose();
  const copy = modeCopy();

  elements.shell.dataset.mode = state.mode;
  elements.rosStatus.textContent = !apiEnabled || state.apiOnline ? "ROS ONLINE" : "ROS PENDING";
  elements.rtabStatus.textContent = !apiEnabled || state.localizationRunning || state.mappingRunning
    ? "RTAB-MAP ALIVE"
    : "RTAB-MAP READY";
  elements.lockStatus.textContent = copy.lock;
  elements.currentMode.textContent = copy.title;
  elements.modeDescription.textContent = copy.description;
  elements.mappingBadge.textContent = state.mappingRunning ? "running" : "idle";
  elements.localizationBadge.textContent = state.localizationRunning ? "running" : "ready";
  elements.correctionBadge.textContent = copy.correction;
  elements.correctionState.textContent = copy.correctionText;
  elements.correctionState.classList.toggle("warn", state.correctionEnabled);
  elements.correctionState.classList.toggle("good", !state.correctionEnabled);
  elements.correctionBadgeWrap.classList.toggle("soft-badge--warning", state.correctionEnabled);
  elements.correctionBadgeWrap.classList.toggle("soft-badge--success", !state.correctionEnabled);

  renderToggle(elements.mapVizToggle, state.mapViz);

  elements.sessionName.value = state.sessionName;
  elements.sessionPath.textContent = sessionDatabasePath();
  elements.posePreset.value = state.activePose;
  elements.selectedPoseYaw.textContent = pose.yaw;

  elements.alignMode.classList.toggle("is-active", state.mode === "align");
  elements.lockTracking.classList.toggle("is-active", state.mode === "tracking");
  elements.alignMode.disabled = !state.localizationRunning;
  elements.lockTracking.disabled = !state.localizationRunning;

  elements.stopMapping.disabled = false;
  elements.stopLocalization.disabled = false;
  elements.sendPose.disabled = false;
}

function bodyWithPose(extra = {}) {
  const pose = selectedPose();
  return {
    preset: pose.label,
    initialPose: pose.initialPose,
    databasePath: "maps/active/rtabmap.db",
    ...extra,
  };
}

function bindEvents() {
  elements.sessionName.addEventListener("input", (event) => {
    state.sessionName = event.target.value;
    render();
  });

  elements.sessionName.addEventListener("change", () => {
    addLog(`Mapping session path: ${sessionDatabasePath()}`);
  });

  elements.mapVizToggle.addEventListener("click", () => {
    state.mapViz = !state.mapViz;
    addLog(`rtabmap_viz set to ${state.mapViz}`);
    render();
  });

  elements.posePreset.addEventListener("change", (event) => {
    state.activePose = event.target.value;
    addLog(`Selected pose preset: ${selectedPose().label}`);
    render();
  });

  elements.startMapping.addEventListener("click", () => {
    command(
      "/api/mapping/start",
      {
        sessionName: sanitizedSessionName(),
        databasePath: sessionDatabasePath(),
        rtabmapViz: state.mapViz,
      },
      () => {
        state.mappingRunning = true;
      },
      `Start mapping session ${sanitizedSessionName()}`
    );
  });

  elements.stopMapping.addEventListener("click", () => {
    command(
      "/api/mapping/stop",
      {},
      () => {
        state.mappingRunning = false;
      },
      "Stop mapping"
    );
  });

  elements.startLocalization.addEventListener("click", () => {
    command(
      "/api/localization/start",
      bodyWithPose({ rtabmapViz: true }),
      () => {
        state.localizationRunning = true;
        state.correctionEnabled = true;
        state.mode = "align";
      },
      `Start localization with ${selectedPose().label}`
    );
  });

  elements.sendPose.addEventListener("click", () => {
    command(
      "/api/localization/pose",
      bodyWithPose(),
      () => {},
      `Send initial pose ${selectedPose().label}`
    );
  });

  elements.stopLocalization.addEventListener("click", () => {
    command(
      "/api/localization/stop",
      {},
      () => {
        state.localizationRunning = false;
      },
      "Stop localization"
    );
  });

  elements.alignMode.addEventListener("click", () => {
    command(
      "/api/correction/align",
      { proximityBySpace: true },
      () => {
        state.correctionEnabled = true;
        state.mode = "align";
      },
      "Enable correction"
    );
  });

  elements.lockTracking.addEventListener("click", () => {
    command(
      "/api/correction/lock",
      { proximityBySpace: false },
      () => {
        state.correctionEnabled = false;
        state.mode = "tracking";
      },
      "Lock tracking"
    );
  });

  elements.killAll.addEventListener("click", () => {
    command(
      "/api/system/kill-all",
      {},
      () => {
        state.mappingRunning = false;
        state.localizationRunning = false;
        state.correctionEnabled = false;
        state.mode = "tracking";
      },
      "Kill all SLAM processes"
    );
  });

  elements.clearLog.addEventListener("click", () => {
    elements.operatorLog.replaceChildren();
    addLog("Operator log cleared");
  });
}

async function pollStatus() {
  if (!apiEnabled) {
    return;
  }

  try {
    const payload = await apiRequest("/api/status");
    state.apiOnline = true;
    applyStatus(payload);
  } catch (_) {
    state.apiOnline = false;
  }
  render();
}

bindEvents();
addLog("Active DB loaded: maps/active/rtabmap.db");
addLog(apiEnabled ? "Dashboard ready: API boundary /api/status" : "Dashboard ready: static preview mode");
render();
pollStatus();
window.setInterval(pollStatus, 3000);
