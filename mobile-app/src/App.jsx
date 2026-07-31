import React, { useEffect, useMemo, useRef, useState } from "react";
import {
  Activity,
  AlertTriangle,
  Battery,
  BatteryCharging,
  Bot,
  CheckCircle2,
  Clock,
  MapPin,
  Lightbulb,
  PlugZap,
  Radio,
  RefreshCw,
  Send,
  Settings,
  Signal,
  Wifi,
  X,
  XCircle,
} from "lucide-react";

const DEFAULT_ESP_URL = "http://192.168.4.1";

const ROUTES = {
  1: {
    name: "Table 1",
    route: "ROUTE:0,3,0,4",
    short: "Straight at first junction → Serve → Dock",
    accent: "cyan",
  },
  2: {
    name: "Table 2",
    route: "ROUTE:2,2,3,1,1,4",
    short: "Right at first junction → Right at second junction → Serve",
    accent: "green",
  },
  3: {
    name: "Table 3",
    route: "ROUTE:2,0,3,0,1,4",
    short: "Right at first junction → Straight at second junction → Serve",
    accent: "pink",
  },
};

const FORWARD_TIMELINES = {
  1: [
    { event: "START", label: "Start", t: 0, x: 120, y: 570 },
    { event: "BEND1", label: "First bend", t: 7.0, x: 300, y: 540 },
    { event: "J1", label: "First junction", t: 12.5, x: 300, y: 390 },
    { event: "TABLE1", label: "Table 1", t: 19.0, x: 300, y: 205 },
  ],
  2: [
    { event: "START", label: "Start", t: 0, x: 120, y: 570 },
    { event: "BEND1", label: "First bend", t: 7.0, x: 300, y: 540 },
    { event: "J1", label: "First junction", t: 12.5, x: 300, y: 390 },
    { event: "J2", label: "Second junction", t: 20.5, x: 540, y: 390 },
    { event: "TABLE2", label: "Table 2", t: 26.5, x: 540, y: 570 },
  ],
  3: [
    { event: "START", label: "Start", t: 0, x: 120, y: 570 },
    { event: "BEND1", label: "First bend", t: 7.0, x: 300, y: 540 },
    { event: "J1", label: "First junction", t: 12.5, x: 300, y: 390 },
    { event: "J2", label: "Second junction", t: 20.5, x: 540, y: 390 },
    { event: "LAST_BEND", label: "Last bend", t: 27.5, x: 790, y: 360 },
    { event: "TABLE3", label: "Table 3", t: 34.5, x: 790, y: 205 },
  ],
};

const RETURN_TIMELINES = {
  1: [
    { event: "RETURNING", label: "Returning", t: 0, x: 300, y: 262 },
    { event: "RETURN_J1", label: "Return junction 1", t: 6.5, x: 300, y: 390 },
    { event: "DOCK_MARKER", label: "Dock marker", t: 19.0, x: 120, y: 570 },
  ],
  2: [
    { event: "RETURNING", label: "Returning", t: 0, x: 540, y: 515 },
    { event: "RETURN_J2", label: "Return junction 2", t: 6.0, x: 540, y: 390 },
    { event: "RETURN_J1", label: "Return junction 1", t: 14.0, x: 300, y: 390 },
    { event: "DOCK_MARKER", label: "Dock marker", t: 26.5, x: 120, y: 570 },
  ],
  3: [
    { event: "RETURNING", label: "Returning", t: 0, x: 790, y: 262 },
    { event: "LAST_BEND", label: "Last bend", t: 7.0, x: 790, y: 360 },
    { event: "RETURN_J2", label: "Return junction 2", t: 14.0, x: 540, y: 390 },
    { event: "RETURN_J1", label: "Return junction 1", t: 22.0, x: 300, y: 390 },
    { event: "DOCK_MARKER", label: "Dock marker", t: 34.5, x: 120, y: 570 },
  ],
};

const EVENT_LABELS = {
  START: "Started",
  BEND1: "First bend",
  J1: "First junction",
  J2: "Second junction",
  LAST_BEND: "Last bend",
  TABLE1: "Reached Table 1",
  TABLE2: "Reached Table 2",
  TABLE3: "Reached Table 3",
  SERVING: "Serving",
  REVERSING: "Reversing",
  UTURN: "Turning back",
  RETURNING: "Returning",
  RETURN_J2: "Return junction 2",
  RETURN_J1: "Return junction 1",
  DOCK_MARKER: "Dock marker",
  DOCK_TURNING: "Dock turning",
  DOCK_UTURN: "Dock U-turn",
  DOCK_REVERSING: "Dock reversing",
  DOCKED: "Docked",
  ERROR: "Error",
};

const EVENT_LISTS = {
  1: ["START", "BEND1", "J1", "TABLE1", "SERVING", "REVERSING", "UTURN", "RETURNING", "RETURN_J1", "DOCK_MARKER", "DOCK_TURNING", "DOCK_REVERSING", "DOCKED"],
  2: ["START", "BEND1", "J1", "J2", "TABLE2", "SERVING", "REVERSING", "UTURN", "RETURNING", "RETURN_J2", "RETURN_J1", "DOCK_MARKER", "DOCK_TURNING", "DOCK_REVERSING", "DOCKED"],
  3: ["START", "BEND1", "J1", "J2", "LAST_BEND", "TABLE3", "SERVING", "REVERSING", "UTURN", "RETURNING", "RETURN_J2", "RETURN_J1", "DOCK_MARKER", "DOCK_TURNING", "DOCK_REVERSING", "DOCKED"],
};

const TIMEOUT_MARGIN_SEC = 1.0;

function normalizeBaseUrl(value) {
  let url = (value || "").trim();
  if (!url) return DEFAULT_ESP_URL;
  if (!url.startsWith("http://") && !url.startsWith("https://")) {
    url = "http://" + url;
  }
  return url.replace(/\/+$/, "");
}

async function apiGet(baseUrl, path) {
  const res = await fetch(`${baseUrl}${path}`, { method: "GET", cache: "no-store" });
  const text = await res.text();
  let data;
  try {
    data = text ? JSON.parse(text) : {};
  } catch {
    data = { ok: false, message: text || "Invalid JSON" };
  }
  if (!res.ok) throw new Error(data.message || `HTTP ${res.status}`);
  return data;
}

function formatUptime(sec) {
  const n = Number(sec || 0);
  const h = Math.floor(n / 3600);
  const m = Math.floor((n % 3600) / 60);
  const s = n % 60;
  if (h) return `${h}h ${m}m`;
  if (m) return `${m}m ${s}s`;
  return `${s}s`;
}

function formatElapsed(ms) {
  const s = Math.max(0, Math.round((ms || 0) / 100) / 10);
  return `${s.toFixed(1)}s`;
}

function clampPercent(value) {
  return Math.max(0, Math.min(100, Number(value || 0)));
}

function tablePoint(table) {
  if (table === 1) return { x: 300, y: 205, angle: 0, forwardAngle: 0, returnAngle: 180 };
  if (table === 2) return { x: 540, y: 570, angle: 180, forwardAngle: 180, returnAngle: 0 };
  if (table === 3) return { x: 790, y: 205, angle: 0, forwardAngle: 0, returnAngle: 180 };
  return { x: 120, y: 570, angle: 90, forwardAngle: 90, returnAngle: -90 };
}

function tableLinePoint(table) {
  /*
    After serving, the robot reverses about its width on the same line before
    doing the U-turn. Show that rotation on the line, not inside the table.
  */
  if (table === 1) return { x: 300, y: 262, angle: 0, forwardAngle: 0, returnAngle: 180 };
  if (table === 2) return { x: 540, y: 515, angle: 180, forwardAngle: 180, returnAngle: 0 };
  if (table === 3) return { x: 790, y: 262, angle: 0, forwardAngle: 0, returnAngle: 180 };
  return { x: 120, y: 570, angle: 90, forwardAngle: 90, returnAngle: -90 };
}

function segmentPath(fromEvent, toEvent) {
  const key = `${fromEvent}->${toEvent}`;
  const map = {
    "START->BEND1": [{ x: 120, y: 570 }, { x: 270, y: 570 }, { x: 300, y: 540 }],
    "BEND1->J1": [{ x: 300, y: 540 }, { x: 300, y: 390 }],
    "J1->TABLE1": [{ x: 300, y: 390 }, { x: 300, y: 205 }],
    "J1->J2": [{ x: 300, y: 390 }, { x: 540, y: 390 }],
    "J2->TABLE2": [{ x: 540, y: 390 }, { x: 540, y: 570 }],
    "J2->LAST_BEND": [{ x: 540, y: 390 }, { x: 760, y: 390 }, { x: 790, y: 360 }],
    "LAST_BEND->TABLE3": [{ x: 790, y: 360 }, { x: 790, y: 205 }],

    "RETURNING->RETURN_J1": [{ x: 300, y: 262 }, { x: 300, y: 390 }],
    "RETURNING->RETURN_J2": [{ x: 540, y: 515 }, { x: 540, y: 390 }],
    "RETURNING->LAST_BEND": [{ x: 790, y: 262 }, { x: 790, y: 360 }],
    "LAST_BEND->RETURN_J2": [{ x: 790, y: 360 }, { x: 760, y: 390 }, { x: 540, y: 390 }],
    "RETURN_J2->RETURN_J1": [{ x: 540, y: 390 }, { x: 300, y: 390 }],
    "RETURN_J1->DOCK_MARKER": [{ x: 300, y: 390 }, { x: 300, y: 540 }, { x: 270, y: 570 }, { x: 120, y: 570 }],
  };
  return map[key] || [];
}

function pointDistance(a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function positionOnPolyline(points, ratio) {
  if (!points || points.length === 0) return { x: 120, y: 570, angle: 90 };
  if (points.length === 1) return { ...points[0], angle: 90 };

  const lengths = [];
  let total = 0;
  for (let i = 0; i < points.length - 1; i++) {
    const len = pointDistance(points[i], points[i + 1]);
    lengths.push(len);
    total += len;
  }

  let target = Math.max(0, Math.min(1, ratio)) * total;

  for (let i = 0; i < lengths.length; i++) {
    if (target <= lengths[i] || i === lengths.length - 1) {
      const a = points[i];
      const b = points[i + 1];
      const r = lengths[i] === 0 ? 0 : target / lengths[i];
      const x = a.x + (b.x - a.x) * r;
      const y = a.y + (b.y - a.y) * r;
      const angle = Math.atan2(b.x - a.x, -(b.y - a.y)) * 180 / Math.PI;
      return { x, y, angle };
    }
    target -= lengths[i];
  }

  const a = points[points.length - 2];
  const b = points[points.length - 1];
  const angle = Math.atan2(b.x - a.x, -(b.y - a.y)) * 180 / Math.PI;
  return { ...b, angle };
}

function completeEventsUpTo(table, event, prevSet) {
  const next = new Set(prevSet);
  const list = EVENT_LISTS[table] || [];
  const idx = list.indexOf(event);
  if (idx >= 0) {
    for (let i = 0; i <= idx; i++) next.add(list[i]);
  } else {
    next.add(event);
  }
  return next;
}

function gatedPosition(timeline, receivedEvents, elapsedMs) {
  if (!timeline || timeline.length === 0) return { x: 120, y: 570, angle: 90 };

  let currentIndex = 0;
  for (let i = 1; i < timeline.length; i++) {
    if (receivedEvents.has(timeline[i].event)) {
      currentIndex = i;
    } else {
      break;
    }
  }

  const current = timeline[currentIndex];
  const next = timeline[currentIndex + 1];

  if (!next) {
    return { x: current.x, y: current.y, angle: current.angle ?? 90 };
  }

  const elapsedSec = Number(elapsedMs || 0) / 1000;
  const ratio = Math.max(0, Math.min(1, (elapsedSec - current.t) / Math.max(0.001, next.t - current.t)));

  const path = segmentPath(current.event, next.event);
  if (path.length >= 2) {
    return positionOnPolyline(path, ratio);
  }

  return positionOnPolyline([{ x: current.x, y: current.y }, { x: next.x, y: next.y }], ratio);
}

function latestTimelineEventRecord(status, timeline, receivedEvents) {
  if (!status || !Array.isArray(status.events) || !timeline) return null;
  const allowed = new Set(timeline.map((p) => p.event));

  for (let i = status.events.length - 1; i >= 0; i--) {
    const item = status.events[i];
    if (item?.event && allowed.has(item.event) && receivedEvents.has(item.event)) {
      return item;
    }
  }

  return null;
}

function pointForEvent(timeline, eventName) {
  return timeline?.find((p) => p.event === eventName) || null;
}

function eventTimedPosition(timeline, status, receivedEvents, fallbackElapsedMs) {
  if (!timeline || timeline.length === 0) return { x: 120, y: 570, angle: 90 };

  const latest = latestTimelineEventRecord(status, timeline, receivedEvents);

  if (!latest) {
    return gatedPosition(timeline, receivedEvents, fallbackElapsedMs);
  }

  const currentPoint = pointForEvent(timeline, latest.event);
  if (!currentPoint) {
    return gatedPosition(timeline, receivedEvents, fallbackElapsedMs);
  }

  const currentIndex = timeline.findIndex((p) => p.event === latest.event);
  const nextPoint = timeline[currentIndex + 1];

  if (!nextPoint) {
    return { x: currentPoint.x, y: currentPoint.y, angle: currentPoint.angle ?? 90 };
  }

  /*
    Use the actual ESP event timestamp as the segment start. This removes the
    pause at return junctions when the real robot reaches an update earlier
    than our rough timing estimate.
  */
  const segmentDurationMs = Math.max(800, (nextPoint.t - currentPoint.t) * 1000);
  const eventMs = Number(latest.ms || 0);
  const elapsedSinceEventMs = Math.max(0, Number(status?.elapsedMs || 0) - eventMs);
  const ratio = Math.max(0, Math.min(1, elapsedSinceEventMs / segmentDurationMs));

  const path = segmentPath(currentPoint.event, nextPoint.event);
  if (path.length >= 2) return positionOnPolyline(path, ratio);

  return positionOnPolyline(
    [{ x: currentPoint.x, y: currentPoint.y }, { x: nextPoint.x, y: nextPoint.y }],
    ratio
  );
}

function getRobotVisualState(table, status, receivedEvents, elapsedMs, returnElapsedMs) {
  const current = status?.currentEvent || "";

  if (!table || receivedEvents.has("DOCKED") || current === "DOCKED") {
    return { x: 120, y: 570, angle: 90, turning: false };
  }

  if (current === "DOCK_MARKER") {
    return { x: 120, y: 570, angle: -90, turning: false };
  }

  if (current === "DOCK_TURNING" || current === "DOCK_UTURN") {
    return { x: 120, y: 570, angle: 90, turning: true };
  }

  if (current === "DOCK_REVERSING") {
    return { x: 120, y: 570, angle: 90, turning: false };
  }

  if (current === "UTURN") {
    const pos = tableLinePoint(table);
    return { ...pos, angle: pos.returnAngle, turning: true };
  }

  if (current === "REVERSING") {
    const pos = tableLinePoint(table);
    return { ...pos, angle: pos.forwardAngle, turning: false };
  }

  if (current === "SERVING") {
    const pos = tablePoint(table);
    return { ...pos, angle: pos.forwardAngle, turning: false };
  }

  if (receivedEvents.has("RETURNING") || current === "RETURNING" || current.startsWith("RETURN_")) {
    const timeline = RETURN_TIMELINES[table] || [];
    if (Array.isArray(status?.events) && status.events.length > 0) {
      return { ...eventTimedPosition(timeline, status, receivedEvents, returnElapsedMs), turning: false };
    }
    return { ...gatedPosition(timeline, receivedEvents, returnElapsedMs), turning: false };
  }

  if (receivedEvents.has(`TABLE${table}`)) {
    const pos = tablePoint(table);
    return { ...pos, angle: pos.forwardAngle, turning: false };
  }

  const timeline = FORWARD_TIMELINES[table] || [];
  if (Array.isArray(status?.events) && status.events.length > 0) {
    return { ...eventTimedPosition(timeline, status, receivedEvents, elapsedMs), turning: false };
  }

  return { ...gatedPosition(timeline, receivedEvents, elapsedMs), turning: false };
}

function getNextExpectedEvent(table, receivedEvents, isReturning) {
  const timeline = isReturning ? RETURN_TIMELINES[table] : FORWARD_TIMELINES[table];
  if (!timeline) return null;
  return timeline.find((p) => !receivedEvents.has(p.event) && p.event !== "START") || null;
}

function Toast({ toast }) {
  if (!toast) return null;
  return (
    <div className={`toast ${toast.type || ""}`}>
      {toast.loading && <span className="spinner" />}
      {toast.message}
    </div>
  );
}

function StatusChip({ online, status }) {
  return (
    <div className={`statusChip ${online ? "online" : "offline"}`}>
      {online ? <CheckCircle2 size={15} /> : <XCircle size={15} />}
      <span>{online ? "ESP Online" : "ESP Offline"}</span>
      <b>{status?.ip || "No IP"}</b>
    </div>
  );
}

function Table({ id, x, y, label, selected, serving, setSelectedTable }) {
  return (
    <g
      className={`table ${selected ? "selected" : ""} ${serving ? "serving" : ""}`}
      onClick={() => setSelectedTable(id)}
      tabIndex="0"
      role="button"
      aria-label={`Select table ${id}`}
      onKeyDown={(e) => {
        if (e.key === "Enter" || e.key === " ") setSelectedTable(id);
      }}
    >
      <ellipse cx={x} cy={y} rx="86" ry="57" className="tableHalo" />
      <ellipse cx={x} cy={y} rx="68" ry="44" className="tableBase" />
      <text x={x} y={y - 4} textAnchor="middle" className="tableName">{label}</text>
      <text x={x} y={y + 19} textAnchor="middle" className="tableCaption">TABLE {id}</text>
      {serving && (
        <text x={x} y={y + 68} textAnchor="middle" className="servingBadge">SERVING</text>
      )}
    </g>
  );
}

function Milestone({ point, received, late }) {
  return (
    <g className={`milestone ${received ? "done" : ""} ${late ? "late" : ""}`}>
      <circle cx={point.x} cy={point.y} r="9" />
    </g>
  );
}

function DiningMap({ selectedTable, setSelectedTable, robotVisual, receivedEvents, alertEvent, servingTable, obstacleActive }) {
  const points = selectedTable ? FORWARD_TIMELINES[selectedTable] : [];

  return (
    <div className="mapCard">
      <div className="mapTop">
        <div>
          <span className="eyebrow">Restaurant floor</span>
          <h2>Select destination from the map</h2>
          <p>Event-gated navigation: robot waits at junctions until update is received.</p>
        </div>
        <div className="mapLegend">
          <span><i className="dot dockDot" /> Dock</span>
          <span><i className="dot routeDot" /> Route</span>
          <span><i className="dot tableDot" /> Table</span>
        </div>
      </div>

      <div className="svgFrame">
        <svg viewBox="0 0 1040 700" className="floorSvg" aria-label="Restaurant map">
          <defs>
            <filter id="glow" x="-70%" y="-70%" width="240%" height="240%">
              <feGaussianBlur stdDeviation="6" result="blur" />
              <feMerge>
                <feMergeNode in="blur" />
                <feMergeNode in="SourceGraphic" />
              </feMerge>
            </filter>

            <linearGradient id="pathGrad" x1="0" y1="0" x2="1" y2="0">
              <stop offset="0%" stopColor="#23e9ff" />
              <stop offset="52%" stopColor="#57ffb1" />
              <stop offset="100%" stopColor="#fff36a" />
            </linearGradient>

            <linearGradient id="tableGrad" x1="0" y1="0" x2="1" y2="1">
              <stop offset="0%" stopColor="#1a2a44" />
              <stop offset="100%" stopColor="#09111f" />
            </linearGradient>
          </defs>

          <rect x="40" y="34" width="960" height="628" rx="38" className="room" />

          <g className="tiles">
            {Array.from({ length: 12 }).map((_, i) => (
              <line key={`v${i}`} x1={80 + i * 75} y1="70" x2={80 + i * 75} y2="626" />
            ))}
            {Array.from({ length: 8 }).map((_, i) => (
              <line key={`h${i}`} x1="76" y1={98 + i * 67} x2="964" y2={98 + i * 67} />
            ))}
          </g>

          <rect x="205" y="125" width="190" height="130" rx="28" className="zone" />
          <rect x="445" y="500" width="190" height="120" rx="28" className="zone" />
          <rect x="705" y="125" width="190" height="130" rx="28" className="zone" />

          <path
            className="trackSoft"
            d="M120 570 L270 570 Q300 570 300 540 L300 390 M300 390 L300 205 M300 390 L540 390 M540 390 L540 570 M540 390 L760 390 Q790 390 790 360 L790 205"
          />
          <path
            className="trackLine"
            d="M120 570 L270 570 Q300 570 300 540 L300 390 M300 390 L300 205 M300 390 L540 390 M540 390 L540 570 M540 390 L760 390 Q790 390 790 360 L790 205"
          />

          <path
            className={`previewRoute ${selectedTable === 1 ? "active" : ""}`}
            d="M120 570 L270 570 Q300 570 300 540 L300 390 L300 205"
          />
          <path
            className={`previewRoute ${selectedTable === 2 ? "active" : ""}`}
            d="M120 570 L270 570 Q300 570 300 540 L300 390 L540 390 L540 570"
          />
          <path
            className={`previewRoute ${selectedTable === 3 ? "active" : ""}`}
            d="M120 570 L270 570 Q300 570 300 540 L300 390 L540 390 L760 390 Q790 390 790 360 L790 205"
          />

          <g className="junctions">
            <circle cx="300" cy="390" r="8" />
            <circle cx="540" cy="390" r="8" />
          </g>

          {points.map((p) => (
            <Milestone
              key={p.event}
              point={p}
              received={receivedEvents.has(p.event)}
              late={alertEvent === p.event}
            />
          ))}

          <g className="dockStation">
            <rect x="62" y="520" width="116" height="82" rx="22" />
            <path d="M92 574 h56" />
            <text x="120" y="552" textAnchor="middle">DOCK</text>
            <text x="120" y="580" textAnchor="middle" className="small">START</text>
          </g>

          <Table id={1} x={300} y={205} label="T1" selected={selectedTable === 1} serving={servingTable === 1} setSelectedTable={setSelectedTable} />
          <Table id={2} x={540} y={570} label="T2" selected={selectedTable === 2} serving={servingTable === 2} setSelectedTable={setSelectedTable} />
          <Table id={3} x={790} y={205} label="T3" selected={selectedTable === 3} serving={servingTable === 3} setSelectedTable={setSelectedTable} />

          <g
            className={`liveRobot ${robotVisual.turning ? "turning" : ""} ${obstacleActive ? "obstacle" : ""}`}
            transform={`translate(${robotVisual.x} ${robotVisual.y}) rotate(${robotVisual.angle || 90})`}
          >
            {obstacleActive && <circle cx="0" cy="0" r="44" className="obstacleBoundary" />}
            <g className="robotInner">
              <circle cx="0" cy="0" r="26" className="liveRobotPulse" />
              <ellipse cx="0" cy="23" rx="34" ry="10" className="robotShadow" />
              <path d="M0 -39 L12 -24 L-12 -24 Z" className="robotNose" />
              <rect x="-31" y="-28" width="62" height="52" rx="18" className="robotBody" />
              <circle cx="-12" cy="-7" r="4.5" />
              <circle cx="12" cy="-7" r="4.5" />
              <rect x="-13" y="7" width="26" height="8" rx="4" />
            </g>
          </g>
        </svg>
      </div>
    </div>
  );
}

function Info({ icon: Icon, label, value }) {
  return (
    <div className="infoRow">
      <span>
        <Icon size={15} />
        {label}
      </span>
      <b>{value}</b>
    </div>
  );
}

function ObstaclePanel({ status, espBaseUrl, refreshStatus, showToast }) {
  const paused = Boolean(status?.obstaclePaused);
  const help = Boolean(status?.obstacleHelpNeeded);

  async function bypass(seconds = 60) {
    try {
      const data = await apiGet(espBaseUrl, `/api/obstacle/bypass?seconds=${seconds}`);
      showToast(data.message || `Obstacle bypass ${seconds}s`);
      await refreshStatus();
    } catch (err) {
      showToast("Bypass failed: " + err.message, "error");
    }
  }

  return (
    <section className={`controlCard obstaclePanel ${paused ? "active" : ""} ${help ? "help" : ""}`}>
      <span className="eyebrow">Obstacle safety</span>
      <div className="obstacleLine">
        <b>{help ? "Help needed" : paused ? "Stopped" : "Clear"}</b>
        <span>{status?.obstacleMessage || "No obstacle detected"}</span>
      </div>
      <div className="distanceGrid">
        <span>Front <b>{Number(status?.frontCm || -1) > 0 ? `${Number(status.frontCm).toFixed(0)} cm` : "--"}</b></span>
        <span>Left <b>{Number(status?.leftCm || -1) > 0 ? `${Number(status.leftCm).toFixed(0)} cm` : "--"}</b></span>
        <span>Right <b>{Number(status?.rightCm || -1) > 0 ? `${Number(status.rightCm).toFixed(0)} cm` : "--"}</b></span>
      </div>
      {(paused || help) && (
        <button className="smallAction warn" onClick={() => bypass(60)}>
          Bypass for this journey
        </button>
      )}
    </section>
  );
}

function SensorTechnicalPanel({ status, espBaseUrl, refreshStatus, showToast }) {
  async function setSensors(front, left, right) {
    try {
      await apiGet(espBaseUrl, `/api/obstacle/enable?front=${front ? 1 : 0}&left=${left ? 1 : 0}&right=${right ? 1 : 0}`);
      showToast("Obstacle sensor settings updated");
      await refreshStatus();
    } catch (err) {
      showToast("Sensor update failed: " + err.message, "error");
    }
  }

  async function bypass(seconds) {
    try {
      const data = await apiGet(espBaseUrl, `/api/obstacle/bypass?seconds=${seconds}`);
      showToast(data.message || "Obstacle bypass enabled");
      await refreshStatus();
    } catch (err) {
      showToast("Bypass failed: " + err.message, "error");
    }
  }

  return (
    <section className="controlCard sensorTech">
      <span className="eyebrow">Ultrasonic sensors</span>
      <p>Front stops at about 15 cm. Side sensors stop only when distance is getting closer during navigation.</p>
      <div className="distanceGrid">
        <span>Front <b>{Number(status?.frontCm || -1) > 0 ? `${Number(status.frontCm).toFixed(1)} cm` : "--"}</b></span>
        <span>Left <b>{Number(status?.leftCm || -1) > 0 ? `${Number(status.leftCm).toFixed(1)} cm` : "--"}</b></span>
        <span>Right <b>{Number(status?.rightCm || -1) > 0 ? `${Number(status.rightCm).toFixed(1)} cm` : "--"}</b></span>
      </div>
      <div className="toggleGrid">
        <button onClick={() => setSensors(!status?.frontSensorEnabled, status?.leftSensorEnabled, status?.rightSensorEnabled)}>
          Front: {status?.frontSensorEnabled ? "ON" : "OFF"}
        </button>
        <button onClick={() => setSensors(status?.frontSensorEnabled, !status?.leftSensorEnabled, status?.rightSensorEnabled)}>
          Left: {status?.leftSensorEnabled ? "ON" : "OFF"}
        </button>
        <button onClick={() => setSensors(status?.frontSensorEnabled, status?.leftSensorEnabled, !status?.rightSensorEnabled)}>
          Right: {status?.rightSensorEnabled ? "ON" : "OFF"}
        </button>
      </div>
      <div className="trayApiGrid">
        <button onClick={() => bypass(30)}>Bypass 30s</button>
        <button onClick={() => bypass(120)}>Bypass 2min</button>
      </div>
    </section>
  );
}

function HallTechnicalPanel({ espBaseUrl, showToast }) {
  const [hallText, setHallText] = useState("");

  async function readHall() {
    try {
      const data = await apiGet(espBaseUrl, "/api/hall/read");
      setHallText(data.hall || JSON.stringify(data));
      showToast("Hall values received");
    } catch (err) {
      showToast("Hall read failed: " + err.message, "error");
    }
  }

  async function updateHall() {
    if (!window.confirm("Update STM32 Hall baseline using the current readings? Place robot on normal line/background condition first.")) return;

    try {
      const data = await apiGet(espBaseUrl, "/api/hall/update");
      setHallText(data.hall || JSON.stringify(data));
      showToast("Hall baseline updated in STM32 RAM");
    } catch (err) {
      showToast("Hall update failed: " + err.message, "error");
    }
  }

  return (
    <section className="controlCard hallTech">
      <span className="eyebrow">Magnetic sensor calibration</span>
      <p>Reads H0-H6 from STM32. Update saves the current values as baseline in RAM until power reset.</p>
      <div className="trayApiGrid">
        <button onClick={readHall}>Read Hall</button>
        <button onClick={updateHall}>Update baseline</button>
      </div>
      <pre>{hallText || "No Hall reading yet."}</pre>
    </section>
  );
}

function TrayStatusMini({ status }) {
  const ready = Boolean(status?.trayReady);
  const weight = Number(status?.trayWeightG || 0);
  const hasFood = Boolean(status?.trayHasFood);
  const waiting = Boolean(status?.servingWaitingForFood);

  return (
    <section className={`controlCard staffTray ${hasFood ? "food" : "empty"} ${waiting ? "waiting" : ""}`}>
      <span className="eyebrow">Tray</span>
      <div className="staffTrayLine">
        <b>{ready ? `${weight.toFixed(0)} g` : "--"}</b>
        <span>{waiting ? "Serving - waiting for pickup" : hasFood ? "Food on tray" : "Tray empty"}</span>
      </div>
      {waiting && <p>Alert buzzer repeats until food is taken.</p>}
    </section>
  );
}


const LED_PATTERNS = [
  { value: "fixed", label: "Fixed" },
  { value: "running", label: "Running" },
  { value: "random", label: "Random sparkle" },
  { value: "rainbow", label: "Rainbow" },
  { value: "breathing", label: "Breathing" },
];

const LED_QUICK_COLORS = [
  { name: "Cyan", r: 40, g: 246, b: 255 },
  { name: "White", r: 255, g: 255, b: 255 },
  { name: "Warm", r: 255, g: 180, b: 90 },
  { name: "Pink", r: 255, g: 79, b: 227 },
  { name: "Green", r: 67, g: 255, b: 172 },
  { name: "Red", r: 255, g: 79, b: 109 },
];

function defaultLedConfig() {
  return { enabled: true, r: 40, g: 246, b: 255, brightness: 70, pattern: "fixed", speed: 45 };
}

function ledConfigForZone(status, zone) {
  const led = status?.led || {};
  if (zone === "left") return { ...defaultLedConfig(), ...(led.left || {}) };
  if (zone === "right") return { ...defaultLedConfig(), ...(led.right || {}) };
  if (zone === "extra") return { ...defaultLedConfig(), ...(led.extra || {}) };
  return { ...defaultLedConfig(), ...(led.right || {}) };
}

function LedControlModal({ status, espBaseUrl, refreshStatus, showToast, onClose }) {
  const [zone, setZone] = useState("sides");
  const [form, setForm] = useState(() => ledConfigForZone(status, "sides"));
  const [linkSides, setLinkSides] = useState(Boolean(status?.led?.linkSides ?? true));
  const [working, setWorking] = useState(false);
  const [dirty, setDirty] = useState(false);

  // The main page refreshes /api/status repeatedly. Previously every refresh
  // copied the ESP values back into this form, so sliders and dropdowns jumped
  // immediately to their old/default values before Apply could be pressed.
  // Keep the latest server status in a ref, but never overwrite active edits.
  const latestStatusRef = useRef(status);
  const initializedFromServerRef = useRef(Boolean(status?.led));

  useEffect(() => {
    latestStatusRef.current = status;

    // Handles the uncommon case where the modal opens before the first status
    // response arrives. This initialization runs once and does not reset edits.
    if (!initializedFromServerRef.current && status?.led) {
      setForm(ledConfigForZone(status, zone));
      setLinkSides(Boolean(status.led.linkSides ?? true));
      initializedFromServerRef.current = true;
      setDirty(false);
    }
  }, [status, zone]);

  useEffect(() => {
    // Changing the control zone intentionally loads that zone's latest saved
    // configuration. Normal background status polling does not run this effect.
    const latest = latestStatusRef.current;
    setForm(ledConfigForZone(latest, zone));
    setLinkSides(Boolean(latest?.led?.linkSides ?? true));
    initializedFromServerRef.current = Boolean(latest?.led);
    setDirty(false);
  }, [zone]);

  function updateField(field, value) {
    setDirty(true);
    setForm((prev) => ({ ...prev, [field]: value }));
  }

  async function applyLed(overrides = {}) {
    const next = { ...form, ...overrides };
    setWorking(true);

    try {
      const params = new URLSearchParams({
        zone,
        enabled: next.enabled ? "1" : "0",
        pattern: next.pattern,
        r: String(next.r),
        g: String(next.g),
        b: String(next.b),
        brightness: String(next.brightness),
        speed: String(next.speed),
        link: linkSides ? "1" : "0",
      });

      const data = await apiGet(espBaseUrl, `/api/led/config?${params.toString()}`);
      setForm(next);
      setDirty(false);
      latestStatusRef.current = data;
      showToast(data.message || "LED settings updated");
      await refreshStatus();
    } catch (err) {
      showToast("LED update failed: " + err.message, "error");
    } finally {
      setWorking(false);
    }
  }

  const previewColor = `rgb(${Number(form.r || 0)}, ${Number(form.g || 0)}, ${Number(form.b || 0)})`;
  const sideCount = Number(status?.led?.sideCount || 88);
  const pin37Count = Number(status?.led?.pin37Count || 0);

  return (
    <div className="modalBackdrop" onMouseDown={(e) => e.target === e.currentTarget && onClose()}>
      <section className="ledModal controlCard">
        <div className="modalHeader">
          <div>
            <span className="eyebrow">Decoration lights</span>
            <h2>WS2812B LED control</h2>
            <p>GPIO36: first {sideCount} LEDs right side, next {sideCount} LEDs left side. GPIO37: optional extra strip ({pin37Count} LEDs).</p>
          </div>
          <button className="iconButton" onClick={onClose} title="Close"><X size={18} /></button>
        </div>

        <div className="ledGrid">
          <label>
            <span>Control zone</span>
            <select value={zone} onChange={(e) => setZone(e.target.value)} disabled={working}>
              <option value="sides">Both side supports</option>
              <option value="right">Right side only</option>
              <option value="left">Left side only</option>
              <option value="extra">GPIO37 extra strip</option>
              <option value="all">All LEDs</option>
            </select>
          </label>

          <label>
            <span>Pattern</span>
            <select value={form.pattern} onChange={(e) => updateField("pattern", e.target.value)} disabled={working}>
              {LED_PATTERNS.map((p) => <option key={p.value} value={p.value}>{p.label}</option>)}
            </select>
          </label>
        </div>

        <div className="ledSwitchRow">
          <button className={form.enabled ? "active" : ""} disabled={working} onClick={() => updateField("enabled", !form.enabled)}>
            {form.enabled ? "LEDs enabled" : "LEDs off"}
          </button>
          <label className="checkLine">
            <input
              type="checkbox"
              checked={linkSides}
              disabled={working}
              onChange={(e) => {
                setLinkSides(e.target.checked);
                setDirty(true);
              }}
            />
            <span>Link right + left when editing one side</span>
          </label>
        </div>

        <div className="ledPreviewRow">
          <div className="ledPreview" style={{ background: previewColor, opacity: form.enabled ? 1 : 0.25 }} />
          <div>
            <b>RGB({form.r}, {form.g}, {form.b})</b>
            <p>Brightness {form.brightness}/255 · Speed {form.speed}/100</p>
          </div>
        </div>

        <div className="sliderStack">
          <label className="rangeLine red"><span>R</span><input type="range" min="0" max="255" value={form.r} onChange={(e) => updateField("r", Number(e.target.value))} /><b>{form.r}</b></label>
          <label className="rangeLine green"><span>G</span><input type="range" min="0" max="255" value={form.g} onChange={(e) => updateField("g", Number(e.target.value))} /><b>{form.g}</b></label>
          <label className="rangeLine blue"><span>B</span><input type="range" min="0" max="255" value={form.b} onChange={(e) => updateField("b", Number(e.target.value))} /><b>{form.b}</b></label>
          <label className="rangeLine"><span>Brightness</span><input type="range" min="0" max="255" value={form.brightness} onChange={(e) => updateField("brightness", Number(e.target.value))} /><b>{form.brightness}</b></label>
          <label className="rangeLine"><span>Speed</span><input type="range" min="1" max="100" value={form.speed} onChange={(e) => updateField("speed", Number(e.target.value))} /><b>{form.speed}</b></label>
        </div>

        <div className="quickColors">
          {LED_QUICK_COLORS.map((c) => (
            <button
              key={c.name}
              disabled={working}
              onClick={() => {
                setDirty(true);
                setForm((prev) => ({ ...prev, r: c.r, g: c.g, b: c.b }));
              }}
            >
              <i style={{ background: `rgb(${c.r}, ${c.g}, ${c.b})` }} /> {c.name}
            </button>
          ))}
        </div>

        <div className="modalActions">
          <span className={`ledSaveState ${dirty ? "dirty" : ""}`}>
            {dirty ? "Unsaved changes" : "Settings synchronized"}
          </span>
          <button className="smallAction warn" disabled={working} onClick={() => applyLed({ enabled: false })}>Turn off</button>
          <button className="smallAction" disabled={working} onClick={() => applyLed({ enabled: true })}>
            {working ? "Applying..." : "Apply settings"}
          </button>
        </div>
      </section>
    </div>
  );
}

function PowerStatusMini({ status }) {
  const percent = clampPercent(status?.batteryPercent);
  const voltage = Number(status?.batteryVoltage || 0);
  const pinVoltage = Number(status?.batteryPinVoltage || 0);
  const charging = Boolean(status?.chargerConnected);
  const low = Boolean(status?.batteryLow) || percent <= 15;
  const Icon = charging ? BatteryCharging : Battery;

  return (
    <section className={`controlCard powerMini ${charging ? "charging" : ""} ${low ? "low" : ""}`}>
      <div className="powerTop">
        <div>
          <span className="eyebrow">Battery</span>
          <p>{charging ? "Charger connected" : low ? "Low battery" : "On battery"}</p>
        </div>
        <div className="batteryPercent">
          <Icon size={22} />
          <b>{Math.round(percent)}%</b>
        </div>
      </div>

      <div className="batteryShell" title={`${voltage.toFixed(2)} V`}>
        <i style={{ width: `${percent}%` }} />
      </div>

      <div className="powerMetaLine">
        <span>{voltage > 0 ? `${voltage.toFixed(2)} V pack` : "Battery --"}</span>
        <span>{pinVoltage > 0 ? `${pinVoltage.toFixed(2)} V ADC` : "ADC --"}</span>
      </div>

      <div className={`chargerPill ${charging ? "on" : ""}`}>
        <PlugZap size={15} />
        <span>{charging ? "Dock charger detected" : "Dock charger not detected"}</span>
      </div>
    </section>
  );
}

function TechnicalNotePanel() {
  return (
    <section className="controlCard technicalNote">
      <span className="eyebrow">Technical page</span>
      <h3>Calibration tools</h3>
      <p>
        Use this page for load-cell tare/calibration and raw values. Staff view only keeps the map,
        live navigation, send, return, and emergency buttons.
      </p>
      <p>
        Magnetic sensor calibration can be added here after STM32 sends Hall raw/min/max values over UART.
      </p>
    </section>
  );
}

function TrayPanel({ status, espBaseUrl, refreshStatus, showToast }) {
  const [knownWeight, setKnownWeight] = useState("1050");
  const [manualScale, setManualScale] = useState("");
  const [rawResult, setRawResult] = useState(null);
  const [working, setWorking] = useState(false);

  const ready = Boolean(status?.trayReady);
  const weight = Number(status?.trayWeightG || 0);
  const threshold = Number(status?.foodThresholdG || 300);
  const hasFood = Boolean(status?.trayHasFood);
  const waiting = Boolean(status?.servingWaitingForFood);

  useEffect(() => {
    const scale = Number(status?.hx711Scale || rawResult?.scale || 0);
    if (scale && !manualScale) {
      setManualScale(Math.abs(scale).toFixed(3));
    }
  }, [status?.hx711Scale, rawResult?.scale, manualScale]);

  async function callWeightApi(path, successMessage) {
    setWorking(true);
    try {
      const data = await apiGet(espBaseUrl, path);
      setRawResult(data);
      showToast(successMessage || data.message || "Weight API updated");
      await refreshStatus();
    } catch (err) {
      showToast("Weight API failed: " + err.message, "error");
    } finally {
      setWorking(false);
    }
  }

  async function readRaw() {
    await callWeightApi("/api/weight/raw?times=10", "Raw reading updated");
  }

  async function tareEmptyTray() {
    if (!window.confirm("Tare now? Make sure the tray is EMPTY.")) return;
    await callWeightApi("/api/weight/tare?times=20", "Empty tray tare saved");
  }

  async function calibrateKnown() {
    const known = Number(knownWeight);
    if (!known || known <= 0) {
      showToast("Enter known weight in grams", "warn");
      return;
    }

    if (!window.confirm(`Calibrate using ${known} g? Place the known weight on the tray first.`)) return;

    await callWeightApi(`/api/weight/calibrate?known=${encodeURIComponent(known)}&times=20`, "Calibration saved");
  }

  async function saveManualScale() {
    const scale = Number(manualScale);
    if (!scale || Math.abs(scale) <= 0) {
      showToast("Enter calibration factor / scale", "warn");
      return;
    }

    await callWeightApi(`/api/weight/setscale?scale=${encodeURIComponent(scale)}`, "Calibration factor saved");
  }

  return (
    <section className={`controlCard trayPanel ${hasFood ? "food" : "empty"} ${waiting ? "waiting" : ""}`}>
      <span className="eyebrow">Tray sensor</span>

      <div className="trayWeight">
        <b>{ready ? `${weight.toFixed(1)} g` : "HX711 --"}</b>
        <span>{hasFood ? "Food detected" : "Tray empty / food taken"}</span>
      </div>

      <div className="trayBar">
        <i style={{ width: `${Math.max(0, Math.min(100, (weight / Math.max(threshold, 1)) * 100))}%` }} />
      </div>

      <p>
        Threshold: <b>{threshold.toFixed(0)} g</b>
        {waiting ? " · Waiting until food is taken or 5 min timeout" : ""}
      </p>

      <div className="trayApiGrid">
        <button disabled={working} onClick={readRaw}>Read raw</button>
        <button disabled={working} onClick={tareEmptyTray}>Tare empty</button>
      </div>

      <div className="calibrateRow">
        <input
          value={knownWeight}
          onChange={(e) => setKnownWeight(e.target.value)}
          inputMode="decimal"
          placeholder="Known g"
        />
        <button disabled={working} onClick={calibrateKnown}>Calibrate</button>
      </div>

      <div className="calibrateRow">
        <input
          value={manualScale}
          onChange={(e) => setManualScale(e.target.value)}
          inputMode="decimal"
          placeholder="Factor / scale"
        />
        <button disabled={working} onClick={saveManualScale}>Save factor</button>
      </div>

      <div className="weightMeta">
        <span>Raw: <b>{rawResult?.raw ?? status?.trayRawAverage ?? "--"}</b></span>
        <span>Offset: <b>{rawResult?.offset ?? status?.trayOffsetRaw ?? "--"}</b></span>
        <span>Scale: <b>{Number(rawResult?.scale ?? status?.hx711Scale ?? 0).toFixed(3)}</b></span>
        <span>Direction: <b>{rawResult?.rawDirection ?? status?.trayRawDirection ?? "--"}</b></span>
      </div>
    </section>
  );
}

function LiveEventWindow({ selectedTable, currentEvent, receivedEvents, elapsedMs }) {
  const list = selectedTable ? (EVENT_LISTS[selectedTable] || []) : [];
  const currentIndex = Math.max(0, list.indexOf(currentEvent));
  const prevEvent = currentIndex > 0 ? list[currentIndex - 1] : "";
  const nextEvent = currentIndex >= 0 && currentIndex < list.length - 1 ? list[currentIndex + 1] : "";

  return (
    <section className="controlCard liveMiniPanel">
      <div className="liveMiniTop">
        <div>
          <span className="eyebrow">Live navigation</span>
          <p>Current route stage</p>
        </div>
        <div className="elapsedLine compact">
          <Clock size={16} />
          <b>{formatElapsed(elapsedMs)}</b>
        </div>
      </div>

      {!selectedTable ? (
        <p>Select and send a table route to show live navigation.</p>
      ) : (
        <div className="miniChain">
          <div className="miniChainItem faded">
            <i>{prevEvent && receivedEvents.has(prevEvent) ? "✓" : "•"}</i>
            <span>{prevEvent ? EVENT_LABELS[prevEvent] : "Waiting"}</span>
          </div>

          <div className="miniChainItem active">
            <i>{currentEvent && receivedEvents.has(currentEvent) ? "✓" : "•"}</i>
            <span>{EVENT_LABELS[currentEvent] || currentEvent || "Waiting"}</span>
          </div>

          <div className="miniChainItem faded">
            <i>→</i>
            <span>{nextEvent ? EVENT_LABELS[nextEvent] : "Complete"}</span>
          </div>
        </div>
      )}
    </section>
  );
}

export default function App() {
  const [espUrlInput, setEspUrlInput] = useState(() => localStorage.getItem("robotEspUrl") || DEFAULT_ESP_URL);
  const espBaseUrl = useMemo(() => normalizeBaseUrl(espUrlInput), [espUrlInput]);
  const [selectedTable, setSelectedTable] = useState(0);
  const [activeTable, setActiveTable] = useState(0);
  const [status, setStatus] = useState(null);
  const [online, setOnline] = useState(false);
  const [busy, setBusy] = useState(false);
  const [toast, setToast] = useState(null);
  const [receivedEvents, setReceivedEvents] = useState(new Set());
  const [alertEvent, setAlertEvent] = useState("");
  const [localStartMs, setLocalStartMs] = useState(0);
  const [returnStartMs, setReturnStartMs] = useState(0);
  const [nowMs, setNowMs] = useState(Date.now());
  const [page, setPage] = useState("staff");
  const [ledModalOpen, setLedModalOpen] = useState(false);
  const lastEventSeqRef = useRef(0);

  const selected = selectedTable ? ROUTES[selectedTable] : null;
  const routeDone = (receivedEvents.has("DOCKED") || status?.currentEvent === "DOCKED") && !selectedTable && !status?.journeyActive;
  const journeyTable = routeDone ? 0 : (status?.journeyActive ? (status.activeTable || activeTable || selectedTable) : (selectedTable || activeTable));
  const elapsedMs = status?.journeyActive ? Number(status.elapsedMs || 0) : (localStartMs ? nowMs - localStartMs : 0);
  const returnElapsedMs = returnStartMs ? nowMs - returnStartMs : 0;
  const robotVisual = getRobotVisualState(journeyTable, status, receivedEvents, elapsedMs, returnElapsedMs);
  const servingTable = ["SERVING", "FOOD_TAKEN_CONFIRMED"].includes(status?.currentEvent || "") || status?.servingWaitingForFood ? journeyTable : 0;
  const currentLiveEvent = status?.currentEvent && !["SENDING", "ROUTE_ACCEPTED", "IDLE"].includes(status.currentEvent)
    ? status.currentEvent
    : (receivedEvents.has("START") ? "START" : "");

  function showToast(message, type = "", loading = false) {
    setToast({ message, type, loading });
    if (!loading) {
      clearTimeout(window.__toastTimer);
      window.__toastTimer = setTimeout(() => setToast(null), 3000);
    }
  }

  function handleSelectTable(table) {
    setSelectedTable(table);

    if (!status?.journeyActive) {
      setActiveTable(0);
      setAlertEvent("");
      setReturnStartMs(0);
      setLocalStartMs(0);
      setReceivedEvents(new Set());
      lastEventSeqRef.current = 0;
    }
  }

  function saveEspUrl() {
    const normalized = normalizeBaseUrl(espUrlInput);
    localStorage.setItem("robotEspUrl", normalized);
    setEspUrlInput(normalized);
    showToast("ESP address saved");
    setTimeout(refreshStatus, 100);
  }

  function applyOneEvent(table, event) {
    if (!event || ["SENDING", "ROUTE_ACCEPTED", "IDLE"].includes(event)) return;

    if (event === "RETURNING") {
      setReturnStartMs((old) => old || Date.now());
    }

    if (event === "DOCKED") {
      setReceivedEvents(new Set(["DOCKED"]));
      setAlertEvent("");
      setSelectedTable(0);
      setActiveTable(0);
      setLocalStartMs(0);
      setReturnStartMs(0);
      showToast("Robot docked");
      return;
    }

    setReceivedEvents((prev) => completeEventsUpTo(table, event, prev));
  }

  function applyStatusEvents(data) {
    const table = data.activeTable || activeTable || selectedTable;
    if (!table) return;

    if (Array.isArray(data.events) && data.events.length > 0) {
      const newEvents = data.events
        .filter((e) => Number(e.seq || 0) > lastEventSeqRef.current)
        .sort((a, b) => Number(a.seq || 0) - Number(b.seq || 0));

      for (const e of newEvents) {
        lastEventSeqRef.current = Math.max(lastEventSeqRef.current, Number(e.seq || 0));
        applyOneEvent(table, e.event);
      }

      return;
    }

    if (data?.currentEvent && data.eventSeq && data.eventSeq !== lastEventSeqRef.current) {
      lastEventSeqRef.current = data.eventSeq;
      applyOneEvent(table, data.currentEvent);
    }
  }

  function checkTimeouts(data = status) {
    const table = data?.activeTable || activeTable || selectedTable;
    if (!table || !data?.journeyActive) {
      setAlertEvent("");
      return;
    }

    const isReturning = receivedEvents.has("RETURNING");
    const next = getNextExpectedEvent(table, receivedEvents, isReturning);
    if (!next) {
      setAlertEvent("");
      return;
    }

    const timeBaseMs = isReturning ? returnElapsedMs : Number(data.elapsedMs || 0);
    const elapsedSec = timeBaseMs / 1000;

    if (elapsedSec > next.t + TIMEOUT_MARGIN_SEC) {
      setAlertEvent(next.event);
    } else {
      setAlertEvent("");
    }
  }

  async function refreshStatus() {
    try {
      const data = await apiGet(espBaseUrl, "/api/status");
      setStatus(data);
      setOnline(true);
      if (data.activeTable) setActiveTable(data.activeTable);
      applyStatusEvents(data);
      checkTimeouts(data);
    } catch {
      setOnline(false);
    }
  }

  async function sendSelected() {
    if (!selectedTable) {
      showToast("Select a table from the map first", "warn");
      return;
    }
    if (!window.confirm(`Send robot to ${selected.name}?`)) return;

    setBusy(true);
    lastEventSeqRef.current = 0;
    setReturnStartMs(0);
    setAlertEvent("");
    setReceivedEvents(new Set());
    showToast(`Sending ${selected.name} route and waiting for STM32 ACK...`, "", true);

    try {
      const data = await apiGet(espBaseUrl, `/api/serve?table=${selectedTable}`);
      setStatus(data);
      setOnline(true);
      setActiveTable(selectedTable);
      setLocalStartMs(Date.now());
      applyStatusEvents(data);
      showToast(data.message || `${selected.name} route acknowledged`);
    } catch (err) {
      setOnline(false);
      setActiveTable(0);
      setLocalStartMs(0);
      setReceivedEvents(new Set());
      showToast("Failed: " + err.message, "error");
    } finally {
      setBusy(false);
    }
  }


  async function sendReturn() {
    if (!window.confirm("Return robot to dock from current position?")) return;

    setBusy(true);
    showToast("Sending RETURN and waiting for STM32 ACK...", "", true);

    try {
      const data = await apiGet(espBaseUrl, "/api/return");
      setStatus(data);
      setOnline(true);
      applyStatusEvents(data);
      showToast(data.message || "Return acknowledged");
    } catch (err) {
      showToast("Return failed: " + err.message, "error");
    } finally {
      setBusy(false);
    }
  }

  async function sendStop() {
    if (!window.confirm("Send STOP command?")) return;
    setBusy(true);
    showToast("Sending STOP...", "", true);

    try {
      const data = await apiGet(espBaseUrl, "/api/stop");
      setStatus(data);
      setReceivedEvents((prev) => new Set([...prev, "STOPPED"]));
      setAlertEvent("");
      showToast(data.message || "STOP sent");
    } catch (err) {
      showToast("Stop failed: " + err.message, "error");
    } finally {
      setBusy(false);
    }
  }

  useEffect(() => {
    refreshStatus();
    const pollTimer = setInterval(refreshStatus, 700);
    const clockTimer = setInterval(() => {
      setNowMs(Date.now());
      checkTimeouts();
    }, 200);

    const keyHandler = (e) => {
      if (e.key === "1") handleSelectTable(1);
      if (e.key === "2") handleSelectTable(2);
      if (e.key === "3") handleSelectTable(3);
      if (e.key === "Enter" && selectedTable) sendSelected();
    };

    window.addEventListener("keydown", keyHandler);
    return () => {
      clearInterval(pollTimer);
      clearInterval(clockTimer);
      window.removeEventListener("keydown", keyHandler);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [espBaseUrl, selectedTable, activeTable, receivedEvents, nowMs, returnStartMs]);

  return (
    <div className="app">
      <Toast toast={toast} />
      {ledModalOpen && (
        <LedControlModal
          status={status}
          espBaseUrl={espBaseUrl}
          refreshStatus={refreshStatus}
          showToast={showToast}
          onClose={() => setLedModalOpen(false)}
        />
      )}
      <header className="header">
        <div className="brand">
          <div className="logo"><Bot size={27} /></div>
          <div>
            <h1>Restaurant Robot</h1>
            <p>Live local WiFi navigation dashboard</p>
          </div>
        </div>
        <div className="headerRight">
          <div className="pageSwitch">
            <button className={page === "staff" ? "active" : ""} onClick={() => setPage("staff")}>Staff</button>
            <button className={page === "technical" ? "active" : ""} onClick={() => setPage("technical")}>Technical</button>
          </div>
          <StatusChip online={online} status={status} />
          <button className="ledHeaderButton" onClick={() => setLedModalOpen(true)} title="LED settings">
            <Lightbulb size={17} />
            LEDs
          </button>
          <button className="iconButton" onClick={refreshStatus} title="Refresh">
            <RefreshCw size={17} />
          </button>
        </div>
      </header>

      {page === "staff" ? (
        <main className="content">
          <DiningMap
            selectedTable={journeyTable || selectedTable}
            setSelectedTable={setSelectedTable}
            robotVisual={robotVisual}
            receivedEvents={receivedEvents}
            alertEvent={alertEvent}
            servingTable={servingTable}
            obstacleActive={Boolean(status?.obstaclePaused || status?.obstacleHelpNeeded)}
          />

          <aside className="controlPanel">
            <section className={`controlCard destination ${selected?.accent || ""}`}>
              <span className="eyebrow">Destination</span>
              <h2>{selected ? selected.name : "Select table"}</h2>
              <p>{selected ? selected.short : "Tap a table on the restaurant map."}</p>

              <button className="sendButton" disabled={!selected || busy} onClick={sendSelected}>
                <Send size={18} />
                Send robot
              </button>

              <button className="returnButton" disabled={busy} onClick={sendReturn}>
                <RefreshCw size={17} />
                Return to dock
              </button>

              <button className="stopButton" disabled={busy} onClick={sendStop}>
                <AlertTriangle size={17} />
                Emergency stop
              </button>
            </section>

            <TrayStatusMini status={status} />

            <PowerStatusMini status={status} />

            <ObstaclePanel status={status} espBaseUrl={espBaseUrl} refreshStatus={refreshStatus} showToast={showToast} />

            <LiveEventWindow
              selectedTable={journeyTable || selectedTable}
              currentEvent={currentLiveEvent}
              receivedEvents={receivedEvents}
              elapsedMs={elapsedMs}
            />
          </aside>
        </main>
      ) : (
        <main className="content technicalContent">
          <aside className="controlPanel technicalPanel">
            <section className="controlCard addressCard">
              <div className="cardTitle">
                <span className="eyebrow">ESP controller</span>
                <Settings size={17} />
              </div>
              <div className="inputRow">
                <input
                  value={espUrlInput}
                  onChange={(e) => setEspUrlInput(e.target.value)}
                  placeholder="http://192.168.4.1"
                />
                <button onClick={saveEspUrl}>Save</button>
              </div>
              <p>Connect to <b>RestaurantRobot_AP</b>, then use <b>http://192.168.4.1</b></p>
            </section>

            <TrayPanel status={status} espBaseUrl={espBaseUrl} refreshStatus={refreshStatus} showToast={showToast} />

            <SensorTechnicalPanel status={status} espBaseUrl={espBaseUrl} refreshStatus={refreshStatus} showToast={showToast} />

            <PowerStatusMini status={status} />

            <HallTechnicalPanel espBaseUrl={espBaseUrl} showToast={showToast} />

            <section className="controlCard telemetry">
              <span className="eyebrow">Telemetry</span>
              <Info icon={Wifi} label="Connection" value={online ? "Online" : "Offline"} />
              <Info icon={Radio} label="Robot API" value={espBaseUrl} />
              <Info icon={Signal} label="STM32 event" value={status?.currentEvent || "--"} />
              <Info icon={MapPin} label="Last route" value={status?.lastRoute || "None"} />
              <Info icon={Battery} label="Battery" value={status ? `${Number(status.batteryPercent || 0).toFixed(0)}% · ${Number(status.batteryVoltage || 0).toFixed(2)} V` : "--"} />
              <Info icon={PlugZap} label="Charger" value={status?.chargerConnected ? "Connected" : "Not connected"} />
              <Info icon={Activity} label="Uptime" value={formatUptime(status?.uptimeSec)} />
            </section>

            <TechnicalNotePanel />
          </aside>
        </main>
      )}
    </div>
  );
}
