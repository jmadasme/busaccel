/* =====================================================================
   BusAccel-8266 — frontend del monitor en tiempo real
   Canvas nativo, sin CDN, dark theme con glow.
   ===================================================================== */
"use strict";

/* ----------------------------- Config ----------------------------- */
const COLORS = {
  z: "#00e5ff",
  x: "#ffb300",
  y: "#b388ff",
  setline: "#9aa4b2",
  leve: "#ffb300",
  severo: "#ff2d55",
  grid: "rgba(139, 152, 169, 0.10)",
  gridTxt: "#5c6a7d",
  text: "#8b98a9",
  marker: {
    "+": "#00e676",
    "-": "#448aff",
    "*": "#ff453a",
    "!": "#ff2dff",
  },
};

const SYM_META = {
  ".": { label: "REPOSO", sub: "sin desviación" },
  "+": { label: "ELEVACIÓN LEVE", sub: "desviación positiva" },
  "-": { label: "CAÍDA LEVE", sub: "desviación negativa" },
  "*": { label: "ELEVACIÓN SEVERA", sub: "desviación ≥ umbral" },
  "!": { label: "CAÍDA SEVERA", sub: "desviación ≥ umbral" },
};

const MARGIN = { top: 16, right: 18, bottom: 30, left: 52 };

/* ----------------------------- Estado ----------------------------- */
const state = {
  points: [],        // todos los puntos recibidos (con t absoluto en ms)
  since: 0,          // indice absoluto del último punto recibido
  t0: null,          // millis del primer punto
  events: [],        // detecciones recientes {sec, sym, z, sz}
  last: null,        // último punto
  status: "idle",
  port: "",
  error: null,
  sim: false,
  counts: { ".": 0, "+": 0, "-": 0, "*": 0, "!": 0 },
  windowSec: 60,
  paused: false,
  showX: false,
  showY: false,
  freezeAnchor: null, // marca de tiempo (sec) al pausar
  leve: 0.20,
  severo: 0.24,
  lastHz: 0,
};

const els = {
  portSelect: document.getElementById("portSelect"),
  baudSelect: document.getElementById("baudSelect"),
  connectBtn: document.getElementById("connectBtn"),
  connPill: document.getElementById("connPill"),
  connLabel: document.getElementById("connLabel"),
  windowRange: document.getElementById("windowRange"),
  windowVal: document.getElementById("windowVal"),
  thLeve: document.getElementById("thLeve"),
  thSevero: document.getElementById("thSevero"),
  chkX: document.getElementById("chkX"),
  chkY: document.getElementById("chkY"),
  chkPause: document.getElementById("chkPause"),
  clearBtn: document.getElementById("clearBtn"),
  simBtn: document.getElementById("simBtn"),
  overlay: document.getElementById("emptyOverlay"),
  badge: document.getElementById("badge"),
  badgeSym: document.getElementById("badgeSym"),
  badgeLabel: document.getElementById("badgeLabel"),
  badgeSub: document.getElementById("badgeSub"),
  statHz: document.getElementById("statHz"),
  statSamples: document.getElementById("statSamples"),
  statT: document.getElementById("statT"),
  statSet: document.getElementById("statSet"),
  cntPlus: document.getElementById("cntPlus"),
  cntMinus: document.getElementById("cntMinus"),
  cntStar: document.getElementById("cntStar"),
  cntExcl: document.getElementById("cntExcl"),
  eventList: document.getElementById("eventList"),
  stPort: document.getElementById("stPort"),
  stError: document.getElementById("stError"),
  stRaw: document.getElementById("stRaw"),
};

const canvas = document.getElementById("chart");
const ctx = canvas.getContext("2d");

/* ----------------------------- Util ----------------------------- */
function fmt(n, d = 2) {
  return Number(n).toFixed(d);
}

function niceStep(range, target = 5) {
  const raw = range / target;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const norm = raw / mag;
  const step = norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10;
  return step * mag;
}

/* ----------------------------- Canvas / HiDPI ----------------------------- */
function resizeCanvas() {
  const wrap = canvas.parentElement;
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(wrap.clientWidth * dpr);
  canvas.height = Math.floor(wrap.clientHeight * dpr);
  canvas.style.width = wrap.clientWidth + "px";
  canvas.style.height = wrap.clientHeight + "px";
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
new ResizeObserver(resizeCanvas).observe(canvas.parentElement);

/* ----------------------------- API ----------------------------- */
async function api(path) {
  try {
    const r = await fetch(path, { cache: "no-store" });
    return await r.json();
  } catch (e) {
    return null;
  }
}

async function poll() {
  if (state.paused) return;
  const data = await api("/api/state?since=" + state.since);
  if (!data) return;
  applyState(data);
  if (data.points && data.points.length) {
    mergePoints(data.points, data.t0, data.wrap);
    state.since = data.since;
  }
}

function applyState(data) {
  state.status = data.status;
  state.port = data.port;
  state.error = data.error;
  state.sim = data.sim;
  state.t0 = data.t0;
  state.counts = data.counts || state.counts;
  state.leve = data.thresholds ? data.thresholds.leve : state.leve;
  state.severo = data.thresholds ? data.thresholds.severo : state.severo;

  els.thLeve.textContent = fmt(state.leve);
  els.thSevero.textContent = fmt(state.severo);
  els.cntPlus.textContent = state.counts["+"] || 0;
  els.cntMinus.textContent = state.counts["-"] || 0;
  els.cntStar.textContent = state.counts["*"] || 0;
  els.cntExcl.textContent = state.counts["!"] || 0;
  els.stPort.textContent = "Puerto: " + (state.sim ? "SIMULADOR" : state.port || "—");
  els.stError.textContent = state.error || "";
  els.stRaw.textContent = "Última línea: " + (data.raw || "—");

  // Estado visual
  const pill = els.connPill;
  pill.className = "pill pill-" + mapStatusClass(state.status);
  els.connLabel.textContent = statusText(state.status, state.sim);
  els.connectBtn.classList.toggle("connected", isLive(state.status));
  els.connectBtn.textContent = isLive(state.status) ? "Desconectar" : "Conectar";
  els.overlay.classList.toggle("hidden", state.status === "connected" || state.status === "simulator");
  els.simBtn.style.display = state.sim ? "none" : "";

  // Lista de puertos
  if (data.ports && data.ports.length && !isLive(state.status)) {
    const cur = els.portSelect.value;
    const opts = data.ports.map((p) => `<option value="${p}">${p}</option>`).join("");
    if (els.portSelect.innerHTML !== opts) {
      els.portSelect.innerHTML = opts;
      if (data.port) els.portSelect.value = data.port;
      else if (cur) els.portSelect.value = cur;
    }
  }

  // Último punto
  if (data.last) {
    updateBadge(data.last.c, data.last);
    const sec = data.t0 != null ? (data.last.t - data.t0) / 1000 : 0;
    els.statT.textContent = fmt(sec, 1);
    els.statSet.textContent = fmt(data.last.sz);
  }
  els.statSamples.textContent = String(data.total || 0);
  els.statHz.textContent = fmt(state.lastHz, 1);
}

function mapStatusClass(s) {
  return s === "connected" ? "connected" : s === "simulator" ? "sim" : s === "connecting" ? "connecting" : s === "error" ? "error" : "idle";
}
function statusText(s, sim) {
  if (sim) return "Simulador activo";
  switch (s) {
    case "connected": return "Conectado";
    case "connecting": return "Conectando…";
    case "error": return "Error";
    default: return "Sin conexión";
  }
}
function isLive(s) {
  return s === "connected" || s === "simulator";
}

function mergePoints(pts, t0, wrap) {
  if (!pts.length) return;
  const first = pts[0];
  // Cálculo de frecuencia con los t del dispositivo
  if (pts.length >= 2) {
    const dt = (pts[pts.length - 1].t - first.t) / (pts.length - 1);
    if (dt > 0) state.lastHz = Math.min(100, 1000 / dt);
  }
  state.points.push(...pts);
  // Retener solo lo necesario (ventana + margen)
  const maxKeep = Math.ceil((state.windowSec + 10) * 20) + 50;
  if (state.points.length > maxKeep) {
    state.points.splice(0, state.points.length - maxKeep);
  }
  // Eventos de detección
  for (const p of pts) {
    if (p.c !== ".") {
      const sec = t0 != null ? (p.t - t0) / 1000 : 0;
      state.events.push({
        sec,
        sym: p.c,
        z: p.z,
        sz: p.sz,
        born: performance.now() / 1000,
      });
    }
  }
  state.events = state.events.filter((e) => performance.now() / 1000 - e.born < 4);
  if (state.events.length > 60) state.events.splice(0, state.events.length - 60);
  pushEventItems(pts, t0);
}

function pushEventItems(pts, t0) {
  const fresh = pts.filter((p) => p.c !== ".");
  if (!fresh.length) return;
  let changed = false;
  for (const p of fresh) {
    const sec = t0 != null ? (p.t - t0) / 1000 : 0;
    const delta = p.z - p.sz;
    const li = document.createElement("li");
    li.className = "event-item";
    const sym = p.c;
    const color = COLORS.marker[sym] || "#9aa4b2";
    li.innerHTML =
      `<span class="event-sym" style="color:${color};text-shadow:0 0 8px ${color}">${sym}</span>` +
      `<span class="event-time">t=${fmt(sec, 1)}s</span>` +
      `<span class="event-delta" style="color:${color}">${delta >= 0 ? "+" : ""}${fmt(delta)} g</span>`;
    els.eventList.prepend(li);
    changed = true;
  }
  if (changed) {
    while (els.eventList.children.length > 8) {
      els.eventList.removeChild(els.eventList.lastChild);
    }
    const empty = els.eventList.querySelector(".event-empty");
    if (empty) empty.remove();
  }
}

function updateBadge(sym, p) {
  const meta = SYM_META[sym] || SYM_META["."];
  els.badge.className = "badge sym-" + (sym === "." ? "dot" : sym === "+" ? "plus" : sym === "-" ? "minus" : sym === "*" ? "star" : "excl");
  els.badgeSym.textContent = sym;
  els.badgeLabel.textContent = meta.label;
  els.badgeSub.textContent = meta.sub;
}

/* ----------------------------- Dibujo ----------------------------- */
function nowSec() {
  return state.t0 != null ? (state.points.length ? (state.points[state.points.length - 1].t - state.t0) / 1000 : 0) : 0;
}

function render() {
  const W = canvas.parentElement.clientWidth;
  const H = canvas.parentElement.clientHeight;
  if (!W || !H) {
    requestAnimationFrame(render);
    return;
  }
  ctx.clearRect(0, 0, W, H);

  const hasData = state.points.length > 0;
  if (!hasData) {
    requestAnimationFrame(render);
    return;
  }

  const anchor = state.paused && state.freezeAnchor != null ? state.freezeAnchor : nowSec();
  const winStart = Math.max(0, anchor - state.windowSec);
  const winEnd = anchor;

  // Puntos visibles en la ventana
  const visible = state.points.filter((p) => {
    const s = (p.t - state.t0) / 1000;
    return s >= winStart && s <= winEnd;
  });
  if (!visible.length) {
    requestAnimationFrame(render);
    return;
  }

  // Setpoint de referencia (el del último punto visible)
  const lastP = visible[visible.length - 1];
  const setZ = lastP.sz;

  // Rango Y
  let minY = Infinity, maxY = -Infinity;
  for (const p of visible) {
    minY = Math.min(minY, p.z, state.showX ? p.x : p.z, state.showY ? p.y : p.z, setZ);
    maxY = Math.max(maxY, p.z, state.showX ? p.x : p.z, state.showY ? p.y : p.z, setZ);
  }
  minY = Math.min(minY, setZ - state.severo);
  maxY = Math.max(maxY, setZ + state.severo);
  const pad = Math.max(0.15, (maxY - minY) * 0.18);
  minY -= pad;
  maxY += pad;
  if (maxY - minY < 0.8) {
    const mid = (maxY + minY) / 2;
    minY = mid - 0.4;
    maxY = mid + 0.4;
  }

  const plotW = W - MARGIN.left - MARGIN.right;
  const plotH = H - MARGIN.top - MARGIN.bottom;
  const plot = {
    x0: MARGIN.left,
    y0: MARGIN.top,
    x1: MARGIN.left + plotW,
    y1: MARGIN.top + plotH,
  };
  const X = (sec) => plot.x0 + ((sec - winStart) / (winEnd - winStart || 1)) * plotW;
  const Y = (g) => plot.y1 - ((g - minY) / (maxY - minY || 1)) * plotH;

  /* --- Zonas de magnitud (leve / severo) --- */
  const zLeveP = Y(setZ + state.leve);
  const zSeveroP = Y(setZ + state.severo);
  const zLeveN = Y(setZ - state.leve);
  const zSeveroN = Y(setZ - state.severo);
  const zoneFill = (yTop, yBot, color, alpha) => {
    const top = Math.max(plot.y0, Math.min(yTop, plot.y1));
    const bot = Math.min(plot.y1, Math.max(yBot, plot.y0));
    if (bot <= top) return;
    ctx.fillStyle = color;
    ctx.globalAlpha = alpha;
    ctx.fillRect(plot.x0, top, plotW, bot - top);
    ctx.globalAlpha = 1;
  };
  // severo (arriba)
  zoneFill(plot.y0, zSeveroP, COLORS.severo, 0.07);
  // leve (arriba)
  zoneFill(zSeveroP, zLeveP, COLORS.leve, 0.06);
  // leve (abajo)
  zoneFill(zLeveN, zSeveroN, COLORS.leve, 0.06);
  // severo (abajo)
  zoneFill(zSeveroN, plot.y1, COLORS.severo, 0.07);
  // borde de las zonas
  ctx.strokeStyle = COLORS.leve;
  ctx.globalAlpha = 0.28;
  ctx.setLineDash([4, 5]);
  ctx.beginPath();
  ctx.moveTo(plot.x0, zLeveP); ctx.lineTo(plot.x1, zLeveP);
  ctx.moveTo(plot.x0, zLeveN); ctx.lineTo(plot.x1, zLeveN);
  ctx.stroke();
  ctx.globalAlpha = 0.3;
  ctx.strokeStyle = COLORS.severo;
  ctx.beginPath();
  ctx.moveTo(plot.x0, zSeveroP); ctx.lineTo(plot.x1, zSeveroP);
  ctx.moveTo(plot.x0, zSeveroN); ctx.lineTo(plot.x1, zSeveroN);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.globalAlpha = 1;

  /* --- Rejilla horizontal --- */
  const step = niceStep(maxY - minY, 5);
  ctx.font = "11px 'JetBrains Mono', Consolas, monospace";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let g = Math.ceil(minY / step) * step; g <= maxY; g += step) {
    const y = Y(g);
    ctx.strokeStyle = COLORS.grid;
    ctx.beginPath();
    ctx.moveTo(plot.x0, y);
    ctx.lineTo(plot.x1, y);
    ctx.stroke();
    ctx.fillStyle = COLORS.gridTxt;
    ctx.fillText(g.toFixed(2), plot.x0 - 8, y);
  }

  /* --- Rejilla vertical (cada 5 s) --- */
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  const tStep = 5;
  for (let s = Math.ceil(winStart / tStep) * tStep; s <= winEnd; s += tStep) {
    const x = X(s);
    ctx.strokeStyle = COLORS.grid;
    ctx.beginPath();
    ctx.moveTo(x, plot.y0);
    ctx.lineTo(x, plot.y1);
    ctx.stroke();
    ctx.fillStyle = COLORS.gridTxt;
    ctx.fillText("-" + Math.round(winEnd - s) + "s", x, plot.y1 + 8);
  }

  /* --- Línea de setpoint (punteada) --- */
  const ySet = Y(setZ);
  ctx.strokeStyle = COLORS.setline;
  ctx.setLineDash([7, 5]);
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(plot.x0, ySet);
  ctx.lineTo(plot.x1, ySet);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = COLORS.setline;
  ctx.textAlign = "left";
  ctx.fillText("SETPOINT " + setZ.toFixed(2) + " g", plot.x0 + 6, ySet - 9);
  // glow sutil
  ctx.shadowColor = "rgba(154,164,178,0.5)";
  ctx.shadowBlur = 6;
  ctx.fillText("", 0, 0);
  ctx.shadowBlur = 0;

  /* --- Curvas --- */
  drawCurve(visible, (p) => p.z, COLORS.z, 2.4, true, X, Y);
  if (state.showX) drawCurve(visible, (p) => p.x, COLORS.x, 1.4, false, X, Y);
  if (state.showY) drawCurve(visible, (p) => p.y, COLORS.y, 1.4, false, X, Y);

  /* --- Punto cabeza + pulso --- */
  const headSec = (lastP.t - state.t0) / 1000;
  const hx = X(headSec), hy = Y(lastP.z);
  const pulse = (performance.now() / 1000) % 1.4;
  ctx.beginPath();
  ctx.arc(hx, hy, 5 + pulse * 7, 0, Math.PI * 2);
  ctx.strokeStyle = "rgba(0,229,255,0.35)";
  ctx.lineWidth = 1.6;
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(hx, hy, 3.4, 0, Math.PI * 2);
  ctx.fillStyle = COLORS.z;
  ctx.shadowColor = COLORS.z;
  ctx.shadowBlur = 12;
  ctx.fill();
  ctx.shadowBlur = 0;

  /* --- Marcadores de detección --- */
  const now = performance.now() / 1000;
  for (const e of state.events) {
    if (e.sec < winStart || e.sec > winEnd) continue;
    const age = now - e.born;
    const fade = Math.max(0, 1 - age / 4);
    const mx = X(e.sec);
    const my = Y(e.z);
    const color = COLORS.marker[e.sym] || "#9aa4b2";
    const r = 6 + age * 2.5;
    ctx.globalAlpha = fade;
    // halo
    const grad = ctx.createRadialGradient(mx, my, 0, mx, my, r * 3);
    grad.addColorStop(0, color + "55");
    grad.addColorStop(1, "transparent");
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(mx, my, r * 3, 0, Math.PI * 2);
    ctx.fill();
    // punto
    ctx.fillStyle = color;
    ctx.shadowColor = color;
    ctx.shadowBlur = 14;
    ctx.beginPath();
    ctx.arc(mx, my, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.shadowBlur = 0;
    // símbolo
    ctx.fillStyle = "#0b0e14";
    ctx.font = "bold 11px 'JetBrains Mono', Consolas, monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(e.sym, mx, my + 0.5);
    ctx.globalAlpha = 1;
  }

  requestAnimationFrame(render);
}

function drawCurve(pts, accessor, color, width, glow, X, Y) {
  ctx.beginPath();
  let started = false;
  for (const p of pts) {
    const s = (p.t - state.t0) / 1000;
    const x = X(s);
    const y = Y(accessor(p));
    if (!started) {
      ctx.moveTo(x, y);
      started = true;
    } else {
      ctx.lineTo(x, y);
    }
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  if (glow) {
    ctx.shadowColor = color;
    ctx.shadowBlur = 10;
  }
  ctx.stroke();
  ctx.shadowBlur = 0;
}

/* ----------------------------- Controles ----------------------------- */
function connect() {
  const port = els.portSelect.value || "/dev/ttyUSB0";
  const baud = els.baudSelect.value;
  api(`/api/connect?port=${encodeURIComponent(port)}&baud=${baud}`);
}

function disconnect() {
  api("/api/disconnect");
}

els.connectBtn.addEventListener("click", () => {
  if (isLive(state.status)) disconnect();
  else connect();
});

els.simBtn.addEventListener("click", () => {
  api("/api/connect?port=SIM&baud=115200");
});

els.portSelect.addEventListener("change", () => {
  if (isLive(state.status)) connect();
});
els.baudSelect.addEventListener("change", () => {
  if (isLive(state.status)) connect();
});

els.windowRange.addEventListener("input", () => {
  state.windowSec = Number(els.windowRange.value);
  els.windowVal.textContent = state.windowSec + " s";
});
els.chkX.addEventListener("change", () => { state.showX = els.chkX.checked; });
els.chkY.addEventListener("change", () => { state.showY = els.chkY.checked; });
els.chkPause.addEventListener("change", () => {
  state.paused = els.chkPause.checked;
  state.freezeAnchor = state.paused ? nowSec() : null;
});

els.clearBtn.addEventListener("click", async () => {
  await api("/api/clear");
  state.points = [];
  state.events = [];
  state.since = 0;
  state.lastHz = 0;
  els.eventList.innerHTML = '<li class="event-empty">Sin eventos todavía</li>';
  updateBadge(".", null);
});

/* ----------------------------- Arranque ----------------------------- */
async function init() {
  // Params de la URL (puerto/baud por defecto inyectados por el backend)
  const q = new URLSearchParams(location.search);
  const defPort = q.get("port") || "/dev/ttyUSB0";
  const defBaud = q.get("baud") || "115200";
  els.baudSelect.value = defBaud;

  const data = await api("/api/state?since=0");
  if (data) {
    if (data.ports && data.ports.length) {
      els.portSelect.innerHTML = data.ports.map((p) => `<option value="${p}">${p}</option>`).join("");
    } else {
      els.portSelect.innerHTML = `<option value="${defPort}">${defPort}</option>`;
    }
    if (data.port) els.portSelect.value = data.port;
    applyState(data);
  } else {
    els.portSelect.innerHTML = `<option value="${defPort}">${defPort}</option>`;
  }

  setInterval(poll, 40);
  requestAnimationFrame(render);
  poll();
}

init();