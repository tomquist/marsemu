/* marsemu dashboard: the X-ray view of a running device, fed by api/stream. */
(() => {
"use strict";
const NS = "http://www.w3.org/2000/svg";
const $ = (id) => document.getElementById(id);
const svg = $("stage");
const css = getComputedStyle(document.documentElement);
const C = (n) => css.getPropertyValue("--" + n).trim();
const reduced = matchMedia("(prefers-reduced-motion: reduce)").matches;
function el(tag, attrs, parent, text) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (text != null) e.textContent = text;
  if (parent) parent.appendChild(e);
  return e;
}
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const sum = (a) => a.reduce((x, y) => x + y, 0);
const pad = (n, w) => String(n).padStart(w, "0");
const hex = (n, w = 0) => "0x" + (n >>> 0).toString(16).toUpperCase().padStart(w, "0");
const sgn = (w) => (w >= 0 ? "+" : "−") + Math.abs(Math.round(w));
function fmtT(t) {
  const h = Math.floor(t / 3600), m = Math.floor(t / 60) % 60, s = t % 60;
  return `${pad(h, 2)}:${pad(m, 2)}:${s.toFixed(1).padStart(4, "0")}`;
}
function esc(s) { return String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;"); }
async function post(url, body, raw) {
  const r = await fetch(url, { method: "POST", body: raw ? body : JSON.stringify(body || {}), headers: raw ? {} : { "Content-Type": "application/json" } });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || `${r.status} ${r.statusText}`);
  return j;
}
function control(cmd) { return post("api/control", cmd).catch((e) => log("EVT", "", "control failed: " + e.message)); }

/* the device as the drawing sees it, from the latest snapshot */
let SN = null;                       // raw snapshot
const S = {
  t: 0, load: 0, sun: [0, 0], pv: [0, 0], pvV: [0, 0], out: [0, 0], outV: [0, 0], outA: [0, 0], target: 0, ac: 0, grid: 0,
  pbat: 0, packV: 0, I: 0, soc: 0, cells: new Array(16).fill(3.3), bal: new Array(16).fill(false),
  fets: { chg: true, dsg: true }, temp: 22, hs: 25, f: { wifi: false, ct: false, heat: false }, ext: [],
};
function fromSnap(sn) {
  SN = sn;
  if (sn.model !== MODEL) buildLayout(sn.model, sn.channels);
  S.t = sn.t; S.load = sn.load; S.grid = sn.grid; S.ac = sn.ac; S.f = sn.faults; S.ext = sn.ext || [];
  if (!sn.pack) {                       // the CT meter: no pack, no house of its own
    S.pbat = 0; S.packV = 0; S.I = 0; S.soc = 0; S.cells = []; S.bal = []; S.temp = 0;
    S.sun = [0, 0]; S.pv = [0, 0]; S.out = [0, 0]; S.target = 0; S.fets = { chg: true, dsg: true }; S.hs = 0;
    return;
  }
  S.pbat = sn.pack.w; S.packV = sn.pack.v; S.I = sn.pack.a; S.soc = sn.pack.soc / 100;
  S.cells = sn.pack.cells.map((v) => v / 1000);
  S.temp = sn.pack.temp;
  if (sn.model === "B2500") {
    S.sun = sn.pv.map((p) => p.avail); S.pv = sn.pv.map((p) => p.w); S.pvV = sn.pv.map((p) => p.v);
    S.out = sn.out.map((o) => o.w); S.outV = sn.out.map((o) => o.v); S.outA = sn.out.map((o) => o.a);
    S.target = sum(sn.out.map((o) => (o.on ? o.i_lim * (o.v || 35) : 0)));
    S.bal = S.cells.map((_, i) => !!(sn.afe.bal & (1 << i)));
    S.fets = { dsg: !!(sn.afe.fets & 1), chg: !!(sn.afe.fets & 2) };
    S.hs = sn.heatsink;
  } else {
    S.sun = sn.mppt ? sn.mppt.avail : [0, 0]; S.pv = sn.mppt ? sn.mppt.pv : [0, 0]; S.out = [0, 0];
    S.target = sn.inverter.reference ?? sn.inverter.ac;
    S.bal = S.cells.map(() => false);
    const mos = sn.bms.mos || {};
    S.fets = { dsg: !!mos.discharge, chg: !!mos.charge };
    S.hs = sn.inverter.temp_inv ?? 0;
  }
}
/* ------------------------------------------------------------ the drawing */
let MODEL = null, COMP = {}, POWER = {}, DATA = {}, CELLS = 14;
const P = {}; // path elements by key
const V = {}; // live text nodes
const SIL = {};
const CELL = [], cellSil = [];
const tracksG = $("tracks"), flowsG = $("flows"), compsG = $("comps"), capsG = $("caps"), senseG = $("sense"), cellsG = $("cellsG"), silG = $("silicon");
const boardsDecor = $("boardsDecor");
// shared positions: the control MCU, its Wi-Fi module and everything outside the box
const RF = {
  rfbroker: { d: "M545 0C700 -44 880 -46 975 -12", c: "rf", rf: true },
  rfphone: { d: "M545 0C760 -70 1010 -70 1100 -14", c: "rf", rf: true },
  rfct: { d: "M545 0C1040 -20 1215 250 1140 562", c: "rf", rf: true },
};
const OUTSIDE = {
  house: { x: 1030, y: 400, w: 120, h: 70, t: "HOUSE", kind: "ext" },
  ct: { x: 1040, y: 540, w: 100, h: 50, t: "CT002 METER", kind: "ext" },
  broker: { x: 975, y: -32, w: 110, h: 44, t: "MQTT BROKER", kind: "ext" },
  phone: { x: 1100, y: -32, w: 90, h: 44, t: "PHONE · BLE", kind: "ext" },
  fc41d: { x: 470, y: 36, w: 150, h: 44, t: "FC41D · WI-FI + BLE", kind: "model" },
};

function clearLayout() {
  for (const g of [tracksG, flowsG, senseG, compsG, cellsG, capsG, silG, boardsDecor]) g.replaceChildren();
  for (const o of [P, V, SIL]) for (const k of Object.keys(o)) delete o[k];
  CELL.length = 0; cellSil.length = 0;
  if (typeof caps !== "undefined") caps.length = 0;
}
function drawPaths() {
  for (const [k, f] of Object.entries(POWER)) {
    el("path", { d: f.d, class: "track hx", "data-k": k, "stroke-width": 7 }, tracksG);
    const halo = el("path", { d: f.d, class: `halo c-${f.c} hx`, "data-k": k }, flowsG);
    const flow = el("path", { d: f.d, class: `flow c-${f.c} hx`, "data-k": k }, flowsG);
    P[k] = { flow, halo, len: flow.getTotalLength(), c: f.c, w: 0, off: 0, dir: 1 };
  }
  for (const [k, f] of Object.entries(DATA)) {
    const p = el("path", { d: f.d, class: (f.rf ? "rftrack " : `dtrack c-${f.c} `) + "hx", "data-k": k }, tracksG);
    P[k] = { path: p, len: p.getTotalLength(), c: f.c };
  }
  el("path", { d: "M545 36V0", stroke: "var(--rf)", "stroke-width": 1.4, fill: "none", class: "hx", "data-k": "ant" }, tracksG);
  el("path", { d: "M537 6l8 -8l8 8", stroke: "var(--rf)", "stroke-width": 1.2, fill: "none" }, tracksG);
}
function comp(key) {
  const c = COMP[key];
  const g = el("g", { class: "comp hx", "data-mod": key, id: "m-" + key, tabindex: 0, role: "button", "aria-label": c.t }, compsG);
  el("rect", { class: "box", x: c.x, y: c.y, width: c.w, height: c.h, rx: 5 }, g);
  el("text", { class: "lab", x: c.x + 9, y: c.y + 15 }, g, c.t);
  if (c.kind === "real") {
    el("rect", { x: c.x + 9, y: c.y + 21, width: 66, height: 12, rx: 2, fill: "rgba(142,241,255,.12)", stroke: "rgba(142,241,255,.5)", "stroke-width": .6 }, g);
    el("text", { class: "tag real", x: c.x + 42, y: c.y + 29.5, "text-anchor": "middle" }, g, "REAL FIRMWARE");
  }
  return g;
}
function valText(key, dx, dy, cls = "val", anchor = "start") {
  const c = COMP[key];
  return el("text", { class: cls, x: c.x + dx, y: c.y + dy, "text-anchor": anchor }, $("m-" + key));
}
function chip(key, title) {
  // a die in the middle of a big part, with pins around it
  const c = COMP[key], g = $("m-" + key);
  for (let i = 0; i < 10; i++) {
    el("rect", { x: c.x + 14 + i * 14.5, y: c.y - 5, width: 6, height: 5, fill: "#2a3d52" }, g);
    el("rect", { x: c.x + 14 + i * 14.5, y: c.y + c.h, width: 6, height: 5, fill: "#2a3d52" }, g);
  }
  for (let i = 0; i < 9; i++) {
    el("rect", { x: c.x - 5, y: c.y + 14 + i * 14.5, width: 5, height: 6, fill: "#2a3d52" }, g);
    el("rect", { x: c.x + c.w, y: c.y + 14 + i * 14.5, width: 5, height: 6, fill: "#2a3d52" }, g);
  }
  el("rect", { x: c.x + 40, y: c.y + 40, width: 90, height: 76, rx: 3, fill: "#0b1520", stroke: "#2b4258" }, g);
  el("circle", { cx: c.x + 50, cy: c.y + 50, r: 2.5, fill: "#2b4258" }, g);
  V.mcuA = el("text", { class: "sub", x: c.x + 85, y: c.y + 68, "text-anchor": "middle" }, g, title);
  V.mcuB = el("text", { class: "val", x: c.x + 85, y: c.y + 89, "text-anchor": "middle", style: "font-size:13px" }, g, "");
  V.mcuC = el("text", { class: "sub", x: c.x + 85, y: c.y + 105, "text-anchor": "middle" }, g, "");
  V.mcuD = el("text", { class: "sub", x: c.x + 9, y: c.y + c.h - 9 }, g, "");
}
function drawOutside() {
  V.fc = valText("fc41d", 9, 34, "sub");
  V.house = valText("house", 9, 44); V.houses = valText("house", 9, 60, "sub");
  V.ct = valText("ct", 9, 37, "sub");
  V.broker = valText("broker", 9, 33, "sub"); V.phone = valText("phone", 9, 33, "sub");
  el("text", { class: "ext-lab", x: 1090, y: 672, "text-anchor": "middle" }, compsG, "GRID");
  el("path", { d: "M1078 668l12 -18l12 18M1082 662h16", stroke: "var(--fg-faint)", fill: "none" }, compsG);
}
function drawCells(x0, cw, cg, y, h, afeX, afeY, afeStep) {
  for (let i = 0; i <= CELLS; i++) {
    const x = i < CELLS ? x0 + i * (cw + cg) + 4 : x0 + (CELLS - 1) * (cw + cg) + cw - 4;
    const ax = afeX + i * afeStep;
    el("path", { d: `M${ax} ${afeY}V${y - 28}L${x} ${y}`, fill: "none", stroke: "var(--i2c)", "stroke-opacity": .22, "stroke-width": .8, class: "hx", "data-k": "sense" }, senseG);
  }
  for (let i = 0; i < CELLS; i++) {
    const x = x0 + i * (cw + cg);
    const g = el("g", { class: "comp hx", "data-mod": "pack", "data-cell": i }, cellsG);
    el("rect", { x, y, width: cw, height: h, rx: 3, fill: "#0c1622", stroke: "#25384c" }, g);
    el("rect", { x: x + cw * .28, y: y - 4, width: 6, height: 4, fill: "#3a5068" }, g);
    el("rect", { x: x + cw * .55, y: y - 4, width: 6, height: 4, fill: "#3a5068" }, g);
    const fill = el("rect", { x: x + 3, y: y + 3, width: cw - 6, height: h - 6, rx: 2, fill: "var(--charge)", "fill-opacity": .16 }, g);
    const lvl = el("rect", { x: x + 3, y: y + 3, width: cw - 6, height: 2, fill: "var(--charge)" }, g);
    el("text", { x: x + cw / 2, y: y + h - 8, "text-anchor": "middle", class: "sub", style: "font-size:8.5px" }, g, String(i + 1));
    const balDot = el("circle", { cx: x + cw / 2, cy: y + 14, r: 3, fill: "var(--warn)", opacity: 0 }, g);
    const hot = el("rect", { x: x - 1, y: y - 1, width: cw + 2, height: h + 2, rx: 4, fill: "none", stroke: "var(--alarm)", "stroke-width": 1.6, opacity: 0 }, g);
    CELL.push({ fill, lvl, balDot, hot, x, y, h });
    cellSil.push(el("text", { class: "sil", x: x + cw / 2, y: y + 26, "text-anchor": "middle", style: "font-size:8px" }, silG, ""));
    cellSil.push(el("text", { class: "sil dim", x: x + cw / 2, y: y + 38, "text-anchor": "middle", style: "font-size:7.5px" }, silG, ""));
  }
}
function silBox(x, y, w, h, lines, key) {
  el("rect", { class: "silbox", x, y, width: w, height: h, rx: 2 }, silG);
  SIL[key] = lines.map((txt, i) => el("text", { class: "sil" + (i === 0 ? " dim" : ""), x: x + 6, y: y + 12 + i * 11 }, silG, txt));
}
function board(x, y, w, h, label) {
  el("rect", { class: "board", x, y, width: w, height: h, rx: 8, style: "fill:url(#pcb)" }, boardsDecor);
  el("rect", { x, y, width: w, height: h, rx: 8, fill: "none", stroke: "#1d3042" }, boardsDecor);
  el("text", { class: "ext-lab", x: x + 12, y: y + 18 }, boardsDecor, label);
}

/* ---- B2500: one MCU does everything */
function buildB2500() {
  CELLS = 14;
  COMP = {
    pv1: { x: 230, y: 80, w: 150, h: 86, t: "PV 1 · BUCK + MPPT", kind: "model" },
    pv2: { x: 230, y: 210, w: 150, h: 86, t: "PV 2 · BUCK + MPPT", kind: "model" },
    mcu: { x: 480, y: 120, w: 170, h: 150, t: "N32G452 · CORTEX-M4F", kind: "real" },
    out1: { x: 770, y: 80, w: 160, h: 86, t: "OUT 1 · CURRENT SOURCE", kind: "model" },
    out2: { x: 770, y: 210, w: 160, h: 86, t: "OUT 2 · CURRENT SOURCE", kind: "model" },
    afe: { x: 255, y: 368, w: 150, h: 56, t: "SH367309 · FRONT END", kind: "model" },
    fets: { x: 535, y: 368, w: 80, h: 44, t: "CHG · DSG", kind: "model" },
    inv: { x: 1010, y: 140, w: 160, h: 130, t: "MICROINVERTER", kind: "ext" },
    panel1: { x: 30, y: 80, w: 140, h: 86, t: "PANEL 1", kind: "ext" },
    panel2: { x: 30, y: 210, w: 140, h: 86, t: "PANEL 2", kind: "ext" },
    extp: { x: 30, y: 470, w: 140, h: 100, t: "EXTENSION PACK 1", kind: "ext" },
    ...OUTSIDE,
  };
  POWER = {
    pv1in: { d: "M170 123H230", c: "sol" }, pv2in: { d: "M170 253H230", c: "sol" },
    pv1bus: { d: "M380 123H410V340", c: "sol" }, pv2bus: { d: "M380 253H410V340", c: "sol" },
    busL: { d: "M410 340H575", c: "sol" }, busR: { d: "M575 340H740", c: "dis" },
    bat: { d: "M575 340V470", c: "chg" },
    o1: { d: "M740 340V123H770", c: "dis" }, o2: { d: "M740 340V253H770", c: "dis" },
    o1inv: { d: "M930 123H985V175H1010", c: "dis" }, o2inv: { d: "M930 253H985V235H1010", c: "dis" },
    acl: { d: "M1090 270V400", c: "dis" }, gridl: { d: "M1090 650V470", c: "dis" },
  };
  DATA = {
    usart: { d: "M480 140H452V58H470", c: "uart" },
    i2c: { d: "M520 270V300H330V368", c: "i2c" },
    uart5: { d: "M630 270V318H222V520H170", c: "ext" },
    pwm1: { d: "M480 175H395V150H380", c: "ana" }, pwm2: { d: "M480 230H395V275H380", c: "ana" },
    dac1: { d: "M650 175H755V150H770", c: "ana" }, dac2: { d: "M650 230H755V275H770", c: "ana" },
    gate: { d: "M405 396H535", c: "i2c" },
    ...RF,
  };
  board(214, 44, 732, 388, "MAIN BOARD");
  board(236, 440, 688, 146, "PACK · 14S LFP · 50 Ah · 2.24 kWh");
  drawPaths();
  Object.keys(COMP).forEach(comp);
  for (const k of ["panel1", "panel2"]) {
    const c = COMP[k], g = $("m-" + k);
    for (let r = 0; r < 3; r++) for (let q = 0; q < 6; q++)
      el("rect", { x: c.x + 10 + q * 20.5, y: c.y + 24 + r * 15, width: 18.5, height: 13, fill: "#0e1d33", stroke: "#1f3656", "stroke-width": .6 }, g);
    V[k] = el("text", { class: "sub", x: c.x + c.w - 8, y: c.y + 15, "text-anchor": "end", style: "fill:var(--solar)" }, g, "");
  }
  chip("mcu", "B2500");
  for (const n of [1, 2]) {
    V["pv" + n] = valText("pv" + n, 9, 42); V["pv" + n + "s"] = valText("pv" + n, 9, 58, "sub"); V["pv" + n + "t"] = valText("pv" + n, 9, 74, "sub");
    V["out" + n] = valText("out" + n, 9, 42); V["out" + n + "s"] = valText("out" + n, 9, 58, "sub"); V["out" + n + "t"] = valText("out" + n, 9, 74, "sub");
  }
  V.afe = valText("afe", 9, 33, "sub"); V.afe2 = valText("afe", 9, 47, "sub");
  V.fets = valText("fets", 40, 33, "sub", "middle");
  V.inv = valText("inv", 9, 50); V.invs = valText("inv", 9, 68, "sub"); V.invt = valText("inv", 9, 84, "sub"); V.invu = valText("inv", 9, 100, "sub");
  V.extp = valText("extp", 9, 44); V.extps = valText("extp", 9, 60, "sub"); V.extpt = valText("extp", 9, 76, "sub");
  drawOutside();
  drawCells(262, 40, 6.3, 474, 98, 268, 424, 9);
  silBox(487, 160, 156, 70, ["USART1 · I2C2 · UART5 · TIM8/TIM2 · DAC", "", "", "", "", ""], "mcu");
  silBox(236, 128, 140, 38, ["TIM8 CH3 / CH4 · ADC1 ch3", "", ""], "pv1");
  silBox(236, 258, 140, 38, ["TIM2 CH4 / CH3 · ADC2 ch5", "", ""], "pv2");
  silBox(776, 128, 150, 38, ["DAC ch1 · ADC1 ch6 / ch1", "", ""], "out1");
  silBox(776, 258, 150, 38, ["DAC ch2 · ADC1 ch7 / ch2", "", ""], "out2");
  silBox(260, 330, 140, 34, ["SH367309 regs", "", ""], "afe");
  silBox(624, 10, 120, 24, ["", ""], "fc");
  [["USART1 · AT", 412, 54], ["I2C2", 450, 296], ["UART5 · 0x73", 300, 314], ["PWM refs", 400, 170], ["DAC", 700, 170]].forEach(([t, x, y]) =>
    el("text", { class: "sil dim", x, y }, silG, t));
}

/* ---- the Venus family and the Jupiter: control, BMS and inverter (and MPPT) on an internal CAN bus */
const FAM = {
  E3: { mcu: "N32G457 · CONTROL", chip: "Venus E 3.0", uart: "UART6", inv: "INVERTER BOARD", invBoard: "INVERTER BOARD (MODEL)", proto: "CAN protocol of Micro v119",
    bms: "N32G45x · BMS", bmsKind: "real", afe: "KA495XX · FRONT END", eeprom: true, rs485: true },
  D: { mcu: "N32G457 · CONTROL", chip: "Venus D", uart: "UART6", inv: "INVERTER BOARD", invBoard: "INVERTER BOARD (MODEL)", proto: "CAN protocol of Micro 116",
    bms: "N32G45x · BMS", bmsKind: "real", afe: "KA49503 · FRONT END", eeprom: true, rs485: true, mppt: "MPPT CONTROLLER" },
  A: { mcu: "N32G457 · CONTROL", chip: "Venus A", uart: "UART6", inv: "INVERTER + MPPT", invBoard: "INVERTER BOARD (MODEL)", proto: "the control's parsers (Micro 1211)",
    bms: "N32G45x · BMS", bmsKind: "real", afe: "SH36735 · FRONT END", eeprom: true, rs485: true, mppt: "MPPT (INVERTER BOARD)" },
  JP: { mcu: "N32G45x · CONTROL", chip: "Jupiter C Plus", uart: "USART1", inv: "INVERTER", invBoard: "INVERTER (MODEL)", proto: "the control's parsers",
    bms: "N32 · BMS (C PLUS)", bmsKind: "real", afe: "KA49503 · FRONT END", mppt: "MPPT" },
  J: { mcu: "N32G45x · CONTROL", chip: "Jupiter C/E", uart: "USART1", inv: "INVERTER", invBoard: "INVERTER (MODEL)", proto: "the control's parsers",
    bms: "BMS (MODEL)", bmsKind: "model", afe: "FRONT END (IN THE MODEL)", mppt: "MPPT" },
};
let FV = FAM.E3;
function packLabel() {
  const p = SN && SN.pack, n = p ? p.cells.length : CELLS, ah = p ? p.capacity_ah : 100;
  const more = SN && SN.packs && SN.packs.length > 1 ? ` · ${SN.packs.length} PACKS` : "";
  return `PACK · ${n}S LFP · ${ah} Ah · ${(n * 3.2 * ah / 1000).toFixed(2)} kWh${more}`;
}
function cellGeometry() {
  // 16 cells: 36 px wide; 13 cells: wider; 14 (B2500) has its own layout
  const n = CELLS, span = 676, cg = n >= 16 ? 6.6 : 8, cw = (span - cg * (n - 1)) / n;
  return [246, cw, cg];
}
function buildVenus(variant) {
  FV = FAM[variant];
  CELLS = SN && SN.pack ? SN.pack.cells.length : 16;
  const pv = !!FV.mppt;
  COMP = {
    mcu: { x: 480, y: 120, w: 170, h: 150, t: FV.mcu, kind: "real" },
    micro: { x: 720, y: 80, w: 210, h: 200, t: FV.inv, kind: "model" },
    bms: { x: 250, y: 362, w: 180, h: 64, t: FV.bms, kind: FV.bmsKind },
    afe: { x: 450, y: 362, w: 150, h: 64, t: FV.afe, kind: "model" },
    fets: { x: 640, y: 368, w: 90, h: 44, t: "CHG · DSG", kind: "model" },
    ...OUTSIDE,
  };
  if (FV.eeprom) COMP.eeprom = { x: 290, y: 104, w: 130, h: 46, t: "24C256 · EEPROM", kind: "model" };
  if (FV.rs485) {
    COMP.rs485 = { x: 290, y: 200, w: 130, h: 52, t: "RS485 · MODBUS", kind: "model" };
    COMP.modbus = { x: 30, y: 196, w: 140, h: 60, t: "MODBUS CLIENT", kind: "ext" };
  }
  if (pv) {
    COMP.mppt = { x: 770, y: 352, w: 160, h: 76, t: FV.mppt, kind: "model" };
    for (let k = 0; k < 4; k++) COMP["string" + (k + 1)] = { x: 995 + (k % 2) * 82, y: 210 + Math.floor(k / 2) * 84, w: 74, h: 72, t: `PV ${k + 1}`, kind: "ext" };
  }
  POWER = {
    bat: { d: "M790 280V300H685V470", c: "chg" },
    acl: { d: pv ? "M930 160H1182V435H1150" : "M930 180H1090V400", c: "dis" },
    gridl: { d: "M1090 650V470", c: "dis" },
  };
  if (pv) {
    POWER.pvin = { d: "M995 250H962V390H930", c: "sol" };
    POWER.mpptbat = { d: "M770 400H730", c: "sol" };
  }
  DATA = {
    usart: { d: "M480 140H452V58H470", c: "uart" },
    caninv: { d: "M565 270V330H860V280", c: "ext" },
    canbms: { d: "M565 270V330H340V362", c: "ext" },
    spi: { d: "M430 394H450", c: "i2c" },
    gate: { d: "M600 390H640", c: "i2c" },
    ...RF,
  };
  if (FV.eeprom) DATA.eep = { d: "M480 127H420", c: "i2c" };
  if (FV.rs485) { DATA.rs = { d: "M480 226H420", c: "ana" }; DATA.rsext = { d: "M290 226H170", c: "ana" }; }
  if (pv) DATA.canmppt = { d: "M565 270V330H850V352", c: "ext" };
  board(214, 44, 470, 270, "CONTROL BOARD");
  board(700, 60, 250, 240, FV.invBoard);
  board(236, 344, 520, 92, FV.bmsKind === "real" ? "BMS BOARD" : "BMS (MODEL)");
  if (pv) board(756, 344, 188, 92, "MPPT (MODEL)");
  board(236, 440, 688, 146, packLabel());
  el("path", { d: "M236 330H940", stroke: "var(--ext)", "stroke-opacity": .5, "stroke-width": 2.2, fill: "none", class: "hx", "data-k": "canbus" }, boardsDecor);
  el("text", { class: "ext-lab", x: 945, y: 334, style: "fill:var(--ext)" }, boardsDecor, "CAN");
  drawPaths();
  Object.keys(COMP).forEach(comp);
  chip("mcu", FV.chip);
  if (FV.eeprom) V.eep = valText("eeprom", 9, 34, "sub");
  if (FV.rs485) {
    V.rs = valText("rs485", 9, 34, "sub"); V.rs2 = valText("rs485", 9, 46, "sub");
    V.modbus = valText("modbus", 9, 36, "sub"); V.modbus2 = valText("modbus", 9, 50, "sub");
  }
  if (pv) {
    V.mppt = valText("mppt", 9, 40); V.mppt2 = valText("mppt", 9, 56, "sub"); V.mppt3 = valText("mppt", 9, 69, "sub");
    for (let k = 1; k <= 4; k++) {
      const c = COMP["string" + k], g = $("m-string" + k);
      for (let r = 0; r < 2; r++) for (let q = 0; q < 3; q++)
        el("rect", { x: c.x + 6 + q * 21, y: c.y + 22 + r * 14, width: 19, height: 12, fill: "#0e1d33", stroke: "#1f3656", "stroke-width": .6 }, g);
      V["str" + k] = valText("string" + k, 9, 66, "sub");
    }
  }
  V.inv = valText("micro", 9, 42); V.invs = valText("micro", 9, 60, "sub"); V.invt = valText("micro", 9, 76, "sub");
  V.invu = valText("micro", 9, 92, "sub"); V.invv = valText("micro", 9, 108, "sub"); V.invw = valText("micro", 9, 124, "sub");
  V.invx = valText("micro", 9, 140, "sub");
  V.bms = valText("bms", 9, FV.bmsKind === "real" ? 46 : 32, "sub"); V.bms2 = valText("bms", 9, FV.bmsKind === "real" ? 58 : 46, "sub");
  V.afe = valText("afe", 9, 33, "sub"); V.afe2 = valText("afe", 9, 47, "sub");
  V.fets = valText("fets", 45, 33, "sub", "middle");
  drawOutside();
  const [x0, cw, cg] = cellGeometry();
  drawCells(x0, cw, cg, 474, 98, 462, 426, 8);
  silBox(487, 160, 156, 70, [`CAN1 · ${FV.uart}${FV.eeprom ? " · I2C · RS485" : ""}`, "", "", "", "", ""], "mcu");
  silBox(726, 196, 198, 70, [FV.proto, "", "", "", "", ""], "micro");
  silBox(256, 300, 172, 30, ["BMS over CAN", ""], "bms");
  silBox(456, 300, 142, 30, ["front end", ""], "afe");
  silBox(624, 10, 120, 24, ["", ""], "fc");
  [[`${FV.uart} · AT`, 412, 54], ["CAN 500 kbit/s", 590, 324], ["SPI", 432, 388]].forEach(([t, x, y]) =>
    el("text", { class: "sil dim", x, y }, silG, t));
}

/* ---- Venus C/E: the control MCU runs the power stage itself */
function buildVenusCE() {
  CELLS = SN && SN.pack ? SN.pack.cells.length : 16;
  COMP = {
    mcu: { x: 480, y: 120, w: 170, h: 150, t: "STM32G474 · CONTROL", kind: "real" },
    relay: { x: 745, y: 60, w: 175, h: 46, t: "GRID RELAY", kind: "model" },
    bridge: { x: 745, y: 126, w: 175, h: 66, t: "INVERTER BRIDGE", kind: "model" },
    dcbus: { x: 745, y: 212, w: 175, h: 56, t: "DC BUS", kind: "model" },
    llc: { x: 745, y: 354, w: 175, h: 66, t: "LLC · BATTERY DC/DC", kind: "model" },
    bms: { x: 250, y: 362, w: 180, h: 64, t: "BMS FIRMWARE", kind: "real" },
    afe: { x: 450, y: 362, w: 150, h: 64, t: "KA495XX · FRONT END", kind: "model" },
    fets: { x: 620, y: 368, w: 90, h: 44, t: "CHG · DSG", kind: "model" },
    ...OUTSIDE,
  };
  POWER = {
    bat: { d: "M832 354V292H665V470", c: "chg" },
    llcbus: { d: "M832 268V354", c: "chg" },
    busbr: { d: "M832 192V212", c: "chg" },
    brrel: { d: "M832 106V126", c: "chg" },
    acl: { d: "M920 83H1090V400", c: "dis" },
    gridl: { d: "M1090 650V470", c: "dis" },
  };
  DATA = {
    usart: { d: "M480 140H452V58H470", c: "uart" },
    hrtimb: { d: "M650 160H745", c: "ana" },
    hrtiml: { d: "M650 250H700V387H745", c: "ana" },
    adc: { d: "M745 240H690V200H650", c: "ana" },
    gpio: { d: "M650 135H720V83H745", c: "ana" },
    canbms: { d: "M565 270V330H340V362", c: "ext" },
    spi: { d: "M430 394H450", c: "i2c" },
    gate: { d: "M600 390H620", c: "i2c" },
    ...RF,
  };
  board(214, 44, 500, 270, "CONTROL BOARD · THE FIRMWARE RUNS THE POWER STAGE");
  board(730, 44, 210, 392, "POWER STAGE (MODEL)");
  board(236, 344, 480, 92, "BMS BOARD");
  board(236, 440, 688, 146, packLabel());
  el("path", { d: "M236 330H716", stroke: "var(--ext)", "stroke-opacity": .5, "stroke-width": 2.2, fill: "none", class: "hx", "data-k": "canbus" }, boardsDecor);
  el("text", { class: "ext-lab", x: 640, y: 324, style: "fill:var(--ext)" }, boardsDecor, "FDCAN");
  drawPaths();
  Object.keys(COMP).forEach(comp);
  chip("mcu", "Venus C/E");
  V.relay = valText("relay", 9, 34, "sub");
  V.bridge = valText("bridge", 9, 38); V.bridge2 = valText("bridge", 9, 54, "sub");
  V.dcbus = valText("dcbus", 9, 40);
  V.llc = valText("llc", 9, 38); V.llc2 = valText("llc", 9, 54, "sub");
  V.bms = valText("bms", 9, 46, "sub"); V.bms2 = valText("bms", 9, 58, "sub");
  V.afe = valText("afe", 9, 33, "sub"); V.afe2 = valText("afe", 9, 47, "sub");
  V.fets = valText("fets", 45, 33, "sub", "middle");
  drawOutside();
  const [x0, cw, cg] = cellGeometry();
  drawCells(x0, cw, cg, 474, 98, 462, 426, 8);
  silBox(487, 160, 156, 70, ["HRTIM · ADC · CORDIC · FDCAN2", "", "", "", "", ""], "mcu");
  silBox(256, 300, 172, 30, ["BMS over CAN", ""], "bms");
  silBox(456, 300, 142, 30, ["front end", ""], "afe");
  silBox(624, 10, 120, 24, ["", ""], "fc");
  [["LPUART1 · AT", 412, 54], ["HRTIM", 668, 156], ["ADC", 690, 196], ["SPI", 432, 388]].forEach(([t, x, y]) =>
    el("text", { class: "sil dim", x, y }, silG, t));
}

/* ---- CT002 / CT003: the meter, polled by virtual batteries */
function buildCt() {
  CELLS = 0;
  COMP = {
    mcu: { x: 480, y: 120, w: 170, h: 150, t: "STM32F0 · METER", kind: "real" },
    flash: { x: 290, y: 104, w: 130, h: 50, t: "SPI FLASH", kind: "model" },
    clampA: { x: 280, y: 420, w: 140, h: 60, t: "CLAMP A", kind: "model" },
    clampB: { x: 470, y: 420, w: 140, h: 60, t: "CLAMP B", kind: "model" },
    clampC: { x: 660, y: 420, w: 140, h: 60, t: "CLAMP C", kind: "model" },
    fc41d: OUTSIDE.fc41d, broker: OUTSIDE.broker, phone: OUTSIDE.phone,
  };
  for (let k = 0; k < 3; k++) COMP["bat" + (k + 1)] = { x: 1010, y: 110 + k * 120, w: 170, h: 90, t: `BATTERY ${k + 1}`, kind: "ext" };
  POWER = { gridl: { d: "M230 530H930", c: "dis" } };
  DATA = {
    usart: { d: "M480 140H452V58H470", c: "uart" },
    spi: { d: "M480 129H420", c: "i2c" },
    adcA: { d: "M350 420V300H500V270", c: "ana" },
    adcB: { d: "M540 420V270", c: "ana" },
    adcC: { d: "M730 420V300H600V270", c: "ana" },
    rfbroker: RF.rfbroker, rfphone: RF.rfphone,
    rfbat1: { d: "M545 0C760 -20 930 40 1010 150", c: "rf", rf: true },
    rfbat2: { d: "M545 0C800 0 940 150 1010 270", c: "rf", rf: true },
    rfbat3: { d: "M545 0C830 20 950 260 1010 390", c: "rf", rf: true },
  };
  board(214, 44, 520, 270, "METER BOARD");
  board(236, 400, 600, 100, "CURRENT CLAMPS · NOT MODELLED");
  el("text", { class: "ext-lab", x: 230, y: 552 }, boardsDecor, "GRID");
  el("text", { class: "ext-lab", x: 930, y: 552, "text-anchor": "end" }, boardsDecor, "HOUSE");
  drawPaths();
  Object.keys(COMP).forEach(comp);
  chip("mcu", "CT002");
  V.flash = valText("flash", 9, 34, "sub"); V.flash2 = valText("flash", 9, 46, "sub");
  for (const k of ["A", "B", "C"]) { V["cl" + k] = valText("clamp" + k, 9, 36); V["cl" + k + "2"] = valText("clamp" + k, 9, 50, "sub"); }
  for (let k = 1; k <= 3; k++) { V["bat" + k] = valText("bat" + k, 9, 38); V["bat" + k + "s"] = valText("bat" + k, 9, 56, "sub"); V["bat" + k + "t"] = valText("bat" + k, 9, 72, "sub"); }
  V.fc = valText("fc41d", 9, 34, "sub");
  V.broker = valText("broker", 9, 33, "sub"); V.phone = valText("phone", 9, 33, "sub");
  silBox(487, 160, 156, 70, ["USART1 · SPI2 · ADC", "", "", "", "", ""], "mcu");
  silBox(624, 10, 120, 24, ["", ""], "fc");
  [["USART1 · AT", 412, 54], ["ADC", 520, 300]].forEach(([t, x, y]) => el("text", { class: "sil dim", x, y }, silG, t));
}

const LAYOUTS = {
  "B2500": buildB2500, "Venus E 3.0": () => buildVenus("E3"), "Venus D": () => buildVenus("D"),
  "Venus A": () => buildVenus("A"), "Jupiter C Plus": () => buildVenus("JP"), "Jupiter C/E": () => buildVenus("J"),
  "Venus C/E": buildVenusCE, "CT002 / CT003": buildCt,
};
const IS_VENUS = (m) => m !== "B2500" && m !== "Venus C/E" && m !== "CT002 / CT003";

/* ------------------------------------------------------------- capsules */
const caps = [];
const lastCap = {};
const CAPCOL = { uart: "uart", i2c: "i2c", ext: "ext", rf: "rf", ana: "analog" };
function capsule(pathKey, dir, label) {
  if (reduced || caps.length > 24 || document.hidden) return;
  const p = P[pathKey];
  if (!p || !p.path) return;
  const now = performance.now();
  if (now - (lastCap[pathKey + dir] || 0) < 220) return;
  lastCap[pathKey + dir] = now;
  const g = el("g", { class: "cap" }, capsG);
  const w = label.length * 5.2 + 10;
  el("rect", { x: -w / 2, y: -7, width: w, height: 14, rx: 3, fill: `var(--${CAPCOL[p.c]})`, stroke: "rgba(0,0,0,.35)" }, g);
  el("text", { x: 0, y: 3, "text-anchor": "middle" }, g, label);
  caps.push({ g, p, dir, born: now, dur: p.len > 500 ? 1500 : p.len > 150 ? 1050 : 800 });
}
function capsuleFor(ev) {
  const t = ev.text || "", d = ev.dir || "";
  switch (ev.ch) {
    case "USART1": {
      const m = t.match(/^(?:AT\+)?(\+?[A-Z0-9]+)/);
      return capsule("usart", d.startsWith("MCU") ? 1 : -1, m ? m[1].slice(0, 10) : "AT");
    }
    case "RF":
      if (d === "FC41D→CT002") return capsule("rfct", 1, t.includes("no answer") ? "poll ✕" : "UDP poll");
      if (d === "CT002→FC41D") return capsule("rfct", -1, t.split("  ")[0]);
      return;
    case "MQTT":
      if (d === "broker→FC41D") return capsule("rfbroker", -1, (t.split("  ")[1] || "msg").slice(0, 12));
      return capsule("rfbroker", 1, t.startsWith("subscribe") ? "SUB" : t.startsWith("connected") ? "CONN" : "PUB");
    case "UART5": return capsule("uart5", 1, "0x73");
    case "UART6": {
      const m = t.match(/^(?:AT\+)?(\+?[A-Z0-9]+)/);
      return capsule("usart", d.startsWith("MCU") ? 1 : -1, m ? m[1].slice(0, 10) : "AT");
    }
    case "CAN": {
      const label = (t.split("  ")[1] || "").split(/[ :·]/)[0] || "CAN";
      if (d === "control→inverter") return capsule("caninv", 1, label.slice(0, 10));
      if (d === "inverter→control") return capsule("caninv", -1, label.slice(0, 10));
      if (d === "control→BMS") return capsule("canbms", 1, "poll");
      if (d === "BMS→control") return capsule("canbms", -1, "BMS");
      if (d === "control→MPPT") return capsule("canmppt", 1, label.slice(0, 10));
      if (d === "MPPT→control") return capsule("canmppt", -1, label.slice(0, 10));
      return;
    }
    case "LPUART1": {
      const m = t.match(/^(?:AT\+)?(\+?[A-Z0-9]+)/);
      return capsule("usart", d.startsWith("MCU") ? 1 : -1, m ? m[1].slice(0, 10) : "AT");
    }
    case "UDP": {
      const mac = (t.match(/c8478c00aa0(\d)/) || [])[1];
      if (!mac) return;
      return capsule("rfbat" + mac, d === "battery→CT" ? -1 : 1, d === "battery→CT" ? "poll" : "answer");
    }
    case "I2C2": return capsule("i2c", -1, "CELL×14");
    case "BLE": return capsule("rfphone", 1, "BLE");
  }
}
function animCaps(now) {
  for (let i = caps.length - 1; i >= 0; i--) {
    const c = caps[i];
    const u = (now - c.born) / c.dur;
    if (u >= 1) { c.g.remove(); caps.splice(i, 1); continue; }
    const e = u < .5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
    const s = c.dir > 0 ? e : 1 - e;
    const pt = c.p.path.getPointAtLength(s * c.p.len);
    c.g.setAttribute("transform", `translate(${pt.x.toFixed(1)},${pt.y.toFixed(1)})`);
    c.g.setAttribute("opacity", u < .08 ? u / .08 : u > .9 ? (1 - u) / .1 : 1);
  }
}

/* ------------------------------------------------------------ flows */
function setFlow(k, watts, cOverride) {
  const p = P[k];
  const a = Math.abs(watts);
  const w = a < 2 ? 0 : 1.4 + 4.2 * Math.sqrt(Math.min(a, 1600) / 800);
  p.w = w; p.dir = watts >= 0 ? 1 : -1; p.speed = 14 + 50 * Math.sqrt(Math.min(a, 1600) / 800);
  const c = cOverride || p.c;
  if (c !== p.cur) {
    p.flow.setAttribute("class", `flow c-${c} hx` + (p.flow.classList.contains("hl") ? " hl" : ""));
    p.halo.setAttribute("class", `halo c-${c} hx` + (p.halo.classList.contains("hl") ? " hl" : ""));
    p.cur = c;
  }
  p.flow.setAttribute("stroke-width", w.toFixed(2));
  p.halo.setAttribute("stroke-width", (w * 3.2).toFixed(2));
  p.flow.setAttribute("stroke-dasharray", w ? `${(w * 1.2).toFixed(1)} ${(w * 3 + 6).toFixed(1)}` : "0 1");
  p.flow.style.display = w ? "" : "none";
  p.halo.style.display = w ? "" : "none";
}
function animFlows(dtReal) {
  const moving = SN && SN.running;
  for (const k in POWER) {
    const p = P[k];
    if (!p.w || reduced || !moving) continue;
    p.off -= p.dir * p.speed * dtReal;
    p.flow.setAttribute("stroke-dashoffset", p.off.toFixed(1));
  }
}

/* ---------------------------------------------------------- render svg */
function render() {
  if (!SN) return;
  if (MODEL === "B2500") renderB2500();
  else if (MODEL === "Venus C/E") renderCE();
  else if (MODEL === "CT002 / CT003") { renderCt(); return; }
  else renderVenus();
  $("shellSoc").textContent = Math.round(S.soc * 100) + "%";
  $("shellLed").setAttribute("stroke-dasharray", `${(138.2 * S.soc).toFixed(1)} 200`);
}
function renderCells(span0, span1) {
  const now = performance.now();
  for (let i = 0; i < CELLS; i++) {
    const c = CELL[i], v = S.cells[i];
    if (!c) continue;
    const f = clamp((v - span0) / (span1 - span0), 0.03, 1);
    const hh = (c.h - 6) * f;
    c.fill.setAttribute("y", (c.y + 3 + (c.h - 6) - hh).toFixed(1));
    c.fill.setAttribute("height", hh.toFixed(1));
    c.lvl.setAttribute("y", (c.y + 3 + (c.h - 6) - hh).toFixed(1));
    const col = S.pbat < -2 ? "var(--discharge)" : "var(--charge)";
    c.fill.setAttribute("fill", col); c.lvl.setAttribute("fill", col);
    c.balDot.setAttribute("opacity", S.bal[i] ? (0.5 + 0.5 * Math.sin(now / 180)).toFixed(2) : 0);
    c.hot.setAttribute("opacity", S.f.heat && S.temp > 50 ? 1 : 0);
    cellSil[2 * i].textContent = (v * 1000).toFixed(0);
    cellSil[2 * i + 1].textContent = S.bal[i] ? "BAL" : "mV";
  }
}
function renderOutside(mq) {
  V.house.textContent = Math.round(S.load) + " W";
  V.houses.textContent = S.load < 0 ? "exporting (solar)" : "load, phase A";
  const ct = SN.ct;
  V.ct.textContent = S.f.ct || S.f.wifi ? "no answer" : ct.last_grid == null ? "waiting for a poll" : `grid ${sgn(ct.last_grid)} W`;
  $("m-ct").classList.toggle("alarm", !!S.f.ct);
  V.broker.textContent = mq ? "connected" : "offline";
  V.phone.textContent = SN.fc41d.ble || "–";
  V.fc.textContent = SN.fc41d.wifi ? `Wi‑Fi · ${mq ? "MQTT" : "no MQTT"} · BLE` : "no Wi‑Fi · no MQTT";
  $("m-fc41d").classList.toggle("alarm", !SN.fc41d.wifi);
  SIL.fc[0].textContent = "AT engine · " + String(SN.fc41d.version).slice(-12);
  SIL.fc[1].textContent = SN.fc41d.wifi ? `MQTT ${mq ? "up" : "down"} · BLE adv` : "Wi‑Fi disconnected";
}
const num = (v, d = 1) => (v == null || Number.isNaN(Number(v)) ? "–" : Number(v).toFixed(d));
function renderVenus() {
  const inv = SN.inverter, bms = SN.bms, mq = (SN.fc41d.mqtt || []).some((s) => s.connected), mp = SN.mppt;
  setFlow("bat", S.pbat, S.pbat >= 0 ? "chg" : "dis");
  setFlow("acl", S.ac, S.ac >= 0 ? "dis" : "chg");
  setFlow("gridl", S.grid, S.grid >= 0 ? "dis" : "sol");
  const irq = sum((SN.mcu.irqs || []).map((x) => x[1]));
  V.mcuA.textContent = `${SN.device_type || SN.profile}${SN.version ? " V" + SN.version : ""}`;
  V.mcuB.textContent = `${sgn(inv.reference)} W`;
  V.mcuC.textContent = "setpoint to inverter";
  V.mcuD.textContent = `IRQ ${(irq / 1000).toFixed(2)}k/s · CAN ${SN.can.frames}`;
  if (V.eep) V.eep.textContent = "settings and statistics";
  if (V.rs) {
    V.rs.textContent = "slave 1, the firmware's"; V.rs2.textContent = "register map";
    V.modbus.textContent = "Modbus TCP via"; V.modbus2.textContent = "marsemu --modbus-port";
  }
  if (mp && V.mppt) {
    setFlow("pvin", sum(mp.pv));
    setFlow("mpptbat", mp.dc);
    V.mppt.textContent = `${Math.round(sum(mp.pv))} W PV`;
    V.mppt2.textContent = `limit ${mp.limit} W · mode ${mp.mode}`;
    V.mppt3.textContent = `into the pack ${Math.round(mp.dc)} W`;
    for (let k = 0; k < 4; k++) V["str" + (k + 1)].textContent = `${Math.round(mp.pv[k])}/${Math.round(mp.avail[k])} W`;
  }
  V.inv.textContent = `${sgn(inv.ac)} W AC`;
  V.invs.textContent = `reference ${sgn(inv.reference)} W · ${inv.connected ? "on grid" : "off grid"}`;
  V.invt.textContent = `DC ${sgn(inv.dc)} W into the pack`;
  V.invu.textContent = `limits ${inv.limit_discharge} / ${inv.limit_charge} W`;
  V.invv.textContent = `LLC ${inv.temp_llc} °C · bridge ${inv.temp_inv} °C`;
  V.invw.textContent = `v${inv.version} protocol (model)`;
  V.invx.textContent = inv.errors.some(Boolean) ? `errors ${inv.errors.map((e) => hex(e, 2)).join(" ")}` : "no errors";
  $("m-micro").classList.toggle("alarm", inv.errors.some(Boolean));
  V.bms.textContent = `SOC ${num(bms.soc)} % · ${num(bms.v, 2)} V`;
  V.bms2.textContent = `${bms.a == null ? "–" : sgn(bms.a)} A · ${bms.real ? (bms.process === "own process" ? "own process" : "real firmware") : "model"}`;
  renderPackSide();
  renderOutside(mq);
  renderCells(3.15, 3.45);
  const m = SIL.mcu;
  m[1].textContent = `PC   ${hex(SN.mcu.pc, 8)}`;
  m[2].textContent = `IRQ  ${Math.round(irq)}/s  ${(SN.mcu.hz / 1e6).toFixed(0)} MHz`;
  m[3].textContent = `CAN  ${SN.can.frames} frames`;
  m[4].textContent = `→inv ${SN.can.by_dir["control→inverter"] || 0}  ←inv ${SN.can.by_dir["inverter→control"] || 0}`;
  m[5].textContent = mp ? `→MPPT ${SN.can.by_dir["control→MPPT"] || 0}  ←BMS ${SN.can.by_dir["BMS→control"] || 0}`
    : `→BMS ${SN.can.by_dir["control→BMS"] || 0}  ←BMS ${SN.can.by_dir["BMS→control"] || 0}`;
  const mi = SIL.micro;
  mi[1].textContent = `0x01 setpoint  ${Math.round(inv.reference)} W`;
  mi[2].textContent = `0x03/0x04 limits ${inv.discharge_limit_w}/${inv.charge_limit_w} W`;
  mi[3].textContent = `0x0A cut-off SOC ${inv.cutoff_soc} %`;
  mi[4].textContent = `work mode ${inv.work_mode} · max SOC ${inv.max_soc} %`;
  mi[5].textContent = `0x10 telemetry ${sgn(inv.ac)} W`;
  SIL.bms[1].textContent = `chg ${num(bms.charge_v)} V ${num(bms.charge_a)} A · dis ${num(bms.discharge_a)} A`;
  SIL.afe[1].textContent = `flags ${hex(bms.flags || 0, 2)} · prot ${hex(bms.protect || 0, 8)}`;
}
function renderPackSide() {
  const minC = Math.min(...S.cells), maxC = Math.max(...S.cells);
  V.afe.textContent = `${(minC * 1000).toFixed(0)}–${(maxC * 1000).toFixed(0)} mV · Δ ${((maxC - minC) * 1000).toFixed(0)}`;
  V.afe2.textContent = `${S.I >= 0 ? "+" : ""}${S.I.toFixed(2)} A · ${S.temp.toFixed(1)} °C`;
  V.fets.textContent = `${S.fets.chg ? "CHG" : "chg"} · ${S.fets.dsg ? "DSG" : "dsg"}`;
  V.fets.style.fill = S.fets.chg && S.fets.dsg ? "var(--ok)" : "var(--alarm)";
  $("m-fets").classList.toggle("alarm", !(S.fets.chg && S.fets.dsg));
}
function renderCE() {
  const st = SN.stage, bms = SN.bms, mq = (SN.fc41d.mqtt || []).some((s) => s.connected);
  const toHouse = -S.pbat;           // + while the pack discharges toward the house
  setFlow("bat", S.pbat, S.pbat >= 0 ? "chg" : "dis");
  for (const k of ["llcbus", "busbr", "brrel"]) setFlow(k, S.pbat, S.pbat >= 0 ? "chg" : "dis");
  setFlow("acl", S.ac, S.ac >= 0 ? "dis" : "chg");
  setFlow("gridl", S.grid, S.grid >= 0 ? "dis" : "sol");
  const irq = sum((SN.mcu.irqs || []).map((x) => x[1]));
  V.mcuA.textContent = `${SN.device_type || SN.profile}${SN.version ? " V" + SN.version : ""}`;
  V.mcuB.textContent = `${sgn(st.ac_w)} W`;
  V.mcuC.textContent = "AC power";
  V.mcuD.textContent = `IRQ ${(irq / 1000).toFixed(1)}k/s · 30 kHz loop`;
  V.relay.textContent = st.relay ? "closed · on the grid" : "open";
  V.relay.style.fill = st.relay ? "var(--ok)" : "var(--fg-dim)";
  V.bridge.textContent = `${sgn(st.ac_w)} W`;
  V.bridge2.textContent = `${st.bridge ? "switching" : "off"} · peak ${num(st.i_peak_a, 2)} A`;
  V.dcbus.textContent = `${num(st.vbus)} V`;
  V.llc.textContent = `${sgn(toHouse)} W`;
  V.llc2.textContent = `${st.llc ? "running" : "off"}${st.bypass ? " · bypass" : ""}`;
  V.bms.textContent = `SOC ${num(bms.soc)} % · ${num(bms.v, 2)} V`;
  V.bms2.textContent = `${bms.a == null ? "–" : sgn(bms.a)} A · own process`;
  renderPackSide();
  renderOutside(mq);
  renderCells(3.15, 3.45);
  const m = SIL.mcu;
  m[1].textContent = `PC   ${hex(SN.mcu.pc, 8)}`;
  m[2].textContent = `IRQ  ${Math.round(irq)}/s  ${(SN.mcu.hz / 1e6).toFixed(0)} MHz`;
  m[3].textContent = `bus ${num(st.vbus)} V · relay ${st.relay ? 1 : 0}`;
  m[4].textContent = `bridge ${st.bridge ? "on" : "off"} · LLC ${st.llc ? "on" : "off"}`;
  m[5].textContent = `CAN ${SN.can.frames} frames`;
  SIL.bms[1].textContent = `chg ${num(bms.charge_v)} V ${num(bms.charge_a)} A · dis ${num(bms.discharge_a)} A`;
  SIL.afe[1].textContent = `flags ${hex(bms.flags || 0, 2)} · prot ${hex(bms.protect || 0, 8)}`;
}
function renderCt() {
  const mq = (SN.fc41d.mqtt || []).some((s) => s.connected), me = SN.meter, bats = SN.batteries || [];
  const irq = sum((SN.mcu.irqs || []).map((x) => x[1]));
  V.mcuA.textContent = `${SN.device_type || SN.profile}${SN.version ? " V" + SN.version : ""}`;
  V.mcuB.textContent = `${me.replies}`;
  V.mcuC.textContent = "answers sent";
  V.mcuD.textContent = `IRQ ${(irq / 1000).toFixed(2)}k/s · id ${me.id}`;
  V.flash.textContent = "factory data:"; V.flash2.textContent = "certificate, key (empty)";
  const last = me.last || [];
  const cl = me.clamps;
  [["A", 4], ["B", 5], ["C", 6]].forEach(([k, i]) => {
    V["cl" + k].textContent = `${last[i] ?? 0} W`;
    V["cl" + k + "2"].textContent = cl ? `${me.source === "p1" ? "P1" : "clamp"} ${sgn(cl[k])} W` : "input not modelled";
  });
  for (let k = 1; k <= 3; k++) {
    const b = bats[k - 1], on = !!b;
    $("m-bat" + k).style.opacity = on ? 1 : .3;
    V["bat" + k].textContent = on ? `${Math.round(b.w)} W` : "not polling";
    V["bat" + k + "s"].textContent = on ? `${b.type} · phase ${b.phase}` : "";
    V["bat" + k + "t"].textContent = on ? `${b.replies} answers` : "";
  }
  V.broker.textContent = mq ? "connected" : "offline";
  V.phone.textContent = SN.fc41d.ble || "–";
  V.fc.textContent = SN.fc41d.wifi ? `Wi‑Fi · UDP 12345 · ${mq ? "MQTT" : "no MQTT"}` : "no Wi‑Fi";
  $("m-fc41d").classList.toggle("alarm", !SN.fc41d.wifi);
  SIL.fc[0].textContent = "AT engine · " + String(SN.fc41d.version).slice(-12);
  SIL.fc[1].textContent = me.socket ? "UDP service on port 12345" : "no UDP service yet";
  const m = SIL.mcu;
  m[1].textContent = `PC   ${hex(SN.mcu.pc, 8)}`;
  m[2].textContent = `IRQ  ${Math.round(irq)}/s  ${(SN.mcu.hz / 1e6).toFixed(0)} MHz`;
  m[3].textContent = `answers ${me.replies}`;
  m[4].textContent = `last to ${last[3] || "–"}`;
  m[5].textContent = `per phase ${last.slice(8, 11).join("/") || "–"}`;
}
function renderB2500() {
  const pvT = sum(S.pv), out = sum(S.out);
  setFlow("pv1in", S.pv[0]); setFlow("pv2in", S.pv[1]);
  setFlow("pv1bus", S.pv[0]); setFlow("pv2bus", S.pv[1]);
  setFlow("busL", pvT);
  setFlow("busR", out);
  setFlow("bat", S.pbat, S.pbat >= 0 ? "chg" : "dis");
  setFlow("o1", S.out[0]); setFlow("o2", S.out[1]);
  setFlow("o1inv", S.out[0]); setFlow("o2inv", S.out[1]);
  setFlow("acl", S.ac);
  setFlow("gridl", S.grid, S.grid >= 0 ? "dis" : "sol");

  V.panel1.textContent = Math.round(S.sun[0]) + " W avail";
  V.panel2.textContent = Math.round(S.sun[1]) + " W avail";
  for (let i = 0; i < 2; i++) {
    const n = i + 1, pv = SN.pv[i], o = SN.out[i];
    V["pv" + n].textContent = Math.round(pv.w) + " W";
    V["pv" + n + "s"].textContent = pv.on ? `${pv.v.toFixed(1)} V · ${pv.a.toFixed(2)} A` : "converter off";
    V["pv" + n + "t"].textContent = `ref ${pv.v_ref.toFixed(1)} V · ≤ ${pv.i_lim.toFixed(1)} A`;
    V["out" + n].textContent = Math.round(o.w) + " W";
    V["out" + n + "s"].textContent = o.on ? `${o.v.toFixed(1)} V · ${o.a.toFixed(2)} A` : "off";
    V["out" + n + "t"].textContent = `heatsink ${S.hs.toFixed(1)} °C`;
  }
  V.mcuA.textContent = `${SN.device_type || SN.profile}${SN.version ? " V" + SN.version : ""}`;
  V.mcuB.textContent = `${Math.round(S.target)} W`;
  V.mcuC.textContent = "output reference";
  const irq = sum((SN.mcu.irqs || []).map((x) => x[1]));
  V.mcuD.textContent = `IRQ ${(irq / 1000).toFixed(2)}k/s · ${(SN.mcu.cycles / 1e6).toFixed(0)} M cycles`;
  const mq = (SN.fc41d.mqtt || []).some((s) => s.connected);
  const minC = Math.min(...S.cells), maxC = Math.max(...S.cells);
  V.afe.textContent = `${(minC * 1000).toFixed(0)}–${(maxC * 1000).toFixed(0)} mV · Δ ${((maxC - minC) * 1000).toFixed(0)}`;
  V.afe2.textContent = `${S.I >= 0 ? "+" : ""}${S.I.toFixed(2)} A · ${S.temp.toFixed(1)} °C`;
  V.fets.textContent = `${S.fets.chg ? "CHG" : "chg"} · ${S.fets.dsg ? "DSG" : "dsg"}`;
  V.fets.style.fill = S.fets.chg && S.fets.dsg ? "var(--ok)" : "var(--alarm)";
  $("m-fets").classList.toggle("alarm", !(S.fets.chg && S.fets.dsg));
  V.inv.textContent = Math.round(S.ac) + " W AC";
  V.invs.textContent = `in 1 ${Math.round(S.out[0])} W · in 2 ${Math.round(S.out[1])} W`;
  V.invt.textContent = `tracker at ${(SN.inv[0] || {}).v_set ?? "–"} V`;
  V.invu.textContent = "model · MPP tracker";
  const ext = S.ext[0];
  $("m-extp").style.opacity = ext ? 1 : .35;
  V.extp.textContent = ext ? ext.soc.toFixed(0) + " %" : "none";
  V.extps.textContent = ext ? `${ext.v.toFixed(2)} V · ${ext.a.toFixed(2)} A` : "not connected";
  V.extpt.textContent = "UART5 · 0x73 frames";
  renderOutside(mq);
  renderCells(3.1, 3.45);
  // silicon readouts: what the firmware wrote into its peripherals
  const r = SN.mcu.regs, m = SIL.mcu;
  const dacTxt = (v) => (v == null ? "off" : hex(v, 3));
  m[1].textContent = `PC   ${hex(SN.mcu.pc, 8)}`;
  m[2].textContent = `IRQ  ${Math.round(irq)}/s  ${(SN.mcu.hz / 1e6).toFixed(0)} MHz`;
  m[3].textContent = `TIM8_CCR3 ${r.TIM8_CCR3}  CCR4 ${r.TIM8_CCR4}`;
  m[4].textContent = `DAC1 ${dacTxt(r.DAC_DHR12R1)}  DAC2 ${dacTxt(r.DAC_DHR12R2)}`;
  m[5].textContent = `I2C2 ${SN.afe.reads} reads`;
  const ccr = [[r.TIM8_CCR3, r.TIM8_CCR4], [r.TIM2_CCR4, r.TIM2_CCR3]];
  const dac = [r.DAC_DHR12R1, r.DAC_DHR12R2];
  for (let i = 0; i < 2; i++) {
    const s = SIL["pv" + (i + 1)], pv = SN.pv[i];
    s[1].textContent = `CCR ${ccr[i][0]} → Vref ${pv.v_ref.toFixed(2)} V`;
    s[2].textContent = `CCR ${ccr[i][1]} → Ilim ${pv.i_lim.toFixed(2)} A`;
    const o = SIL["out" + (i + 1)], ou = SN.out[i];
    o[1].textContent = dac[i] == null ? "DAC off" : `DAC ${hex(dac[i], 3)} → ${(ou.i_lim * 1000).toFixed(0)} mA`;
    o[2].textContent = `out ${ou.v.toFixed(2)} V · ${ou.a.toFixed(3)} A`;
  }
  const bs = SN.afe.bstatus;
  SIL.afe[1].textContent = `BSTATUS ${hex(bs[0], 2)} ${hex(bs[1], 2)} ${hex(bs[2], 2)}`;
  SIL.afe[2].textContent = `BAL ${hex(SN.afe.bal, 4)} FET ${S.fets.chg ? "C" : "-"}${S.fets.dsg ? "D" : "-"}`;
}

/* ------------------------------------------------------------- HUD */
function renderHudCt() {
  const me = SN.meter, bats = SN.batteries || [], last = me.last || [];
  const cell = (id, v, u, note, color) => { $(id).innerHTML = `${v}<u>${u}</u>`; $(id).style.color = color || ""; $(id + "N").textContent = note; };
  cell("hGrid", last[7] ?? 0, "W", SN.meter.clamps ? "total in the last answer" : "total in the last answer (clamps not modelled)");
  cell("hAc", sum(bats.map((b) => b.w)), "W", `reported by ${bats.length} batteries`, "var(--discharge)");
  cell("hPv", me.replies, "", "answers to battery polls", "var(--xray)");
  cell("hBat", last.slice(8, 11).join("/") || "–", "", "batteries per phase A/B/C");
  cell("hSoc", last[12] ?? "–", "dBm", "RSSI the meter reports", "var(--xray)");
}
function renderHud() {
  if (!SN) return;
  document.querySelectorAll(".hud small").forEach((e, k) => { e.textContent = (MODEL === "CT002 / CT003"
    ? ["Grid (meter)", "Batteries deliver", "Answers", "Batteries per phase", "Wi‑Fi"] : ["Grid", "AC to house", "Solar", "Battery", "State of charge"])[k]; });
  if (MODEL === "CT002 / CT003") { renderHudCt(); renderHudCommon(); return; }
  const g = S.grid;
  $("hGrid").innerHTML = `${sgn(g)}<u>W</u>`;
  $("hGrid").style.color = Math.abs(g) < 25 ? "var(--ok)" : g > 0 ? "var(--fg)" : "var(--solar)";
  $("hGridN").textContent = Math.abs(g) < 25 ? "balanced" : g > 0 ? "import from grid" : "export to grid";
  $("hAc").innerHTML = `${Math.round(S.ac)}<u>W</u>`;
  $("hAcN").textContent = MODEL === "Venus C/E" ? `bridge ${SN.stage.bridge ? "switching" : "off"} · relay ${SN.stage.relay ? "closed" : "open"}`
    : `${MODEL === "B2500" ? "reference" : "setpoint"} ${Math.round(S.target)} W`;
  $("hPv").innerHTML = `${Math.round(sum(S.pv))}<u>W</u>`;
  $("hPvN").textContent = MODEL === "B2500" || SN.mppt ? S.pv.map((w) => Math.round(w)).filter((w, k) => k < 2 || w).join(" + ") + " W" : "no PV input";
  const b = S.pbat;
  $("hBat").innerHTML = `${sgn(b)}<u>W</u>`;
  $("hBat").style.color = Math.abs(b) < 3 ? "var(--fg-dim)" : b > 0 ? "var(--charge)" : "var(--discharge)";
  $("hBatN").textContent = Math.abs(b) < 3 ? "idle" : `${b > 0 ? "charging" : "discharging"} · ${S.I.toFixed(2)} A`;
  $("hSoc").innerHTML = `${(S.soc * 100).toFixed(1)}<u>%</u>`;
  const kwh = SN.pack.capacity_ah * S.packV / 1000;
  $("hSocN").textContent = `${S.packV.toFixed(2)} V · ${(S.soc * kwh).toFixed(2)} of ${kwh.toFixed(2)} kWh`;
  renderHudCommon();
}
function renderHudCommon() {
  $("clock").firstChild.nodeValue = "T+" + fmtT(S.t);
  $("rate").textContent = SN.error ? "stopped: error" : !SN.running ? "paused" : `${SN.rate.toFixed(1)}× real time`;
  const set = (id, bad, txt) => { const c = $(id); c.classList.remove("off"); c.classList.toggle("bad", bad); c.lastChild.nodeValue = txt; };
  const mq = (SN.fc41d.mqtt || []).some((s) => s.connected);
  set("chipWifi", !SN.fc41d.wifi, SN.fc41d.wifi ? "Wi‑Fi" : "Wi‑Fi down");
  set("chipMqtt", !mq, mq ? "MQTT connected" : "MQTT offline");
  const ct = SN.ct, lost = S.f.ct || S.f.wifi || ct.age == null || ct.age > 10;
  if (MODEL === "CT002 / CT003") set("chipCt", !SN.meter.socket, SN.meter.socket ? "UDP 12345 open" : "no UDP service");
  else set("chipCt", lost, ct.age == null ? "CT002 waiting" : lost ? `CT002 lost ${ct.age.toFixed(0)} s` : `CT002 ${ct.age.toFixed(1)} s ago`);
  set("chipBle", false, "BLE " + (SN.fc41d.ble || "–"));
  $("crumb").innerHTML = `${esc(SN.model)} <em>${esc(SN.device_type || "")}</em> · firmware <em>${esc(SN.version ? "V" + SN.version : "starting")}</em> · profile <em>${esc(SN.profile)}</em> · <span title="${esc(SN.image)}">${esc(SN.image.length > 28 ? SN.image.slice(0, 26) + "…" : SN.image)}</span>`;
  const stateTxt = SN.error ? "error" : SN.running ? "running" : "paused";
  document.querySelectorAll(".dev[data-models]").forEach((d) => {
    const on = d.dataset.models.split("|").includes(MODEL);
    d.classList.toggle("on", on); d.lastElementChild.textContent = on ? stateTxt : "ready";
  });
  $("playBtn").textContent = SN.running ? "❚❚" : "▶";
  $("playBtn").setAttribute("aria-label", SN.running ? "Pause" : "Run");
  document.querySelectorAll("[data-speed]").forEach((x) => x.setAttribute("aria-pressed", String(+x.dataset.speed === SN.speed)));
  for (const [id, k] of [["fWifi", "wifi"], ["fCt", "ct"], ["fHot", "heat"]]) if (document.activeElement !== $(id)) $(id).checked = !!S.f[k];
}
/* --------------------------------------------------------- inspector */
let sel = "mcu";
const DESC = {
  mcu: ["N32G452 microcontroller", "Runs the unmodified firmware image: MPPT, steering against the CT002, MQTT, BLE, protection.", "real"],
  fc41d: ["FC41D Wi‑Fi/BLE module", "Model of the Quectel module on USART1. Answers the firmware's AT commands; Wi‑Fi, MQTT, UDP and BLE go through it.", "model"],
  afe: ["SH367309 battery front end", "Model of the front end on I2C2: measures the 14 cells, the current and the temperatures, balances, switches the FETs.", "model"],
  fets: ["Charge and discharge FETs", "Switched by the front end on the firmware's command. Off on over- or under-voltage, over-current or temperature.", "model"],
  pack: ["Battery pack", "14 LFP cells in series. Cell model: OCV curve, resistance, polarisation, temperature.", "model"],
  pv1: ["PV input 1", "Analog buck converter. The firmware sets its voltage reference and current limit through filtered PWM and runs the MPPT itself.", "model"],
  pv2: ["PV input 2", "Analog buck converter. The firmware sets its voltage reference and current limit through filtered PWM and runs the MPPT itself.", "model"],
  out1: ["Output 1", "Current source whose reference is the DAC. The firmware shapes it like a PV panel so the microinverter settles at the power it wants.", "model"],
  out2: ["Output 2", "Current source whose reference is the DAC. The firmware shapes it like a PV panel so the microinverter settles at the power it wants.", "model"],
  inv: ["Microinverter", "Two-input microinverter with a perturb-and-observe MPP tracker on each input.", "ext"],
  house: ["House", "The load the battery covers. Set it with the slider, a scenario, or in Home Assistant from one of your sensors.", "ext"],
  ct: ["CT002 meter", "Measures the grid at the meter and answers the battery's UDP polls: built in, or AstraMeter.", "ext"],
  panel1: ["Solar panel 1", "Available power, set with the sun slider or a scenario.", "ext"],
  panel2: ["Solar panel 2", "Available power, set with the sun slider or a scenario.", "ext"],
  extp: ["Extension pack", "A second pack on the UART5 extension bus, polled with 0x73 frames.", "ext"],
  broker: ["MQTT broker", "Where the device's MQTT session goes: offline (recorded here) or your broker. hm2mqtt polls it with cd=1.", "ext"],
  phone: ["Phone (BLE)", "A BLE central. Tools connect through marsemu's BLE bridge.", "ext"],
};
const DESC_V = {
  mcu: ["N32G457 control", "Runs the unmodified control firmware: Wi‑Fi, MQTT, BLE, the CT poll, phase detection, steering, schedules and limits, the Modbus slave.", "real"],
  fc41d: DESC.fc41d,
  eeprom: ["24C256 EEPROM", "Model of the I2C EEPROM where the control keeps its settings and statistics.", "model"],
  rs485: ["RS485 port", "The control firmware's Modbus RTU slave (address 1). marsemu serves it as Modbus TCP with --modbus-port.", "model"],
  modbus: ["Modbus client", "Home Assistant's Modbus integration or any other client, through marsemu's Modbus gateway.", "ext"],
  micro: ["Inverter board", "Model of the inverter board (STM32G474, Micro v119): its CAN protocol, setpoint clamp, grid-connect delay, slew and losses. Its firmware is not emulated.", "model"],
  bms: ["N32G45x BMS", "Runs the unmodified BMS firmware in its own process: SOC, limits, protections and MOSFETs, reported to the control over CAN.", "real"],
  afe: ["KA495XX battery front end", "Model of the front end on the BMS's SPI bus: measures the 16 cells, the current and the temperatures.", "model"],
  fets: ["Charge and discharge MOSFETs", "Switched by the BMS firmware. Off on protection.", "model"],
  pack: ["Battery pack", "16 LFP cells in series, 100 Ah. Cell model: OCV curve, resistance, polarisation, temperature.", "model"],
  house: DESC.house, ct: DESC.ct, broker: DESC.broker, phone: DESC.phone,
  mppt: ["MPPT", "Model of the PV charge controller on the CAN bus: answers the control's polls with its status block and obeys its PV power limit and charge voltage. Its firmware is not emulated.", "model"],
  string1: ["PV string 1", "Available power, set with the sun slider or a scenario.", "ext"], string2: ["PV string 2", "Available power, set with the sun slider or a scenario.", "ext"],
  string3: ["PV string 3", "Available power (set through the API).", "ext"], string4: ["PV string 4", "Available power (set through the API).", "ext"],
  relay: ["Grid relay", "Closed by the control firmware once the PLL has locked and the DC bus is up.", "model"],
  bridge: ["Inverter bridge", "Driven by the control firmware's HRTIM at 30 kHz; its control loop runs in the firmware.", "model"],
  dcbus: ["DC bus", "Between the LLC and the bridge, measured by the firmware's ADC. Capacitance estimated.", "model"],
  llc: ["LLC converter", "The battery DC/DC, switched by the firmware's HRTIM. Ratio estimated.", "model"],
  flash: ["SPI flash", "External NOR flash where a CT keeps its factory data (TLS certificate and key). Empty here: the module claims the certificates.", "model"],
  clampA: ["Clamp A", "The current clamp on phase A. Not modelled: the meter measures 0 W.", "model"],
  clampB: ["Clamp B", "The current clamp on phase B. Not modelled: the meter measures 0 W.", "model"],
  clampC: ["Clamp C", "The current clamp on phase C. Not modelled: the meter measures 0 W.", "model"],
  bat1: ["Battery 1", "A virtual battery that polls the meter over UDP every 2.7 s, as a real one does.", "ext"],
  bat2: ["Battery 2", "A virtual battery that polls the meter over UDP every 2.7 s, as a real one does.", "ext"],
  bat3: ["Battery 3", "A virtual battery that polls the meter over UDP every 2.7 s, as a real one does.", "ext"],
};
const TITLES = {
  "Venus D": { afe: ["KA49503 battery front end", "Model of the pack's front end on the BMS's SPI bus: 16 cells, current, temperatures.", "model"],
    micro: ["Inverter board", "Model of the inverter board (Micro 116 protocol): setpoint clamp, grid-connect delay, slew, losses.", "model"] },
  "Venus A": { afe: ["SH36735 battery front end", "Model of the front end of the Venus A's BMS: 13 cells, with the bit-banged I2C EEPROM that holds the master marker.", "model"],
    micro: ["Inverter board", "Model of the inverter board (Micro 1211, an NS800RT that is not emulated) with its four PV inputs and 1450 W limits.", "model"],
    mppt: ["MPPT", "The four PV inputs of the Venus A's inverter board, modelled from what the control sends and parses.", "model"] },
  "Jupiter C Plus": { mcu: ["N32G45x control", "Runs the unmodified Jupiter control firmware: Wi‑Fi, MQTT, BLE, the CT poll, steering, the MPPT and inverter commands.", "real"],
    bms: ["C Plus BMS", "Runs the unmodified C Plus BMS firmware on a model of its board (the Venus D's front end and EEPROM). It answers the control's broadcast.", "real"],
    afe: ["KA49503 battery front end", "Model of the C Plus BMS's front end: 16 cells, current, temperatures.", "model"],
    micro: ["Inverter", "Model built from what the control sends and parses; the inverter firmware is not available.", "model"] },
  "Jupiter C/E": { mcu: ["N32G45x control", "Runs the unmodified Jupiter control firmware: Wi‑Fi, MQTT, BLE, the CT poll, steering, the MPPT and inverter commands.", "real"],
    bms: ["BMS model", "Answers the control's 0x305 poll with the Pylontech-style set. Its limits are simple rules (50 A, half above 95 % SOC), not firmware.", "model"],
    afe: ["Front end", "Part of the BMS model.", "model"],
    micro: ["Inverter", "Model built from what the control sends and parses; the inverter firmware is not available.", "model"] },
  "Venus C/E": { mcu: ["STM32G474 control", "Runs the unmodified control firmware, power stage included: the 30 kHz control loop on the HRTIM, the PLL, the relays, the LLC and bridge, plus Wi‑Fi, MQTT, BLE, the CT poll and steering.", "real"] },
  "CT002 / CT003": { mcu: ["STM32F0 meter", "Runs the unmodified CT firmware: answers the batteries' UDP polls, reports over MQTT and HTTP.", "real"] },
};
const CHMAP_V = { mcu: null, fc41d: ["UART6", "USART1", "LPUART1", "UDP", "MQTT", "RF", "BLE"], mppt: ["CAN"], relay: ["FW"], bridge: ["FW"], dcbus: ["FW"], llc: ["FW"],
  flash: ["FW"], clampA: ["UDP"], clampB: ["UDP"], clampC: ["UDP"], bat1: ["UDP"], bat2: ["UDP"], bat3: ["UDP"], micro: ["CAN"], bms: ["CAN", "ALARM"], afe: ["CAN"], pack: ["CAN", "ALARM"], fets: ["CAN", "ALARM"], eeprom: ["FW"], rs485: ["FW"], modbus: ["FW"], ct: ["RF"], broker: ["MQTT"], phone: ["BLE"], house: ["EVT"] };
const CHMAP = { mcu: null, fc41d: ["USART1", "MQTT", "RF", "BLE"], afe: ["I2C2", "ALARM"], pack: ["I2C2", "UART5", "ALARM"], fets: ["I2C2", "ALARM"], extp: ["UART5"], ct: ["RF"], broker: ["MQTT"], phone: ["BLE"], inv: ["FW", "EVT"], house: ["EVT"], out1: ["FW"], out2: ["FW"], pv1: ["FW", "EVT"], pv2: ["FW", "EVT"], panel1: ["EVT"], panel2: ["EVT"] };
const BS1 = ["OV", "UV", "OCD1", "OCD2", "OCC", "SC", "PF", "WDT"], BS2 = ["UTC", "OTC", "UTD", "OTD"];
function kv(rows) { return `<dl class="kv">${rows.map(([a, b]) => `<dt>${a}</dt><dd>${b}</dd>`).join("")}</dl>`; }
function regs(rows, title, note) {
  return `<div class="blk"><h4>${title}<span>${note || ""}</span></h4><table class="regs"><tbody>${rows.map((r) => `<tr><td>${r[0]}</td><td>${r[1]}</td><td>${r[2] || ""}</td></tr>`).join("")}</tbody></table></div>`;
}
function traffic(key) {
  const chs = (MODEL === "B2500" ? CHMAP : CHMAP_V)[key];
  const lines = LOG.filter((l) => !chs || chs.includes(l.ch)).slice(-6).reverse();
  return `<div class="blk"><h4>Recent traffic<span>${chs ? chs.join(" · ") : "all buses"}</span></h4><div class="mini">${lines.map((l) => `<div><span style="color:var(--fg-faint)">${l.t.toFixed(1).padStart(7)}</span> <span class="ch-${l.ch}">${l.ch}</span> ${esc(l.text)}</div>`).join("") || "<div>nothing yet</div>"}</div></div>`;
}
function cellsGrid() {
  const mn = Math.min(...S.cells), mx = Math.max(...S.cells);
  return `<div class="cells">${S.cells.map((v, i) => `<div class="cell${S.bal[i] ? " bal" : ""}${v === mn ? " min" : ""}${v === mx ? " max" : ""}"><small>C${i + 1}</small>${(v * 1000).toFixed(0)}</div>`).join("")}</div>`;
}
function flags(list) { return `<div class="flags">${list.map(([n, on, bad]) => `<span class="flag ${bad ? "bad" : on ? "on" : ""}">${n}</span>`).join("")}</div>`; }
function inspector() {
  if (!SN) { $("ibody").innerHTML = '<div class="empty">Start a device to inspect it.</div>'; return; }
  const dd = MODEL === "B2500" ? DESC : DESC_V;
  const d = (TITLES[MODEL] || {})[sel] || dd[sel] || dd.mcu, kind = d[2], r = SN.mcu.regs || {};
  let h = `<div class="ititle"><div><h2>${d[0]}</h2><p>${d[1]}</p></div><span class="badge ${kind}">${kind === "real" ? "real firmware" : kind === "ext" ? "outside the box" : "model"}</span></div>`;
  if (MODEL !== "B2500") {
    h += inspectVenus();
    h += traffic(sel);
    $("ibody").innerHTML = h;
    return;
  }
  if (sel === "mcu") {
    h += kv([["Image", esc(SN.image)], ["Profile", SN.profile], ["Firmware version", SN.version ? "V" + SN.version : "not reported yet"],
      ["Device type", SN.device_type || "–"], ["Clock", `${(SN.mcu.hz / 1e6).toFixed(0)} MHz · ${(SN.mcu.cycles / 1e6).toFixed(1)} M cycles`],
      ["Program counter", hex(SN.mcu.pc, 8)], ["Output reference", Math.round(S.target) + " W"]]);
    const dac = (v) => (v == null ? "off" : hex(v, 3));
    h += regs([
      ["TIM8_CCR3", r.TIM8_CCR3, `PV1 Vref ${SN.pv[0].v_ref.toFixed(2)} V`], ["TIM8_CCR4", r.TIM8_CCR4, `PV1 I limit ${SN.pv[0].i_lim.toFixed(2)} A`],
      ["TIM2_CCR4", r.TIM2_CCR4, `PV2 Vref ${SN.pv[1].v_ref.toFixed(2)} V`], ["TIM2_CCR3", r.TIM2_CCR3, `PV2 I limit ${SN.pv[1].i_lim.toFixed(2)} A`],
      ["DAC_DHR12R1", dac(r.DAC_DHR12R1), `OUT1 ${SN.out[0].i_lim.toFixed(3)} A`], ["DAC_DHR12R2", dac(r.DAC_DHR12R2), `OUT2 ${SN.out[1].i_lim.toFixed(3)} A`],
    ], "Peripheral registers", "what the firmware wrote");
    h += `<div class="formula">Vref = (4374 − CCR) · 500/31 mV · DAC = zero + (mA − 5) / 5.9</div>`;
    h += regs((SN.mcu.irqs || []).map(([n, rt]) => [n, rt.toFixed(0) + " /s", ""]), "Interrupts", "per emulated second");
    const adc = Object.entries(SN.mcu.adc || {});
    if (adc.length) h += regs(adc.map(([k, v]) => [k, v, ""]), "ADC inputs", "what the firmware reads");
  } else if (sel === "fc41d") {
    const ms = SN.fc41d.mqtt || [];
    h += kv([["Module firmware", esc(SN.fc41d.version)], ["Wi‑Fi", SN.fc41d.wifi ? "connected" : "disconnected"], ["BLE name", esc(SN.fc41d.ble || "–")],
      ...ms.map((s, i) => [`MQTT ${i}`, `${s.connected ? "connected" : "closed"} · ${esc(s.host)}:${s.port}`]),
      ["Client ID", esc((ms[0] || {}).client_id || "–")], ["Command topic", esc(SN.sub_topic || "not subscribed yet")]]);
  } else if (sel === "afe" || sel === "pack" || sel === "fets") {
    const mn = Math.min(...S.cells), mx = Math.max(...S.cells), bs = SN.afe.bstatus;
    h += kv([["State of charge", (S.soc * 100).toFixed(2) + " %"], ["Pack", `${S.packV.toFixed(2)} V · ${S.I.toFixed(2)} A · ${Math.round(S.pbat)} W`],
      ["Cell spread", `${((mx - mn) * 1000).toFixed(0)} mV`], ["Temperature", S.temp.toFixed(1) + " °C"],
      ["Moved", `+${SN.pack.ah_in.toFixed(2)} Ah · −${SN.pack.ah_out.toFixed(2)} Ah`], ["Register reads", SN.afe.reads]]);
    h += `<div class="blk"><h4>Cells<span>mV · min / max / balancing</span></h4>${cellsGrid()}</div>`;
    h += `<div class="blk"><h4>Front end status<span>BSTATUS1–3</span></h4>${flags([
      ["CHG", S.fets.chg, !S.fets.chg], ["DSG", S.fets.dsg, !S.fets.dsg],
      ...BS1.map((n, i) => [n, false, !!(bs[0] & (1 << i))]), ...BS2.map((n, i) => [n, false, !!(bs[1] & (1 << i))]),
      ["CHGING", !!(bs[2] & 0x80)], ["DSGING", !!(bs[2] & 0x40)], ["BAL", !!SN.afe.bal]])}</div>`;
  } else if (sel === "pv1" || sel === "pv2") {
    const i = sel === "pv1" ? 0 : 1, pv = SN.pv[i];
    h += kv([["Available", Math.round(pv.avail) + " W"], ["Harvested", Math.round(pv.w) + " W"], ["Operating point", `${pv.v.toFixed(2)} V · ${pv.a.toFixed(2)} A`],
      ["Voltage reference", pv.v_ref.toFixed(2) + " V"], ["Current limit", pv.i_lim.toFixed(2) + " A"], ["Converter", pv.on ? "on" : "off"]]);
    h += regs([[i ? "TIM2_CCR4" : "TIM8_CCR3", i ? r.TIM2_CCR4 : r.TIM8_CCR3, "voltage reference"], [i ? "TIM2_CCR3" : "TIM8_CCR4", i ? r.TIM2_CCR3 : r.TIM8_CCR4, "current limit"]], "What the firmware sets", "");
  } else if (sel === "out1" || sel === "out2") {
    const i = sel === "out1" ? 0 : 1, o = SN.out[i], dac = i ? r.DAC_DHR12R2 : r.DAC_DHR12R1;
    h += kv([["Power", Math.round(o.w) + " W"], ["Voltage · current", o.on ? `${o.v.toFixed(2)} V · ${o.a.toFixed(3)} A` : "off"], ["Current reference", o.i_lim.toFixed(3) + " A"], ["Heatsink", S.hs.toFixed(1) + " °C"]]);
    h += regs([[`DAC_DHR12R${i + 1}`, dac == null ? "off" : hex(dac, 3), "current reference"]], "What the firmware sets", "");
  } else if (sel === "inv") {
    h += kv([["AC out", Math.round(S.ac) + " W"], ...SN.inv.map((x, i) => [`Input ${i + 1}`, `${Math.round(S.out[i])} W · tracker at ${x.v_set} V`])]);
  } else if (sel === "ct") {
    const ct = SN.ct;
    h += kv([["Grid at the meter", sgn(S.grid) + " W"], ["Last answer", ct.age == null ? "none yet" : `${ct.age.toFixed(1)} s ago`], ["Polls", ct.polls], ["Answered", ct.answered], ["Meter MAC", ct.mac], ["Offline (fault)", S.f.ct ? "yes" : "no"]]);
  } else if (sel === "house") {
    h += kv([["Load", Math.round(S.load) + " W"], ["Covered by the battery", Math.round(Math.min(S.ac, S.load)) + " W"], ["From the grid", Math.round(Math.max(0, S.grid)) + " W"], ["To the grid", Math.round(Math.max(0, -S.grid)) + " W"]]);
  } else if (sel === "extp") {
    h += S.ext.length ? kv(S.ext.flatMap((e, i) => [[`Pack ${i + 1} charge`, e.soc.toFixed(1) + " %"], [`Pack ${i + 1}`, `${e.v.toFixed(2)} V · ${e.a.toFixed(2)} A`]]))
      : '<div class="empty">No extension pack. Start the device with one to see it here.</div>';
    if (SN.bus && Object.keys(SN.bus).length) h += kv(Object.entries(SN.bus).slice(0, 10).map(([k, v]) => [k, esc(Array.isArray(v) ? v.join(" ") : v)]));
  } else if (sel === "broker") {
    const s = (SN.fc41d.mqtt || [])[0];
    h += kv([["Session", s ? `${s.connected ? "connected" : "closed"} · ${esc(s.host)}:${s.port}` : "none"], ["Command topic", esc(SN.sub_topic || "–")]]);
  } else if (sel === "phone") {
    h += kv([["Advertising as", esc(SN.fc41d.ble || "–")], ["Central", "connect with marsemu's BLE bridge"]]);
  } else if (sel === "panel1" || sel === "panel2") {
    const i = sel === "panel1" ? 0 : 1;
    h += kv([["Available", Math.round(S.sun[i]) + " W"], ["Harvested", Math.round(S.pv[i]) + " W"]]);
  }
  h += traffic(sel);
  $("ibody").innerHTML = h;
}
function inspectCt() {
  const me = SN.meter, last = me.last || [];
  if (sel === "mcu" || sel === "flash") {
    return kv([["Image", esc(SN.image)], ["Profile", SN.profile], ["Firmware version", SN.version ? "V" + SN.version : "–"],
      ["Device type", SN.device_type || "–"], ["Meter id (BLE MAC)", me.id], ["Answers sent", me.replies],
      ["Clock", `${(SN.mcu.hz / 1e6).toFixed(0)} MHz`], ["Program counter", hex(SN.mcu.pc, 8)]])
      + regs((SN.mcu.irqs || []).map(([n, rt]) => [n, rt.toFixed(0) + " /s", ""]), "Interrupts", "per emulated second")
      + (last.length ? regs([["model", last[0]], ["meter", last[1]], ["to", `${last[2]} ${last[3]}`], ["A / B / C", `${last[4]} / ${last[5]} / ${last[6]} W`],
        ["total", `${last[7]} W`], ["batteries A/B/C/?", last.slice(8, 12).join(" / ")], ["RSSI", last[12]], ["fields 13–", last.slice(13).join(" ")]], "Last answer", "relay mode") : "");
  }
  if (sel.startsWith("bat")) {
    const b = (SN.batteries || [])[+sel.slice(3) - 1];
    if (!b) return '<div class="empty">Not polling. Raise "Batteries polling the meter".</div>';
    return kv([["Device type", b.type], ["MAC", b.mac], ["Phase", b.phase], ["Reports", `${Math.round(b.w)} W`], ["Answers", b.replies]])
      + (b.last ? regs([["A / B / C", `${b.last[4]} / ${b.last[5]} / ${b.last[6]} W`], ["batteries per phase", b.last.slice(8, 11).join(" / ")]], "Its last answer", "") : "");
  }
  if (sel.startsWith("clamp")) {
    const ph = sel.slice(-1), cl = SN.meter.clamps;
    return kv([["Measured", cl ? `${sgn(cl[ph])} W (house ${Math.round(SN.meter.house_load / 3)} W minus batteries on ${ph})` : "0 W (not modelled)"],
      ["In the answers", `${last[{ clampA: 4, clampB: 5, clampC: 6 }[sel]] ?? 0} W`],
      ["Path", !cl ? "–" : SN.meter.source === "p1" ? "utility meter → P1 telegram (DSMR, USART3) → firmware" : "metering chip (BL09xx, bit-banged SPI) → firmware"]]);
  }
  if (sel === "fc41d") return kv([["Module firmware", esc(SN.fc41d.version)], ["Wi‑Fi", SN.fc41d.wifi ? "connected" : "disconnected"], ["UDP service", me.socket ? "port 12345" : "not open yet"], ["BLE name", esc(SN.fc41d.ble || "–")], ["Command topic", esc(SN.sub_topic || "–")]]);
  if (sel === "broker") { const s0 = (SN.fc41d.mqtt || [])[0]; return kv([["Session", s0 ? `${s0.connected ? "connected" : "closed"} · ${esc(s0.host)}:${s0.port}` : "none"]]); }
  return "";
}
function inspectCE() {
  const st = SN.stage;
  if (sel === "relay" || sel === "bridge" || sel === "dcbus" || sel === "llc") {
    return kv([["AC power", sgn(st.ac_w) + " W"], ["Battery power", sgn(st.bat_w) + " W"], ["DC bus", `${num(st.vbus)} V`],
      ["Grid relay", st.relay ? "closed" : "open"], ["Bridge", st.bridge ? "switching" : "off"], ["LLC", st.llc ? "running" : "off"],
      ["Bypass", st.bypass ? "closed" : "open"], ["Peak current", `${num(st.i_peak_a, 2)} A`]]);
  }
  return null;
}
function inspectVenus() {
  if (MODEL === "CT002 / CT003") return inspectCt();
  if (MODEL === "Venus C/E") { const r = inspectCE(); if (r != null) return r; }
  const inv = SN.inverter, bms = SN.bms;
  if (sel === "mppt" || sel.startsWith("string")) {
    const mp = SN.mppt;
    return kv([["PV harvested", `${Math.round(sum(mp.pv))} W`], ...mp.pv.map((w, k) => [`String ${k + 1}`, `${Math.round(w)} of ${Math.round(mp.avail[k])} W`]),
      ["PV limit from the control", `${mp.limit} W`], ["Into the pack", `${Math.round(mp.dc)} W`], ["Mode", mp.mode], ["Protocol", "v" + mp.version]]);
  }
  if (sel === "mcu") {
    return kv([["Image", esc(SN.image)], ["Profile", SN.profile], ["Firmware version", SN.version ? "V" + SN.version : "not reported yet"],
      ["Device type", SN.device_type || "–"], ["Clock", `${(SN.mcu.hz / 1e6).toFixed(0)} MHz · ${(SN.mcu.cycles / 1e6).toFixed(1)} M cycles`],
      ["Program counter", hex(SN.mcu.pc, 8)], SN.stage ? ["AC power (its own bridge)", sgn(SN.stage.ac_w) + " W"] : ["Setpoint to the inverter", sgn(inv.reference) + " W"]])
      + regs((SN.mcu.irqs || []).map(([n, rt]) => [n, rt.toFixed(0) + " /s", ""]), "Interrupts", "per emulated second")
      + regs(Object.entries(SN.can.by_dir).map(([k, v]) => [k, v, "frames"]), "CAN traffic", `${SN.can.frames} frames`);
  }
  if (sel === "micro" && SN.stage) return inspectCE();
  if (sel === "micro") {
    return kv([["AC power", sgn(inv.ac) + " W"], ["Reference", sgn(inv.reference) + " W"], ["DC into the pack", sgn(inv.dc) + " W"],
      ["On the grid", inv.connected ? "yes" : "no"], ["Allowed now", `${inv.limit_discharge} W out · ${-inv.limit_charge} W in`],
      ["Configured limits", `${inv.discharge_limit_w} W out · ${inv.charge_limit_w} W in`], ["Cut-off / max SOC", `${inv.cutoff_soc} % / ${inv.max_soc} %`],
      ["Work mode", inv.work_mode], ["Temperatures", `LLC ${inv.temp_llc} °C · bridge ${inv.temp_inv} °C`],
      ["Errors", inv.errors.map((e) => hex(e, 2)).join(" ")], ["Protocol", "Micro v" + inv.version]]);
  }
  if (sel === "bms" || sel === "afe" || sel === "pack" || sel === "fets") {
    const mn = Math.min(...S.cells), mx = Math.max(...S.cells), mos = bms.mos || {};
    const packs = (SN.packs || []).length > 1 ? regs(SN.packs.map((p, k) => [`pack ${k + 1}`, `${p.soc} %`, `${p.v} V · ${p.a} A · ${p.mos.charge ? "C" : "-"}${p.mos.discharge ? "D" : "-"}`]), "Packs in the stack", "") : "";
    return packs + kv([["BMS image", esc(bms.image || "none (model)")], ["Runs", bms.process], ["SOC (BMS)", num(bms.soc) + " %"],
      ["SOC (pack model)", (S.soc * 100).toFixed(2) + " %"], ["Pack", `${S.packV.toFixed(2)} V · ${S.I.toFixed(2)} A · ${Math.round(S.pbat)} W`],
      ["Charge to", `${bms.charge_v} V at ${bms.charge_a} A`], ["Discharge up to", `${bms.discharge_a} A`],
      ["Cell spread", `${((mx - mn) * 1000).toFixed(0)} mV`], ["Temperature", S.temp.toFixed(1) + " °C"],
      ["Last BMS frame", bms.seen == null ? "none yet" : `${bms.seen} s ago`]])
      + `<div class="blk"><h4>Cells<span>mV · min / max</span></h4>${cellsGrid()}</div>`
      + `<div class="blk"><h4>Protection<span>as the BMS reports it</span></h4>${flags([
        ["CHG", !!mos.charge, !mos.charge], ["DSG", !!mos.discharge, !mos.discharge], ["PRE", !!mos.precharge],
        ["may charge", !!(bms.flags & 1), !(bms.flags & 1)], ["may discharge", !!(bms.flags & 2), !(bms.flags & 2)],
        ["protection " + hex(bms.protect, 8), false, !!bms.protect]])}</div>`;
  }
  if (sel === "fc41d") {
    const ms = SN.fc41d.mqtt || [];
    return kv([["Module firmware", esc(SN.fc41d.version)], ["On", SN.fc41d.uart], ["Wi‑Fi", SN.fc41d.wifi ? "connected" : "disconnected"],
      ["BLE name", esc(SN.fc41d.ble || "–")], ...ms.map((s, i) => [`MQTT ${i}`, `${s.connected ? "connected" : "closed"} · ${esc(s.host)}:${s.port}`]),
      ["Command topic", esc(SN.sub_topic || "not subscribed yet")]]);
  }
  if (sel === "ct") {
    const ct = SN.ct;
    return kv([["Grid at the meter", sgn(S.grid) + " W"], ["Last answer", ct.age == null ? "none yet" : `${ct.age.toFixed(1)} s ago`], ["Polls", ct.polls], ["Answered", ct.answered], ["Meter MAC", ct.mac]]);
  }
  if (sel === "house") return kv([["Load", Math.round(S.load) + " W"], ["From the grid", Math.round(Math.max(0, S.grid)) + " W"], ["To the grid", Math.round(Math.max(0, -S.grid)) + " W"]]);
  if (sel === "broker") {
    const s0 = (SN.fc41d.mqtt || [])[0];
    return kv([["Session", s0 ? `${s0.connected ? "connected" : "closed"} · ${esc(s0.host)}:${s0.port}` : "none"], ["Command topic", esc(SN.sub_topic || "–")]]);
  }
  if (sel === "phone") return kv([["Advertising as", esc(SN.fc41d.ble || "–")]]);
  return "";
}
function select(key) {
  sel = key;
  document.querySelectorAll(".comp.sel").forEach((e) => e.classList.remove("sel"));
  if (key === "pack") cellsG.querySelectorAll(".comp").forEach((e) => e.classList.add("sel"));
  else { const e = $("m-" + key); if (e) e.classList.add("sel"); }
  $("inspHint").textContent = key;
  inspector();
}
svg.addEventListener("click", (e) => { const g = e.target.closest("[data-mod]"); if (g) select(g.dataset.mod); });
svg.addEventListener("keydown", (e) => {
  if ((e.key === "Enter" || e.key === " ") && e.target.dataset && e.target.dataset.mod) { e.preventDefault(); select(e.target.dataset.mod); }
});

/* ------------------------------------------------------------ console */
const LOG = []; const LOGMAX = 600;
let CHANNELS = ["FW", "I2C2", "USART1", "UART5", "MQTT", "RF", "BLE", "EVT", "ALARM"];
const shown = new Set(["FW", "I2C2", "USART1", "UART5", "UART6", "LPUART1", "CAN", "UDP", "MQTT", "RF", "BLE", "EVT", "ALARM"]);
let logDirty = true;
function log(ch, dir, text, t) {
  LOG.push({ t: t ?? S.t, ch, dir, text });
  if (LOG.length > LOGMAX) LOG.shift();
  logDirty = true;
}
const filt = $("filters");
function buildFilters(channels) {
  CHANNELS = channels;
  filt.replaceChildren();
  CHANNELS.forEach((ch) => {
    const b = document.createElement("button");
    b.textContent = ch; b.className = "ch-" + ch; b.setAttribute("aria-pressed", String(shown.has(ch)));
    b.addEventListener("click", () => { if (shown.has(ch)) shown.delete(ch); else shown.add(ch); b.setAttribute("aria-pressed", shown.has(ch)); logDirty = true; renderLog(); });
    filt.appendChild(b);
  });
}
buildFilters(CHANNELS);
const logEl = $("log");
let stick = true;
logEl.addEventListener("scroll", () => { stick = logEl.scrollTop + logEl.clientHeight >= logEl.scrollHeight - 8; });
function renderLog() {
  if (!logDirty) return;
  logDirty = false;
  const rows = LOG.filter((l) => shown.has(l.ch)).slice(-200);
  logEl.innerHTML = rows.map((l) => `<div class="ln" title="${esc(l.dir)}"><span class="t">${l.t.toFixed(1).padStart(7)}</span><span class="c ch-${l.ch}">${l.ch}</span><span class="x">${l.dir ? `<span style="color:var(--fg-faint)">${esc(l.dir)}</span>  ` : ""}${esc(l.text)}</span></div>`).join("");
  if (stick) logEl.scrollTop = logEl.scrollHeight;
}

/* ------------------------------------------------------------- history */
const hist = []; const HMAX = 600;
let pendingMark = null;
/* ------------------------------------------------------------- charts */
const charts = [
  { cv: $("c1"), series: [["grid", "fg"], ["ac", "discharge"], ["pv", "solar"], ["bat", "charge"]], zero: true },
  { cv: $("c2"), series: [["soc", "xray"]], right: ["v", "fg-dim"] },
  { cv: $("c3"), band: ["cmin", "cmax"] },
];
let hoverX = null;
charts.forEach((ch) => {
  ch.cv.addEventListener("pointermove", (e) => { const r = ch.cv.getBoundingClientRect(); hoverX = (e.clientX - r.left) / r.width; });
  ch.cv.addEventListener("pointerleave", () => { hoverX = null; $("hoverRead").textContent = ""; });
});
function drawCharts() {
  if (!hist.length) return;
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const t1 = hist[hist.length - 1].t, t0 = t1 - HMAX * 0.5;
  let hi = null;
  if (hoverX != null) {
    const tt = t0 + hoverX * (t1 - t0);
    hi = hist.reduce((a, b) => (Math.abs(b.t - tt) < Math.abs(a.t - tt) ? b : a));
  }
  charts.forEach((ch, ci) => {
    const cv = ch.cv, W = cv.clientWidth, H = cv.clientHeight;
    if (!W || !H) return;
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
    const g = cv.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    const L = 36, R = ch.right ? 36 : 6, T = 8, B = 16;
    const x = (t) => L + (t - t0) / (t1 - t0) * (W - L - R);
    let lo, hiV;
    if (ch.band) {
      lo = Math.min(...hist.map((h) => h.cmin)) - 4; hiV = Math.max(...hist.map((h) => h.cmax)) + 4;
    } else {
      const vals = hist.flatMap((h) => ch.series.map(([k]) => h[k]));
      lo = Math.min(0, ...vals); hiV = Math.max(...vals, ch.zero ? 100 : 1);
      if (ci === 1) { lo = Math.max(0, Math.floor(Math.min(...vals) - 2)); hiV = Math.min(100, Math.ceil(Math.max(...vals) + 2)); }
    }
    if (hiV - lo < 1e-6) hiV = lo + 1;
    const nice = (v) => { const p = Math.pow(10, Math.floor(Math.log10(v))); const m = v / p; return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p; };
    const step = nice((hiV - lo) / 3);
    lo = Math.floor(lo / step) * step; hiV = Math.ceil(hiV / step) * step;
    const y = (v) => T + (1 - (v - lo) / (hiV - lo)) * (H - T - B);
    g.font = `500 9.5px ${C("f-mono")}`;
    g.textBaseline = "middle";
    for (let v = lo; v <= hiV + 1e-9; v += step) {
      g.strokeStyle = v === 0 && ch.zero ? C("line-2") : C("line");
      g.lineWidth = 1;
      g.beginPath(); g.moveTo(L, Math.round(y(v)) + .5); g.lineTo(W - R, Math.round(y(v)) + .5); g.stroke();
      g.fillStyle = C("fg-faint"); g.textAlign = "right";
      g.fillText(!ch.band && Math.abs(v) >= 1000 ? (v / 1000).toFixed(1) + "k" : String(Math.round(v * 10) / 10), L - 5, y(v));
    }
    g.textAlign = "left"; g.textBaseline = "alphabetic"; g.fillStyle = C("fg-faint");
    g.fillText("−5 min", L, H - 3); g.textAlign = "right"; g.fillText("now", W - R, H - 3);
    // event marks
    hist.forEach((h) => { if (h.ev) { g.strokeStyle = C("warn"); g.globalAlpha = .55; g.setLineDash([2, 3]); g.beginPath(); g.moveTo(x(h.t) + .5, T); g.lineTo(x(h.t) + .5, H - B); g.stroke(); g.setLineDash([]); g.globalAlpha = 1; if (ci === 0) { g.fillStyle = C("warn"); g.textAlign = "left"; g.fillText(h.ev, x(h.t) + 3, T + 9); } } });
    const line = (k, col, yy = y, width = 1.4) => {
      g.strokeStyle = C(col); g.lineWidth = width; g.beginPath();
      hist.forEach((h, i) => { const px = x(h.t), py = yy(h[k]); i ? g.lineTo(px, py) : g.moveTo(px, py); });
      g.stroke();
      const last = hist[hist.length - 1];
      g.fillStyle = C(col); g.beginPath(); g.arc(x(last.t), yy(last[k]), 2.4, 0, 7); g.fill();
    };
    if (ch.band) {
      g.fillStyle = "rgba(142,241,255,.08)";
      g.beginPath();
      hist.forEach((h, i) => { const px = x(h.t), py = y(h.cmax); i ? g.lineTo(px, py) : g.moveTo(px, py); });
      for (let i = hist.length - 1; i >= 0; i--) g.lineTo(x(hist[i].t), y(hist[i].cmin));
      g.closePath(); g.fill();
      line("cmin", "charge"); line("cmax", "discharge");
      g.fillStyle = C("warn");
      hist.forEach((h) => { if (h.bal) g.fillRect(x(h.t) - .5, H - B - 3, 1.5, 3); });
    } else {
      if (ch.zero) {
        // area under the AC line
        g.fillStyle = "rgba(255,181,71,.07)"; g.beginPath(); g.moveTo(x(hist[0].t), y(0));
        hist.forEach((h) => g.lineTo(x(h.t), y(h.ac))); g.lineTo(x(hist[hist.length - 1].t), y(0)); g.closePath(); g.fill();
      }
      ch.series.forEach(([k, col]) => line(k, col));
      if (ch.right) {
        const vals = hist.map((h) => h.v); let a = Math.min(...vals) - .1, b = Math.max(...vals) + .1;
        const yr = (v) => T + (1 - (v - a) / (b - a)) * (H - T - B);
        line("v", "fg-dim", yr, 1);
        g.fillStyle = C("fg-faint"); g.textAlign = "left"; g.textBaseline = "middle";
        g.fillText(b.toFixed(1) + " V", W - R + 4, yr(b)); g.fillText(a.toFixed(1) + " V", W - R + 4, yr(a));
      }
    }
    if (hi) {
      g.strokeStyle = C("xray"); g.globalAlpha = .6; g.lineWidth = 1;
      g.beginPath(); g.moveTo(x(hi.t) + .5, T); g.lineTo(x(hi.t) + .5, H - B); g.stroke(); g.globalAlpha = 1;
    }
  });
  if (hi) $("hoverRead").textContent = `T+${fmtT(hi.t)} · grid ${Math.round(hi.grid)} W · AC ${Math.round(hi.ac)} W · PV ${Math.round(hi.pv)} W · bat ${Math.round(hi.bat)} W · SOC ${hi.soc.toFixed(1)} % · cells ${hi.cmin.toFixed(0)}–${hi.cmax.toFixed(0)} mV`;
}

/* --------------------------------------------------------------- lens */
const lens = { on: true, x: 565, y: 195, r: 92, m: 1.8 };
const ticksG = $("lensTicks");
for (let i = 0; i < 24; i++) {
  const a = i / 24 * Math.PI * 2, r1 = 92, r2 = i % 6 === 0 ? 103 : 98;
  el("line", { class: "lenstick", x1: 0, y1: 0, x2: 0, y2: 0, "data-a": a, "data-r1": r1, "data-r2": r2, opacity: i % 6 === 0 ? .9 : .4 }, ticksG);
}
function placeLens() {
  const { x, y, r, m } = lens;
  $("lensClipC").setAttribute("cx", x); $("lensClipC").setAttribute("cy", y);
  $("lensBgC").setAttribute("cx", x); $("lensBgC").setAttribute("cy", y);
  $("lensRing").setAttribute("cx", x); $("lensRing").setAttribute("cy", y);
  $("lensRing2").setAttribute("cx", x); $("lensRing2").setAttribute("cy", y);
  $("lensLab").setAttribute("x", x); $("lensLab").setAttribute("y", y + r + 20);
  $("lensInner").setAttribute("transform", `translate(${x} ${y}) scale(${m}) translate(${-x} ${-y})`);
  ticksG.querySelectorAll("line").forEach((t) => {
    const a = +t.dataset.a, r1 = +t.dataset.r1, r2 = +t.dataset.r2;
    t.setAttribute("x1", x + Math.cos(a) * r1); t.setAttribute("y1", y + Math.sin(a) * r1);
    t.setAttribute("x2", x + Math.cos(a) * r2); t.setAttribute("y2", y + Math.sin(a) * r2);
  });
}
function toSvg(e) {
  const pt = svg.createSVGPoint(); pt.x = e.clientX; pt.y = e.clientY;
  return pt.matrixTransform(svg.getScreenCTM().inverse());
}
svg.addEventListener("pointermove", (e) => {
  if (!lens.on || (e.pointerType === "touch" && e.buttons === 0)) return;
  const p = toSvg(e); lens.x = clamp(p.x, 0, 1200); lens.y = clamp(p.y, -40, 680); placeLens();
});
function setLens(on) { lens.on = on; $("lens").style.display = on ? "" : "none"; $("lensTog").checked = on; }
$("lensTog").addEventListener("change", (e) => setLens(e.target.checked));

/* ------------------------------------------------------------- depth */
function setDepth(d) {
  svg.classList.remove("d0", "d1", "d2"); svg.classList.add("d" + d);
  document.querySelectorAll("[data-depth]").forEach((b) => b.setAttribute("aria-pressed", String(+b.dataset.depth === d)));
  $("lensLab").textContent = d === 0 ? "BOARDS + SILICON ×1.8" : "SILICON ×1.8";
}
document.querySelectorAll("[data-depth]").forEach((b) => b.addEventListener("click", () => setDepth(+b.dataset.depth)));
$("flowTog").addEventListener("change", (e) => svg.classList.toggle("noflow", !e.target.checked));
$("dataTog").addEventListener("change", (e) => svg.classList.toggle("nodata", !e.target.checked));

/* -------------------------------------------------------- run controls */
document.querySelectorAll("[data-speed]").forEach((b) => b.addEventListener("click", () => control({ op: "speed", x: +b.dataset.speed })));
$("playBtn").addEventListener("click", () => control({ op: "pause", on: !!(SN && SN.running) }));
$("stepBtn").addEventListener("click", () => control({ op: "step", s: 1 }));
$("stopBtn").addEventListener("click", () => post("api/stop").catch((e) => log("EVT", "", "stop failed: " + e.message)));
$("askBtn").addEventListener("click", () => control({ op: "mqtt", payload: "cd=1" }));
const timers = {};
function debounced(key, fn) { clearTimeout(timers[key]); timers[key] = setTimeout(fn, 160); }
function bind(id, out, fmt, send) {
  const i = $(id);
  i.addEventListener("input", () => { $(out).textContent = fmt(+i.value); debounced(id, () => send(+i.value)); });
}
bind("sLoad", "oLoad", (v) => v + " W", (v) => control({ op: "load", w: v }));
const sendPv = () => {
  const rest = SN && SN.mppt ? SN.mppt.avail.slice(2) : [];
  control({ op: "pv", w: [+$("sPv1").value, +$("sPv2").value, ...rest] });
};
bind("sBats", "oBats", (v) => String(v), (v) => control({ op: "batteries", n: v }));
bind("sBatW", "oBatW", (v) => v + " W", (v) => control({ op: "battery_w", w: v }));
bind("sPv1", "oPv1", (v) => v + " W", sendPv);
bind("sPv2", "oPv2", (v) => v + " W", sendPv);
bind("sAmb", "oAmb", (v) => v + " °C", (v) => control({ op: "ambient", c: v }));
function syncSliders() {
  if (!SN) return;
  const put = (id, out, v, fmt) => { const i = $(id); if (document.activeElement !== i && !timers[id + "busy"]) { i.value = v; $(out).textContent = fmt(v); } };
  put("sLoad", "oLoad", Math.round(SN.load), (v) => v + " W");
  const avail = SN.pv ? SN.pv.map((p) => p.avail) : SN.mppt ? SN.mppt.avail : null;
  if (avail) {
    put("sPv1", "oPv1", Math.round(avail[0]), (v) => v + " W");
    put("sPv2", "oPv2", Math.round(avail[1]), (v) => v + " W");
  }
  if (SN.batteries) {
    put("sBats", "oBats", SN.batteries.length, (v) => String(v));
    if (SN.batteries[0]) put("sBatW", "oBatW", Math.round(SN.batteries[0].w), (v) => v + " W");
  }
  put("sAmb", "oAmb", Math.round(SN.ambient), (v) => v + " °C");
}
$("scStep").addEventListener("click", () => control({ op: "scenario", name: "load_step" }));
$("scCloud").addEventListener("click", () => control({ op: "scenario", name: "cloud" }));
$("scExport").addEventListener("click", () => control({ op: "scenario", name: "surplus" }));
$("scHouseExport").addEventListener("click", () => control({ op: "scenario", name: "export" }));
for (const [id, name] of [["fWifi", "wifi"], ["fCt", "ct"], ["fHot", "heat"]])
  $(id).addEventListener("change", (e) => control({ op: "fault", name, on: e.target.checked }));

/* -------------------------------------------------------------- trace */
const TRACE = [
  { h: "The CT002 measures the grid", p: () => `At the meter the house draws ${Math.round(S.load)} W and the battery delivers ${Math.round(S.ac)} W, so the grid shows ${sgn(S.grid)} W.`, hl: ["m-ct", "gridl", "m-house"], sel: "ct" },
  { h: "The firmware asks the meter", p: () => `The MCU sends AT+QISEND to the Wi‑Fi module, which sends a UDP poll to the meter. ${SN.ct.polls} polls so far.`, hl: ["m-mcu", "usart", "m-fc41d", "ant", "rfct", "m-ct"], sel: "fc41d", go: () => capsule("usart", 1, "QISEND") },
  { h: "The meter answers over Wi‑Fi", p: () => `The last answer said ${SN.ct.last_grid == null ? "nothing yet" : sgn(SN.ct.last_grid) + " W on phase A"}. The module hands it to the MCU on USART1.`, hl: ["m-ct", "rfct", "ant", "m-fc41d", "usart", "m-mcu"], sel: "fc41d", go: () => capsule("rfct", -1, SN.ct.last_grid == null ? "reply" : `A ${sgn(SN.ct.last_grid)} W`) },
  { h: "The firmware decides", p: () => `This is the real firmware, not a model. Its answer right now: an output reference of ${Math.round(S.target)} W, split over both outputs.`, hl: ["m-mcu"], sel: "mcu" },
  { h: "It sets the output currents", p: () => `It writes the DAC: ${SN.mcu.regs.DAC_DHR12R1 == null ? "off" : hex(SN.mcu.regs.DAC_DHR12R1, 3)} and ${SN.mcu.regs.DAC_DHR12R2 == null ? "off" : hex(SN.mcu.regs.DAC_DHR12R2, 3)}, the current references of the two output stages. The ADC reads back voltage and current.`, hl: ["m-mcu", "dac1", "dac2", "m-out1", "m-out2"], sel: "out1" },
  { h: "The microinverter follows", p: () => "Each output looks like a PV panel to the microinverter, whose own MPP tracker settles where the firmware wants it.", hl: ["m-out1", "m-out2", "o1inv", "o2inv", "m-inv"], sel: "inv" },
  { h: "AC reaches the house", p: () => `${Math.round(S.ac)} W AC flows to the house, and the grid at the meter moves toward 0 W (now ${sgn(S.grid)} W).`, hl: ["m-inv", "acl", "m-house", "gridl", "m-ct"], sel: "house" },
  { h: "The battery makes up the difference", p: () => `PV gives ${Math.round(sum(S.pv))} W and the outputs take ${Math.round(sum(S.out))} W, so the pack ${S.pbat >= 0 ? "charges with" : "gives"} ${Math.abs(Math.round(S.pbat))} W through the FETs. The front end measures every cell on I2C2.`, hl: ["m-pv1", "m-pv2", "pv1bus", "pv2bus", "busL", "busR", "bat", "m-fets", "m-afe", "i2c", "sense", "cells"], sel: "pack" },
];
const TRACE_V = [
  { h: "The CT002 measures the grid", p: () => `At the meter the house draws ${Math.round(S.load)} W and the battery delivers ${Math.round(S.ac)} W, so the grid shows ${sgn(S.grid)} W.`, hl: ["m-ct", "gridl", "m-house"], sel: "ct" },
  { h: "The control asks the meter", p: () => `The control MCU sends AT+QISEND to the Wi‑Fi module on ${SN.fc41d.uart}, which sends a UDP poll to the meter. ${SN.ct.polls} polls so far.`, hl: ["m-mcu", "usart", "m-fc41d", "ant", "rfct", "m-ct"], sel: "fc41d", go: () => capsule("usart", 1, "QISEND") },
  { h: "The meter answers over Wi‑Fi", p: () => `The last answer said ${SN.ct.last_grid == null ? "nothing yet" : sgn(SN.ct.last_grid) + " W on phase A"}.`, hl: ["m-ct", "rfct", "ant", "m-fc41d", "usart", "m-mcu"], sel: "fc41d", go: () => capsule("rfct", -1, SN.ct.last_grid == null ? "reply" : `A ${sgn(SN.ct.last_grid)} W`) },
  { h: "The control firmware decides", p: () => `This is the real firmware. Its setpoint for the inverter right now: ${sgn(SN.inverter.reference)} W (positive: deliver, negative: charge from the grid).`, hl: ["m-mcu"], sel: "mcu" },
  { h: "The setpoint goes over CAN", p: () => `Function 0x01 to the inverter board, which clamps it to what the BMS allows (${SN.inverter.limit_discharge} W out, ${-SN.inverter.limit_charge} W in) and echoes it.`, hl: ["m-mcu", "caninv", "canbus", "m-micro"], sel: "micro", go: () => capsule("caninv", 1, "setpoint") },
  { h: "The BMS sets the limits", p: () => `The BMS firmware, in its own process, answers the control's poll with voltage, current, SOC and limits: charge to ${SN.bms.charge_v} V at ${SN.bms.charge_a} A, discharge up to ${SN.bms.discharge_a} A.`, hl: ["m-bms", "canbms", "canbus", "m-mcu", "m-micro"], sel: "bms", go: () => capsule("canbms", -1, "BMS") },
  { h: "AC reaches the house", p: () => `${sgn(S.ac)} W AC from the inverter board; the grid at the meter moves toward 0 W (now ${sgn(S.grid)} W).`, hl: ["m-micro", "acl", "m-house", "gridl", "m-ct"], sel: "house" },
  { h: "The pack follows", p: () => `${S.pbat >= 0 ? "Into" : "Out of"} the pack: ${Math.abs(Math.round(S.pbat))} W through the MOSFETs. The front end measures all 16 cells for the BMS on SPI.`, hl: ["m-micro", "bat", "m-fets", "m-afe", "m-bms", "spi", "sense", "cells"], sel: "pack" },
];
let tri = -1;
const dotsEl = $("trDots");
const TRACE_CE = [
  { h: "The CT002 measures the grid", p: () => `At the meter the house draws ${Math.round(S.load)} W and the device delivers ${Math.round(S.ac)} W: the grid shows ${sgn(S.grid)} W.`, hl: ["m-ct", "gridl", "m-house"], sel: "ct" },
  { h: "The control asks the meter", p: () => `AT+QISEND on LPUART1 to the Wi‑Fi module, which sends a UDP poll. ${SN.ct.polls} polls so far.`, hl: ["m-mcu", "usart", "m-fc41d", "ant", "rfct", "m-ct"], sel: "fc41d", go: () => capsule("usart", 1, "QISEND") },
  { h: "The meter answers", p: () => `The last answer said ${SN.ct.last_grid == null ? "nothing yet" : sgn(SN.ct.last_grid) + " W"}.`, hl: ["m-ct", "rfct", "ant", "m-fc41d", "usart", "m-mcu"], sel: "fc41d", go: () => capsule("rfct", -1, "reply") },
  { h: "One MCU decides and drives", p: () => "Real firmware: the same STM32G474 that runs the steering runs the 30 kHz control loop of the power stage.", hl: ["m-mcu"], sel: "mcu" },
  { h: "The bridge and the relay", p: () => `The HRTIM switches the inverter bridge (${SN.stage.bridge ? "switching" : "off"}); the grid relay is ${SN.stage.relay ? "closed" : "open"}.`, hl: ["m-mcu", "hrtimb", "gpio", "m-bridge", "m-relay"], sel: "bridge" },
  { h: "The LLC and the DC bus", p: () => `The LLC moves power between the pack and the DC bus (${num(SN.stage.vbus)} V), which the ADC measures.`, hl: ["m-mcu", "hrtiml", "adc", "m-llc", "m-dcbus"], sel: "llc" },
  { h: "AC reaches the house", p: () => `${sgn(S.ac)} W AC; the grid at the meter moves toward 0 W (now ${sgn(S.grid)} W).`, hl: ["m-relay", "acl", "m-house", "gridl", "m-ct"], sel: "house" },
  { h: "The pack and its BMS", p: () => `${S.pbat >= 0 ? "Into" : "Out of"} the pack: ${Math.abs(Math.round(S.pbat))} W. The BMS firmware reports over CAN from its own process.`, hl: ["bat", "llcbus", "m-fets", "m-afe", "m-bms", "canbms", "spi", "sense", "cells"], sel: "pack" },
];
const TRACE_CT = [
  { h: "A battery polls the meter", p: () => `Every 2.7 s each battery sends a UDP poll to port 12345 with its phase and the power it delivers. ${(SN.batteries || []).length} are polling.`, hl: ["m-bat1", "rfbat1", "ant", "m-fc41d"], sel: "bat1", go: () => capsule("rfbat1", -1, "poll") },
  { h: "The module hands it over", p: () => "The Wi‑Fi module passes the datagram to the meter's MCU on USART1 (+QIURC, then AT+QIRD).", hl: ["m-fc41d", "usart", "m-mcu"], sel: "fc41d", go: () => capsule("usart", -1, "+QIURC") },
  { h: "The firmware answers", p: () => `Real firmware: grid per phase (${!SN.meter.clamps ? "0 W, the clamps are not modelled" : SN.meter.source === "p1" ? "parsed from the utility meter's P1 telegram" : "read from the metering chip"}), the total, how many batteries report on each phase (${((SN.meter.last || []).slice(8, 11).join("/")) || "–"}), its RSSI.`, hl: ["m-mcu", "m-clampA", "m-clampB", "m-clampC", "adcA", "adcB", "adcC"], sel: "mcu" },
  { h: "The answer goes back", p: () => `AT+QISEND, then UDP to the battery. ${SN.meter.replies} answers so far.`, hl: ["m-mcu", "usart", "m-fc41d", "ant", "rfbat1", "m-bat1"], sel: "bat1", go: () => capsule("rfbat1", 1, "answer") },
  { h: "It reports to the cloud stand-in", p: () => "Over MQTT and HTTP the meter reports what it measures and how many batteries it serves; marsemu answers both locally.", hl: ["m-mcu", "usart", "m-fc41d", "ant", "rfbroker", "m-broker"], sel: "broker" },
];
const traceSteps = () => (MODEL === "B2500" ? TRACE : MODEL === "Venus C/E" ? TRACE_CE : MODEL === "CT002 / CT003" ? TRACE_CT : TRACE_V);
function traceGo(i) {
  const TR = traceSteps();
  if (!SN || i < 0 || i >= TR.length) return traceEnd();
  if (dotsEl.children.length !== TR.length) { dotsEl.replaceChildren(); TR.forEach(() => dotsEl.appendChild(document.createElement("i"))); }
  tri = i;
  svg.classList.add("tracing");
  $("trace").hidden = false;
  const s = TR[i];
  $("trN").textContent = i + 1; $("trH").textContent = s.h; $("trP").textContent = s.p();
  [...dotsEl.children].forEach((d, j) => d.classList.toggle("on", j <= i));
  svg.querySelectorAll(".hl").forEach((e) => e.classList.remove("hl"));
  s.hl.forEach((k) => {
    if (k === "cells") cellsG.querySelectorAll(".comp").forEach((e) => e.classList.add("hl"));
    const byId = $(k); if (byId) byId.classList.add("hl");
    svg.querySelectorAll(`[data-k="${k}"]`).forEach((e) => e.classList.add("hl"));
  });
  select(s.sel);
  if (s.go) s.go();
  $("trPrev").disabled = i === 0;
  $("trNext").textContent = i === TR.length - 1 ? "Done" : "Next";
}
function traceEnd() {
  tri = -1; svg.classList.remove("tracing"); $("trace").hidden = true;
  svg.querySelectorAll(".hl").forEach((e) => e.classList.remove("hl"));
}
$("traceBtn").addEventListener("click", () => traceGo(0));
$("trNext").addEventListener("click", () => traceGo(tri + 1));
$("trPrev").addEventListener("click", () => traceGo(Math.max(0, tri - 1)));
$("trEnd").addEventListener("click", traceEnd);
document.addEventListener("keydown", (e) => {
  if (e.target.matches("input,select,textarea")) return;
  if (e.key === "Escape") traceEnd();
  else if (e.key === "l" || e.key === "L") setLens(!lens.on);
  else if (e.key === "0" || e.key === "1" || e.key === "2") setDepth(+e.key);
  else if ((e.key === "t" || e.key === "T") && SN) traceGo(0);
  else if (e.key === "ArrowRight" && tri >= 0) traceGo(tri + 1);
  else if (e.key === "ArrowLeft" && tri > 0) traceGo(tri - 1);
});

/* ------------------------------------------------------------ layouts */
const LEGENDS = {
  "B2500": [["solar", "PV"], ["charge", "into the pack"], ["discharge", "toward the house"], ["i2c", "I2C2"], ["uart", "USART1"], ["ext", "UART5"], ["analog", "PWM / DAC / ADC"], ["rf", "Wi‑Fi / BLE"]],
  "Venus E 3.0": [["charge", "into the pack"], ["discharge", "toward the house"], ["ext", "CAN"], ["uart", "UART6"], ["i2c", "I2C / SPI"], ["analog", "RS485"], ["rf", "Wi‑Fi / BLE"]],
  "Venus D": [["solar", "PV"], ["charge", "into the pack"], ["discharge", "toward the house"], ["ext", "CAN"], ["uart", "UART6"], ["i2c", "I2C / SPI"], ["analog", "RS485"], ["rf", "Wi‑Fi / BLE"]],
  "Jupiter C/E": [["solar", "PV"], ["charge", "into the pack"], ["discharge", "toward the house"], ["ext", "CAN"], ["uart", "USART1"], ["i2c", "SPI"], ["rf", "Wi‑Fi / BLE"]],
  "Venus C/E": [["charge", "into the pack"], ["discharge", "toward the house"], ["ext", "FDCAN"], ["uart", "LPUART1"], ["analog", "HRTIM / ADC / GPIO"], ["i2c", "SPI"], ["rf", "Wi‑Fi / BLE"]],
  "CT002 / CT003": [["uart", "USART1"], ["i2c", "SPI flash"], ["analog", "metering chip / P1 port"], ["rf", "Wi‑Fi: UDP, MQTT, BLE"]],
};
LEGENDS["Venus A"] = LEGENDS["Venus D"]; LEGENDS["Jupiter C Plus"] = LEGENDS["Jupiter C/E"];
const SHELL = { "B2500": ["B2500", "2.24 kWh · 800 W"], "Venus E 3.0": ["VENUS E", "5.12 kWh · 2.5 kW"],
  "Venus D": ["VENUS D", "2.2 kW · 4 MPPT"], "Venus A": ["VENUS A", "1.45 kW · 4 MPPT"], "Venus C/E": ["VENUS", "5.12 kWh · 2.5 kW"],
  "Jupiter C/E": ["JUPITER", "inverter · MPPT · pack"], "Jupiter C Plus": ["JUPITER C+", "inverter · MPPT · pack"],
  "CT002 / CT003": ["CT002", "smart meter"] };
function buildLayout(model, channels) {
  if (!LAYOUTS[model]) model = "B2500";
  traceEnd();
  MODEL = model;
  clearLayout();
  LAYOUTS[model]();
  $("legend").innerHTML = LEGENDS[model].map(([c, t]) => `<span><i style="background:var(--${c})"></i>${t}</span>`).join("");
  $("shellName").textContent = SHELL[model][0]; $("shellSub").textContent = SHELL[model][1];
  const pv = model === "B2500" || !!(SN && SN.mppt), ct = model === "CT002 / CT003";
  document.querySelectorAll(".only-pv").forEach((e) => { e.hidden = !pv; });
  document.querySelectorAll(".only-nopv").forEach((e) => { e.hidden = pv || ct; });
  document.querySelectorAll(".only-ct").forEach((e) => { e.hidden = !ct; });
  document.querySelectorAll(".only-house").forEach((e) => { e.hidden = ct; });
  // a CT002's clamps see the house: its load is a control there too
  $("sLoad").parentElement.hidden = ct && !(SN && SN.meter && SN.meter.clamps);
  $("houseHead").textContent = ct ? "House and batteries" : pv ? "House and sun" : "House";
  $("scStep").hidden = ct; $("fHot").parentElement.hidden = ct; $("fCt").parentElement.hidden = ct;
  if (channels) buildFilters(channels);
  sel = "mcu";
  const e = $("m-mcu"); if (e) e.classList.add("sel");
  $("inspHint").textContent = "mcu";
}

/* ------------------------------------------------------- start screen */
let images = [], chosen = null, canUpload = true;
let bmsFor = null;   // the control image the BMS choice was made for
function renderImages() {
  const box = $("imgs");
  const devs = images.filter((i) => i.role !== "bms"), bmss = images.filter((i) => i.role === "bms");
  if (!devs.length) box.innerHTML = '<div class="empty">No firmware images in the firmware folder yet.</div>';
  else box.innerHTML = devs.map((im) => `<button class="img" data-img="${esc(im.name)}" aria-pressed="${im.name === chosen}" ${im.supported ? "" : "disabled"}>
      <small class="${im.supported ? "ok" : ""}">${esc(im.supported ? im.model : "not yet")}</small>
      <span class="nm" title="${esc(im.name)}">${esc(im.name)}</span>
      <small>${esc(im.profile || im.error || "unknown")}</small></button>`).join("");
  box.querySelectorAll("[data-img]").forEach((b) => b.addEventListener("click", () => { chosen = b.dataset.img; renderImages(); }));
  const ch = images.find((i) => i.name === chosen);
  const needBms = !!(ch && ch.needs_bms);
  $("bmsPick").hidden = !needBms;
  if (needBms) {
    const cur = $("bmsSel").value;
    $("bmsSel").innerHTML = bmss.length ? bmss.map((b) => `<option value="${esc(b.name)}">${esc(b.name)}</option>`).join("")
      : '<option value="">add the BMS image too</option>';
    // a newly chosen control image gets the BMS image of its own model (VNSD-0 with VNSD-0)
    const tag = (chosen.match(/[A-Z]{3,}\d*-\d+/) || [])[0];
    const own = tag && bmss.find((b) => b.name.includes(tag));
    const keep = chosen === bmsFor && bmss.find((b) => b.name === cur);
    const pick = keep || own || bmss.find((b) => /bms/i.test(b.name)) || bmss[0];
    bmsFor = chosen;
    if (pick) $("bmsSel").value = pick.name;
  }
  const optBms = !!(ch && ch.bms_optional);
  if (optBms) {
    $("bmsPick").hidden = false;
    const cur = $("bmsSel").value;
    $("bmsSel").innerHTML = '<option value="">none: the BMS model</option>' + bmss.map((b) => `<option value="${esc(b.name)}">${esc(b.name)}</option>`).join("");
    if (bmss.some((b) => b.name === cur)) $("bmsSel").value = cur;
  }
  const pvIn = !ch || ch.pv_inputs > 0, ct = !!(ch && ch.model === "CT002 / CT003");
  $("oPvA").parentElement.hidden = !pvIn; $("oPvB").parentElement.hidden = !pvIn;
  $("oLoadS").parentElement.hidden = ct; $("oSoc").parentElement.hidden = ct;
  $("lPacks").hidden = !(ch && ch.packs); $("lBats").hidden = !ct;
  $("startBtn").disabled = !chosen || (needBms && !bmss.length);
  $("drop").hidden = !canUpload;
}
function showStart(on) { $("start").hidden = !on; document.body.classList.toggle("starting", on); if (on) { traceEnd(); renderImages(); } }
async function upload(file) {
  $("startErr").textContent = "";
  try {
    const info = await post("api/firmware?name=" + encodeURIComponent(file.name), file, true);
    const st = await (await fetch("api/state")).json();
    images = st.images; chosen = info.supported ? info.name : chosen;
    if (!info.supported && info.role !== "bms") $("startErr").textContent = `${info.name}: ${info.profile || "unknown image"} is not in the dashboard yet.`;
    renderImages();
  } catch (e) { $("startErr").textContent = e.message; }
}
const drop = $("drop");
drop.addEventListener("click", () => $("file").click());
drop.addEventListener("keydown", (e) => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); $("file").click(); } });
$("file").addEventListener("change", (e) => { if (e.target.files[0]) upload(e.target.files[0]); e.target.value = ""; });
drop.addEventListener("dragover", (e) => { e.preventDefault(); drop.classList.add("over"); });
drop.addEventListener("dragleave", () => drop.classList.remove("over"));
drop.addEventListener("drop", (e) => { e.preventDefault(); drop.classList.remove("over"); if (e.dataTransfer.files[0]) upload(e.dataTransfer.files[0]); });
$("startBtn").addEventListener("click", async () => {
  $("startErr").textContent = ""; $("startBtn").disabled = true; $("startBtn").textContent = "Starting…";
  try {
    const ch = images.find((i) => i.name === chosen);
    await post("api/start", { image: chosen, bms: ch && (ch.needs_bms || ch.bms_optional) ? $("bmsSel").value || undefined : undefined,
      pv: [+$("oPvA").value, +$("oPvB").value], load: +$("oLoadS").value, soc: +$("oSoc").value,
      packs: ch && ch.packs ? +$("oPacks").value : undefined, batteries: ch && ch.model === "CT002 / CT003" ? +$("oBats2").value : undefined });
  } catch (e) { $("startErr").textContent = e.message; }
  $("startBtn").disabled = !chosen; $("startBtn").textContent = "Start";
});

/* ------------------------------------------------------------- stream */
let fresh = true;
function onState(st) {
  images = st.images || []; canUpload = st.upload !== false;
  if (!chosen && images.some((i) => i.supported)) chosen = (images.find((i) => i.name === st.image) || images.find((i) => i.supported)).name;
  if (!MODEL) buildLayout("B2500");
  if (st.running) {
    showStart(false);
    if (st.snapshot) { fromSnap(st.snapshot); syncSliders(); }
  } else {
    SN = null; showStart(true);
    for (const [id, txt] of [["chipWifi", "Wi‑Fi"], ["chipMqtt", "MQTT"], ["chipCt", "CT002"], ["chipBle", "BLE"]]) {
      const c = $(id); c.classList.add("off"); c.classList.remove("bad"); c.lastChild.nodeValue = txt;
    }
    $("crumb").textContent = "no device running"; $("devState").textContent = "ready";
  }
  LOG.length = 0; hist.length = 0; logDirty = true; fresh = true;
}
function onTick(r) {
  fromSnap(r.snapshot);
  for (const ev of r.events) {
    log(ev.ch, ev.dir, ev.text, ev.t);
    if (!fresh && $("dataTog").checked) capsuleFor(ev);
  }
  for (const h of r.history) { hist.push(h); if (hist.length > HMAX) hist.shift(); }
  fresh = false;
}
let es = null;
function connect() {
  es = new EventSource("api/stream");
  const link = (ok, txt) => { const c = $("chipLink"); c.classList.toggle("bad", !ok); c.lastChild.nodeValue = txt; };
  es.onopen = () => link(true, "live");
  es.onerror = () => link(false, "reconnecting");
  es.addEventListener("hello", (e) => onState(JSON.parse(e.data)));
  es.addEventListener("idle", (e) => { images = JSON.parse(e.data).images || []; if (!SN) renderImages(); });
  es.addEventListener("tick", (e) => onTick(JSON.parse(e.data)));
}

/* --------------------------------------------------------------- loop */
let last = performance.now(), slowT = 0;
function frame(now) {
  const dt = clamp((now - last) / 1000, 0, 0.1); last = now;
  animFlows(dt);
  animCaps(now);
  if (SN) render();
  if (!reduced && SN) $("beam").setAttribute("x", (170 + ((now / 7000) % 1) * 820).toFixed(1));
  if (now - slowT > 250) { slowT = now; renderHud(); syncSliders(); inspector(); renderLog(); drawCharts(); }
  else if (hoverX != null) drawCharts();
  requestAnimationFrame(frame);
}
placeLens(); setDepth(1); connect();
requestAnimationFrame(frame);
window.addEventListener("resize", () => drawCharts());
})();
