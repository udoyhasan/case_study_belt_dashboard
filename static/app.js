/* Conveyor line HMI client: WebSocket state -> mimic, trends, faults, events, table. No dependencies. */
(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const SVGNS = "http://www.w3.org/2000/svg";
  const KEEP_S = 600;

  const FAULT_LABEL = {
    OVERLOAD: "Overload", OVER_CURRENT: "Over-current", ABN_CURRENT: "Current high for load (stall/jam)",
    BELT_SLIP: "Belt slip", VIBRATION: "Vibration", OVERHEAT: "Overheat",
    OVERSPEED: "Overspeed", STALL: "Motor stall / Jam",
  };
  // which physical part / reading a fault belongs to
  const FAULT_PART = {
    OVERLOAD: "load", OVER_CURRENT: "current", ABN_CURRENT: "current", BELT_SLIP: "current",
    VIBRATION: "vib", OVERHEAT: "temp", OVERSPEED: "current", STALL: "current",
  };
  const PARTS = {
    load: ["g-load", "tag-load", "ld-load", "parcels"],
    current: ["g-motor", "tag-current", "ld-current"],
    vib: ["g-vib", "tag-vib", "ld-vib"],
    temp: ["g-temp", "tag-temp", "ld-temp"],
  };
  const MODE_TEXT = {
    NORMAL: (run) => (run ? "Running normally" : "Stopped"),
    ADAPTING: () => "Adapting speed",
    ALARM: () => "Alarm",
    TRIPPED: () => "Stopped by protection",
    "E-STOP": () => "Emergency stop",
  };

  let rows = [];
  let S = null;              // latest server state
  let clockOffset = 0;       // server time - client time
  let windowS = 120;
  let hoverT = null;
  let lastTableT = -1;
  const holdUntil = { speed: 0, auto_adapt: 0, load_aware: 0 };

  const fmt = (v, d = 0) => (v === null || v === undefined || Number.isNaN(v) ? "–"
    : Number(v).toLocaleString(undefined, { minimumFractionDigits: d, maximumFractionDigits: d }));
  const nowS = () => Date.now() / 1000 + clockOffset;
  const clock = (t) => new Date(t * 1000).toLocaleTimeString([], { hour12: false });
  const expectedCurrent = (L, load) => L.cur_idle + L.cur_per_kg * Math.max(load || 0, 0) / 1000;
  const currentTol = (L, exp) => Math.max(L.cur_floor, exp * L.cur_tol);
  const vibLimit = (L, load) => L.vib_base + L.vib_per_kg * Math.max(load || 0, 0) / 1000;

  // ------------------------------------------------------------------ socket
  let ws;
  function connect() {
    ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
    ws.onmessage = (e) => onState(JSON.parse(e.data));
    ws.onclose = () => {
      setNotice("Lost connection to the dashboard server. Retrying…");
      $("mode").textContent = "Offline";
      $("mode").className = "mode is-bad";
      setTimeout(connect, 1500);
    };
  }
  const send = (action, value) => ws && ws.readyState === 1 && ws.send(JSON.stringify({ action, value }));

  function onState(s) {
    S = s;
    clockOffset = s.now - Date.now() / 1000;
    if (s.rows.length) {
      rows.push(...s.rows);
      const cut = s.now - KEEP_S;
      let i = 0;
      while (i < rows.length && rows[i].t < cut) i++;
      if (i) rows = rows.slice(i);
    }
    render();
  }

  // ------------------------------------------------------------------ render
  function render() {
    const last = rows[rows.length - 1];
    renderBar();
    renderAlarm();
    renderMimic(last);
    renderFaults();
    renderEvents();
    renderCharts();
    if (last && last.t !== lastTableT) { lastTableT = last.t; renderTable(); }
  }

  function renderBar() {
    const s = S;
    $("lnk-broker").className = "link " + (s.broker ? "is-on" : "is-off");
    const devOn = s.device === "online" && s.comm_ok;
    $("lnk-device").className = "link " + (devOn ? "is-on" : "is-off");
    $("lnk-device").lastChild.textContent = devOn ? "ESP32" : s.device === "unknown" ? "ESP32 not seen" : "ESP32 offline";

    const last = rows[rows.length - 1];
    const running = last ? last.run : false;
    const m = $("mode");
    m.textContent = (MODE_TEXT[s.mode] || (() => s.mode))(running);
    m.className = "mode" + (s.mode === "ADAPTING" ? " is-warn" : ["ALARM", "TRIPPED", "E-STOP"].includes(s.mode) ? " is-bad" : "");

    const c = s.control, t = Date.now();
    if (t > holdUntil.speed && document.activeElement !== $("speed")) {
      $("speed").value = c.target; $("speed-out").textContent = `${Math.round(c.target)}%`;
    }
    if (t > holdUntil.auto_adapt) $("sw-adapt").checked = c.auto_adapt;
    if (t > holdUntil.load_aware) $("sw-load").checked = c.load_aware;
    $("btn-start").disabled = s.mode === "E-STOP";

    if (s.mode === "E-STOP") setNotice("Emergency stop is latched on the machine. Release it with the START button on the conveyor.");
    else if (!s.comm_ok && s.last_rx) setNotice(`No data from the ESP32 for ${Math.round(s.now - s.last_rx)} s. Check its power and WiFi connection.`);
    else if (!s.broker) setNotice("The dashboard can't reach the MQTT broker. Start it with ./start_broker.sh.");
    else setNotice(null);
  }

  function setNotice(text) {
    const n = $("notice");
    n.hidden = !text;
    if (text) n.textContent = text;
  }

  function renderAlarm() {
    const a = S.alarm, el = $("alarm");
    el.hidden = !a.active;
    if (!a.active) { siren(false); return; }
    const loud = !a.silenced;
    el.classList.toggle("is-loud", loud);
    $("alarm-title").textContent = loud ? "Alarm: attention required" : "Alarm acknowledged";
    let reason = a.reason.replace(/\.?$/, ".") + (loud ? "" : " Waiting for the fault to clear.");
    if (loud && !audio.unlocked) reason += " Click anywhere on the page to turn on the alarm sound.";
    $("alarm-reason").textContent = reason;
    $("btn-ack").hidden = !loud;
    siren(loud);
  }

  function renderMimic(last) {
    const mimic = $("mimic");
    mimic.classList.toggle("has-data", !!last);
    if (!last) return;
    const L = S.limits;
    const exp = expectedCurrent(L, last.load_g);

    $("v-current").textContent = fmt(last.current_ma);
    $("s-current").textContent = last.run ? `Expected ${fmt(exp)} mA at this load` : "Motor idle";
    $("v-speed").textContent = fmt(last.cmd_speed);
    $("s-speed").textContent = last.run ? `Step delay ${fmt(last.step_delay_us)} µs` : last.estop ? "Emergency stop" : "Belt stopped";
    $("v-power").textContent = fmt((last.power_mw || 0) / 1000, 2);
    $("s-power").textContent = `${fmt(last.bus_v, 2)} V supply`;
    $("v-temp").textContent = fmt(last.temp_c, 1);
    $("s-temp").textContent = `Warning at ${fmt(L.temp_warn)} °C`;
    $("v-load").textContent = fmt(last.load_g);
    $("s-load").textContent = `Warning at ${fmt(L.load_warn)} g`;
    $("v-vib").textContent = fmt(last.vib_rms_g, 3);
    $("s-vib").textContent = `Limit ${fmt(vibLimit(L, last.load_g), 3)} g at this load`;

    // part highlighting
    const sev = {};
    for (const f of S.faults) {
      const p = FAULT_PART[f.code];
      if (p) sev[p] = Math.max(sev[p] || 0, f.severity);
    }
    for (const [part, ids] of Object.entries(PARTS)) {
      for (const id of ids) {
        const el = $(id);
        if (!el) continue;
        el.classList.toggle("is-warn", sev[part] === 1);
        el.classList.toggle("is-bad", sev[part] >= 2);
      }
    }
    mimic.classList.toggle("is-estop", !!last.estop);

    // belt motion follows the commanded speed
    const moving = last.run && !last.estop;
    const dur = (1.3 - 0.9 * Math.min(Math.max(last.cmd_speed || 0, 0), 100) / 100).toFixed(2) + "s";
    const root = document.documentElement.style;
    if (root.getPropertyValue("--belt-dur") !== dur) root.setProperty("--belt-dur", dur);
    root.setProperty("--belt-play", moving ? "running" : "paused");
  }

  function renderFaults() {
    const ul = $("faults");
    if (!S.faults.length) {
      const msg = rows.length ? "No faults. All readings are inside their limits." : "No data yet.";
      ul.innerHTML = `<li class="none"><span class="ok">${msg}</span></li>`;
      return;
    }
    ul.innerHTML = S.faults.map((f) =>
      `<li><span class="pri ${f.severity >= 2 ? "pri--crit" : "pri--warn"}" title="${f.severity >= 2 ? "Critical" : "Warning"}"></span>` +
      `<b>${esc(f.name)} <span class="sr">(${f.severity >= 2 ? "critical" : "warning"})</span></b><p>${esc(f.message)}</p></li>`).join("");
  }

  function renderEvents() {
    const ol = $("events");
    const html = S.events.map((e) => `<li><time>${esc(e.time)}</time><span class="k-${e.kind}">${esc(e.text)}</span></li>`).join("");
    if (ol.dataset.h !== html) { ol.innerHTML = html || "<li><span></span><span>Nothing yet.</span></li>"; ol.dataset.h = html; }
  }

  // ------------------------------------------------------------------ trends
  const CHARTS = [
    { id: "load", label: "Load on belt", unit: "g", key: "load_g", d: 0,
      lines: (L) => [["c-warn", L.load_warn], ["c-crit", L.load_crit]] },
    { id: "current", label: "Motor current", unit: "mA", key: "current_ma", d: 0, band: true,
      note: "Shaded band is normal for the load" },
    { id: "vib", label: "Frame vibration", unit: "g", key: "vib_rms_g", d: 3, limit: true,
      note: "Dashed line is the limit for the load" },
    { id: "temp", label: "Motor temperature", unit: "°C", key: "temp_c", d: 1,
      lines: (L) => [["c-warn", L.temp_warn], ["c-crit", L.temp_crit]] },
    { id: "speed", label: "Belt speed", unit: "%", key: "cmd_speed", d: 0, fixed: [0, 100],
      note: "Solid is commanded, dashed is what the ESP32 reports" },
  ];

  function buildCharts() {
    const wrap = $("charts");
    wrap.innerHTML = CHARTS.map((c, i) => `
      <div class="chart${i === CHARTS.length - 1 ? " chart--last" : ""}" data-id="${c.id}">
        <div><span class="chart__k">${c.label}</span><span class="chart__v" id="cv-${c.id}">–</span>
          <span class="chart__n" id="cn-${c.id}">${c.note || ""}</span></div>
        <svg id="cs-${c.id}" preserveAspectRatio="none" aria-hidden="true"></svg>
      </div>`).join("");
    const area = (ev) => {
      const svg = ev.target.closest("svg");
      if (!svg) return null;
      const r = svg.getBoundingClientRect();
      return { x: ev.clientX - r.left, w: r.width - AXIS_W };
    };
    wrap.addEventListener("pointermove", (ev) => {
      const a = area(ev);
      if (!a || a.x < 0 || a.x > a.w) { hoverT = null; renderCharts(); return; }
      const t1 = nowS();
      hoverT = t1 - windowS + (a.x / a.w) * windowS;
      renderCharts();
    });
    wrap.addEventListener("pointerleave", () => { hoverT = null; renderCharts(); });
  }

  const AXIS_W = 44;

  function el(tag, attrs, parent) {
    const n = document.createElementNS(SVGNS, tag);
    for (const k in attrs) n.setAttribute(k, attrs[k]);
    parent.appendChild(n);
    return n;
  }

  function renderCharts() {
    if (!S) return;
    const L = S.limits;
    const t1 = nowS(), t0 = t1 - windowS;
    const win = rows.filter((r) => r.t >= t0 - 1);
    // fault spans
    const spans = [];
    let start = null;
    win.forEach((r, i) => {
      const f = r.fault && r.fault !== "NORMAL";
      if (f && start === null) start = r.t;
      if ((!f || i === win.length - 1) && start !== null) { spans.push([start, r.t]); start = null; }
    });
    let hoverRow = null;
    if (hoverT !== null && win.length) {
      hoverRow = win.reduce((b, r) => (Math.abs(r.t - hoverT) < Math.abs(b.t - hoverT) ? r : b), win[0]);
    }

    for (const c of CHARTS) {
      const svg = $(`cs-${c.id}`);
      if (!svg) continue;
      const W = svg.clientWidth, H = svg.clientHeight;
      if (!W || !H) continue;
      const pw = W - AXIS_W;
      const isLast = c.id === "speed";
      const ph = H - (isLast ? 18 : 4);
      svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
      svg.textContent = "";

      // y domain
      const vals = win.map((r) => r[c.key]).filter((v) => v !== null && v !== undefined);
      let lo, hi;
      if (c.fixed) [lo, hi] = c.fixed;
      else {
        lo = Math.min(...vals); hi = Math.max(...vals);
        if (c.band) for (const r of win) { const e = expectedCurrent(L, r.load_g), tl = currentTol(L, e); lo = Math.min(lo, e - tl); hi = Math.max(hi, e + tl); }
        if (c.limit) for (const r of win) hi = Math.max(hi, vibLimit(L, r.load_g));
        if (c.lines) for (const [, y] of c.lines(L)) if (vals.length && hi > 0.6 * y) hi = Math.max(hi, y);
        if (!vals.length || !Number.isFinite(lo)) { lo = 0; hi = 1; }
        const pad = (hi - lo) * 0.12 || Math.abs(hi) * 0.1 || 1;
        lo -= pad; hi += pad;
        if (c.key !== "temp_c" && lo < 0) lo = 0;
      }
      const X = (t) => ((t - t0) / windowS) * pw;
      const Y = (v) => ph - ((v - lo) / (hi - lo)) * (ph - 4) - 2;

      // grid + axis labels
      for (const f of [0, 0.5, 1]) {
        const v = lo + (hi - lo) * f, y = Y(v);
        el("line", { x1: 0, x2: pw, y1: y, y2: y, class: "c-grid" }, svg);
        const tx = el("text", { x: W, y: y + 4, "text-anchor": "end", class: "c-axis" }, svg);
        tx.textContent = fmt(v, c.d > 1 ? 2 : c.d === 1 ? 0 : 0);
      }
      if (isLast) {
        for (let k = 0; k <= 4; k++) {
          const t = t0 + (windowS * k) / 4;
          const tx = el("text", { x: X(t), y: H - 2, "text-anchor": k === 0 ? "start" : k === 4 ? "end" : "middle", class: "c-axis" }, svg);
          tx.textContent = k === 4 ? "now" : clock(t).slice(0, 8);
        }
      }
      for (const [a, b] of spans) el("rect", { x: X(Math.max(a, t0)), y: 0, width: Math.max(2, X(b) - X(Math.max(a, t0))), height: ph, class: "c-fault" }, svg);

      if (c.band && win.length) {
        const up = win.map((r) => { const e = expectedCurrent(L, r.load_g); return `${X(r.t)},${Y(e + currentTol(L, e))}`; });
        const dn = win.slice().reverse().map((r) => { const e = expectedCurrent(L, r.load_g); return `${X(r.t)},${Y(Math.max(lo, e - currentTol(L, e)))}`; });
        el("polygon", { points: up.concat(dn).join(" "), class: "c-band" }, svg);
      }
      if (c.lines) for (const [cls, y] of c.lines(L)) if (y > lo && y < hi) el("line", { x1: 0, x2: pw, y1: Y(y), y2: Y(y), class: cls }, svg);
      if (c.limit && win.length) el("path", { d: pathOf(win, X, (r) => Y(vibLimit(L, r.load_g))), class: "c-warn", fill: "none" }, svg);

      if (c.id === "speed") {
        el("path", { d: pathOf(win, X, (r) => (r.run ? Y(r.speed_pct) : null), true), class: "c-line c-line--app" }, svg);
        el("path", { d: pathOf(win, X, (r) => Y(r.cmd_speed), true), class: "c-line c-line--cmd" }, svg);
      } else {
        el("path", { d: pathOf(win, X, (r) => (r[c.key] === null || r[c.key] === undefined ? null : Y(r[c.key]))), class: "c-line" }, svg);
      }

      const show = hoverRow || win[win.length - 1];
      if (hoverRow) {
        el("line", { x1: X(hoverRow.t), x2: X(hoverRow.t), y1: 0, y2: ph, class: "c-hover" }, svg);
        const v = hoverRow[c.key];
        if (v !== null && v !== undefined) el("circle", { cx: X(hoverRow.t), cy: Y(v), r: 3, class: "c-dot" }, svg);
      }
      $(`cv-${c.id}`).innerHTML = show ? `${fmt(show[c.key], c.d)}<small>${c.unit}</small>` : "–";
      $(`cn-${c.id}`).textContent = hoverRow ? `at ${clock(hoverRow.t)}` : c.note || "";
    }
  }

  // polyline with gaps where samples are missing (> 2 s apart)
  function pathOf(list, X, Yf, step = false) {
    let d = "", prev = null, prevY = null;
    for (const r of list) {
      const y = Yf(r);
      if (y === null || !Number.isFinite(y)) { prev = null; continue; }
      const x = X(r.t).toFixed(1);
      if (prev === null || r.t - prev > 2) d += `M${x},${y.toFixed(1)}`;
      else d += step ? `H${x}V${y.toFixed(1)}` : `L${x},${y.toFixed(1)}`;
      prev = r.t; prevY = y;
    }
    return d || "M0,0";
  }

  // ------------------------------------------------------------------ table
  function renderTable() {
    const onlyFaults = $("sw-faults-only").checked;
    const L = S.limits;
    const list = [];
    for (let i = rows.length - 1; i >= 0 && list.length < 120; i--) {
      const r = rows[i];
      if (onlyFaults && (!r.fault || r.fault === "NORMAL")) continue;
      list.push(r);
    }
    if (!list.length) {
      $("rows").innerHTML = `<tr><td class="empty" colspan="11">${onlyFaults ? "No faulty samples in the last 10 minutes." : "No samples yet."}</td></tr>`;
      return;
    }
    $("rows").innerHTML = list.map((r) => {
      const faulty = r.fault && r.fault !== "NORMAL";
      const bad = ["ALARM", "TRIPPED", "E-STOP"].includes(r.mode);
      const cls = faulty ? (bad ? "f-bad" : "f-warn") : "";
      const belt = r.estop ? "E-stop" : r.run ? "Running" : "Stopped";
      const fault = faulty ? r.fault.split(",").map((c) => FAULT_LABEL[c] || c).join(", ") : "";
      return `<tr class="${cls}"><td>${clock(r.t)}</td><td>${fmt(r.load_g)}</td><td>${fmt(r.current_ma)}</td>` +
        `<td>${fmt(expectedCurrent(L, r.load_g))}</td><td>${fmt((r.power_mw || 0) / 1000, 2)}</td><td>${fmt(r.vib_rms_g, 3)}</td>` +
        `<td>${fmt(r.temp_c, 1)}</td><td>${fmt(r.cmd_speed)}</td><td>${r.run ? fmt(r.step_delay_us) : "–"}</td>` +
        `<td>${belt}</td><td class="f">${esc(fault)}</td></tr>`;
    }).join("");
  }

  function esc(s) {
    return String(s ?? "").replace(/[&<>"]/g, (ch) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[ch]));
  }

  // ------------------------------------------------------------------ alarm sound (Web Audio, no files)
  const audio = { ctx: null, unlocked: false, osc: null, gain: null, timer: null };
  function unlockAudio() {
    if (audio.unlocked) return;
    try {
      audio.ctx = new (window.AudioContext || window.webkitAudioContext)();
      audio.ctx.resume();
      audio.unlocked = true;
      if (S) renderAlarm();
    } catch (_) { /* no audio available */ }
  }
  function siren(on) {
    if (!audio.unlocked) return;
    if (on && !audio.osc) {
      const ctx = audio.ctx;
      audio.gain = ctx.createGain(); audio.gain.gain.value = 0.18; audio.gain.connect(ctx.destination);
      audio.osc = ctx.createOscillator(); audio.osc.type = "square"; audio.osc.connect(audio.gain); audio.osc.start();
      let hiTone = false;
      const tick = () => { hiTone = !hiTone; audio.osc.frequency.setTargetAtTime(hiTone ? 960 : 640, ctx.currentTime, 0.05); };
      tick();
      audio.timer = setInterval(tick, 450);
    } else if (!on && audio.osc) {
      clearInterval(audio.timer);
      audio.osc.stop(); audio.osc.disconnect(); audio.gain.disconnect();
      audio.osc = audio.gain = null;
    }
  }
  ["pointerdown", "keydown"].forEach((ev) => window.addEventListener(ev, unlockAudio, { passive: true }));

  // ------------------------------------------------------------------ controls
  $("btn-start").addEventListener("click", () => send("start"));
  $("btn-stop").addEventListener("click", () => send("stop"));
  $("btn-tare").addEventListener("click", () => send("tare"));
  $("btn-ack").addEventListener("click", () => send("ack"));
  $("speed").addEventListener("input", (e) => { $("speed-out").textContent = `${e.target.value}%`; holdUntil.speed = Date.now() + 1500; });
  $("speed").addEventListener("change", (e) => { send("speed", Number(e.target.value)); holdUntil.speed = Date.now() + 1500; });
  $("sw-adapt").addEventListener("change", (e) => { send("auto_adapt", e.target.checked); holdUntil.auto_adapt = Date.now() + 1500; });
  $("sw-load").addEventListener("change", (e) => { send("load_aware", e.target.checked); holdUntil.load_aware = Date.now() + 1500; });
  $("sw-faults-only").addEventListener("change", () => S && renderTable());
  $("window").addEventListener("click", (e) => {
    const b = e.target.closest("button");
    if (!b) return;
    windowS = Number(b.dataset.s);
    for (const x of $("window").children) x.setAttribute("aria-checked", x === b ? "true" : "false");
    renderCharts();
  });
  window.addEventListener("resize", () => renderCharts());

  buildCharts();
  connect();
})();
