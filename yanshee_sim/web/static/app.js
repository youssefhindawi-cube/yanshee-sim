/* Yanshee relay GUI (ARCHITECTURE.md section 5). Plain JS, no build step, no network beyond this server. */
"use strict";
(() => {
  // ------------------------------------------------------------------ constants
  const SERVO_GROUPS = [
    { id: "head", label: "Head", names: ["NeckLR"] },
    { id: "larm", label: "Left arm", names: ["LeftShoulderRoll", "LeftShoulderFlex", "LeftElbowFlex"] },
    { id: "rarm", label: "Right arm", names: ["RightShoulderRoll", "RightShoulderFlex", "RightElbowFlex"] },
    { id: "lleg", label: "Left leg", names: ["LeftHipLR", "LeftHipFB", "LeftKneeFlex", "LeftAnkleFB", "LeftAnkleUD"] },
    { id: "rleg", label: "Right leg", names: ["RightHipLR", "RightHipFB", "RightKneeFlex", "RightAnkleFB", "RightAnkleUD"] },
  ];
  const NAMES = SERVO_GROUPS.flatMap((g) => g.names);
  // fallback until the first state arrives (yanshee_sim/servos.py)
  const DEFAULT_LIMITS = {
    RightShoulderRoll: [0, 180], RightShoulderFlex: [0, 180], RightElbowFlex: [0, 180],
    LeftShoulderRoll: [0, 180], LeftShoulderFlex: [0, 180], LeftElbowFlex: [0, 180],
    RightHipLR: [0, 120], RightHipFB: [10, 180], RightKneeFlex: [0, 180], RightAnkleFB: [0, 180], RightAnkleUD: [65, 180],
    LeftHipLR: [60, 180], LeftHipFB: [0, 170], LeftKneeFlex: [0, 180], LeftAnkleFB: [0, 180], LeftAnkleUD: [0, 115],
    NeckLR: [15, 165],
  };
  const LED_COLORS = { button: ["white", "red", "green", "blue", "yellow", "purple", "cyan"], camera: ["red", "green", "blue"] };
  const LED_MODES = { button: ["on", "off", "blink", "breath"], camera: ["on", "off", "blink"] };
  const CSS_COLOR = { white: "#f2f4f7", red: "#ff3b30", green: "#2fd158", blue: "#2f7bff", yellow: "#ffd60a",
    purple: "#b45cff", cyan: "#32d6e6", orange: "#ff9f0a", black: "#111", pink: "#ff6fb1", gray: "#8e8e93", grey: "#8e8e93" };
  const CATEGORIES = ["gestures", "locomotion", "fight", "football", "get-up", "dances", "v2 layered", "user"];
  const CAMS = [{ id: "front", label: "Front" }, { id: "side", label: "Side" }, { id: "free", label: "Orbit" },
    { id: "head", label: "Head cam", title: "First-person view from the head camera (the robot's camera in Robot mode)" }];
  const VISION = [
    { task: "face:recognition", label: "Who is it?" }, { task: "face:quantity", label: "Count faces" },
    { task: "face:age", label: "Age" }, { task: "face:age_group", label: "Age group" },
    { task: "face:gender", label: "Gender" }, { task: "face:expression", label: "Expression" },
    { task: "face:mask", label: "Mask?" }, { task: "face:glass", label: "Glasses?" },
    { task: "gesture", label: "Hand gesture" }, { task: "color", label: "Colours" },
    { task: "object", label: "Objects" }, { task: "qr", label: "QR code" },
  ];
  const PHRASES = ["Hello! I am Yanshee.", "Welcome to our stand!", "Watch me dance!", "Thank you, goodbye!"];
  // hold-to-walk directions: [speed_v sign (+ forward), speed_h sign (+ robot's left)]
  const DIRS = { f: [1, 0], b: [-1, 0], l: [0, 1], r: [0, -1], fl: [1, 1], fr: [1, -1], bl: [-1, 1], br: [-1, -1] };
  const KEY_DIRS = { ArrowUp: "f", KeyW: "f", ArrowDown: "b", KeyS: "b", ArrowLeft: "l", KeyA: "l", ArrowRight: "r", KeyD: "r" };
  const TABS = ["motion", "servos", "settings", "voice", "log"];
  const NAME_RE = /^[A-Za-z0-9_-]+$/;
  // routine warnings that go to the log without a toast (they repeat on every slider move)
  const QUIET_EVENTS = /^runtime .*(stretched|clamped)|: clamped$|manual control takes over/;
  const PREVIEW_NAME = "_gui_preview"; // scratch motion file used to play unsaved recorder content

  // ------------------------------------------------------------------ helpers
  const $ = (sel, root = document) => root.querySelector(sel);
  const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));
  function h(tag, attrs, ...children) {
    const el = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs || {})) {
      if (v == null || v === false) continue;
      if (k === "class") el.className = v;
      else if (k === "text") el.textContent = v;
      else if (k === "style") el.style.cssText = v;
      else if (k.startsWith("on") && typeof v === "function") el.addEventListener(k.slice(2), v);
      else if (k in el && typeof v !== "string") el[k] = v;
      else el.setAttribute(k, v === true ? "" : v);
    }
    for (const c of children.flat()) if (c != null && c !== false) el.append(c instanceof Node ? c : String(c));
    return el;
  }
  const store = {
    get(k, d) { try { const v = localStorage.getItem("yanshee." + k); return v == null ? d : JSON.parse(v); } catch (e) { return d; } },
    set(k, v) { try { localStorage.setItem("yanshee." + k, JSON.stringify(v)); } catch (e) { /* private mode */ } },
  };
  const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));
  const fmt = (x, d = 1) => (typeof x === "number" && isFinite(x) ? x.toFixed(d) : "--");
  const pretty = (s) => String(s).replace(/_/g, " ");
  const servoLabel = (n) => n.replace(/^(Left|Right)/, "").replace(/([a-z])([A-Z])/g, "$1 $2")
    .replace(/\bL R\b/, "L/R").replace(/\bF B\b/, "F/B").replace(/\bU D\b/, "U/D");
  const clock = (t) => { const d = t ? new Date(t * 1000) : new Date(); return d.toTimeString().slice(0, 8); };
  const listFrom = (res, ...keys) => {
    if (Array.isArray(res)) return res;
    if (!res || typeof res !== "object") return [];
    for (const k of keys) if (Array.isArray(res[k])) return res[k];
    for (const v of Object.values(res)) if (Array.isArray(v)) return v;
    return [];
  };
  function throttle(fn, ms) {
    let last = 0, timer = null;
    return (...args) => {
      const wait = last + ms - Date.now();
      clearTimeout(timer);
      if (wait <= 0) { last = Date.now(); fn(...args); } else timer = setTimeout(() => { last = Date.now(); fn(...args); }, wait);
    };
  }

  // ------------------------------------------------------------------ toasts
  const toastSeen = new Map();
  function toast(text, level = "info", title) {
    if (!text) return;
    text = String(text);
    const now = Date.now();
    if (now - (toastSeen.get(text) || 0) < 4000) return; // same message already on screen
    toastSeen.set(text, now);
    const box = $("#toasts");
    const head = title || { error: "Error", warn: "Warning", ok: "Done", info: "" }[level];
    const el = h("div", { class: "toast " + level, role: level === "error" ? "alert" : "status" }, head ? h("b", { text: head }) : null, text);
    const close = () => { el.classList.add("out"); setTimeout(() => el.remove(), 220); };
    el.addEventListener("click", close);
    box.prepend(el);
    while (box.children.length > 5) box.lastChild.remove();
    setTimeout(close, { error: 8000, warn: 6000 }[level] || 3500);
  }
  function failTitle(err) {
    return /refus|collision|battery|limit|not available|cancel/i.test(err || "") ? "Refused" : "Error";
  }
  function report(res, okText) {
    if (!res) return res;
    if (res.ok === false) toast(res.error || "Request failed", "error", failTitle(res.error));
    else if (okText) toast(okText, "ok");
    const warns = Array.isArray(res.warnings) ? res.warnings : [];
    warns.slice(0, 3).forEach((w) => toast(w, "warn"));
    return res;
  }

  // ------------------------------------------------------------------ HTTP
  async function api(method, path, body, opts = {}) {
    const { quiet = false, timeout = 20000, okText } = opts;
    const init = { method, headers: {}, keepalive: !!opts.keepalive };
    if (body !== undefined) { init.headers["Content-Type"] = "application/json"; init.body = JSON.stringify(body); }
    const ctl = new AbortController();
    const tm = setTimeout(() => ctl.abort(), timeout);
    if (!opts.keepalive) init.signal = ctl.signal;
    let res;
    try {
      const r = await fetch(path, init);
      const ct = r.headers.get("content-type") || "";
      if (ct.includes("json")) {
        res = await r.json();
        if (Array.isArray(res)) res = { ok: r.ok, items: res };
      } else {
        const t = await r.text();
        res = { ok: r.ok, text: t };
        if (!r.ok) res.error = `HTTP ${r.status}: ${t.slice(0, 200)}`;
      }
      if (res && typeof res === "object" && res.ok === undefined) res.ok = r.ok;
      res.status = r.status;
      if (!r.ok && res.ok !== false) res.ok = false;
      if (res.ok === false && !res.error) {
        const d = res.detail;
        res.error = Array.isArray(d) ? d.map((x) => `${(x.loc || []).slice(1).join(".")}: ${x.msg}`).join("; ")
          : (d || `HTTP ${r.status}`);
      }
    } catch (e) {
      res = { ok: false, error: e.name === "AbortError" ? `${path}: no answer from the server after ${timeout / 1000} s`
        : `server unreachable (${e.message})` };
    } finally { clearTimeout(tm); }
    if (!quiet) report(res, okText);
    return res;
  }
  async function busy(btn, fn) {
    if (btn) { if (btn.classList.contains("busy")) return; btn.classList.add("busy"); btn.setAttribute("aria-busy", "true"); }
    try { return await fn(); } finally { if (btn) { btn.classList.remove("busy"); btn.removeAttribute("aria-busy"); } }
  }
  function flash(el) { if (!el) return; el.classList.remove("fired"); void el.offsetWidth; el.classList.add("fired"); }

  // ------------------------------------------------------------------ confirm dialog
  $("#confirmStop").addEventListener("click", () => { doStop(); $("#confirmDlg").close("cancel"); });
  function confirmBox(title, body, okLabel = "OK") {
    const dlg = $("#confirmDlg");
    if (!dlg || typeof dlg.showModal !== "function") return Promise.resolve(window.confirm(title + "\n\n" + body));
    $("#confirmTitle").textContent = title;
    $("#confirmBody").textContent = body;
    $("#confirmOk").textContent = okLabel;
    return new Promise((resolve) => {
      dlg.addEventListener("close", () => resolve(dlg.returnValue === "ok"), { once: true });
      dlg.returnValue = "cancel";
      dlg.showModal();
      $("#confirmCancel").focus();
    });
  }

  // ------------------------------------------------------------------ app state
  let S = null; // latest state snapshot
  let limits = { ...DEFAULT_LIMITS };
  const ui = { tab: "motion", cam: store.get("cam", "front"), connectedKey: "", modeKey: "" };
  const robotConnected = () => !!(S && S.robot && S.robot.connected);
  const mode = () => (S ? S.mode : "sim");

  // ------------------------------------------------------------------ tabs (#hash routing)
  function showTab(id, push) {
    if (!TABS.includes(id)) id = "motion";
    ui.tab = id;
    $$(".tabpanel").forEach((p) => { p.hidden = p.dataset.panel !== id; });
    $$("#tabs a").forEach((a) => a.setAttribute("aria-selected", String(a.dataset.tab === id)));
    if (push && location.hash !== "#" + id) history.replaceState(null, "", "#" + id);
    if (id !== "motion") walkRelease();
    if (id === "log") { logUnseen = 0; renderLogBadge(); renderLog(); }
    if (id === "settings") { loadInfo(); sensorsTick(); }
    if (id === "servos") { loadMotions(); renderServoReadback(true); }
    if (id === "settings") requestAnimationFrame(drawChart);
  }
  $$("#tabs a").forEach((a) => a.addEventListener("click", (e) => { e.preventDefault(); showTab(a.dataset.tab, true); }));
  window.addEventListener("hashchange", () => showTab(location.hash.slice(1)));

  // ------------------------------------------------------------------ top bar
  $$("#modeSeg button").forEach((b) => b.addEventListener("click", () => setMode(b.dataset.mode, b)));
  async function setMode(m, btn) {
    if (m === mode()) return;
    if (m !== "sim") {
      if (!robotConnected()) { toast("Connect a robot first (IP field at the top), then switch to " + m + ".", "warn", "No robot"); $("#ipInput").focus(); return; }
      const what = m === "robot" ? "Commands will move the REAL robot (the simulator shows its readback)."
        : "Commands will move the simulator AND the REAL robot.";
      const ok = await confirmBox(`Switch to ${m.toUpperCase()} mode?`,
        `${what} Make sure the robot stands on a flat surface with room around it, and that the emergency stop (STOP button / Esc, or the robot's power switch) is within reach. Switching stops everything first.`,
        `Switch to ${m}`);
      if (!ok) return;
    }
    walkRelease();
    await busy(btn, () => api("POST", "/api/mode", { mode: m }, { okText: `Mode: ${m}` }));
    refreshState();
  }
  $("#connectForm").addEventListener("submit", (e) => {
    e.preventDefault();
    const ip = $("#ipInput").value.trim();
    if (!ip) { toast("Type the robot's IP address (shown in the Yanshee app), e.g. 192.168.3.13", "warn"); return; }
    if (!/^[A-Za-z0-9.\-]+(:\d{1,5})?$/.test(ip)) { toast(`"${ip}" does not look like an IP address (or IP:port)`, "warn"); return; }
    store.set("ip", ip);
    busy($("#connectBtn"), async () => {
      const r = await api("POST", "/api/connect", { ip }, { timeout: 30000 });
      if (r.ok) {
        toast(`Connected to ${r.ip || ip}` + (r.battery != null ? `, battery ${r.battery}%` : ""), "ok");
        refreshState(); loadActions(); loadMusic();
      }
    });
  });
  $("#disconnectBtn").addEventListener("click", (e) => busy(e.currentTarget, async () => {
    walkRelease();
    const r = await api("POST", "/api/disconnect", undefined, { okText: "Robot disconnected" });
    if (r.ok) { refreshState(); loadActions(); loadMusic(); }
  }));
  $("#ipInput").value = store.get("ip", "");

  // STOP: fire on pointerdown for the fastest reaction; ignore the click that follows
  let stopDownAt = 0;
  const stopBtn = $("#stopBtn");
  stopBtn.addEventListener("pointerdown", (e) => { if (e.button === 0) { stopDownAt = Date.now(); doStop(); } });
  stopBtn.addEventListener("click", () => { if (Date.now() - stopDownAt > 800) doStop(); });
  function doStop() {
    stopBtn.classList.add("flash"); setTimeout(() => stopBtn.classList.remove("flash"), 200);
    walk.held.clear(); walk.keys.clear(); walkPaint();
    walk.active = false; walk.lastKey = ""; walkKeepalive(false);
    try { if (window.speechSynthesis) speechSynthesis.cancel(); } catch (e) { /* ignore */ }
    api("POST", "/api/stop", undefined, { timeout: 10000 }).then((r) => { if (r.ok) { walk.sent = false; toast("Stopped", "ok"); } });
  }
  $("#resetBtn").addEventListener("click", (e) => busy(e.currentTarget, async () => {
    walkRelease();
    await api("POST", "/api/reset");
  }));

  function renderTop(s) {
    document.body.dataset.mode = s.mode;
    $$("#modeSeg button").forEach((b) => {
      b.setAttribute("aria-checked", String(b.dataset.mode === s.mode));
      b.classList.toggle("na", b.dataset.mode !== "sim" && !s.robot.connected);
    });
    const r = s.robot || {};
    const dot = $("#robotDot");
    dot.className = "dot" + (r.connected ? (r.online === false ? " off" : " on") : "");
    dot.title = r.connected ? (r.online === false ? "robot not answering" : "robot online") : "no robot";
    $("#connectBtn").hidden = !!r.connected;
    $("#disconnectBtn").hidden = !r.connected;
    const ipIn = $("#ipInput");
    ipIn.readOnly = !!r.connected;
    if (r.connected && r.ip && document.activeElement !== ipIn) ipIn.value = r.ip;
    battery($("#simBatt"), s.sim ? s.sim.battery : null, false);
    battery($("#robotBatt"), r.connected ? r.battery : null, r.charging);
    $("#simBattBig").textContent = s.sim ? Math.round(s.sim.battery) + "%" : "--%";
    // activity
    const a = s.activity || { kind: "idle" };
    const act = $("#activity");
    const idle = !a.kind || a.kind === "idle";
    act.classList.toggle("busy", !idle);
    $("#actKind").textContent = idle ? "Idle" : a.kind;
    $("#actName").textContent = idle ? "" : [a.name, a.target ? `on ${a.target}` : "", a.detail ? `- ${a.detail}` : ""].filter(Boolean).join(" ");
    act.title = $("#actName").textContent;
    $("#actBar").style.width = idle ? "0" : Math.round(100 * clamp(a.progress || 0, 0, 1)) + "%";
    // pause / resume: robot built-ins only (the robot's motion_control pause)
    const pausable = a.kind === "action" && (a.target === "robot" || (a.target === "both" && /follows|paused/.test(a.detail || "")));
    const pb = $("#actPause");
    pb.hidden = !pausable;
    pb.textContent = a.detail === "paused" ? "Resume" : "Pause";
  }
  $("#actPause").addEventListener("click", (e) => busy(e.currentTarget, () =>
    api("POST", e.currentTarget.textContent === "Resume" ? "/api/action/resume" : "/api/action/pause")));
  function battery(el, pct, charging) {
    const has = typeof pct === "number" && isFinite(pct);
    el.classList.toggle("na", !has);
    el.classList.toggle("low", has && pct < 30);
    $("i", el).style.width = has ? clamp(pct, 0, 100) + "%" : "0";
    $("b", el).textContent = has ? Math.round(pct) + "%" + (charging ? " +" : "") : "--";
    if (has) el.title = `${el.id === "simBatt" ? "Simulated" : "Robot"} battery ${pct}%${charging ? " (charging)" : ""}${pct < 30 ? " - motions are refused below 30%" : ""}`;
  }

  // ------------------------------------------------------------------ live view
  const streamImg = $("#stream");
  let streamGen = 0, streamRetry = null;
  CAMS.forEach((c) => $("#camSeg").append(h("button", { type: "button", role: "radio", "data-cam": c.id,
    "aria-checked": String(c.id === ui.cam), title: c.title, onclick: () => setCam(c.id) }, c.label)));
  if (!CAMS.some((c) => c.id === ui.cam)) ui.cam = "front";
  function setCam(cam) {
    ui.cam = cam; store.set("cam", cam);
    $$("#camSeg button").forEach((b) => b.setAttribute("aria-checked", String(b.dataset.cam === cam)));
    startStream();
    renderSource();
  }
  function startStream() {
    clearTimeout(streamRetry);
    streamGen++;
    $("#vpPlaceholder").hidden = false;
    $("#vpPlaceholder").textContent = "Connecting to the video stream...";
    streamImg.removeAttribute("src");
    if (document.hidden) return; // freed while hidden (browsers allow few parallel connections)
    streamImg.src = `/api/stream/${encodeURIComponent(ui.cam)}.mjpg?g=${streamGen}`;
  }
  function stopStream() { clearTimeout(streamRetry); streamGen++; streamImg.removeAttribute("src"); }
  streamImg.addEventListener("load", () => { $("#vpPlaceholder").hidden = true; });
  streamImg.addEventListener("error", () => {
    if (!streamImg.getAttribute("src")) return;
    $("#vpPlaceholder").hidden = false;
    $("#vpPlaceholder").textContent = "No video from the server. Retrying...";
    clearTimeout(streamRetry);
    streamRetry = setTimeout(startStream, 2500);
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) { stopStream(); walkRelease(); } else startStream();
  });
  function renderSource() {
    const m = mode();
    let src = "Simulation";
    if (m === "robot") src = ui.cam === "head" ? "Robot camera" : "Digital twin (robot readback)";
    else if (m === "both") src = "Simulation + robot";
    $("#vpSource").textContent = src + " - " + (CAMS.find((c) => c.id === ui.cam) || {}).label;
  }
  function renderLive(s) {
    renderSource();
    const sim = s.sim || {};
    $("#vpFallen").hidden = !sim.fallen;
    const tilt = $("#tiltInfo");
    tilt.textContent = `tilt ${fmt(sim.tilt_deg)}°` + (sim.fixed_base ? " (fixed base)" : "");
    tilt.classList.toggle("bad", !!sim.fallen || sim.tilt_deg > 25);
    const rerr = $("#rendererError");
    rerr.hidden = !sim.renderer_error;
    if (sim.renderer_error) rerr.textContent = "Sim camera: " + sim.renderer_error;
    for (const [type, el] of [["button", $("#ledButton")], ["camera", $("#ledCamera")]]) {
      const led = (s.leds || {})[type] || {};
      el.style.setProperty("--led", CSS_COLOR[led.color] || "#888");
      el.className = "led-ind m-" + (led.mode || "on");
      el.title = `${type === "button" ? "Chest" : "Eye"} LED: ${led.color || "?"}, ${led.mode || "?"}`;
    }
    const r = s.robot || {};
    $("#robotInfo").textContent = r.connected
      ? `Robot ${r.ip || ""} ${r.online === false ? "(not answering!)" : "online"}` : "Robot: not connected";
  }

  // ------------------------------------------------------------------ walking (hold to walk)
  // active: a direction is held and a walk was asked for; sent: a /api/gait went out and no stop has been confirmed
  // since (a gait request that timed out on this side may still have started the robot walking on the server)
  const walk = { held: new Map(), keys: new Set(), active: false, sent: false, lastKey: "", chain: Promise.resolve(), timer: null };
  const WALK_KEEPALIVE_MS = 400; // the server stops an open-ended walk after 1.5 s without one (dead-man)
  function walkKeepalive(on) {
    clearInterval(walk.timer); walk.timer = null;
    if (on) walk.timer = setInterval(() => {
      if (!walk.active) { walkKeepalive(false); return; }
      api("POST", "/api/gait/keepalive", undefined, { quiet: true, timeout: 1500 });
    }, WALK_KEEPALIVE_MS);
  }
  function walkVector() {
    let v = 0, hh = 0;
    for (const d of [...walk.held.values(), ...walk.keys]) { v += DIRS[d][0]; hh += DIRS[d][1]; }
    return [Math.sign(v), Math.sign(hh)];
  }
  function walkPaint() {
    const on = new Set([...walk.held.values(), ...walk.keys]);
    $$("#dpad button[data-dir]").forEach((b) => b.classList.toggle("held", on.has(b.dataset.dir)));
  }
  function walkEnqueue(fn) { walk.chain = walk.chain.then(fn, fn).catch(() => {}); return walk.chain; }
  function walkUpdate() {
    walkPaint();
    const [sv, sh] = walkVector();
    if (!sv && !sh) {
      if (walk.active || walk.sent) walkSendStop();
      return;
    }
    const sp = +$("#walkSpeed").value || 3;
    const body = { speed_v: sv * sp, speed_h: sh * sp, steps: 0, period: +$("#walkPeriod").value || 1, wave: $("#walkWave").checked };
    const key = JSON.stringify(body);
    if (walk.active && key === walk.lastKey) return;
    walk.active = true; walk.lastKey = key;
    walkKeepalive(true);
    walkEnqueue(async () => {
      if (!walk.active || walk.lastKey !== key) return; // released or changed before this ran
      walk.sent = true;
      // the server may first wait for the previous activity (up to 6 s) and then the robot (its timeout)
      const r = await api("POST", "/api/gait", body, { timeout: 15000 });
      if (r.ok || walk.lastKey !== key) return;
      if (r.status) { walk.active = false; walk.lastKey = ""; walkKeepalive(false); } // refused: nothing walks
      else walkSendStop(); // no answer: the walk may have started anyway, so stop it now
    });
  }
  function walkSendStop() {
    walk.active = false; walk.lastKey = ""; walkKeepalive(false);
    const done = (r) => { if (r && r.ok && !walk.active) walk.sent = false; };
    api("POST", "/api/gait/stop", undefined, { quiet: true }).then(done); // right away, even if a gait call hangs
    walkEnqueue(() => api("POST", "/api/gait/stop", undefined, { timeout: 8000 }).then(done)); // and after any queued gait call
  }
  function walkRelease() {
    const was = walk.active || walk.sent || walk.held.size || walk.keys.size;
    walk.held.clear(); walk.keys.clear(); walkPaint();
    if (was) walkSendStop();
  }
  $$("#dpad button[data-dir]").forEach((b) => {
    const dir = b.dataset.dir;
    b.addEventListener("pointerdown", (e) => {
      if (e.pointerType === "mouse" && e.button !== 0) return;
      e.preventDefault();
      // drop implicit touch capture so sliding off the button counts as leaving it
      try { if (b.hasPointerCapture(e.pointerId)) b.releasePointerCapture(e.pointerId); } catch (err) { /* ignore */ }
      walk.held.set(e.pointerId, dir);
      walkUpdate();
    });
    const up = (e) => { if (walk.held.delete(e.pointerId)) walkUpdate(); };
    ["pointerup", "pointercancel", "pointerleave", "lostpointercapture"].forEach((t) => b.addEventListener(t, up));
    b.addEventListener("contextmenu", (e) => e.preventDefault());
    // keyboard users: Enter/Space on a focused d-pad button walks while held
    b.addEventListener("keydown", (e) => {
      if ((e.key === "Enter" || e.key === " ") && !e.repeat) { e.preventDefault(); walk.keys.add(dir); walkUpdate(); }
    });
    b.addEventListener("keyup", (e) => {
      if (e.key === "Enter" || e.key === " ") { walk.keys.delete(dir); walkUpdate(); }
    });
    b.addEventListener("blur", () => { if (walk.keys.delete(dir)) walkUpdate(); });
  });
  // safety net: a pointer released anywhere stops what that pointer was holding
  ["pointerup", "pointercancel"].forEach((t) => window.addEventListener(t, (e) => { if (walk.held.delete(e.pointerId)) walkUpdate(); }));
  window.addEventListener("blur", walkRelease);
  window.addEventListener("pagehide", () => {
    if (walk.active || walk.sent) api("POST", "/api/gait/stop", undefined, { quiet: true, keepalive: true });
  });
  $("#dpadStop").addEventListener("click", () => { walkRelease(); api("POST", "/api/gait/stop"); });
  $("#walkSpeed").addEventListener("input", (e) => {
    $("#walkSpeedOut").textContent = e.target.value; store.set("walkSpeed", +e.target.value);
    if (walk.active) walkUpdate(); // the server changes speed in place
  });
  $("#walkWave").checked = !!store.get("walkWave", false);
  $("#walkWave").addEventListener("change", (e) => { store.set("walkWave", e.target.checked); if (walk.active) walkUpdate(); });
  $("#walkSpeed").value = store.get("walkSpeed", 3);
  $("#walkSpeedOut").textContent = $("#walkSpeed").value;
  $$("[data-turn]").forEach((b) => b.addEventListener("click", () => {
    const a = actions.find((x) => x.name === "turn around");
    if (a && !availability(a, b.dataset.turn).ok) { toast(availability(a, b.dataset.turn).why, "info"); return; }
    flash(b);
    playAction({ name: "turn around", version: "v1" }, b.dataset.turn);
  }));

  // global keys: Esc = STOP anywhere; arrows/WASD walk on the Motion tab
  const typing = (el) => el && (el.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(el.tagName));
  window.addEventListener("keydown", (e) => {
    if (e.key === "Escape") { // STOP anywhere, also while a confirm dialog is open (it closes as Cancel)
      const dlg = $("#confirmDlg");
      if (dlg.open) { doStop(); dlg.close("cancel"); return; }
      e.preventDefault(); doStop(); return;
    }
    if (ui.tab !== "motion" || typing(document.activeElement) || e.ctrlKey || e.metaKey || e.altKey) return;
    const d = KEY_DIRS[e.code];
    if (!d) return;
    e.preventDefault();
    if (e.repeat || walk.keys.has(d)) return;
    walk.keys.add(d); walkUpdate();
  });
  window.addEventListener("keyup", (e) => {
    const d = KEY_DIRS[e.code];
    if (d && walk.keys.delete(d)) walkUpdate();
  });

  // ------------------------------------------------------------------ actions
  let actions = [];
  ui.cat = store.get("cat", "gestures");
  function availability(a, dir) {
    const m = mode();
    if (m === "sim") {
      const sims = a.sim_directions || [];
      const ok = dir ? sims.includes(dir) : !!a.sim;
      return { ok, why: ok ? "" : `"${pretty(a.label || a.name)}${dir ? " " + dir : ""}" is not available in simulation. Connect a robot and use Robot or Both mode.` };
    }
    if (!robotConnected()) return { ok: false, why: "No robot connected." };
    return { ok: a.robot !== false, why: a.robot === false ? "Not available on the robot." : "" };
  }
  function badges(a, compact) {
    const out = [];
    if (a.music && !compact) out.push(h("span", { class: "badge music" }, "music"));
    if (a.version === "v2") out.push(h("span", { class: "badge" }, "v2"));
    if (a.kind === "file") out.push(h("span", { class: "badge" }, a.source || "file"));
    else if (!a.sim) out.push(h("span", { class: "badge robot" }, "robot only"));
    else if (a.directions && a.sim_directions && a.sim_directions.length < a.directions.length) out.push(h("span", { class: "badge sim" }, "partly sim"));
    else out.push(h("span", { class: "badge sim" }, "sim"));
    return out;
  }
  async function playAction(a, dir, el) {
    const speed = $("#actSpeed").value;
    const repeat = clamp(parseInt($("#actRepeat").value, 10) || 1, 1, 100);
    flash(el);
    walkRelease();
    return api("POST", "/api/action", { name: a.name, direction: dir || "", speed, repeat, version: a.version || "v1" });
  }
  function actionTile(a, big) {
    const dirs = a.directions || [];
    if (!dirs.length) {
      const av = availability(a);
      const el = h("button", { type: "button", class: "tile" + (av.ok ? "" : " na"), title: av.why || `Play ${a.label || a.name}` },
        h("span", { class: "t-name" }, pretty(a.label || a.name)), h("span", { class: "t-badges" }, badges(a, big)));
      el.addEventListener("click", () => {
        const av2 = availability(a);
        if (!av2.ok) { toast(av2.why, "info"); return; }
        playAction(a, "", el);
      });
      return el;
    }
    const row = h("div", { class: "dirs" });
    for (const d of dirs) {
      const av = availability(a, d);
      const b = h("button", { type: "button", class: av.ok ? "" : "na", title: av.why || `${a.label || a.name} ${d}` }, d);
      b.addEventListener("click", () => {
        const av2 = availability(a, d);
        if (!av2.ok) { toast(av2.why, "info"); return; }
        playAction(a, d, b);
      });
      row.append(b);
    }
    const anyOk = dirs.some((d) => availability(a, d).ok);
    return h("div", { class: "tile" + (anyOk ? "" : " na") + (big ? " big" : "") },
      h("span", { class: "t-name" }, pretty(a.label || a.name)), row, h("span", { class: "t-badges" }, badges(a)));
  }
  async function loadActions() {
    const r = await api("GET", "/api/actions", undefined, { quiet: true });
    if (!r.ok) { $("#actionGrid").replaceChildren(h("p", { class: "empty" }, "Could not load actions: " + r.error)); return; }
    actions = listFrom(r, "actions").filter((a) => a && a.name && !String(a.name).startsWith("_"));
    renderActions();
  }
  function renderActions() {
    const q = $("#actSearch").value.trim().toLowerCase();
    const counts = {};
    actions.forEach((a) => { counts[a.category] = (counts[a.category] || 0) + 1; });
    const cats = CATEGORIES.filter((c) => counts[c]).concat(Object.keys(counts).filter((c) => !CATEGORIES.includes(c)));
    if (!cats.includes(ui.cat)) ui.cat = cats[0] || "gestures";
    $("#catChips").replaceChildren(...cats.map((c) => h("button", { type: "button", role: "tab", "aria-selected": String(!q && c === ui.cat),
      onclick: () => { ui.cat = c; store.set("cat", c); $("#actSearch").value = ""; renderActions(); } }, c, h("small", {}, counts[c]))));
    const list = q ? actions.filter((a) => (a.name + " " + (a.label || "")).toLowerCase().includes(q)) : actions.filter((a) => a.category === ui.cat);
    $("#actionGrid").replaceChildren(...(list.length ? list.map((a) => actionTile(a)) : [h("p", { class: "empty" }, q ? "No action matches." : "No actions in this category.")]));
    const fallback = actions.some((a) => a.source === "fallback");
    $("#actionNote").textContent = mode() === "sim"
      ? "Sim mode: dimmed actions only exist on the real robot." + (fallback ? " The robot list shown is the captured one; connect a robot to load its own." : "")
      : mode() === "both" ? "Both mode: robot-only actions play on the robot and the simulator follows its readback." : "Robot mode: actions play on the real robot.";
    // dances + get-up live on the same tab
    const dances = actions.filter((a) => a.category === "dances");
    $("#danceGrid").replaceChildren(...(dances.length ? dances.map((a) => actionTile(a, true)) : [h("p", { class: "empty" }, "No dances listed.")]));
    const getups = actions.filter((a) => a.category === "get-up");
    const resetB = h("button", { type: "button", class: "btn primary", title: "Stand up / reset pose" }, "Stand up (reset)");
    resetB.addEventListener("click", () => busy(resetB, () => api("POST", "/api/reset")));
    $("#getupRow").replaceChildren(resetB, ...getups.map((a) => {
      const av = availability(a);
      const b = h("button", { type: "button", class: "btn" + (av.ok ? "" : " na"), title: av.why || "" }, pretty(a.label || a.name));
      b.addEventListener("click", () => { const av2 = availability(a); if (!av2.ok) { toast(av2.why, "info"); return; } playAction(a, "", b); });
      return b;
    }));
    const turn = actions.find((a) => a.name === "turn around");
    $$("[data-turn]").forEach((b) => {
      const av = turn ? availability(turn, b.dataset.turn) : { ok: mode() !== "sim", why: "Turning is not available in simulation." };
      b.classList.toggle("na", !av.ok); b.title = av.why || "";
    });
  }
  $("#actSearch").addEventListener("input", renderActions);
  $("#actSpeed").value = store.get("actSpeed", "normal");
  $("#actSpeed").addEventListener("change", (e) => store.set("actSpeed", e.target.value));

  // music
  async function loadMusic() {
    const r = await api("GET", "/api/music", undefined, { quiet: true });
    const sel = $("#musicSelect");
    const names = r.ok ? listFrom(r, "music").map((m) => (typeof m === "string" ? m : m.name)).filter(Boolean) : [];
    const keep = sel.value;
    sel.replaceChildren(...names.map((n) => h("option", { value: n }, pretty(n))));
    if (names.includes(keep)) sel.value = keep;
    $("#musicNote").textContent = !r.ok ? "Music list unavailable: " + r.error
      : r.source === "sim" ? "No robot: the simulator has no audio (songs are listed for the dances)." : "";
  }
  $("#musicPlay").addEventListener("click", (e) => {
    const name = $("#musicSelect").value;
    if (!name) { toast("No song selected", "warn"); return; }
    busy(e.currentTarget, async () => {
      const r = await api("POST", "/api/music", { name });
      if (r.ok && r.simulated) toast(`"${pretty(name)}" (the simulator has no audio)`, "info");
    });
  });
  $("#musicStop").addEventListener("click", (e) => busy(e.currentTarget, () => api("POST", "/api/music/stop")));

  // ------------------------------------------------------------------ servos
  const servoEls = {};
  const servoPending = {};
  let servoInFlight = false, servoLastEdit = {};
  function buildServos() {
    const wrap = $("#servoGroups");
    wrap.replaceChildren();
    for (const g of SERVO_GROUPS) {
      const limp = h("button", { type: "button", class: "btn small ghost", title: `Make the ${g.label.toLowerCase()} limp (pose by hand)` }, "Limp");
      const hold = h("button", { type: "button", class: "btn small ghost", title: `Hold the ${g.label.toLowerCase()} in place` }, "Hold");
      const box = h("div", { class: "sgroup", "data-group": g.id },
        h("div", { class: "sgroup-head" }, h("h3", {}, g.label), h("div", { class: "btn-row" }, limp, hold)));
      limp.addEventListener("click", () => busy(limp, () => servoMode("program", g.names, [box])));
      hold.addEventListener("click", () => busy(hold, () => servoMode("work", g.names, [box])));
      for (const n of g.names) {
        const [lo, hi] = limits[n] || [0, 180];
        const id = "sv-" + n;
        const range = h("input", { type: "range", id, min: lo, max: hi, step: 1, value: 90, "aria-label": n });
        const num = h("input", { type: "number", min: lo, max: hi, step: 1, value: 90, "aria-label": n + " degrees" });
        const rb = h("div", { class: "rb" }, h("span", { class: "rb-sim" }, "sim --"), h("span", { class: "rb-robot" }, ""),
          h("span", { class: "lim" }, `${lo}-${hi}°`));
        const row = h("div", { class: "srow" }, h("label", { for: id, title: n }, servoLabel(n)), num, range, rb);
        range.addEventListener("input", () => { num.value = range.value; servoEdit(n, +range.value); });
        range.addEventListener("change", () => servoEdit(n, +range.value, true));
        num.addEventListener("change", () => {
          const v = clamp(Math.round(+num.value), lo, hi);
          if (!isFinite(v)) return;
          num.value = v; range.value = v; servoEdit(n, v, true);
        });
        servoEls[n] = { range, num, rb, lo, hi };
        box.append(row);
      }
      wrap.append(box);
    }
  }
  async function servoMode(m, names, boxes) {
    const legs = names.some((n) => /Hip|Knee|Ankle/.test(n));
    if (m === "program" && legs && mode() !== "sim") {
      const ok = await confirmBox("Make the robot's legs limp?",
        "The REAL robot cannot stand on limp legs: it falls unless you hold it or lay it down first. Hold makes the servos stiff again.",
        "Limp: I am holding it");
      if (!ok) return;
    }
    const r = await api("POST", "/api/servos/mode", { mode: m, servos: names });
    if (r.ok) {
      boxes.forEach((b) => b.classList.toggle("limp", m === "program"));
      toast(m === "program" ? "Limp: pose by hand, then Capture frame" : "Holding position", "ok");
      if (m === "program" && mode() !== "robot" && legs) {
        toast("Limp legs cannot hold the simulated robot up: it will sink or fall. Press Hold, then Reset.", "warn");
      }
    }
  }
  $("#allLimp").addEventListener("click", (e) => busy(e.currentTarget, () => servoMode("program", NAMES, $$(".sgroup"))));
  $("#allHold").addEventListener("click", (e) => busy(e.currentTarget, () => servoMode("work", NAMES, $$(".sgroup"))));
  function servoEdit(n, v, now) {
    servoPending[n] = v;
    servoLastEdit[n] = Date.now();
    if (now) servoFlush(); else servoFlushThrottled();
  }
  async function servoFlush() {
    if (servoInFlight || !Object.keys(servoPending).length) return;
    const angles = { ...servoPending };
    Object.keys(servoPending).forEach((k) => delete servoPending[k]);
    servoInFlight = true;
    const runtime = clamp(parseInt($("#servoRuntime").value, 10) || 400, 200, 4000);
    try {
      const r = await api("POST", "/api/servos", { angles, runtime }, { timeout: 8000, quiet: true });
      if (!r.ok) toast(r.error, "error", failTitle(r.error));
      if (!r.ok && r.blocked && r.blocked.length) {
        for (const n of Object.keys(angles)) servoLastEdit[n] = 0; // snap back to the readback
      }
    } finally {
      servoInFlight = false;
      if (Object.keys(servoPending).length) setTimeout(servoFlush, 50);
    }
  }
  const servoFlushThrottled = throttle(servoFlush, 220);
  function servoSource(s) {
    return (s.mode !== "sim" && s.servos && s.servos.robot) ? s.servos.robot : (s.servos && s.servos.sim) || {};
  }
  function renderServoReadback(force) {
    if (!S || (ui.tab !== "servos" && !force)) return;
    const sim = (S.servos && S.servos.sim) || {}, rob = S.servos && S.servos.robot;
    const src = servoSource(S);
    const follow = $("#servoFollow").checked;
    const now = Date.now();
    for (const n of NAMES) {
      const el = servoEls[n];
      if (!el) continue;
      el.rb.children[0].textContent = `sim ${fmt(sim[n], 0)}°`;
      el.rb.children[1].textContent = rob ? `robot ${fmt(rob[n], 0)}°` : "";
      const editing = now - (servoLastEdit[n] || 0) < 1500 || document.activeElement === el.num || document.activeElement === el.range && el.range.matches(":active");
      if ((follow || force === "init") && !editing && typeof src[n] === "number") {
        const v = Math.round(src[n]);
        el.range.value = v; el.num.value = v;
      }
    }
  }
  function applyLimits(lim) {
    if (!lim) return;
    let changed = false;
    for (const n of NAMES) if (Array.isArray(lim[n]) && String(lim[n]) !== String(limits[n])) { limits[n] = lim[n]; changed = true; }
    if (changed) { buildServos(); renderServoReadback("init"); }
  }

  // ------------------------------------------------------------------ recorder
  const rec = Object.assign({ name: "", description: "", end_in_reset: true, frames: [], dirty: false }, store.get("draft", {}));
  let openFrames = new Set();
  function saveDraft() { store.set("draft", rec); }
  function setDirty() { rec.dirty = true; saveDraft(); $("#checkReport").hidden = true; }
  function recFromInputs() {
    rec.name = $("#recName").value.trim();
    rec.description = $("#recDesc").value;
    rec.end_in_reset = $("#recEndReset").checked;
  }
  function recToInputs() {
    $("#recName").value = rec.name || "";
    $("#recDesc").value = rec.description || "";
    $("#recEndReset").checked = rec.end_in_reset !== false;
  }
  ["#recName", "#recDesc"].forEach((s) => $(s).addEventListener("input", () => { recFromInputs(); setDirty(); validateName(); }));
  $("#recEndReset").addEventListener("change", () => { recFromInputs(); setDirty(); });
  function validateName() {
    const n = $("#recName").value.trim();
    $("#recName").classList.toggle("invalid", !!n && !NAME_RE.test(n));
    return n && NAME_RE.test(n) ? n : null;
  }
  function motionBody() {
    recFromInputs();
    const frames = rec.frames.map((f) => {
      const out = {};
      if (f.angles && Object.keys(f.angles).length) out.angles = { ...f.angles };
      out.runtime = clamp(parseInt(f.runtime, 10) || 500, 200, 4000);
      if (+f.hold > 0) out.hold = Math.round(+f.hold);
      return out;
    });
    const m = { frames, end_in_reset: rec.end_in_reset };
    if (rec.name) m.name = rec.name;
    if (rec.description) m.description = rec.description;
    return m;
  }
  function frameSummary(f) {
    const keys = Object.keys(f.angles || {});
    if (!keys.length) return "pause";
    if (keys.length >= NAMES.length) return "full pose (17 servos)";
    return keys.length <= 3 ? keys.map((k) => `${servoLabel(k)} ${f.angles[k]}°`).join(", ") : `${keys.length} servos`;
  }
  function renderFrames(flashIdx) {
    const list = $("#keyframes");
    $("#kfEmpty").hidden = rec.frames.length > 0;
    list.replaceChildren(...rec.frames.map((f, i) => {
      const rt = h("input", { type: "number", min: 200, max: 4000, step: 50, value: f.runtime, "aria-label": `Frame ${i + 1} runtime ms` });
      const hd = h("input", { type: "number", min: 0, max: 60000, step: 50, value: f.hold || 0, "aria-label": `Frame ${i + 1} hold ms` });
      rt.addEventListener("change", () => { f.runtime = clamp(parseInt(rt.value, 10) || 500, 200, 4000); rt.value = f.runtime; setDirty(); });
      hd.addEventListener("change", () => { f.hold = clamp(parseInt(hd.value, 10) || 0, 0, 60000); hd.value = f.hold; setDirty(); });
      const btn = (label, title, fn, cls = "") => h("button", { type: "button", class: "btn ghost " + cls, title, onclick: fn }, label);
      const open = openFrames.has(f);
      const anglesBox = h("div", { class: "kf-angles", hidden: !open });
      if (open) {
        for (const n of NAMES) {
          const [lo, hi] = limits[n] || [0, 180];
          const inp = h("input", { type: "number", min: lo, max: hi, step: 1, placeholder: "-", value: f.angles && n in f.angles ? f.angles[n] : "", "aria-label": n });
          inp.addEventListener("change", () => {
            f.angles = f.angles || {};
            if (inp.value === "") delete f.angles[n];
            else { const v = clamp(Math.round(+inp.value), lo, hi); if (!isFinite(v)) return; f.angles[n] = v; inp.value = v; }
            setDirty();
            const sum = list.children[i] && $(".kf-sum", list.children[i]);
            if (sum) sum.textContent = frameSummary(f);
          });
          anglesBox.append(h("label", { title: n }, servoLabel(n) + (n.startsWith("Left") ? " L" : n.startsWith("Right") ? " R" : ""), inp));
        }
      }
      const li = h("li", { class: "kf" + (i === flashIdx ? " flash" : "") },
        h("div", { class: "kf-row" },
          h("span", { class: "kf-idx" }, String(i + 1)),
          h("label", {}, "runtime", rt), h("label", {}, "hold", hd),
          h("span", { class: "kf-sum" }, frameSummary(f)),
          h("div", { class: "kf-btns" },
            btn("▲", "Move up", () => moveFrame(i, -1)),
            btn("▼", "Move down", () => moveFrame(i, 1)),
            btn("Go to", "Move the current target to this pose", (e) => gotoFrame(f, e.currentTarget)),
            btn("Retake", "Replace this frame's angles with the current pose", (e) => retakeFrame(f, i, e.currentTarget)),
            btn("Copy", "Duplicate this frame", () => { rec.frames.splice(i + 1, 0, JSON.parse(JSON.stringify(f))); setDirty(); renderFrames(i + 1); }),
            btn(open ? "Angles ▴" : "Angles ▾", "Edit angles", () => { open ? openFrames.delete(f) : openFrames.add(f); renderFrames(); }),
            btn("✕", "Delete this frame", () => { rec.frames.splice(i, 1); openFrames.delete(f); setDirty(); renderFrames(); }, "danger"))),
        anglesBox);
      return li;
    }));
  }
  function moveFrame(i, d) {
    const j = i + d;
    if (j < 0 || j >= rec.frames.length) return;
    [rec.frames[i], rec.frames[j]] = [rec.frames[j], rec.frames[i]];
    setDirty(); renderFrames(j);
  }
  async function gotoFrame(f, btn) {
    if (!f.angles || !Object.keys(f.angles).length) { toast("This frame is a pause (no angles)", "info"); return; }
    await busy(btn, () => api("POST", "/api/servos", { angles: f.angles, runtime: clamp(+f.runtime || 600, 200, 4000) }));
  }
  async function capture() {
    const runtime = clamp(parseInt($("#capRuntime").value, 10) || 600, 200, 4000);
    const hold = clamp(parseInt($("#capHold").value, 10) || 0, 0, 60000);
    $("#capHold").value = hold;
    const r = await api("POST", "/api/record", { runtime, hold });
    if (!r.ok) return null;
    const fr = r.frame || r;
    if (!fr.angles) { toast("The server returned no frame", "error"); return null; }
    return { angles: fr.angles, runtime: fr.runtime || runtime, hold: fr.hold || hold };
  }
  async function retakeFrame(f, i, btn) {
    await busy(btn, async () => {
      const fr = await capture();
      if (fr) { f.angles = fr.angles; setDirty(); renderFrames(i); }
    });
  }
  $("#captureBtn").addEventListener("click", (e) => busy(e.currentTarget, async () => {
    const fr = await capture();
    if (fr) { rec.frames.push(fr); setDirty(); renderFrames(rec.frames.length - 1); toast(`Frame ${rec.frames.length} captured`, "ok"); }
  }));
  $("#addPauseBtn").addEventListener("click", () => {
    rec.frames.push({ runtime: clamp(parseInt($("#capRuntime").value, 10) || 600, 200, 4000), hold: 0 });
    setDirty(); renderFrames(rec.frames.length - 1);
  });
  $("#recNew").addEventListener("click", async () => {
    if (rec.dirty && rec.frames.length && !(await confirmBox("Start a new motion?", "Unsaved keyframes will be discarded.", "Discard"))) return;
    Object.assign(rec, { name: "", source: "", description: "", end_in_reset: true, frames: [], dirty: false });
    openFrames = new Set(); saveDraft(); recToInputs(); renderFrames(); $("#checkReport").hidden = true;
  });
  function renderReport(r, title) {
    const box = $("#checkReport");
    box.hidden = false;
    if (!r.ok && r.safe === undefined && !r.errors) {
      box.className = "report unsafe";
      box.replaceChildren(h("strong", {}, title || "Check failed"), h("p", {}, r.error || ""));
      return;
    }
    const errors = r.errors || [], warnings = r.warnings || [], cols = r.collisions || [];
    const safe = r.safe !== undefined ? r.safe : !errors.length && !cols.length;
    box.className = "report " + (safe ? "safe" : "unsafe");
    const head = [safe ? "Safe to play" : "Not safe for the robot"];
    if (r.duration_s != null) head.push(`${fmt(r.duration_s)} s`);
    if (r.start) head.push(`checked from the ${r.start} pose`);
    box.replaceChildren(...[h("strong", {}, (title ? title + ": " : "") + head.join(" · ")),
      errors.length ? h("ul", { class: "r-err" }, errors.map((e) => h("li", {}, e))) : null,
      cols.length ? h("ul", { class: "r-err" }, cols.map((c) => h("li", {}, `${c.label || `Frame ${c.frame + 1}`}${c.at != null ? `, ${Math.round(c.at * 100)}% of the move` : ""}: ${(c.bodies || []).join(" hits ")}`))) : null,
      warnings.length ? h("ul", { class: "r-warn" }, warnings.map((w) => h("li", {}, w))) : null].filter(Boolean));
  }
  async function checkMotion(btn) {
    if (!rec.frames.length) { toast("Capture some keyframes first", "warn"); return null; }
    return busy(btn, async () => {
      const r = await api("POST", "/api/motions/check", motionBody(), { quiet: true });
      renderReport(r, "Check");
      if (!r.ok && r.error) toast(r.error, "error");
      return r;
    });
  }
  $("#recCheck").addEventListener("click", (e) => checkMotion(e.currentTarget));
  async function playEditor(btn, simOnly) {
    if (!rec.frames.length) { toast("Capture some keyframes first", "warn"); return; }
    const speed = clamp(+$("#recSpeed").value || 1, 0.1, 5);
    if (simOnly && mode() !== "sim") {
      if (!(await confirmBox("Preview in the simulator?", "This switches the relay to Sim mode (the robot stops and stays still).", "Switch to Sim"))) return;
      const m = await api("POST", "/api/mode", { mode: "sim" });
      if (!m.ok) return;
      await refreshState();
    }
    await busy(btn, async () => {
      walkRelease();
      const name = !rec.dirty && rec.source && validateName() === rec.source ? rec.source : null;
      let r;
      if (name) r = await api("POST", `/api/motions/${encodeURIComponent(name)}/play`, { speed }, { quiet: true });
      else {
        // unsaved keyframes: POST /api/motions/play {motion, speed}; servers without it get a scratch file
        r = await api("POST", "/api/motions/play", { motion: motionBody(), speed }, { quiet: true });
        if (r.status === 404 || r.status === 405) {
          const sv = await api("PUT", "/api/motions/" + PREVIEW_NAME, motionBody(), { quiet: true });
          if (!sv.ok) { renderReport(sv, "Cannot play"); toast(sv.error, "error", failTitle(sv.error)); return; }
          r = await api("POST", `/api/motions/${PREVIEW_NAME}/play`, { speed }, { quiet: true });
          api("DELETE", "/api/motions/" + PREVIEW_NAME, undefined, { quiet: true });
        }
      }
      if (!r.ok) { renderReport(r, "Refused"); toast(r.error, "error", failTitle(r.error)); return; }
      (r.warnings || []).slice(0, 3).forEach((w) => toast(w, "warn"));
      toast(`Playing on ${r.target || mode()}${r.duration_s ? ` (${fmt(r.duration_s)} s)` : ""}`, "ok");
    });
  }
  $("#recPreview").addEventListener("click", (e) => playEditor(e.currentTarget, true));
  $("#recPlay").addEventListener("click", (e) => playEditor(e.currentTarget, false));
  $("#recSave").addEventListener("click", (e) => {
    const name = validateName();
    if (!name) { toast("Give the motion a name: letters, digits, _ and - only", "warn"); $("#recName").focus(); return; }
    if (!rec.frames.length) { toast("Capture some keyframes first", "warn"); return; }
    busy(e.currentTarget, async () => {
      const exists = motions.some((m) => m.name === name) && rec.source !== name;
      if (exists && !(await confirmBox(`Overwrite "${name}"?`, "A motion file with this name already exists.", "Overwrite"))) return;
      const r = await api("PUT", "/api/motions/" + encodeURIComponent(name), motionBody(), { quiet: true });
      if (!r.ok) { renderReport(r, "Not saved"); toast(r.error, "error"); return; }
      rec.dirty = false; rec.source = name; saveDraft();
      renderReport({ ok: true, errors: r.errors || [], warnings: r.warnings || [], collisions: [] }, `Saved ${name}`);
      toast(`Saved ${name}`, "ok");
      loadMotions(); loadActions();
    });
  });

  // motion files
  let motions = [];
  async function loadMotions() {
    const r = await api("GET", "/api/motions", undefined, { quiet: true });
    const box = $("#motionList");
    if (!r.ok) { box.replaceChildren(h("p", { class: "empty" }, "Could not list motion files: " + r.error)); return; }
    motions = listFrom(r, "motions", "files").filter((m) => m && m.name && !m.name.startsWith("_"));
    if (!motions.length) { box.replaceChildren(h("p", { class: "empty" }, "No motion files yet.")); return; }
    box.replaceChildren(...motions.map((m) => {
      const load = h("button", { type: "button", class: "btn small" }, "Load");
      const play = h("button", { type: "button", class: "btn small" }, "Play");
      const del = h("button", { type: "button", class: "btn small ghost danger" }, "Delete");
      load.addEventListener("click", () => busy(load, () => loadIntoEditor(m.name)));
      play.addEventListener("click", () => busy(play, async () => {
        walkRelease();
        const r2 = await api("POST", `/api/motions/${encodeURIComponent(m.name)}/play`, { speed: clamp(+$("#recSpeed").value || 1, 0.1, 5) }, { quiet: true });
        if (!r2.ok) { renderReport(r2, `Refused ${m.name}`); toast(r2.error, "error", failTitle(r2.error)); }
        else (r2.warnings || []).slice(0, 3).forEach((w) => toast(w, "warn"));
      }));
      del.addEventListener("click", () => busy(del, async () => {
        if (!(await confirmBox(`Delete "${m.name}"?`, "The motion file is removed from disk. This cannot be undone.", "Delete"))) return;
        const r2 = await api("DELETE", "/api/motions/" + encodeURIComponent(m.name), undefined, { okText: `Deleted ${m.name}` });
        if (r2.ok) { loadMotions(); loadActions(); }
      }));
      const meta = m.error ? m.error : [`${m.frames ?? "?"} frames`, m.duration_s != null ? `${fmt(m.duration_s)} s` : "", m.description || ""].filter(Boolean).join(" · ");
      return h("div", { class: "frow" }, h("span", { class: "f-name" }, m.name),
        m.dir === "recorded" ? h("span", { class: "badge" }, "recorded") : null,
        h("span", { class: "f-meta" }, meta), h("div", { class: "btn-row" }, load, play, del));
    }));
  }
  async function loadIntoEditor(name) {
    if (rec.dirty && rec.frames.length && !(await confirmBox(`Load "${name}"?`, "Unsaved keyframes in the editor will be discarded.", "Load"))) return;
    const r = await api("GET", "/api/motions/" + encodeURIComponent(name));
    if (!r.ok) return;
    const mo = r.motion || r;
    if (!Array.isArray(mo.frames)) { toast(`"${name}" has no frames list`, "error"); return; }
    Object.assign(rec, { name, source: name, description: mo.description || "", end_in_reset: mo.end_in_reset !== false, dirty: false,
      frames: mo.frames.map((f) => ({ angles: f && f.angles ? { ...f.angles } : undefined, runtime: (f && f.runtime) || 500, hold: (f && f.hold) || 0 })) });
    openFrames = new Set(); saveDraft(); recToInputs(); renderFrames(); $("#checkReport").hidden = true;
    toast(`Loaded ${name} (${rec.frames.length} frames)`, "ok");
  }
  $("#motionsRefresh").addEventListener("click", (e) => busy(e.currentTarget, loadMotions));

  // ------------------------------------------------------------------ settings
  const volume = $("#volume");
  let volumeEditing = 0;
  volume.addEventListener("input", () => { $("#volumeOut").textContent = volume.value; volumeEditing = Date.now(); });
  volume.addEventListener("change", () => { volumeEditing = Date.now(); api("POST", "/api/volume", { volume: +volume.value }); });
  $$("#langSeg button").forEach((b) => b.addEventListener("click", () => busy(b, () => api("POST", "/api/language", { language: b.dataset.lang }))));
  $("#fallMgmt").addEventListener("change", async (e) => {
    const r = await api("POST", "/api/fall", { enable: e.target.checked });
    if (!r.ok) e.target.checked = !e.target.checked;
  });
  $("#chargeBtn").addEventListener("click", (e) => busy(e.currentTarget, () => api("POST", "/api/sim/charge", undefined, { okText: "Sim battery charged" })));
  for (const set of $$(".led-set")) {
    const type = set.dataset.led;
    $(".swatches", set).replaceChildren(...LED_COLORS[type].map((c) => h("button", { type: "button", role: "radio", "data-color": c,
      "aria-label": c, title: c, style: `--sw:${CSS_COLOR[c]}`, onclick: () => setLed(type, c, null) })));
    $(".modes", set).replaceChildren(...LED_MODES[type].map((m) => h("button", { type: "button", role: "radio", "data-mode": m,
      onclick: () => setLed(type, null, m) }, m)));
  }
  function setLed(type, color, m) {
    const cur = (S && S.leds && S.leds[type]) || {};
    color = color || cur.color || LED_COLORS[type][0];
    m = m || cur.mode || "on";
    if (!LED_COLORS[type].includes(color) || !LED_MODES[type].includes(m)) { toast(`The ${type} LED cannot do ${color}/${m}`, "warn"); return; }
    api("POST", "/api/led", { type, color, mode: m });
  }
  function renderSettings(s) {
    if (Date.now() - volumeEditing > 1500 && typeof s.volume === "number" && document.activeElement !== volume) {
      volume.value = s.volume; $("#volumeOut").textContent = s.volume;
    }
    $$("#langSeg button").forEach((b) => b.setAttribute("aria-checked", String(b.dataset.lang === s.language)));
    if (typeof s.fall_management === "boolean") $("#fallMgmt").checked = s.fall_management;
    for (const set of $$(".led-set")) {
      const led = (s.leds || {})[set.dataset.led] || {};
      $$(".swatches button", set).forEach((b) => b.setAttribute("aria-checked", String(b.dataset.color === led.color)));
      $$(".modes button", set).forEach((b) => b.setAttribute("aria-checked", String(b.dataset.mode === led.mode)));
    }
  }

  // info
  function kv(obj, depth = 0) {
    const dl = h("dl", { class: "kv" });
    for (const [k, v] of Object.entries(obj || {})) {
      if (v && typeof v === "object" && !Array.isArray(v) && depth < 1) {
        dl.append(h("dt", {}, pretty(k)), h("dd", {}, kv(v, depth + 1)));
      } else {
        dl.append(h("dt", {}, pretty(k)), h("dd", {}, v == null ? "--" : typeof v === "object" ? JSON.stringify(v) : String(v)));
      }
    }
    return dl;
  }
  async function loadInfo() {
    const r = await api("GET", "/api/info", undefined, { quiet: true });
    const box = $("#infoBox");
    if (!r.ok) { box.replaceChildren(h("p", { class: "empty" }, "Info unavailable: " + r.error)); return; }
    box.replaceChildren(h("div", { class: "kv-title" }, "Simulator"), kv(r.sim),
      h("div", { class: "kv-title" }, "Robot"), r.robot ? kv(r.robot) : h("p", { class: "empty" }, "Not connected."));
  }
  $("#infoRefresh").addEventListener("click", (e) => busy(e.currentTarget, loadInfo));

  // sensors + rolling chart
  const chart = { buf: [], max: 240, series: "euler", target: "sim" };
  let sensorsBusy = false, sensorsTimer = null;
  function pick(g, series) {
    if (!g || typeof g !== "object" || g.error) return null;
    const k = series === "euler" ? "euler" : "gyro";
    const out = ["x", "y", "z"].map((a) => g[`${k}-${a}`]);
    return out.every((x) => typeof x === "number") ? out : null;
  }
  async function sensorsTick() {
    clearTimeout(sensorsTimer);
    if (ui.tab !== "settings" || document.hidden) return;
    if (!sensorsBusy) {
      sensorsBusy = true;
      const r = await api("GET", "/api/sensors", undefined, { quiet: true, timeout: 8000 });
      sensorsBusy = false;
      if (r.ok) renderSensors(r);
    }
    clearTimeout(sensorsTimer);
    if (ui.tab === "settings") sensorsTimer = setTimeout(sensorsTick, robotConnected() ? 1000 : 300);
  }
  function renderSensors(r) {
    const sim = r.sim || {};
    const rob = r.robot && typeof r.robot === "object" ? r.robot : null;
    chart.buf.push({ t: Date.now(), sim: { euler: pick(sim.gyro, "euler"), gyro: pick(sim.gyro, "gyro") },
      robot: rob ? { euler: pick(rob.gyro, "euler"), gyro: pick(rob.gyro, "gyro") } : null });
    const cutoff = Date.now() - 60000;
    while (chart.buf.length && (chart.buf.length > chart.max || chart.buf[0].t < cutoff)) chart.buf.shift();
    drawChart();
    const imuCard = (title, g) => {
      if (!g || typeof g !== "object") return h("div", {}, h("div", { class: "kv-title" }, title), h("p", { class: "empty" }, typeof g === "string" ? g : "no data"));
      const rows = {};
      for (const k of ["euler", "gyro", "accel"]) {
        const v = ["x", "y", "z"].map((a) => g[`${k}-${a}`]);
        if (v.some((x) => typeof x === "number")) rows[k === "euler" ? "angles (deg)" : k === "gyro" ? "gyro (deg/s)" : "accel (g)"] = v.map((x) => fmt(x, k === "euler" ? 1 : 2)).join("  ");
      }
      if (g.error) rows.error = g.error;
      return h("div", {}, h("div", { class: "kv-title" }, title), kv(rows));
    };
    $("#imuNums").replaceChildren(imuCard("Simulator", sim.gyro), imuCard("Robot", rob ? rob.gyro : (typeof r.robot === "string" ? r.robot : "not connected")));
    const add = [];
    if (sim.distance_m) add.push(h("div", { class: "kv-title" }, "Simulator distance sensors"), kv(Object.fromEntries(Object.entries(sim.distance_m).map(([k, v]) => [k, typeof v === "number" && v < 0 ? "nothing in range" : `${fmt(v, 3)} m`]))));
    if (rob && rob.addons) {
      const entries = Object.entries(rob.addons);
      if (rob.addons.error) add.push(h("p", { class: "empty" }, "Robot sensors: " + rob.addons.error));
      else if (!entries.length) add.push(h("p", { class: "empty" }, "No add-on sensors plugged into the robot."));
      else for (const [k, v] of entries) add.push(h("div", { class: "kv-title" }, k), Array.isArray(v) ? h("div", {}, v.map((x) => kv(x))) : (v && typeof v === "object" ? kv(v) : h("p", {}, String(v))));
    } else add.push(h("p", { class: "empty" }, "Robot add-on sensors (infrared, ultrasonic, touch, pressure, environment): robot not connected."));
    $("#addons").replaceChildren(...add);
  }
  function drawChart() {
    const cv = $("#imuChart");
    if (!cv || cv.offsetParent === null) return;
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, hh = cv.clientHeight;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(hh * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(hh * dpr); }
    const ctx = cv.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, hh);
    const css = getComputedStyle(document.documentElement);
    const col = (n) => css.getPropertyValue(n).trim();
    const pts = chart.buf.map((p) => ({ t: p.t, v: p[chart.target] && p[chart.target][chart.series] })).filter((p) => p.v);
    let span = chart.series === "euler" ? 10 : 30;
    pts.forEach((p) => p.v.forEach((x) => { span = Math.max(span, Math.abs(x) * 1.15); }));
    const padL = 42, padR = 8, padT = 8, padB = 18;
    const X = (t) => padL + (w - padL - padR) * (1 - (Date.now() - t) / 60000);
    const Y = (v) => padT + (hh - padT - padB) * (0.5 - v / (2 * span));
    ctx.font = "11px system-ui, sans-serif";
    ctx.fillStyle = col("--muted"); ctx.strokeStyle = col("--border"); ctx.lineWidth = 1;
    for (const v of [span, span / 2, 0, -span / 2, -span]) {
      ctx.beginPath(); ctx.moveTo(padL, Y(v)); ctx.lineTo(w - padR, Y(v)); ctx.stroke();
      ctx.fillText((Math.abs(span) >= 10 ? Math.round(v) : v.toFixed(2)), 4, Y(v) + 4);
    }
    ctx.fillText("-60 s", padL, hh - 4);
    ctx.fillText("now", w - padR - 22, hh - 4);
    if (!pts.length) {
      ctx.fillText(chart.target === "robot" && !robotConnected() ? "Robot not connected" : "Waiting for data...", padL + 10, padT + 16);
      return;
    }
    ["--x", "--y", "--z"].forEach((c, i) => {
      ctx.strokeStyle = col(c); ctx.lineWidth = 2; ctx.lineJoin = "round"; ctx.beginPath();
      pts.forEach((p, j) => { const x = X(p.t), y = Y(p.v[i]); j ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.stroke();
    });
  }
  function segSelect(sel, attr, fn) {
    $$(sel + " button").forEach((b) => b.addEventListener("click", () => {
      $$(sel + " button").forEach((x) => x.setAttribute("aria-checked", String(x === b)));
      fn(b.dataset[attr]);
    }));
  }
  segSelect("#imuSeries", "series", (v) => { chart.series = v; drawChart(); });
  segSelect("#imuTarget", "target", (v) => { chart.target = v; drawChart(); });
  window.addEventListener("resize", throttle(drawChart, 150));

  // ------------------------------------------------------------------ voice & vision
  function speakInBrowser(text) {
    if (!("speechSynthesis" in window)) { toast("This browser cannot speak (no speechSynthesis)", "warn"); return; }
    try {
      speechSynthesis.cancel();
      const u = new SpeechSynthesisUtterance(text);
      u.lang = S && S.language === "zh" ? "zh-CN" : "en-US";
      const v = speechSynthesis.getVoices().find((x) => x.lang && x.lang.toLowerCase().startsWith(u.lang.slice(0, 2).toLowerCase()));
      if (v) u.voice = v;
      speechSynthesis.speak(u);
    } catch (e) { toast("Browser speech failed: " + e.message, "warn"); }
  }
  async function say(text, btn) {
    text = (text || "").trim();
    if (!text) { toast("Type something to say", "warn"); return; }
    await busy(btn, async () => {
      const r = await api("POST", "/api/tts", { text });
      if (r.ok && r.speak_in_browser) speakInBrowser(r.speak_in_browser);
    });
  }
  $("#ttsBtn").addEventListener("click", (e) => say($("#ttsText").value, e.currentTarget));
  $("#ttsText").addEventListener("keydown", (e) => { if (e.key === "Enter" && (e.ctrlKey || e.metaKey)) say($("#ttsText").value, $("#ttsBtn")); });
  $("#ttsStopBtn").addEventListener("click", () => {
    try { if (window.speechSynthesis) speechSynthesis.cancel(); } catch (e) { /* ignore */ }
    if (robotConnected()) api("POST", "/api/tts/stop"); // robot TTS only (stop_voice_tts); dances keep going
  });
  $("#phrases").replaceChildren(...PHRASES.map((p) => h("button", { type: "button", class: "btn ghost", onclick: (e) => { $("#ttsText").value = p; say(p, e.currentTarget); } }, p)));

  function renderVision() {
    $("#visionGrid").replaceChildren(...VISION.map((v) => {
      const ok = mode() !== "sim" ? robotConnected() : v.task === "color";
      const why = ok ? "" : mode() === "sim" ? `"${v.label}" needs the robot (only Colours works in simulation).` : "No robot connected.";
      const b = h("button", { type: "button", class: "tile" + (ok ? "" : " na"), title: why || v.task },
        h("span", { class: "t-name" }, v.label), h("span", { class: "t-badges" }, h("span", { class: "badge " + (v.task === "color" ? "sim" : "robot") }, v.task === "color" ? "sim + robot" : "robot")));
      b.addEventListener("click", () => { if (!ok) { toast(why, "info"); return; } runVision(v, b); });
      return b;
    }));
  }
  function visionBody(task, data) {
    if (data && Array.isArray(data.color)) {
      if (!data.color.length) return h("p", { class: "empty" }, "No strong colours seen.");
      return h("div", { class: "colors" }, data.color.map((c) => h("span", {}, h("i", { style: `--sw:${CSS_COLOR[c.name] || c.name}` }),
        `${c.name}${typeof c.fraction === "number" ? ` ${Math.round(c.fraction * 100)}%` : ""}`)));
    }
    if (data == null) return h("p", { class: "empty" }, "No result.");
    return h("pre", {}, JSON.stringify(data, null, 2));
  }
  async function runVision(v, btn) {
    const timeout = clamp(parseInt($("#visionTimeout").value, 10) || 8, 1, 30);
    const head = h("div", { class: "result-head" }, v.label, h("span", { class: "badge" }, "running..."), h("time", {}, clock()));
    const card = h("div", { class: "result" }, head);
    const box = $("#visionResults");
    box.prepend(card);
    while (box.children.length > 12) box.lastChild.remove();
    await busy(btn, async () => {
      const r = await api("POST", "/api/vision", { task: v.task, timeout }, { quiet: true, timeout: (timeout + 15) * 1000 });
      head.children[0].replaceWith(h("span", { class: "badge " + (r.ok ? "sim" : "robot") }, r.ok ? (r.target || "done") : "failed"));
      if (!r.ok) { card.classList.add("err"); card.append(h("p", {}, r.error || "failed")); toast(r.error, "error", failTitle(r.error)); if (r.data) card.append(visionBody(v.task, r.data)); return; }
      card.append(visionBody(v.task, r.data));
    });
  }
  $("#faceForm").addEventListener("submit", (e) => {
    e.preventDefault();
    const name = $("#faceName").value.trim();
    if (!name) { toast("Type the person's name first", "warn"); return; }
    busy($("button", e.currentTarget), () => api("POST", "/api/face_entry", { name }, { timeout: 40000, okText: `Face of ${name} enrolled` }));
  });
  let photoUrl = null;
  $("#photoRes").value = store.get("photoRes", "640x480");
  if (!$("#photoRes").value) $("#photoRes").value = "640x480";
  $("#photoBtn").addEventListener("click", (e) => busy(e.currentTarget, async () => {
    try {
      const res = $("#photoRes").value;
      store.set("photoRes", res);
      const r = await fetch("/api/photo?resolution=" + encodeURIComponent(res), { cache: "no-store" });
      const ct = r.headers.get("content-type") || "";
      if (!ct.startsWith("image/")) {
        let msg = `HTTP ${r.status}`;
        try { const j = await r.json(); msg = j.error || j.detail || msg; } catch (err) { /* not JSON */ }
        toast(msg, "error"); return;
      }
      const blob = await r.blob();
      if (photoUrl) URL.revokeObjectURL(photoUrl);
      photoUrl = URL.createObjectURL(blob);
      $("#photoBox").replaceChildren(h("img", { src: photoUrl, alt: "Photo from the " + (mode() === "sim" ? "simulated head camera" : "robot camera") }));
      const save = $("#photoSave");
      save.href = photoUrl; save.download = `yanshee-${new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19)}.jpg`; save.hidden = false;
    } catch (err) { toast("Photo failed: " + err.message, "error"); }
  }));

  // ------------------------------------------------------------------ log
  const events = [];
  const seenEvents = new Set();
  let logUnseen = 0, eventsSynced = false;
  ui.logLevel = "all";
  function addEvent(ev, live) {
    if (!ev || !ev.text) return;
    const key = ev.id != null ? "i" + ev.id : `t${ev.t}|${ev.text}`;
    if (seenEvents.has(key)) return;
    seenEvents.add(key);
    events.push(ev);
    if (events.length > 500) seenEvents.delete((() => { const o = events.shift(); return o.id != null ? "i" + o.id : `t${o.t}|${o.text}`; })());
    if (live && (ev.level === "warn" || ev.level === "error") && !QUIET_EVENTS.test(ev.text)) {
      toast(ev.text, ev.level, ev.level === "error" ? failTitle(ev.text) : "Warning");
      if (ui.tab !== "log") { logUnseen++; renderLogBadge(); }
    }
    if (ui.tab === "log") scheduleLog();
  }
  let logPending = false;
  function scheduleLog() { if (!logPending) { logPending = true; requestAnimationFrame(() => { logPending = false; renderLog(); }); } }
  function renderLogBadge() { const b = $("#logBadge"); b.hidden = !logUnseen; b.textContent = logUnseen; }
  function renderLog() {
    const q = $("#logSearch").value.trim().toLowerCase();
    const lv = ui.logLevel;
    const list = events.filter((e) => (lv === "all" || e.level === lv) && (!q || e.text.toLowerCase().includes(q))).slice(-300).reverse();
    $("#logList").replaceChildren(...(list.length ? list.map((e) => h("li", { class: "l-" + (e.level || "info") },
      h("time", {}, clock(e.t)), h("span", { class: "lvl" }, e.level || "info"), h("span", { class: "txt" }, e.text)))
      : [h("li", {}, h("span", {}), h("span", {}), h("span", { class: "txt empty" }, "No events."))]));
  }
  segSelect("#logFilter", "level", (v) => { ui.logLevel = v; renderLog(); });
  $("#logSearch").addEventListener("input", renderLog);
  $("#logClear").addEventListener("click", () => { events.length = 0; renderLog(); });

  // ------------------------------------------------------------------ state + websocket
  function applyState(s) {
    if (!s || typeof s !== "object" || !s.sim) return;
    const first = !S;
    S = s;
    s.robot = s.robot || { connected: false };
    applyLimits(s.limits);
    renderTop(s);
    renderLive(s);
    renderSettings(s);
    renderServoReadback(first ? "init" : false);
    const modeKey = s.mode + "|" + !!s.robot.connected;
    if (modeKey !== ui.modeKey) {
      const prevConnected = ui.modeKey.endsWith("true");
      ui.modeKey = modeKey;
      renderActions(); renderVision();
      $("#ttsNote").textContent = s.mode === "sim"
        ? "Sim mode: this browser speaks the text. Speech recognition (ASR) on the robot understands Chinese only."
        : "The robot speaks the text. Speech recognition (ASR) on the robot understands Chinese only.";
      if (!first && prevConnected !== !!s.robot.connected) { loadActions(); loadMusic(); }
      if (!first && ui.cam === "head") startStream(); // the head stream source changes with the mode
    }
    for (const ev of s.events || []) addEvent(ev, eventsSynced);
    eventsSynced = true;
  }
  async function refreshState() {
    const r = await api("GET", "/api/state", undefined, { quiet: true, timeout: 8000 });
    if (r.ok !== false && r.sim) applyState(r);
    return r;
  }

  let ws = null, wsDelay = 500, wsLast = 0, wsReconnectTimer = null, offlineTimer = null, wasOffline = false, pollTimer = null;
  function setOnline(on) {
    if (on) {
      clearTimeout(offlineTimer); offlineTimer = null;
      clearTimeout(pollTimer);
      $("#offlineBanner").hidden = true;
      if (wasOffline) { wasOffline = false; toast("Reconnected to the server", "ok"); startStream(); loadActions(); loadMusic(); loadMotions(); }
    } else if (!offlineTimer && $("#offlineBanner").hidden) {
      // one debounce per outage: failed reconnects (every 0.5-1.3 s at first) must not keep postponing it
      offlineTimer = setTimeout(() => { offlineTimer = null; $("#offlineBanner").hidden = false; wasOffline = true; walkRelease(); }, 1500);
    }
  }
  const VIDEO_PAUSED = "Video paused: no connection to the server.";
  function wsConnect() {
    clearTimeout(wsReconnectTimer);
    const url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws";
    try { ws = new WebSocket(url); } catch (e) { wsSchedule(); return; }
    ws.onopen = () => { wsDelay = 500; wsLast = Date.now(); setOnline(true); };
    ws.onmessage = (e) => {
      wsLast = Date.now();
      let m;
      try { m = JSON.parse(e.data); } catch (err) { return; }
      if (m.type === "state") applyState(m.state || m);
      else if (m.type === "event") addEvent(m.event || m, true);
      else if (m.type === "events" && Array.isArray(m.events)) m.events.forEach((ev) => addEvent(ev, true));
    };
    ws.onclose = () => { ws = null; setOnline(false); wsSchedule(); pollState(); };
    ws.onerror = () => { try { ws && ws.close(); } catch (e) { /* ignore */ } };
  }
  function wsSchedule() {
    clearTimeout(wsReconnectTimer);
    $("#offlineText").textContent = `Reconnecting in ${Math.round(wsDelay / 1000) || 1} s... (is scripts/web_gui.py still running?)`;
    wsReconnectTimer = setTimeout(wsConnect, wsDelay);
    wsDelay = Math.min(5000, wsDelay * 1.6);
  }
  // while the WebSocket is down, keep the state live over plain HTTP (and tell "server down" from "link down")
  async function pollState() {
    clearTimeout(pollTimer);
    if (ws && ws.readyState === WebSocket.OPEN) return;
    const r = await refreshState();
    const up = r.ok !== false && !!r.sim;
    const ph = $("#vpPlaceholder");
    if (!up) { ph.hidden = false; ph.textContent = VIDEO_PAUSED; } // the last frame would look live
    else if (!ph.hidden && ph.textContent === VIDEO_PAUSED) startStream();
    const banner = $("#offlineBanner");
    banner.classList.toggle("soft", up);
    $("#offlineTitle").textContent = up ? "Live link interrupted." : "Server connection lost.";
    if (up) $("#offlineText").textContent = "The page updates every second over HTTP while it reconnects.";
    if (!(ws && ws.readyState === WebSocket.OPEN)) pollTimer = setTimeout(pollState, 1000);
  }
  setInterval(() => { // a silent socket (no state for 6 s) is treated as dead
    if (ws && ws.readyState === WebSocket.OPEN && Date.now() - wsLast > 6000) { try { ws.close(); } catch (e) { /* ignore */ } }
  }, 2000);

  // ------------------------------------------------------------------ boot
  function measureTopbar() {
    document.documentElement.style.setProperty("--topbar-h", $("#topbar").offsetHeight + "px");
  }
  window.addEventListener("resize", throttle(measureTopbar, 100));
  if (window.ResizeObserver) new ResizeObserver(measureTopbar).observe($("#topbar"));
  buildServos();
  recToInputs();
  renderFrames();
  renderVision();
  showTab(location.hash.slice(1) || "motion");
  measureTopbar();
  refreshState();
  loadActions();
  loadMusic();
  wsConnect();
  // start the MJPEG stream after the page has loaded so the load event is not held up by the endless stream
  if (document.readyState === "complete") startStream(); else window.addEventListener("load", startStream, { once: true });
})();
