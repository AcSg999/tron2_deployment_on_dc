"use strict";

(() => {
  const $ = (id) => document.getElementById(id);
  const copy = {
    zh: {
      helper: "标定采集助手", eyebrow: "CALIBRATION · 采集引导",
      title: "一步一步，完成标定采集", subtitle: "先看画面，再换位置采集，最后计算结果。",
      intrinsicsTitle: "看清标定板，采集相机内参", handeyeTitle: "变换腕部姿态，采集外参",
      intrinsicsSubtitle: "移动标定板，覆盖不同画面区域和倾斜角度。",
      handeyeSubtitle: "通过现有控制方式调整腕部姿态，静止后采集。保持相机与头部位置不变。",
      previewTitle: "相机画面", notCaptured: "尚未采集", detected: "已识别标定板", notDetected: "未识别标定板",
      emptyTitle: "把标定板放入画面", emptyHint: "点击“查看画面（不保存）”，检查标定板是否被识别。",
      previewAlt: "当前标定板画面与检测角点", previewHint: "检测到的角点会标在图上。画面只在点击后更新。",
      previewButton: "查看画面（不保存）", currentStep: "现在做什么", stepPreview: "看清标定板", stepSave: "换位置采集", stepSolve: "计算结果",
      startingNotice: "请先查看当前相机画面。", startingNext: "保持标定板完整可见，等待画面稳定。",
      saved: "已保存样本", coverageTitle: "画面位置", coverageHint: "尝试不同区域和倾斜角度。",
      coverageLabel: "已采集的画面区域", stepsLabel: "标定采集步骤",
      saveButton: "重新采集并保存样本", saveHint: "保存时会采集新画面，请保持标定板和机械臂静止。",
      solveButton: "计算结果", solveHint: "采集满足条件后，即可计算。", solveReady: "可以计算，也可以继续补充不同姿态。",
      resultEyebrow: "采集完成 · 下一步", resultTitle: "结果已保存",
      resultNote: "拟合误差用于检查采集质量；实际精度还需用独立测量点验证。",
      outputFile: "结果文件", commandLabel: "在终端继续", copyButton: "复制命令", copied: "已复制。", selectCopy: "已选中命令，请手动复制。",
      footer: "标定 → 部署 → 预抓取", manual: "采集由你操作；助手不会移动机械臂。",
      connecting: "正在读取采集状态…", disconnected: "未连接 · 刷新页面重试", ready: "已连接 · 点击按钮才会采集", mockReady: "模拟模式 · 使用合成样本练习操作",
      requestBusy: "正在处理，请保持静止…", previewBusy: "正在采集画面…", saveBusy: "正在采集并保存新样本…", solveBusy: "正在计算标定结果…",
      mock: "模拟数据", real: "实际采集", intrinsics: "相机内参", handeye: "相机到基座外参",
      left: "左腕", right: "右腕", board: "棋盘", innerCorners: "个内角点", square: "方格边长",
      arucoMarker: "ArUco 单码", arucoBoard: "ArUco 集成板", markerSize: "标记边长", markerGap: "标记间隙", visibleMarkers: "可见标记",
      requestError: "操作未完成", networkError: "无法连接采集服务，请检查终端中的服务状态。",
      spread: "相对首个样本的最大腕部转角变化", noSpread: "采集不同腕部姿态，避免只做平移。", selected: "已覆盖", unselected: "尚未覆盖",
      regions: ["左上", "上中", "右上", "左中", "中心", "右中", "左下", "下中", "右下"],
    },
    en: {
      helper: "Calibration helper", eyebrow: "CALIBRATION · CAPTURE GUIDE",
      title: "Complete calibration, one sample at a time", subtitle: "Check the view, collect different poses, then solve.",
      intrinsicsTitle: "Keep the board in view. Calibrate the camera.", handeyeTitle: "Change wrist poses. Calibrate the camera to base.",
      intrinsicsSubtitle: "Move the board across the image and vary its tilt.",
      handeyeSubtitle: "Use your existing controls to position the wrist, then let it settle. Keep the camera and head fixed.",
      previewTitle: "Camera view", notCaptured: "No capture yet", detected: "Board detected", notDetected: "Board not detected",
      emptyTitle: "Place the board in view", emptyHint: "Select Preview (do not save) to check that the board is detected.",
      previewAlt: "Current calibration board view with detected corners", previewHint: "Detected corners appear on the image. The view updates only when you click.",
      previewButton: "Preview (do not save)", currentStep: "What to do now", stepPreview: "Check the board", stepSave: "Vary and capture", stepSolve: "Solve",
      startingNotice: "Preview the current camera view first.", startingNext: "Keep the entire board visible and let the view settle.",
      saved: "Saved samples", coverageTitle: "Image positions", coverageHint: "Try different regions and tilts.",
      coverageLabel: "Sampled image regions", stepsLabel: "Calibration capture steps",
      saveButton: "Capture and save a new sample", saveHint: "Saving takes a fresh capture. Keep the board and arm still.",
      solveButton: "Solve", solveHint: "Solve becomes available when capture requirements are met.", solveReady: "You can solve now or add more varied poses.",
      resultEyebrow: "CAPTURE COMPLETE · NEXT STEP", resultTitle: "Result saved",
      resultNote: "Fit error checks capture quality. Use independent measured points to verify real accuracy.",
      outputFile: "Result file", commandLabel: "Continue in the terminal", copyButton: "Copy command", copied: "Copied.", selectCopy: "Command selected. Copy it manually.",
      footer: "Calibration → deployment → pregrasp", manual: "You control capture. This helper does not move the arm.",
      connecting: "Reading capture status…", disconnected: "Not connected · reload the page to retry", ready: "Connected · capture starts when you click", mockReady: "Mock mode · practice with synthetic samples",
      requestBusy: "Working. Please keep still…", previewBusy: "Capturing the view…", saveBusy: "Capturing and saving a new sample…", solveBusy: "Solving calibration…",
      mock: "MOCK DATA", real: "Real capture", intrinsics: "Camera intrinsics", handeye: "Camera to base",
      left: "Left wrist", right: "Right wrist", board: "Board", innerCorners: "inner corners", square: "square size",
      arucoMarker: "ArUco marker", arucoBoard: "ArUco board", markerSize: "marker size", markerGap: "marker gap", visibleMarkers: "visible markers",
      requestError: "Action could not be completed", networkError: "Cannot connect to the capture service. Check its status in the terminal.",
      spread: "Largest wrist rotation change from the first sample", noSpread: "Vary wrist rotation as well as position.", selected: "sampled", unselected: "not sampled",
      regions: ["Top left", "Top center", "Top right", "Middle left", "Center", "Middle right", "Bottom left", "Bottom center", "Bottom right"],
    },
  };
  const regionIds = ["top-left", "top-center", "top-right", "middle-left", "middle-center", "middle-right", "bottom-left", "bottom-center", "bottom-right"];
  let language = "zh";
  try { language = localStorage.getItem("tron2-calibration-language") === "en" ? "en" : "zh"; } catch (_) { /* Storage may be unavailable in private browsing. */ }
  let state = null;
  let busy = false;
  let activity = null;
  let disconnected = false;
  const localized = (value) => typeof value === "string" ? value : value?.[language] || value?.en || value?.zh || "";

  function render() {
    const t = copy[language];
    document.documentElement.lang = language === "zh" ? "zh-CN" : "en";
    document.title = `TRON2 · ${t.helper}`;
    document.querySelectorAll("[data-text]").forEach((element) => { element.textContent = t[element.dataset.text]; });
    $("language").textContent = language === "zh" ? "English" : "中文";
    $("language").setAttribute("aria-label", language === "zh" ? "Switch to English" : "切换到中文");
    $("preview-image").alt = t.previewAlt;
    document.querySelector(".steps").setAttribute("aria-label", t.stepsLabel);
    $("coverage-grid").setAttribute("aria-label", t.coverageLabel);
    $("title").textContent = state ? t[`${state.stage}Title`] || t.title : t.title;
    $("subtitle").textContent = state ? t[`${state.stage}Subtitle`] || t.subtitle : t.subtitle;
    $("connection").textContent = busy ? t[activity] || t.requestBusy : disconnected ? t.disconnected : state ? (state.mock ? t.mockReady : t.ready) : t.connecting;
    $("notice").textContent = localized(state?.notice) || t.startingNotice;
    $("next").textContent = localized(state?.next) || t.startingNext;
    document.querySelector(".instruction").classList.toggle("warning", ["error", "warning"].includes(state?.notice?.level));
    $("saved-count").textContent = String(state?.saved_count ?? 0);
    $("stage").textContent = state ? t[state.stage] || "" : "—";
    if (state?.stage === "handeye" && ["left", "right"].includes(state.side)) $("stage").textContent = `${t[state.side]} · ${t.handeye}`;
    const pattern = state?.pattern;
    const target = state?.target;
    const hasBoardSpec = Array.isArray(pattern) && pattern.length === 2 && pattern.every(Number.isFinite) && Number.isFinite(state?.square_m);
    const millimetres = (value) => Number((value * 1000).toFixed(3));
    let spec = "";
    if (hasBoardSpec) {
      spec = `${t.board}: ${pattern.join(" × ")} ${t.innerCorners} · ${t.square} ${millimetres(state.square_m)} mm`;
    } else if (target) {
      const visible = Array.isArray(state?.detected_marker_ids) ? state.detected_marker_ids.length : 0;
      spec = `${target.kind === "marker" ? t.arucoMarker : t.arucoBoard}: ${target.dictionary} · `;
      spec += target.kind === "marker"
        ? `ID ${target.marker_ids.join(", ")} · ${t.markerSize} ${millimetres(target.marker_length_m)} mm`
        : `${target.markers_x} × ${target.markers_y} · ${t.markerSize} ${millimetres(target.marker_length_m)} mm · ${t.markerGap} ${millimetres(target.marker_separation_m)} mm · ${t.visibleMarkers} ${visible}/${target.marker_ids.length}`;
    }
    $("board-spec").hidden = !spec;
    $("board-spec").textContent = spec;
    $("mode").hidden = !state;
    $("mode").textContent = state?.mock ? t.mock : t.real;
    $("mode").className = `badge${state?.mock ? " mock" : ""}`;
    const detected = state?.board_detected;
    $("detection").textContent = detected === true ? t.detected : detected === false ? t.notDetected : t.notCaptured;
    $("detection").className = `badge${detected === true ? " ready" : detected === false ? " warning" : ""}`;
    const preview = state?.preview_image;
    const hasPreview = typeof preview === "string" && /^data:image\/(?:png|jpeg);base64,[A-Za-z0-9+/=\s]+$/.test(preview);
    if (hasPreview && $("preview-image").getAttribute("src") !== preview) $("preview-image").src = preview;
    if (!hasPreview) $("preview-image").removeAttribute("src");
    $("preview-image").hidden = !hasPreview;
    $("empty-preview").hidden = hasPreview;
    $("intrinsic-coverage").hidden = state?.stage !== "intrinsics";
    const regions = new Set(state?.coverage_regions || []);
    $("coverage-grid").replaceChildren(...regionIds.map((region, index) => {
      const cell = document.createElement("span");
      const covered = regions.has(region);
      cell.className = `coverage-cell${covered ? " filled" : ""}`;
      cell.title = `${t.regions[index]} · ${covered ? t.selected : t.unselected}`;
      cell.setAttribute("aria-label", cell.title);
      return cell;
    }));
    $("handeye-spread").hidden = state?.stage !== "handeye";
    const spread = state?.rotation_spread_deg;
    $("handeye-spread").textContent = typeof spread === "number" && Number.isFinite(spread) ? `${t.spread}: ${spread.toFixed(1)}°` : t.noSpread;
    $("step-preview").classList.toggle("active", !state?.saved_count && !state?.result);
    $("step-save").classList.toggle("active", Boolean(state?.saved_count) && !state?.can_solve && !state?.result);
    $("step-solve").classList.toggle("active", Boolean(state?.can_solve || state?.result));
    $("solve-hint").textContent = state?.can_solve ? t.solveReady : t.solveHint;

    const result = state?.result;
    $("result").hidden = !result;
    $("metric-label").textContent = localized(result?.metric_label);
    $("metric-value").textContent = result?.metric_value == null ? "" : String(result.metric_value);
    $("result-next").textContent = localized(result?.next);
    $("result-path").textContent = result?.path || "";
    $("next-command").hidden = !result?.command;
    $("command").value = result?.command || "";
    document.querySelectorAll("button").forEach((button) => { button.disabled = busy; });
    $("preview").disabled = busy || !state;
    $("save").disabled = busy || !state;
    $("solve").disabled = busy || !state?.can_solve;
    $("copy").disabled = busy || !result?.command;
    $("preview").textContent = busy && activity === "previewBusy" ? t.previewBusy : t.previewButton;
    $("save").textContent = busy && activity === "saveBusy" ? t.saveBusy : t.saveButton;
    $("solve").textContent = busy && activity === "solveBusy" ? t.solveBusy : t.solveButton;
  }

  async function request(path, method = "GET") {
    const options = { method, cache: "no-store", credentials: "same-origin" };
    if (method === "POST") {
      options.headers = { "Content-Type": "application/json" };
      options.body = "{}";
    }
    const response = await fetch(path, options);
    let body;
    try { body = await response.json(); } catch (_) { throw new Error(`${copy[language].requestError} (HTTP ${response.status})`); }
    if (!response.ok) throw new Error(localized(body.error) || `${copy[language].requestError} (HTTP ${response.status})`);
    return body;
  }

  async function act(action) {
    if (busy) return;
    busy = true;
    activity = action ? `${action}Busy` : "connecting";
    $("error").hidden = true;
    $("copy-status").textContent = "";
    render();
    try {
      state = await request(action ? `/api/${action}` : "/api/status", action ? "POST" : "GET");
      disconnected = false;
    } catch (error) {
      $("error").textContent = error instanceof TypeError ? copy[language].networkError : error.message;
      $("error").hidden = false;
      disconnected = !state;
      // A failed capture may invalidate a previous result. Read the actual state again.
      if (action) {
        if (state) state = { ...state, result: null };
        try {
          state = await request("/api/status");
          disconnected = false;
        } catch (_) {
          disconnected = true; // Keep the error and last known capture visible.
        }
      }
    } finally {
      busy = false;
      activity = null;
      render();
    }
  }

  $("language").addEventListener("click", () => {
    language = language === "zh" ? "en" : "zh";
    try { localStorage.setItem("tron2-calibration-language", language); } catch (_) { /* Preference is optional. */ }
    $("copy-status").textContent = "";
    render();
  });
  ["preview", "save", "solve"].forEach((action) => $(action).addEventListener("click", () => act(action)));
  $("copy").addEventListener("click", async () => {
    try {
      await navigator.clipboard.writeText($("command").value);
      $("copy-status").textContent = copy[language].copied;
    } catch (_) {
      $("command").focus();
      $("command").select();
      $("copy-status").textContent = copy[language].selectCopy;
    }
  });
  act(null); // Status only: opening this page never starts a camera capture.
})();
