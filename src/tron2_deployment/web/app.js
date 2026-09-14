"use strict";
const $ = (id) => document.getElementById(id);
const state = { status: null, frame: null, image: null, box: null, drag: null, mask: null, overlay: null,
  observation: null, plan: null, handoff: null, revision: 0, operation: null, error: "", job: null, timer: null };
const ready = () => state.status?.workflow === "object_pregrasp" && !state.status.profile_error;
const running = () => state.job?.state === "running";
const fmt = (value) => Array.isArray(value) ? value.map((n) => Number(n).toFixed(4)).join("  ") : String(value);

function badge(id, value, good = false) { $(id).textContent = value; $(id).className = `badge${good ? " ready" : ""}`; }
function render() {
  const busy = Boolean(state.operation) || running();
  $("capture").disabled = !ready() || busy;
  $("mask").disabled = busy || !state.frame || !state.box;
  $("clear").disabled = busy || !state.frame || !state.box;
  $("estimate").disabled = busy || !state.mask;
  $("plan").disabled = busy || !state.observation || (!$("left").checked && !$("right").checked);
  $("download").disabled = busy || !state.plan;
  $("box").disabled = busy || !state.frame;
  $("reviewed").disabled = busy || !state.plan;
  $("execute").disabled = busy || !state.status?.mock || !state.plan || !$("reviewed").checked
    || state.job?.plan_id === state.plan?.plan_id;
  $("stop").disabled = !running() || !state.status?.mock;
  $("empty").hidden = Boolean(state.image);
  $("error").hidden = !state.error;
  $("error").textContent = state.error;
  $("activity").textContent = state.operation?.message || (running() ? "正在执行模拟计划，可随时停止。" : state.plan ? "计划已生成。请在 RViz 审核完整路径与停止位置。" : state.observation ? "物体已定位，可以读取腕部状态并规划。" : state.mask ? "遮罩已就绪，可以估计物体位姿。" : state.frame ? "请框选目标物体，并生成遮罩。" : "手动采集当前顶部 RGB-D 帧开始工作。");
  badge("connection", ready() ? "服务已连接" : "服务未就绪", ready());
  badge("observation-badge", state.observation ? "物体已定位" : state.mask ? "遮罩已就绪" : state.frame ? "已采集" : "等待采集", Boolean(state.observation));
  badge("plan-badge", state.plan ? "计划已生成" : state.observation ? "可规划" : "等待定位", Boolean(state.plan));
  const jobLabel = {running:"模拟运行中",completed:"模拟已完成",rejected:"模拟执行被拒绝",failed:"模拟执行失败",stopped:"模拟已停止"}[state.job?.state];
  badge("execution-badge", jobLabel || (state.plan ? "等待 RViz 审核" : "等待计划"), state.job?.state === "completed");
  $("frame-info").textContent = state.frame ? `${state.frame.width} × ${state.frame.height} · ${state.frame.source === "mock" ? "模拟 RGB-D" : "实际 RGB-D"} · base_Link` : "尚未采集";
  $("mask-info").textContent = state.mask ? `遮罩已就绪 · ${Number(state.mask.area_px || 0).toLocaleString()} px` : "尚未生成遮罩";
  $("position").textContent = state.observation ? fmt(state.observation.pose7.slice(0, 3)) : "—";
  $("quaternion").textContent = state.observation ? fmt(state.observation.pose7.slice(3)) : "—";
  $("source").textContent = state.observation ? `${state.observation.source === "mock" ? "模拟观测" : "FoundationPose"} · ${new Date(state.observation.capture_timestamp_s * 1000).toLocaleTimeString("zh-CN", {hour12:false})}` : "—";
  $("plan-result").hidden = !state.plan;
  $("rviz-handoff").hidden = !state.plan;
  $("job").hidden = !state.job;
  $("job").textContent = state.job ? JSON.stringify(state.job, null, 2) : "";
  $("execution-note").textContent = state.status?.mock ? "模拟模式：审核后仅运行模拟机器人，并在预抓取位置停止。" : "当前连接实际设备：浏览器仅观测与规划。真实执行请使用审核后的命令行流程。";
}
function invalidate(level) {
  state.revision += 1;
  state.operation?.controller.abort(); state.operation = null; state.error = "";
  state.plan = state.handoff = null; $("reviewed").checked = false;
  $("plan-json").textContent = ""; $("rviz-command").textContent = ""; $("plan-summary").replaceChildren();
  if (level !== "plan") state.observation = null;
  if (["frame", "mask"].includes(level)) state.mask = state.overlay = null;
  if (level === "frame") { state.frame = state.image = state.box = state.drag = null; $("box").value = ""; }
  draw(); render();
}
async function api(path, body, signal) {
  const response = await fetch(path, {method: body === undefined ? "GET" : "POST", headers:{"Content-Type":"application/json"}, body: body === undefined ? undefined : JSON.stringify(body), signal});
  let data; try { data = await response.json(); } catch { throw new Error(`服务响应无法读取（HTTP ${response.status}）`); }
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}
async function operation(message, action) {
  if (state.operation || running()) return;
  const task = {message, revision:state.revision, controller:new AbortController()}; state.operation = task; state.error = ""; render();
  const current = () => state.operation === task && state.revision === task.revision;
  try { await action(task.controller.signal, current); }
  catch (error) { if (current() && error.name !== "AbortError") state.error = error.message; }
  finally { if (current()) { state.operation = null; draw(); render(); } }
}
function profileSettings(profile) {
  $("profile-name").textContent = profile.profile_name;
  $("fingerprint").textContent = profile.profile_fingerprint;
  $("mesh").value = profile.object?.mesh_id || "未配置";
  $("mode").textContent = profile.mock ? "模拟模式 · 合成观测 / 模拟腕部反馈 · 不连接机器人" : "实际观测与腕部反馈 · 浏览器执行关闭";
  const labels = {symmetry:"物体对称性",standoff_m:"物体退让距离（m）",lift_clearance_m:"抬升余量（m）",anchor_object:"物体坐标系接近锚点",outward_axis_object:"物体向外方向",radius_m:"轴对称半径（m）",axial_offset_m:"轴向偏移（m）",wrist_to_tcp_pose7:"腕部 → TCP（XYZ / WXYZ）"};
  for (const side of ["left","right"]) {
    const list = $(`${side}-settings`); list.replaceChildren();
    for (const [key,label] of Object.entries(labels)) {
      const value = profile.pregrasp?.[side]?.[key]; if (value === undefined) continue;
      const term = document.createElement("dt"), detail = document.createElement("dd");
      term.textContent = label; detail.textContent = key === "symmetry" ? value === "axial" ? "轴对称（按当前腕部方向接近）" : "非轴对称（使用配置锚点）" : fmt(value);
      list.append(term,detail);
    }
  }
}
async function refresh() {
  const revision = state.revision;
  try {
    const status = await api("/api/status");
    if (status.workflow !== "object_pregrasp") throw new Error("连接的服务不是当前独立预抓取工作台。");
    state.status = status; state.job = status.job;
    if (["rejected","failed"].includes(state.job?.state)) state.error = state.job.error || (state.job.report?.errors || []).join("；") || "模拟执行未成功，请查看运行记录。";
    if (!state.operation && revision === state.revision) {
      if (state.frame && (state.frame.frame_ref !== status.latest_frame_ref || state.frame.profile_fingerprint !== status.profile_fingerprint)) invalidate("frame");
      else if (state.mask && state.mask.mask_ref !== status.latest_mask_ref) invalidate("mask");
      else if (state.observation && state.observation.observation_id !== status.latest_observation_id) invalidate("observation");
      else if (state.plan && state.plan.plan_id !== status.latest_plan_id) invalidate("plan");
    }
    if (status.profile_error) { invalidate("frame"); state.error = status.profile_error; }
    profileSettings(status);
  } catch (error) { state.status = null; invalidate("frame"); state.error = error.message; }
  render(); clearTimeout(state.timer);
  if (running()) state.timer = setTimeout(refresh, 600);
}
function image(source) {
  return new Promise((resolve,reject) => {
    if (typeof source !== "string" || !source.startsWith("data:image/")) return reject(new Error("服务未返回有效的内嵌图像。"));
    const img = new Image(); img.onload = () => resolve(img); img.onerror = () => reject(new Error("图像解码失败。")); img.src = source;
  });
}
function draw() {
  const canvas = $("frame"), ctx = canvas.getContext("2d"); ctx.clearRect(0,0,canvas.width,canvas.height);
  if (!state.image) return;
  ctx.drawImage(state.image,0,0,canvas.width,canvas.height);
  if (state.overlay) ctx.drawImage(state.overlay,0,0);
  const box = state.drag || state.box;
  if (box) { ctx.strokeStyle="#70ffd0"; ctx.fillStyle="#4bd2a71c"; ctx.lineWidth=Math.max(2,canvas.width/260); ctx.setLineDash([8,5]); ctx.fillRect(box[0],box[1],box[2]-box[0],box[3]-box[1]); ctx.strokeRect(box[0],box[1],box[2]-box[0],box[3]-box[1]); ctx.setLineDash([]); }
}
function point(event) {
  const r = $("frame").getBoundingClientRect(), scale = Math.min(r.width/state.frame.width,r.height/state.frame.height);
  const left = r.left+(r.width-state.frame.width*scale)/2, top = r.top+(r.height-state.frame.height*scale)/2;
  return [Math.round(Math.max(0,Math.min(state.frame.width,(event.clientX-left)/scale))),Math.round(Math.max(0,Math.min(state.frame.height,(event.clientY-top)/scale)))];
}
function boxOrder(b) { return [Math.min(b[0],b[2]),Math.min(b[1],b[3]),Math.max(b[0],b[2]),Math.max(b[1],b[3])]; }
function setBox(box) { state.box = box[2]-box[0]>=2 && box[3]-box[1]>=2 ? box : null; state.drag = null; $("box").value = state.box ? state.box.join(" ") : ""; draw(); render(); }
$("capture").onclick = () => {
  invalidate("frame"); operation("正在采集当前 RGB-D…",async(signal,current) => {
    const frame = await api("/api/capture",{},signal); if(!current())return;
    if (!frame.frame_ref || frame.reference_frame !== "base_Link") throw new Error("相机响应缺少当前帧或 base_Link 标定信息。");
    const img = await image(frame.image); if(!current())return;
    if (img.naturalWidth !== frame.width || img.naturalHeight !== frame.height) throw new Error("图像尺寸与相机元数据不一致。");
    state.frame = frame; state.image = img; $("frame").width = frame.width; $("frame").height = frame.height;
  });
};
$("mask").onclick = () => {
  if(!state.box)return; invalidate("mask"); operation("正在生成 SAM 物体遮罩…",async(signal,current) => {
    const result = await api("/api/mask",{frame_ref:state.frame.frame_ref,prompt:{type:"box",xyxy:state.box,coordinates:"pixels"}},signal); if(!current())return;
    if(!result.mask_ref || result.frame_ref !== state.frame.frame_ref)throw new Error("遮罩不属于当前 RGB-D 帧。");
    const img = await image(result.mask); if(!current())return;
    if(img.naturalWidth!==state.frame.width || img.naturalHeight!==state.frame.height)throw new Error("遮罩尺寸不匹配。");
    const overlay=document.createElement("canvas"); overlay.width=img.naturalWidth;overlay.height=img.naturalHeight;
    const ctx=overlay.getContext("2d",{willReadFrequently:true});ctx.drawImage(img,0,0);const pixels=ctx.getImageData(0,0,overlay.width,overlay.height);let area=0;
    for(let i=0;i<pixels.data.length;i+=4){const foreground=pixels.data[i]>0&&pixels.data[i+3]>0;if(foreground)area++;pixels.data[i]=24;pixels.data[i+1]=192;pixels.data[i+2]=145;pixels.data[i+3]=foreground?94:0;}
    if(!area)throw new Error("遮罩为空，请重新框选目标物体。");ctx.putImageData(pixels,0,0);state.mask=result;state.overlay=overlay;
  });
};
$("estimate").onclick = () => {
  invalidate("observation"); operation("FoundationPose 正在定位物体…",async(signal,current) => {
    const result=await api("/api/estimate",{frame_ref:state.frame.frame_ref,mask_ref:state.mask.mask_ref,mesh_id:$("mesh").value},signal);if(!current())return;
    if(!result.observation_id || result.frame_ref!==state.frame.frame_ref || result.mask_ref!==state.mask.mask_ref || result.reference_frame!=="base_Link" || !Array.isArray(result.pose7) || result.pose7.length!==7 || !result.pose7.every(Number.isFinite))throw new Error("位姿响应不属于当前观测，或缺少有效的 base_Link 位姿。");
    state.observation=result;
  });
};
$("plan").onclick = () => {
  invalidate("plan"); operation("正在读取腕部反馈并检查接近路径…",async(signal,current) => {
    const sides=["left","right"].filter(side=>$(side).checked);
    const result=await api("/api/plan",{observation_id:state.observation.observation_id,sides},signal);if(!current())return;
    if(!result.plan?.plan_id || !result.plan_path || !result.rviz_command)throw new Error("规划器未返回完整计划与 RViz 交接信息。");
    state.plan=result.plan;state.handoff=result;$("plan-id").textContent=`计划 ID：${result.plan.plan_id}`;
    $("plan-json").textContent=JSON.stringify(result.plan,null,2);$("rviz-command").textContent=result.rviz_command;
    const summary=$("plan-summary");summary.replaceChildren();
    for(const [side,target] of Object.entries(result.plan.targets||{})){
      const card=document.createElement("div"),title=document.createElement("strong");title.textContent=side==="left"?"左腕预抓取目标":"右腕预抓取目标";card.append(title);
      for(const [label,value] of [["腕部位置（m）",target.wrist_pregrasp_pose7_base?.slice(0,3)],["TCP 位置（m）",target.tcp_pregrasp_pose7_base?.slice(0,3)],["物体退让距离（m）",target.standoff_m]]){
        if(value===undefined)continue;const p=document.createElement("p");p.textContent=`${label}：${fmt(value)}`;card.append(p);
      }summary.append(card);
    }
  });
};
$("download").onclick=()=>{if(!state.plan)return;const url=URL.createObjectURL(new Blob([JSON.stringify(state.plan,null,2)+"\n"],{type:"application/json"}));const a=document.createElement("a");a.href=url;a.download=`pregrasp_${state.plan.plan_id.replace(/[^a-zA-Z0-9_-]/g,"")}.json`;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);};
$("execute").onclick=()=>{if(!state.plan||!$("reviewed").checked)return;operation("正在启动模拟执行…",async(signal,current)=>{const result=await api("/api/mock-execute",{plan_id:state.plan.plan_id,reviewed_plan_id:state.plan.plan_id},signal);if(!current())return;state.job=result;state.timer=setTimeout(refresh,400);});};
$("stop").onclick=async()=>{try{await api("/api/stop",{});await refresh();}catch(error){state.error=error.message;render();}};
$("reviewed").onchange=render;
for(const side of ["left","right"])$(side).onchange=()=>invalidate("plan");
$("clear").onclick=()=>{invalidate("mask");state.box=state.drag=null;$("box").value="";draw();render();};
$("box").oninput=()=>{if(!state.frame)return;invalidate("mask");state.box=null;const text=$("box").value.trim();if(text){const box=text.split(/[\s,，]+/).map(Number);if(box.length===4&&box.every(Number.isFinite)){const ordered=boxOrder(box.map(Math.round));if(ordered[0]>=0&&ordered[1]>=0&&ordered[2]<=state.frame.width&&ordered[3]<=state.frame.height&&ordered[2]-ordered[0]>=2&&ordered[3]-ordered[1]>=2)state.box=ordered;else state.error="框选范围应在图像内，宽高至少为 2 像素。";}else state.error="请输入四个有限的像素坐标。";}draw();render();};
$("frame").onpointerdown=(event)=>{if(!state.frame||state.operation||running()||(event.pointerType==="mouse"&&event.button!==0))return;invalidate("mask");state.box=null;const p=point(event);state.drag=[...p,...p];$("frame").setPointerCapture(event.pointerId);draw();render();};
$("frame").onpointermove=(event)=>{if(!state.drag)return;state.drag=[...state.drag.slice(0,2),...point(event)];draw();};
$("frame").onpointerup=(event)=>{if(!state.drag)return;setBox(boxOrder([...state.drag.slice(0,2),...point(event)]));if($("frame").hasPointerCapture(event.pointerId))$("frame").releasePointerCapture(event.pointerId);};
$("frame").onpointercancel=()=>{state.drag=state.box=null;$("box").value="";draw();render();};
$("refresh").onclick=refresh;
render();refresh();
