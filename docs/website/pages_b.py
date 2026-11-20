# -*- coding: utf-8 -*-
"""pages_b.py —— 其余页面内容（由 build.py 引入 EXTRA_PAGES）。"""

# ============================================================
# 仿真实验室
# ============================================================
SIM_BODY = '''
<div class="card">
  <h2><span class="no">🏭</span>仿真实验室：浏览器里的实时 2D SLAM</h2>
  <div class="sub">自动探索 AI + 360° 激光 + log-odds 栅格生长 + 扫描匹配纠偏 —— slam_toolbox 的浏览器缩影</div>
  <p>点「▶ 开始探索」，看机器人（青色三角）在六边形世界里自动巡航：<b class="wa">橙色射线</b>是激光逐帧扫描，
     <b class="cy">浅蓝灰区域</b>是正在生长的占据栅格地图，<b class="pk">粉色虚线</b>是"里程计航位推算"的原始轨迹
     （会漂移），<b class="ok">绿色实线</b>是扫描匹配纠偏后的轨迹（SLAM 真正相信的位置）。
     每走过 1.5m，引擎做一次<b>相关性扫描匹配</b>把累计漂移拉回来——
     <b>注意看粉绿两线如何周期性"分叉又汇合"</b>：分叉 = 里程计漂移，汇合 = 匹配纠偏生效。</p>
  <canvas id="simcv" width="980" height="560"></canvas>
  <div class="ctrls">
    <button id="simPlay" onclick="SIM.toggle()">▶ 开始探索</button>
    <label>速度</label>
    <input type="range" id="simSpeed" min="1" max="6" step="1" value="3" oninput="document.getElementById('spdV').textContent=this.value+'×'">
    <span id="spdV" style="color:var(--dim);font-size:13px">3×</span>
    <button class="ghost" onclick="SIM.toggleRays()">激光射线</button>
    <button class="ghost" onclick="SIM.toggleTruth()">纠偏轨迹</button>
    <button class="ghost" onclick="SIM.toggleGrid()">占据栅格</button>
    <button class="ghost" onclick="SIM.newWorld()">换一个世界</button>
    <button class="ghost" onclick="SIM.reset()">重置</button>
  </div>
  <div class="readout" id="simOut"></div>
  <h3>这个仿真内部在做什么（与真实 slam_toolbox 一一对应）</h3>
  <table>
    <tr><th>仿真组件</th><th>数学</th><th>真实对应</th></tr>
    <tr><td>探索 AI（前进/避障转向/环扫）</td><td>状态机 + 扫描锥检测</td><td>建图遥操 / explore_lite（doc 20）</td></tr>
    <tr><td>360° 激光（120 束，含噪声）</td><td>射线-圆/线段求交</td><td>TB3 gpu_lidar（5Hz·360束）</td></tr>
    <tr><td>航位推算（粉色虚线轨迹）</td><td>v/ω 积分 + 漂移项 → 必然偏</td><td>轮式里程计（doc 21 §1）</td></tr>
    <tr><td>占据栅格生长（浅蓝灰格）</td><td>log-odds 逐格投票（doc 21 §3）</td><td>/map 的 OccupancyGrid</td></tr>
    <tr><td>扫描匹配纠偏（绿线汇合）</td><td>correlative 匹配，窗口 ±0.15m/±10°</td><td>slam_toolbox 前端（doc 21 §6）</td></tr>
    <tr><td>关键帧</td><td>每 1.5m 里程取一帧</td><td>minimum_travel_distance（doc 22）</td></tr>
  </table>
  <h3>地图上的"遮挡阴影"不是 bug</h3>
  <p>每个圆柱背后拖着的深色区域 = <b>激光遮挡阴影</b>：柱子挡住了后面的视野，
     那些格子从未被任何一束激光扫到，保持"未知"。真实 SLAM 地图同款。
     消掉它的方法只有：<b>机器人绕到柱子后面去</b>——这就是探索策略（覆盖完整性）的重要性。</p>
  <div class="warnbox">⚠ 观察要点：粉色虚线（航位推算）会慢慢偏离真实走过的路——这就是 doc 21 §1 手算的
     「轮径磨损 3% → 5m 漂 15cm」在动图里的样子。每次绿线与它分叉又汇合，
     就是扫描匹配在纠偏。把速度调到 6×，观察纠偏频率是否够用。</div>
  <details><summary>引擎实现细节（想深挖的看）</summary>
    <p>世界 = 六边形墙（6 条线段）+ 9 个圆柱（种子随机微扰）；激光 = 120 束射线，
    每束对全部线段/圆求交取最近（加微噪声）；栅格 = 136×136 @ 5cm，逐束沿光线采样写
    log-odds（束中 −0.4、束端 +1.2，限幅 ±4）；纠偏 = 每 1.5m 关键帧触发一次
    correlative 匹配（±0.15m/±0.15rad 窗口、0.075 步长，每 4 束取一个采样点，
    目标函数 = 扫描端点的 Σ log-odds，取最大）——与真实 slam_toolbox 前端同构，只是维度和窗口缩小。
    机器人带<b>硬碰撞约束</b>（半径 0.11 对墙/柱）——AI 失效也不会穿墙。</p>
  </details>
</div>
'''

SIM_SCRIPT = '''
<script>
const SIM=(()=>{
  const cv=document.getElementById('simcv'), ctx=cv.getContext('2d');
  const RES=0.05, GW=136, GH=136;
  const GOX=-3.4, GOY=-3.4;
  const MAXR=6.0, NRAYS=120;
  let world=null, running=false, speed=3, frame=0;
  let rob=null, deadTrail=[], corrTrail=[], keyframes=0, corrections=0, corrSum=0;
  let grid=null, showRays=true, showTruth=true, showGrid=true;
  function makeWorld(seed){
    const w={segs:[],circs:[]};
    const R=3.0, pts=[];
    for(let i=0;i<6;i++){ const a=i*Math.PI/3; pts.push([R*Math.cos(a),R*Math.sin(a)]); }
    for(let i=0;i<6;i++) w.segs.push({a:pts[i],b:pts[(i+1)%6]});
    const rs=(s=>()=> (s=(s*1664525+1013904223)%4294967296)/4294967296)(seed||12345);
    for(let ix=-1;ix<=1;ix++) for(let iy=-1;iy<=1;iy++){
      w.circs.push({x:ix*1.1+(rs()-.5)*.06, y:iy*1.1+(rs()-.5)*.06, r:0.15});
    }
    return w;
  }
  function rayCircle(ox,oy,dx,dy,c){
    const fx=ox-c.x, fy=oy-c.y;
    const b=fx*dx+fy*dy, cc=fx*fx+fy*fy-c.r*c.r;
    const disc=b*b-cc; if(disc<0) return Infinity;
    const t=-b-Math.sqrt(disc); return t>0.01?t:Infinity;
  }
  function raySeg(ox,oy,dx,dy,s){
    const ax=s.a[0],ay=s.a[1],bx=s.b[0],by=s.b[1];
    const ex=bx-ax,ey=by-ay;
    const den=dx*ey-dy*ex; if(Math.abs(den)<1e-12) return Infinity;
    const t=((ax-ox)*ey-(ay-oy)*ex)/den;
    const u=((ax-ox)*dy-(ay-oy)*dx)/den;
    if(t>0.01 && u>=0 && u<=1) return t;
    return Infinity;
  }
  function castRay(ox,oy,ang,maxr){
    const dx=Math.cos(ang), dy=Math.sin(ang); let t=maxr;
    for(const s of world.segs){ const tt=raySeg(ox,oy,dx,dy,s); if(tt<t)t=tt; }
    for(const c of world.circs){ const tt=rayCircle(ox,oy,dx,dy,c); if(tt<t)t=tt; }
    return t;
  }
  function collides(x,y){
    const RR=0.11;
    for(const c of world.circs) if(Math.hypot(x-c.x,y-c.y)<c.r+RR) return true;
    for(const s of world.segs){
      const ax=s.a[0],ay=s.a[1],bx=s.b[0],by=s.b[1];
      const ex=bx-ax,ey=by-ay,L2=ex*ex+ey*ey;
      let t=((x-ax)*ex+(y-ay)*ey)/L2; t=Math.max(0,Math.min(1,t));
      if(Math.hypot(x-(ax+t*ex),y-(ay+t*ey))<RR) return true;
    }
    return false;
  }
  function cellIdx(x,y){
    const c=Math.floor((x-GOX)/RES), r=Math.floor((y-GOY)/RES);
    return (c>=0&&c<GW&&r>=0&&r<GH)?{c,r}:null;
  }
  function gridAdd(x,y,val){
    const ci=cellIdx(x,y); if(!ci) return;
    const i=ci.r*GW+ci.c;
    grid.l[i]=Math.max(-4,Math.min(4,grid.l[i]+val));
  }
  function insertScan(pose,rays){
    const [x,y,th]=pose;
    for(const [ang,r] of rays){
      const rr=Math.min(r,MAXR-0.05);
      const steps=Math.floor(rr/RES);
      for(let s=0;s<steps;s++){
        const d=(s+0.5)*RES;
        gridAdd(x+Math.cos(th+ang)*d, y+Math.sin(th+ang)*d, -0.4);
      }
      gridAdd(x+Math.cos(th+ang)*rr, y+Math.sin(th+ang)*rr, +1.2);
    }
  }
  function reset(){
    world=makeWorld(Math.floor(Math.random()*1e6));
    grid={l:new Float32Array(GW*GH)};
    rob={x:-2.0,y:-0.5,th:0, dx:-2.0,dy:-0.5,dth:0,
         lastKF:0, state:'DRIVE', turnRem:0, stuckT:performance.now(),
         backUntil:0, winPos:[-2.0,-0.5], winT:performance.now(), stuck:0, turnT0:0};
    deadTrail=[]; corrTrail=[]; keyframes=0; corrections=0; corrSum=0; frame=0;
    const rays=scanRays(rob);
    insertScan([rob.x,rob.y,rob.th],rays);
    corrTrail.push([rob.x,rob.y]); deadTrail.push([rob.dx,rob.dy]);
  }
  function scanRays(r){
    const rays=[];
    for(let i=0;i<NRAYS;i++){
      const ang=-Math.PI + i*(2*Math.PI/NRAYS);
      const noisy=1+(Math.sin(frame*13+i*7)*0.004);
      rays.push([ang, castRay(r.x,r.y,r.th+ang,MAXR)*noisy]);
    }
    return rays;
  }
  function matchCorrect(rays,drPose){
    let best=null;
    for(let ddx=-0.15;ddx<=0.151;ddx+=0.075)
    for(let ddy=-0.15;ddy<=0.151;ddy+=0.075)
    for(let dt=-0.15;dt<=0.151;dt+=0.075){
      const px=drPose[0]+ddx, py=drPose[1]+ddy, pt=drPose[2]+dt;
      let sc=0;
      for(let i=0;i<rays.length;i+=4){
        const ang=rays[i][0], r=rays[i][1];
        const ci=cellIdx(px+Math.cos(pt+ang)*r, py+Math.sin(pt+ang)*r);
        if(ci) sc+=grid.l[ci.r*GW+ci.c];
      }
      if(!best || sc>best.sc) best={sc,x:px,y:py,th:pt};
    }
    return best;
  }
  function step(dt){
    const r=rob;
    let v=0.22, om=0;
    const rays=scanRays(r);
    let front=Infinity, left=Infinity, right=Infinity, bestAng=0, bestR=0;
    rays.forEach(p=>{
      const deg=p[0]*180/Math.PI, rr=p[1];
      if(rr>bestR){ bestR=rr; bestAng=p[0]; }
      if(Math.abs(deg)<32) front=Math.min(front,rr);
      else if(deg>=35&&deg<95) left=Math.min(left,rr);
      else if(deg<=-35&&deg>-95) right=Math.min(right,rr);
    });
    const now=performance.now();
    const moved=Math.hypot(r.x-r.winPos[0], r.y-r.winPos[1]);
    if(now-r.winT>3000){
      if(moved<0.25){ r.stuck=(r.stuck||0)+1;
        if(r.stuck>=2){ r.backUntil=now+800; r.stuck=0; } }
      else r.stuck=0;
      r.winPos=[r.x,r.y]; r.winT=now;
    }
    const stuck = moved<0.15 && now-r.winT>2500;
    if(r.state==='DRIVE'){
      if(stuck || (front<0.38 && r.lastKF>0)){
        const dir = bestR>0.8 ? Math.sign(bestAng||1) : (left<=right? -1 : 1);
        r.turnRem = dir*((70+Math.random()*70)*Math.PI/180);
        r.state='TURN'; r.turnT0=now;
      } else { v=Math.min(0.22, Math.max(0.05,(front-0.25))); }
    }
    if(r.state==='TURN'){
      if(now<(r.backUntil||0)) v=-0.15;
      const stepRot=Math.min(0.9*dt, Math.abs(r.turnRem));
      om=Math.sign(r.turnRem)*stepRot/dt;
      r.turnRem-=Math.sign(r.turnRem)*stepRot;
      if(Math.abs(r.turnRem)<0.03 && now>(r.backUntil||0)){
        r.state='DRIVE'; r.winPos=[r.x,r.y]; r.winT=now;
      }
      if(now-r.turnT0>4000){ r.turnRem=0; r.state='DRIVE'; r.winPos=[r.x,r.y]; r.winT=now; }
    }
    const nx=r.x+v*Math.cos(r.th)*dt, ny=r.y+v*Math.sin(r.th)*dt;
    if(!collides(nx,ny)){ r.x=nx; r.y=ny; }
    r.th+=om*dt;
    const drift=1+Math.sin(frame*0.7)*0.006;
    r.dx+=v*Math.cos(r.dth)*dt*drift;
    r.dy+=v*Math.sin(r.dth)*dt*drift;
    r.dth+=om*dt;
    if(r.state==='DRIVE') r.lastKF+=v*dt;
    if(r.lastKF>=1.5 || (r.state==='TURN'&&r.lastKF>0.8)){
      keyframes++;
      const best=matchCorrect(rays,[r.dx,r.dy,r.dth]);
      if(best){
        const err=Math.hypot(best.x-r.dx,best.y-r.dy);
        corrSum+=err; corrections++;
        r.dx=best.x; r.dy=best.y; r.dth=best.th;
        r.lastKF=0;
        insertScan([r.x,r.y,r.th],rays);
      }
    } else { insertScan([r.x,r.y,r.th],rays); }
    deadTrail.push([r.dx,r.dy]); if(deadTrail.length>3000) deadTrail=deadTrail.filter((_,i)=>i%2===0);
    corrTrail.push([r.x,r.y]);   if(corrTrail.length>3000) corrTrail=corrTrail.filter((_,i)=>i%2===0);
    frame++;
  }
  function draw(){
    ctx.clearRect(0,0,cv.width,cv.height);
    const scale=Math.min(cv.width,cv.height)/(RES*GW);
    const mx=(cv.width-RES*GW*scale)/2;
    const my=(cv.height-RES*GH*scale)/2;
    const x2px=v=>mx+(v-GOX)*scale;
    const y2px=v=>my+(v-GOY)*scale;
    if(showGrid){
      const off=document.createElement('canvas'); off.width=GW; off.height=GH;
      const octx=off.getContext('2d'), img=octx.createImageData(GW,GH);
      for(let i=0;i<GW*GH;i++){
        const p=1/(1+Math.exp(-grid.l[i]));
        let R,G,B;
        if(p<0.35){ R=126;G=142;B=172; }
        else if(p<0.65){ R=17;G=22;B=36; }
        else { R=251;G=113;B=133; }
        img.data[i*4]=R; img.data[i*4+1]=G; img.data[i*4+2]=B; img.data[i*4+3]=230;
      }
      octx.putImageData(img,0,0);
      ctx.imageSmoothingEnabled=false;
      ctx.drawImage(off, x2px(GOX), y2px(GOY), RES*GW*scale, RES*GH*scale);
      ctx.imageSmoothingEnabled=true;
    }
    ctx.strokeStyle='rgba(255,107,129,0.55)'; ctx.lineWidth=2;
    for(const s of world.segs){ ctx.beginPath();
      ctx.moveTo(x2px(s.a[0]),y2px(s.a[1])); ctx.lineTo(x2px(s.b[0]),y2px(s.b[1])); ctx.stroke(); }
    for(const c of world.circs){ ctx.beginPath();
      ctx.arc(x2px(c.x),y2px(c.y),c.r*scale,0,7); ctx.stroke(); }
    ctx.lineWidth=1;
    if(showTruth){ ctx.strokeStyle='rgba(52,211,153,0.95)'; ctx.lineWidth=2; ctx.beginPath();
      corrTrail.forEach(([x,y],i)=>{ const px=x2px(x),py=y2px(y); i?ctx.lineTo(px,py):ctx.moveTo(px,py); });
      ctx.stroke(); ctx.lineWidth=1; }
    ctx.strokeStyle='rgba(244,114,182,0.85)'; ctx.lineWidth=1.5; ctx.setLineDash([4,3]);
    ctx.beginPath(); deadTrail.forEach(([x,y],i)=>{ const px=x2px(x),py=y2px(y); i?ctx.lineTo(px,py):ctx.moveTo(px,py); });
    ctx.stroke(); ctx.setLineDash([]); ctx.lineWidth=1;
    if(showRays){
      const rays=scanRays(rob);
      ctx.strokeStyle='rgba(251,191,36,0.25)';
      rays.forEach(p=>{ ctx.beginPath();
        ctx.moveTo(x2px(rob.x),y2px(rob.y));
        ctx.lineTo(x2px(rob.x+p[1]*Math.cos(rob.th+p[0])), y2px(rob.y+p[1]*Math.sin(rob.th+p[0])));
        ctx.stroke(); });
    }
    const rx=x2px(rob.x), ry=y2px(rob.y);
    ctx.save(); ctx.translate(rx,ry); ctx.rotate(rob.th);
    ctx.fillStyle='#22d3ee'; ctx.beginPath();
    ctx.moveTo(10,0); ctx.lineTo(-7,6); ctx.lineTo(-7,-6); ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.fillStyle='#f472b6'; ctx.beginPath();
    ctx.arc(x2px(rob.dx),y2px(rob.dy),4,0,7); ctx.fill();
    ctx.font='12px sans-serif'; ctx.textAlign='left';
    const ly=cv.height-46;
    ctx.fillStyle='#22d3ee'; ctx.fillText('▲ 机器人(真值)', 20, ly);
    ctx.fillStyle='#f472b6'; ctx.fillText('● 航位推算(漂移)', 150, ly);
    ctx.fillStyle='#34d399'; ctx.fillText('── 纠偏后轨迹', 310, ly);
    ctx.fillStyle='#fbbf24'; ctx.fillText('── 激光射线', 440, ly);
    ctx.fillStyle='#fb7185'; ctx.fillText('■ 占据(墙/柱)', 560, ly);
    ctx.fillStyle='#7e8eac'; ctx.fillText('▢ 自由', 675, ly);
    ctx.fillStyle='#546079'; ctx.fillText('▨ 未知/遮挡阴影', 745, ly);
    const stat=document.getElementById('simOut');
    const avg=corrections?(corrSum/corrections).toFixed(3):'—';
    const dr=Math.hypot(rob.dx-rob.x,rob.dy-rob.y).toFixed(3);
    let covered=0; for(let i=0;i<grid.l.length;i++) if(Math.abs(grid.l[i])>0.3) covered++;
    stat.textContent=
`时间帧 ${frame}   关键帧 ${keyframes}   扫描匹配纠偏 ${corrections} 次（平均修正 ${avg} m）
当前航位推算漂移 ${dr} m —— 粉虚线与绿线的分叉-汇合 = 漂移与纠偏的循环
已覆盖栅格 ${covered} / ${GW*GH}`;
  }
  setInterval(function tick(){
    if(!running) return;
    for(let i=0;i<speed;i++) step(0.05);
    draw();
  }, 50);
  return {
    toggle(){ running=!running;
      document.getElementById('simPlay').textContent=running?'⏸ 暂停':'▶ 继续探索';
      if(running) draw(); },
    toggleRays(){ showRays=!showRays; draw(); },
    toggleTruth(){ showTruth=!showTruth; draw(); },
    toggleGrid(){ showGrid=!showGrid; draw(); },
    newWorld(){ reset(); draw(); },
    reset(){ reset(); draw(); },
    ensure(){ if(!world){ reset(); } draw(); },
    draw(){ draw(); }
  };
})();
SIM.ensure();
</script>
'''

# ============================================================
# 模块 1：占据栅格
# ============================================================
OG_BODY = '''
<div class="card">
  <h2><span class="no">①</span>占据栅格建图：log-odds 投票（详细版）</h2>
  <div class="sub">已知位姿怎么画地图？—— 从"扔硬币"推导到完整数值表</div>
  <h3>1.1 直觉：投票箱</h3>
  <p>激光一束就是一次"探索"：<b style="color:var(--bad)">光束打到的地方有墙，光束路过的地方没墙</b>。
  但传感器有噪声——这一次说"有墙"，下一次可能说"没墙"。所以不能非黑即白，
  每个格子维护一个<b>占据概率</b>，像投票箱一样累积证据：
  占据票多 → 概率升；自由票多 → 概率降；票数对冲 → 回到中间。</p>
  <p>问题：概率的更新是<b>乘法</b>（贝叶斯：新概率 ∝ 旧概率 × 观测似然），
  连乘一堆 0.x 的小数会<b>数值下溢</b>，而且"证据对冲"的加减关系在乘法里很难看清楚。
  解法：换一个坐标系——<b>log-odds</b>。</p>
  <h3>1.2 log-odds：把乘法变加法</h3>
  <div class="formula">定义：  l = ln( p / (1−p) )                （"是"的几率取对数）
互逆：  p = 1 / (1 + e^(−l))

对照表：
  p=0.50 → l=0            （完全未知）
  p=0.70 → l=ln(7/3)=0.847（一次"占据"票）
  p=0.30 → l=−0.847       （一次"自由"票）
  p=0.845 → l=1.694       （两次占据票）
  p=0.927 → l=2.540       （三次占据票）</div>
  <p>为什么妙：贝叶斯更新 p_new ∝ p_old × P(z|cell) 在 l 坐标里变成
  <b>l_new = l_old + ln(P(z|cell)/(1−P(z|cell)))</b> —— 乘法变加法，
  "证据对冲"变成正负数相加，数值上还不会下溢。工程与优雅兼得。</p>
  <h3>1.3 反演传感器模型（一束激光留下什么信息）</h3>
  <div class="formula">光束终点附近的格子（±1 格）→ 被占据    P(occ)=p_occ=0.70 → l=+0.847
光束沿途穿过的格子        → 是自由的 P(occ)=p_free=0.30 → l=−0.847
光束没扫到的格子          → 不更新     P(occ)=0.50   → l 不变</div>
  <p>为什么叫"反演"：正向模型是"已知地图，预测传感器会读到什么"（用于定位）；
  这里反过来用——"已知读数，推断地图"。</p>
  <h3>1.4 完整手算：一面墙的诞生（12 格 1D 世界）</h3>
  <p>机器人站格子 2，朝右发 6 格激光束 → 束中格 3-7 得自由票，束端格 8 得占据票。
  连测 3 次，然后一束噪声假回波（束端变成格 5）：</p>
  <table>
    <tr><th>格子</th><th>初始 l/p</th><th>束1 后</th><th>束2 后</th><th>束3 后</th><th>噪声束后</th></tr>
    <tr><td>3（束中）</td><td>0 / 0.50</td><td>−0.847 / 0.30</td><td>−1.695 / 0.155</td><td>−2.540 / 0.073</td><td>+0.847 → −1.693 / 0.155</td></tr>
    <tr><td>5（束中）</td><td>0 / 0.50</td><td>−0.847 / 0.30</td><td>−1.695 / 0.155</td><td>−2.540 / 0.073</td><td><b style="color:var(--ok)">−1.695 / 0.155</b></td></tr>
    <tr><td>7（束中）</td><td>0 / 0.50</td><td>−0.847 / 0.30</td><td>−1.695 / 0.155</td><td>−2.540 / 0.073</td><td>−2.540 / 0.073（噪声束没扫到它）</td></tr>
    <tr><td>8（墙）</td><td>0 / 0.50</td><td>+0.847 / 0.70</td><td>+1.695 / 0.845</td><td><b style="color:var(--ok)">+2.540 / 0.927</b></td><td>不变 / 0.927</td></tr>
  </table>
  <p>读表格的三条线：<b>① 墙格</b>指数逼近 0.927 但永不到 1（每次证据都在打折扣）；
  <b>② 噪声格 5</b>：之前两次"自由"（−1.695）+ 一次错误"占据"（+0.847）= −1.695 ——
  <b>证据完全对冲，错误被洗掉</b>；<b>③ 格子 7</b>：噪声束没扫到它，它保持原值——没证据就不动。</p>
  <h3>1.5 交互演示</h3>
  <canvas id="ogcv" width="980" height="190"></canvas>
  <div class="ctrls">
    <button onclick="OG.beam(0)">发射正常束</button>
    <button onclick="OG.beam(1)">发射噪声束</button>
    <button onclick="OG.reset()">重置</button>
  </div>
  <div class="readout" id="ogOut"></div>
  <h3>1.6 参数对应（真实系统）</h3>
  <table>
    <tr><th>本演示的概念</th><th>slam_toolbox 参数</th><th>作用</th></tr>
    <tr><td>0.7/0.3 的票强度</td><td>（内置于栅格更新）</td><td>票太强 → 噪声焊死格子；太弱 → 收敛慢</td></tr>
    <tr><td>p&gt;阈值 判占据</td><td>occupancy_threshold: 0.1</td><td>输出 /map 的黑白判定</td></tr>
    <tr><td>穿过 ≥2 次才算自由</td><td>min_pass_through: 2</td><td>抗单次噪声（证据对冲的输出端）</td></tr>
    <tr><td>最大建图距离</td><td>max_laser_range: 20</td><td>远端点不可信，截断</td></tr>
  </table>
  <h3>1.7 常见误区</h3>
  <p>① <b>一次观测焊死格子</b>：用 p=0.99/0.01 的票 → 噪声永久污染（doc 22 抗干扰的反例）；
  ② <b>把未知当自由</b>：没扫到的格子不能涂白——探索不完整时"未知"必须保留（导航规划对待未知区要保守）；
  ③ <b>直接乘概率</b>：数值下溢 + 无法对冲，log-odds 不是炫技是刚需。</p>
  <details><summary>练习 1：某格子 2 次占据票、3 次自由票，最终 p=?</summary>
    <p>l = 2×0.847 − 3×0.847 = −0.847 → p = 1/(1+e^0.847) = 1/2.333 = <b>0.30</b>。
    恰好等于单次自由票的模型值——2 占 3 自由 = 净 1 自由票。</p>
  </details>
  <details><summary>练习 2：为什么 P(occ) 永远到不了 1？</summary>
    <p>l = l₀ + n×0.847 有限 → p = σ(l) &lt; 1。只有"无限次一致观测"才逼近 1。
    这是贝叶斯谦虚性：任何有限证据都不能给出绝对确定——给绝对确定会让后续
    矛盾观测永远无法修正（p=1 时 l=∞，加任何负票都拉不回来）。</p>
  </details>
</div>
'''

OG_SCRIPT = '''
<script>
const OG=(()=>{
  const cv=document.getElementById('ogcv'), ctx=cv.getContext('2d');
  const N=12, ROBOT=2, BEAM=6;
  let l=new Array(N).fill(0), pocc=0.7, count=0, lastEnd=null;
  function pOf(i){ return 1/(1+Math.exp(-l[i])); }
  function draw(){
    ctx.clearRect(0,0,cv.width,cv.height);
    const pad=46, w=(cv.width-2*pad)/N, y0=64, h=52;
    for(let i=0;i<N;i++){
      const p=pOf(i), x=pad+i*w;
      ctx.fillStyle = p<0.35?`rgba(79,163,255,${0.25+0.5*(p/0.35)})`
                    : p<0.65?`rgba(100,110,140,0.5)`
                    : `rgba(255,107,129,${0.4+0.6*((p-0.65)/0.35)})`;
      ctx.fillRect(x+2,y0,w-4,h);
      ctx.strokeStyle='#2a3550'; ctx.strokeRect(x+2,y0,w-4,h);
      ctx.fillStyle='#dbe4f5'; ctx.font='11px monospace'; ctx.textAlign='center';
      ctx.fillText(p.toFixed(2), x+w/2, y0+h+16);
      ctx.fillText('l='+l[i].toFixed(2), x+w/2, y0+h+32);
      if(i===ROBOT){ ctx.fillStyle='#3ddc84'; ctx.beginPath();
        ctx.arc(x+w/2,y0-14,7,0,7); ctx.fill();
        ctx.fillText('机器人', x+w/2, y0-30); }
    }
    if(lastEnd!==null){
      const xr=pad+ROBOT*w+w/2, xe=pad+lastEnd*w+w/2;
      ctx.strokeStyle='rgba(251,191,36,0.9)'; ctx.setLineDash([6,4]); ctx.lineWidth=2;
      ctx.beginPath(); ctx.moveTo(xr,y0+h/2); ctx.lineTo(xe,y0+h/2); ctx.stroke();
      ctx.setLineDash([]); ctx.lineWidth=1;
      ctx.fillStyle='#ffb454'; ctx.beginPath(); ctx.arc(xe,y0+h/2,6,0,7); ctx.fill();
    }
    ctx.fillStyle='#8b98b8'; ctx.font='12px sans-serif'; ctx.textAlign='left';
    ctx.fillText('12 格 1D 世界 · 格色 = 占据概率（蓝=自由 灰=未知 红=占据）', pad, 24);
  }
  function beam(noise){
    count++;
    const end = noise? ROBOT+3 : ROBOT+BEAM;
    const name= noise? '噪声假回波（束端=格5）' : '正常回波（束端=格8）';
    const lo=Math.log(pocc/(1-pocc)), lf=Math.log((1-pocc)/pocc);
    const rows=[];
    for(let c=0;c<N;c++){
      if(c===end){ l[c]+=lo; rows.push(`格${c}: l=${l[c].toFixed(3)} p=${pOf(c).toFixed(3)} 【束端→占据】`); }
      else if(c>ROBOT&&c<end){ l[c]+=lf; rows.push(`格${c}: l=${l[c].toFixed(3)} p=${pOf(c).toFixed(3)} 【束中→自由】`); }
    }
    lastEnd=end;
    draw();
    document.getElementById('ogOut').textContent=
      `第 ${count} 束（${name}）\\n`+rows.join('\\n')
      +`\\n墙格8 当前: l=${l[8].toFixed(3)} → p=${pOf(8).toFixed(3)}`
      +`\\n噪声对冲演示：格5 若先累积 2 次自由（l=-1.694），一次误占（+0.847）→ 回到 -0.847（p=0.30）`;
  }
  function reset(){ l=new Array(N).fill(0); count=0; lastEnd=null; draw();
    document.getElementById('ogOut').textContent='已重置（全部 l=0, p=0.5）'; }
  reset();
  return { beam, reset };
})();
</script>
'''

# ============================================================
# 模块 2：贝叶斯滤波
# ============================================================
BF_BODY = '''
<div class="card">
  <h2><span class="no">②</span>贝叶斯滤波：噪声世界里合并证据（详细版）</h2>
  <div class="sub">信念是分布不是一个数 —— 两步循环：预测（我动了）+ 观测更新（我看到了）</div>
  <h3>2.1 直觉：为什么必须用分布</h3>
  <p>确定性思维说"机器人在 (2.3, 1.1)"。但轮子会打滑、激光有噪声、地图有误差——
  你对位置的知识<b>永远不完美</b>。硬给一个数 = 假装自己不糊涂，而且没法表达
  "我有 60% 把握在这、30% 把握在那"。正确表达是<b>概率分布</b>——
  SLAM 全文都在回答一个问题：这个分布怎么随「我动了」「我看到了」而更新？</p>
  <h3>2.2 场景设定</h3>
  <p>三格走廊，格子 1、3 有门。传感器只能报"门/非门"：
  P(报门|真有门)=<b>0.8</b>（正常），P(报门|没门)=<b>0.2</b>（误报率）。
  运动模型：命令右移一格 → <b>80%</b> 成功右移、<b>20%</b> 打滑原地；
  格子 3 的右墙挡住，向右命令在格子 3 等价于原地。</p>
  <div class="formula"> ┌─────┬─────┬─────┐
 │ 门1 │     │ 门3 │      初始信念（完全不知道）：
 │ (1) │ (2) │ (3) │      bel = [1/3, 1/3, 1/3]
 └─────┴─────┴─────┘</div>
  <h3>2.3 两步循环的数学</h3>
  <div class="formula">预测（运动更新）：  bel_bar(x) = ∫ P(x|u,x') · bel(x') dx'
                     —— 运动模型把信念"摊开"（卷积）
更新（观测）：      bel(x) = η · P(z|x) · bel_bar(x)
                     —— 传感器模型重加权（η = 归一化常数）</div>
  <p>所有 SLAM 算法都是这两步的不同实现——区别只在<b>信念怎么表示</b>：
  高斯 → EKF；一群粒子 → 粒子滤波；位姿图 → 图优化。</p>
  <h3>2.4 完整手算（四步表格）</h3>
  <table>
    <tr><th>步骤</th><th>格子1</th><th>格子2</th><th>格子3</th><th>发生了什么</th></tr>
    <tr><td>初始（完全不知道）</td><td>33.3%</td><td>33.3%</td><td>33.3%</td><td>均匀先验</td></tr>
    <tr><td>观测1「报门」</td><td><b>44.4%</b></td><td>11.1%</td><td><b>44.4%</b></td><td><b style="color:var(--wa)">多峰！门1门3平分——世界对称无法区分</b></td></tr>
    <tr><td>预测「右移」</td><td>8.9%</td><td>37.8%</td><td>53.3%</td><td>两个峰被运动摊开</td></tr>
    <tr><td>观测2「报门」</td><td>12.4%</td><td>13.2%</td><td><b style="color:var(--ok)">74.4%</b></td><td><b>锁定格子 3</b>——一次运动+一次观测打破对称</td></tr>
  </table>
  <p><b>逐步展开（观测1）</b>：未归一化 = 旧信念 × 传感器模型——
  格子1：0.333×0.8(有门)=0.2667；格子2：0.333×0.2(没门)=0.0667；格子3：0.333×0.8=0.2667。
  η = 0.6 → 归一化：0.444 / 0.111 / 0.444。</p>
  <p><b>逐步展开（预测）</b>：概率流——格子1 的 0.444 有 80% 流向格子 2、20% 留原地；
  格子 3 的 0.444 因右墙 100% 留下。汇总：格子1 = 0.0889；
  格子2 = 0.3556+0.0222 = 0.3778；格子3 = 0.0889+0.4444 = 0.5333。</p>
  <h3>2.5 交互演示</h3>
  <canvas id="bfcv" width="980" height="250"></canvas>
  <div class="ctrls">
    <button onclick="BF.measure()">观测（报门）</button>
    <button onclick="BF.move()">右移一格</button>
    <button class="ghost" onclick="BF.reset()">重置</button>
    <button class="ghost" onclick="BF.auto()">自动完整序列</button>
  </div>
  <div class="readout" id="bfOut"></div>
  <h3>2.6 关键读数与直觉</h3>
  <p><b style="color:var(--wa)">多峰</b>：只测一次"门"，门 1 和门 3 无法区分——分布诚实地保留两个假设。
  这不是算法失败，是信息不足的真实反映。<b>打破对称靠运动</b>：右移一格后，
  两个峰的相对位置变化不同（一个流走、一个留下），第二次观测就能区分。
  <b>这就是"边走边测"的价值</b>——单独的观测或单独的运动都不够，两者交替才收敛。</p>
  <div class="note">📌 AMCL（本仓库一直在用的定位器）= 这一套的粒子版：
  把连续分布换成 N 个粒子（每个粒子是一个位置假设），粒子密度就是信念。
  见模块 4。</div>
  <h3>2.7 常见误区</h3>
  <p>① 忘记归一化（η）——信念和不再是 1，后面全部失真；
  ② 运动模型忘记"撞墙=原地"的边界处理——概率泄漏到世界之外；
  ③ 误报率设 0——传感器永不犯错是不现实的，设 0 会让一次观测把信念"焊死"在错误格子。</p>
  <details><summary>练习：预测步手算（先自己算）</summary>
    <p>信念 [0.444, 0.111, 0.444]，命令右移一格（80% 成功/20% 原地，格 3 撞墙=原地）。
    预测后各格信念？</p>
    <p>答案：格子1 = 0.444×0.2 = 0.0889；格子2 = 0.444×0.8 + 0.111×0.2 = 0.3778；
    格子3 = 0.111×0.8 + 0.444×1.0 = 0.5333。和 = 1.0 ✓（概率守恒）。
    用"观测1 后"的数值接着算，与表中"预测「右移」"行一致。</p>
  </details>
</div>
'''

BF_SCRIPT = '''
<script>
const BF=(()=>{
  const cv=document.getElementById('bfcv'), ctx=cv.getContext('2d');
  const doors=[true,false,true];
  let bel=[1/3,1/3,1/3], step=0, msg='';
  const P_MOVE=0.8,P_STAY=0.2,PDD=0.8,PDN=0.2;
  function draw(){
    ctx.clearRect(0,0,cv.width,cv.height);
    const w=200,gap=120,x0=(cv.width-3*w-2*gap)/2,y0=56,h=70;
    ctx.font='13px sans-serif';
    for(let i=0;i<3;i++){
      const x=x0+i*(w+gap);
      ctx.fillStyle='#1e2740'; ctx.fillRect(x,y0,w,h);
      ctx.strokeStyle='#2a3550'; ctx.strokeRect(x,y0,w,h);
      if(doors[i]){ ctx.fillStyle='#ffb454'; ctx.font='22px sans-serif'; ctx.textAlign='center';
        ctx.fillText('⌂',x+w/2,y0+34); ctx.font='11px sans-serif'; ctx.fillText('门',x+w/2,y0+52); }
      ctx.fillStyle='#dbe4f5'; ctx.font='13px sans-serif';
      ctx.fillText(`格子 ${i+1}`,x+w/2,y0-10);
      const bh=bel[i]*130;
      ctx.fillStyle='rgba(79,163,255,0.85)'; ctx.fillRect(x+20,y0+h+10+(130-bh),w-40,bh);
      ctx.strokeStyle='#2a3550'; ctx.strokeRect(x+20,y0+h+10,w-40,130);
      ctx.fillStyle='#4fa3ff'; ctx.font='13px monospace';
      ctx.fillText((bel[i]*100).toFixed(1)+'%',x+w/2,y0+h+10+130+18);
    }
    ctx.fillStyle='#8b98b8'; ctx.textAlign='left';
    ctx.fillText('信念条 = 机器人对各格子的置信度',x0,24);
    if(msg){ ctx.fillStyle='#ffb454'; ctx.font='14px sans-serif'; ctx.fillText(msg,x0,42); }
  }
  function show(m){ msg=m; draw();
    document.getElementById('bfOut').textContent=m+'\\n当前信念: ['
      +bel.map(b=>(b*100).toFixed(1)+'%').join(', ')+']'; }
  return {
    measure(){ step++;
      const unn=bel.map((b,i)=>b*(doors[i]?PDD:PDN));
      const eta=unn.reduce((a,b)=>a+b,0); bel=unn.map(u=>u/eta);
      show(`第${step}步 观测（报门）——未归一化 [${unn.map(u=>u.toFixed(3)).join(', ')}], η=${eta.toFixed(3)}`); },
    move(){ step++;
      const pred=[0,0,0];
      bel.forEach((b,i)=>{ const d=Math.min(i+1,2); pred[d]+=b*P_MOVE; pred[i]+=b*P_STAY; });
      bel=pred; show(`第${step}步 预测（右移一格：80%成功/20%打滑；格3右墙=原地）`); },
    reset(){ bel=[1/3,1/3,1/3]; step=0; show('重置（均匀先验：完全不知道在哪）'); },
    auto(){ this.reset();
      [()=>this.measure(),()=>this.move(),()=>this.measure()].forEach((f,i)=>setTimeout(f,800*(i+1))); },
    draw(){ draw(); }
  };
})();
BF.reset();
</script>
'''

EXTRA_PAGES = {
    'sim.html': ('仿真实验室', SIM_BODY, SIM_SCRIPT),
    'og.html': ('① 占据栅格建图', OG_BODY, OG_SCRIPT),
    'bf.html': ('② 贝叶斯滤波', BF_BODY, BF_SCRIPT),
}

# ============================================================
# 模块 3：EKF
# ============================================================
EKF_BODY = '''
<div class="card">
  <h2><span class="no">③</span>EKF-SLAM：高斯世界的"同时"估计（详细版）</h2>
  <div class="sub">机器人 + 路标打包进一个高斯分布；观测让不确定性收缩、并诞生"关联"</div>
  <h3>3.1 直觉：蒙眼摸消防栓</h3>
  <p>蒙眼被带进一栋楼，走了几步——<b>预测：你对位置更糊涂了</b>（不确定性膨胀）。
  摸到一个"消防栓"——<b>更新：所有和消防栓有关的位置推理都变准了</b>（收缩），
  而且你对"自己走了多远"和"消防栓在哪"的判断从此<b>绑定</b>（关联诞生）。</p>
  <p>EKF 用<b>一个高斯分布</b>（均值 + 协方差矩阵 Σ）描述对全部状态的认知：
  均值 = "最可能在哪"，协方差 = "不确定性多大 + 哪些量相互关联"。</p>
  <h3>3.2 为什么需要"扩展"（E）KF</h3>
  <p>运动和观测往往是<b>非线性</b>的（角度、三角函数），高斯过非线性函数后不再是高斯。
  EKF 的"E"= 在当前估计点做<b>线性化</b>：用雅可比矩阵（偏导数）把非线性函数
  近似成直线，然后套用线性卡尔曼公式。本篇的例子里观测方程 z = 路标 − 机器人
  恰好是线性的，所以 H 是常数矩阵——真实 SLAM 里 H 每步都要重算。</p>
  <h3>3.3 公式与完整手算</h3>
  <div class="formula">状态 x = [机器人位置, 路标位置]      先验 x̄=[1.0, 3.0]，Σ=[[1.5, 0], [0, 0.5]]

预测（机器人前进 u=1，运动噪声 σ²=0.5；路标不动）：
  x̄ = [2.0, 3.0]                    （均值平移）
  Σ̄ = [[1.5+0.5, 0], [0, 0.5]]      （膨胀：只有机器人的方差变大）

更新（观测 z = 路标 − 机器人 = 1.2，观测噪声 R=0.25）：
  H = [−1, +1]                       （z 对 x 求导 −1、对 m 求导 +1）
  残差 = z − H·x̄ = 1.2 − 1.0 = 0.2
  S = H·Σ̄·Hᵀ + R = 2.0 + 0.5 + 0.25 = 2.75
  K = Σ̄·Hᵀ/S = [−2.0, 0.5]/2.75 = [−0.727, +0.182]
  x = x̄ + K×0.2 = [1.855, 3.036]
  Σ = (I − K·H)·Σ̄ = [[0.545, 0.364], [0.364, 0.409]]</div>
  <h3>3.4 卡尔曼增益的直觉：谁更准听谁的</h3>
  <p>把"观测推断出的机器人位置"（路标 3.0 − 观测 1.2 = 1.8，方差 0.25）
  与"运动预测"（2.0，方差 2.0）做精度加权平均：</p>
  <div class="formula">融合 = (2.0/2.0 + 1.8/0.25) / (1/2.0 + 1/0.25) = (1.0 + 7.2) / 4.5 = 1.822
—— 观测方差只有 0.25（准），运动方差 2.0（糙）→ 融合结果偏向观测。
（上面是"路标完全已知"的简化情形；演示里路标也有不确定性 0.5，
  联合更新给出 1.855——路标的不确定性"传染"给了机器人位置。）</div>
  <h3>3.5 交互演示（拖动噪声滑块，看增益与收缩实时变化）</h3>
  <canvas id="ekfcv" width="980" height="260"></canvas>
  <div class="ctrls">
    <label>运动噪声 σ²=<span id="ekmV">0.50</span></label>
    <input type="range" id="ekm" min="0.05" max="3" step="0.05" value="0.5" oninput="EKF.draw()">
    <label>观测噪声 R=<span id="ekrV">0.25</span></label>
    <input type="range" id="ekr" min="0.05" max="2" step="0.05" value="0.25" oninput="EKF.draw()">
    <button class="ghost" onclick="EKF.reset()">重置</button>
  </div>
  <div class="readout" id="ekfOut"></div>
  <h3>3.6 关键读数</h3>
  <p>机器人方差 <b>2.0 → 0.545</b>（观测吸走不确定性）；
  交叉协方差 <b>0.364 ≠ 0</b> —— 机器人和路标从此"关联"；
  路标估计 3.0 → 3.036 —— <b>机器人的位置不确定性"传染"给了路标</b>。</p>
  <h3>3.7 EKF-SLAM 的宿命：相关性链条与规模爆炸</h3>
  <p>每观测一个新路标，它就通过机器人位姿与<b>所有旧路标</b>产生关联——
  协方差矩阵几乎全满。n 个路标 → O(n²) 矩阵，1000 个路标 = 100 万个数每次更新。
  这就是 EKF-SLAM 被淘汰的原因。但它的<b>相关性洞察</b>统治了后续所有 SLAM：
  一旦回环，全图一起校正（模块 6 会再见到这一幕）。</p>
  <h3>3.8 常见误区</h3>
  <p>① 忘记预测步的方差膨胀——只更新均值会让机器人"过度自信"；
  ② 把 H 当常数用在强非线性系统——线性化只在估计点附近有效；
  ③ 忘记路标初始化（第一次观测到新路标时要用观测值初始化它的均值与方差）。</p>
  <details><summary>练习：路标完全已知时，机器人方差收缩到多少？</summary>
    <p>S = 2.0 + 0.25 = 2.25，K = [−2.0, 0]/2.25 = [−0.889, 0]。
    I − KH = [[0.111, 0.889], [0, 1]] → Σ' = [[0.222, 0], [0, 0]]。
    机器人方差 2.0 → <b>0.222</b>。直觉校验（精度加权）：
    1/(1/2.0 + 1/0.25) = 0.222 ✓。</p>
  </details>
</div>
'''

EKF_SCRIPT = '''
<script>
const EKF=(()=>{
  const cv=document.getElementById('ekfcv'), ctx=cv.getContext('2d');
  const sm=document.getElementById('ekm'), sr=document.getElementById('ekr');
  function calc(){
    const Q=+sm.value, R=+sr.value, X0=1.0, U=1.0, LM=3.0, Z=1.2;
    let x=[X0,LM], S=[[1.5,0],[0,0.5]];
    x=[x[0]+U, LM]; S=[[1.5+Q,0],[0,0.5]];
    const H=[-1,1];
    const innov=Z-(H[0]*x[0]+H[1]*x[1]);
    const Sh=H[0]*H[0]*S[0][0]+H[0]*H[1]*S[0][1]+H[1]*H[0]*S[1][0]+H[1]*H[1]*S[1][1]+R;
    const K0=(S[0][0]*H[0]+S[0][1]*H[1])/Sh, K1=(S[1][0]*H[0]+S[1][1]*H[1])/Sh;
    x=[x[0]+K0*innov, x[1]+K1*innov];
    const KH=[[K0*H[0],K0*H[1]],[K1*H[0],K1*H[1]]];
    const S2=[[0,0],[0,0]];
    for(let i=0;i<2;i++) for(let j=0;j<2;j++){
      let v=0; for(let k=0;k<2;k++) v+=((i===k?1:0)-KH[i][k])*S[k][j]; S2[i][j]=v;
    }
    return {x,S:S2,K:[K0,K1],innov,Sh,Sp:[[1.5+Q,0],[0,0.5]],pr:1.5+Q};
  }
  function draw(){
    const c=calc();
    ctx.clearRect(0,0,cv.width,cv.height);
    const x2px=v=>90+v*160, y0=90;
    ctx.strokeStyle='#2a3550'; ctx.beginPath(); ctx.moveTo(70,y0); ctx.lineTo(cv.width-70,y0); ctx.stroke();
    ctx.fillStyle='#8b98b8'; ctx.font='12px sans-serif'; ctx.textAlign='center';
    for(let v=0;v<=5;v++) ctx.fillText(v+'m',x2px(v),y0+20);
    const bw=Math.sqrt(c.Sp[0][0])*2*160;
    ctx.fillStyle='rgba(79,163,255,0.16)'; ctx.fillRect(x2px(2.0)-bw,y0-16,bw*2,32);
    ctx.fillStyle='#4fa3ff'; ctx.beginPath(); ctx.arc(x2px(2.0),y0,9,0,7); ctx.fill();
    ctx.fillText('预测 x̄=2.0',x2px(2.0),y0-26);
    ctx.fillStyle='#34d399'; ctx.beginPath(); ctx.arc(x2px(c.x[0]),y0,9,0,7); ctx.fill();
    ctx.fillText('更新 x='+c.x[0].toFixed(3),x2px(c.x[0]),y0+42);
    ctx.fillStyle='#fbbf24'; ctx.beginPath(); ctx.arc(x2px(3.0),y0,9,0,7); ctx.fill();
    ctx.fillText('路标 3.0 → '+c.x[1].toFixed(3),x2px(3.0),y0-26);
    ctx.strokeStyle='rgba(52,211,153,0.6)'; ctx.setLineDash([5,4]);
    ctx.beginPath(); ctx.moveTo(x2px(c.x[0]),y0); ctx.lineTo(x2px(3.0),y0); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle='#34d399'; ctx.fillText('观测 z=1.2（残差 '+c.innov.toFixed(2)+'）',
      (x2px(c.x[0])+x2px(3.0))/2,y0+60);
    document.getElementById('ekfOut').textContent=
`预测后 Σ̄ = [[${c.Sp[0][0].toFixed(2)}, 0], [0, 0.5]]     残差 = ${c.innov.toFixed(2)}
S = H·Σ̄·Hᵀ + R = ${c.Sh.toFixed(3)}     K = [${c.K[0].toFixed(3)}, ${c.K[1].toFixed(3)}]
更新后 x = [${c.x[0].toFixed(3)}, ${c.x[1].toFixed(3)}]
Σ' = [[${c.S[0][0].toFixed(3)}, ${c.S[0][1].toFixed(3)}], [${c.S[1][0].toFixed(3)}, ${c.S[1][1].toFixed(3)}]]
机器人方差 ${c.pr.toFixed(2)} → ${c.S[0][0].toFixed(3)}   交叉协方差 ${c.S[0][1].toFixed(3)} ≠ 0（关联诞生）`;
  }
  return { draw, reset(){ sm.value=0.5; sr.value=0.25;
    document.getElementById('ekmV').textContent='0.50';
    document.getElementById('ekrV').textContent='0.25'; draw(); } };
})();
['ekm','ekr'].forEach(id=>document.getElementById(id).addEventListener('input',e=>{
  document.getElementById(id+'V').textContent=(+e.target.value).toFixed(2); }));
EKF.draw();
</script>
'''

# ============================================================
# 模块 4：粒子滤波
# ============================================================
PF_BODY = '''
<div class="card">
  <h2><span class="no">④</span>粒子滤波：200 个"平行宇宙"从迷失到锁定（详细版）</h2>
  <div class="sub">高斯假设扛不住多峰（走廊两头长得一样）—— 粒子群天生表示多峰</div>
  <h3>4.1 直觉：平行宇宙</h3>
  <p>高斯分布只有一个"峰"——它只能表达"我大概在这"。但走廊两头长得一样时，
  你的信念其实是<b>两个峰</b>："我在 A 或 B，各 50%"。强行用单峰高斯等于对一半可能性装瞎。
  粒子滤波用 <b>N 个加权假设（粒子）</b>采样整个信念分布——每个粒子是一个
  "如果我在这里呢？"的平行宇宙。粒子密集的地方 = 信念强的地方。</p>
  <h3>4.2 三步循环（伪代码）</h3>
  <div class="formula">循环：
  ① 预测   每个粒子按运动模型抖动：x_p += u + 噪声
  ② 加权   按观测给每个粒子打分：w_p = P(z | x_p)   （似然）
  ③ 重采样 按权重抽 N 个新粒子（权重高的繁殖，低的淘汰）
每一步的估计 = 粒子的加权平均（位置/方差都从粒子群读出）</div>
  <h3>4.3 重采样的数学（轮盘赌）</h3>
  <p>按权重归一化后，每个粒子被抽中的概率 = w_p/Σw。手算：两个粒子
  w_A=0.12、w_B=0.88 → 重采样 200 个新粒子时，A 的期望副本 = 200×0.12×(1/2)…
  按比例：A 期望 24 个、B 期望 176 个。B 大幅繁殖、A 接近淘汰——
  <b>重采样就是"适者生存"的算术化</b>。副作用：反复重采样会让低权重粒子完全消失，
  多样性下降（粒子贫化）——真实实现要加"粗糙化"噪声。</p>
  <h3>4.4 交互演示（200 粒子，均匀撒满 [0,7]，门在 1.0 和 3.3）</h3>
  <canvas id="pfcv" width="980" height="240"></canvas>
  <div class="ctrls">
    <button onclick="PF.measure()">测距 + 加权重采样</button>
    <button onclick="PF.move()">前进 0.7m</button>
    <button class="ghost" onclick="PF.reset()">重置</button>
    <button class="ghost" onclick="PF.auto()">自动完整序列</button>
    <button class="ghost" onclick="PF.tTruth()" id="pfTruthBtn">显示真值</button>
  </div>
  <div class="readout" id="pfOut"></div>
  <h3>4.5 三幕剧读数</h3>
  <table>
    <tr><th>幕</th><th>粒子分布</th><th>说明</th></tr>
    <tr><td>第一幕：初始均匀</td><td>撒满 [0,7]</td><td>完全迷失——粒子滤波的诚实起点</td></tr>
    <tr><td>第二幕：第一次测距</td><td><b>多个簇存活</b></td><td>"到最近门 0.85m"的位置有多个——<b style="color:var(--wa)">多峰不是 bug，是诚实</b></td></tr>
    <tr><td>第三幕：移动+测距×2</td><td>只剩一个簇</td><td>门间距的非对称性开始起作用，假设大批死亡 → <b style="color:var(--ok)">锁定</b>（粒子标准差 = 置信度）</td></tr>
  </table>
  <h3>4.6 FastSLAM / Gmapping：粒子 + 每粒子一张地图</h3>
  <p><b>FastSLAM</b>（2003）的条件独立性洞察：已知机器人轨迹，各路标独立 →
  每个粒子带 N 个独立小 EKF，把 EKF-SLAM 的 O(n²) 打成 O(n·logN)。
  <b>Gmapping</b>（2007）：RBPF + 栅格地图（每粒子背一张全地图）+ 改进提议分布。
  ROS1 时代默认，已停维护——但"粒子=假设集"的思想活在 <b>AMCL</b>（本仓库一直在用）里。
  代价：覆盖多峰需要粒子数、粒子带地图内存爆炸 → 世界一大就轮到图优化（模块 6）。</p>
  <h3>4.7 常见误区</h3>
  <p>① 粒子贫化：重采样后所有粒子都是少数几个的副本 → 多样性丢失。对策：重采样后加小噪声；
  ② 粒子数太少 → 信念估计粗糙；太多 → CPU 爆（AMCL 默认 500-2000 个）；
  ③ 权重全趋近 0（观测与任何粒子都不匹配）→ 重采样退化——要检测并注入随机粒子。</p>
  <details><summary>练习：权重计算</summary>
    <p>路标在 3.0m。粒子 A 假设位姿 1.0（预测距离 2.0），粒子 B 假设 2.5（预测 0.5）。
    观测 z=0.6（σ=0.2）。各自的权重？</p>
    <p>答案：w = exp(−(z−预测)²/(2σ²))。w_A = exp(−(0.6−2.0)²/0.08) = exp(−24.5) ≈ 0；
    w_B = exp(−(0.6−0.5)²/0.08) = exp(−0.125) ≈ 0.88。重采样后 A 淘汰、B 繁殖——
    假设 A 被观测"处决"。</p>
  </details>
</div>
'''

PF_SCRIPT = '''
<script>
const PF=(()=>{
  const cv=document.getElementById('pfcv'), ctx=cv.getContext('2d');
  const DOORS=[1.0,3.3], WORLD=7;
  let particles=[], trueX=2.15, showT=false, seed=42;
  function rng(){ seed=(seed*1664525+1013904223)%4294967296; return seed/4294967296; }
  function nearest(x){ return Math.min(Math.abs(x-DOORS[0]),Math.abs(x-DOORS[1])); }
  function reset(){ particles=[]; trueX=2.15; seed=42;
    for(let i=0;i<200;i++) particles.push(rng()*WORLD); draw(); }
  function measure(){
    const z=nearest(trueX)+(rng()-0.5)*0.1;
    const w=particles.map(p=>Math.exp(-((nearest(p)-z)**2)/(2*0.05**2)));
    const sum=w.reduce((a,b)=>a+b,0);
    const acc=[]; let c=0;
    w.forEach(ww=>{c+=ww;acc.push(c);});
    particles=particles.map(()=>{ const r=rng()*c; let j=0;
      while(j<acc.length-1&&acc[j]<r)j++; return particles[j]; })
      .map(p=>p+(rng()-0.5)*0.02);
    draw();
  }
  function move(){ trueX+=0.7+(rng()-0.5)*0.06;
    particles=particles.map(p=>p+0.7+(rng()-0.5)*0.06); draw(); }
  function draw(){
    ctx.clearRect(0,0,cv.width,cv.height);
    const x2px=v=>50+v*(cv.width-100)/WORLD;
    const bins=70,h0=90,hh=54;
    const hist=new Array(bins).fill(0);
    particles.forEach(p=>{ const b=Math.min(bins-1,Math.floor(p/WORLD*bins)); hist[b]++; });
    const mx=Math.max(...hist,1);
    for(let b=0;b<bins;b++){
      const t=hist[b]/mx;
      ctx.fillStyle=t>0?`rgba(79,163,255,${0.25+0.75*t})`:'rgba(20,26,40,0.5)';
      const bx=x2px(b/bins*WORLD), bw=(cv.width-100)/bins;
      ctx.fillRect(bx,h0+hh*(1-t),bw-1,hh*t);
    }
    ctx.strokeStyle='#2a3550'; ctx.strokeRect(x2px(0),h0,cv.width-100,hh);
    ctx.fillStyle='#8b98b8'; ctx.font='12px sans-serif'; ctx.textAlign='left';
    ctx.fillText('粒子分布直方图（上）· 世界 [0,7m] 与两扇门（下）',50,24);
    const w0=170,wh=34;
    ctx.strokeStyle='#2a3550'; ctx.beginPath(); ctx.moveTo(x2px(0),w0); ctx.lineTo(x2px(WORLD),w0); ctx.stroke();
    DOORS.forEach(d=>{ ctx.fillStyle='#ffb454'; ctx.fillRect(x2px(d)-3,w0-wh,6,wh);
      ctx.fillText('门 '+d.toFixed(1),x2px(d)-22,w0-wh-8); });
    particles.forEach(p=>{ ctx.fillStyle='rgba(79,163,255,0.8)';
      ctx.fillRect(x2px(p)-1,w0-3,2,6); });
    if(showT){ ctx.fillStyle='#ff6b81'; ctx.beginPath();
      ctx.moveTo(x2px(trueX),w0-6); ctx.lineTo(x2px(trueX)-6,w0-18); ctx.lineTo(x2px(trueX)+6,w0-18);
      ctx.closePath(); ctx.fill(); ctx.fillText('真值',x2px(trueX)-14,w0-24); }
    const mean=particles.reduce((a,b)=>a+b,0)/particles.length;
    const sd=Math.sqrt(particles.reduce((a,b)=>a+(b-mean)**2,0)/particles.length);
    ctx.fillStyle='#34d399'; ctx.fillText(`粒子均值 ${mean.toFixed(2)}  标准差 ${sd.toFixed(3)}（置信度）`,50,150);
    document.getElementById('pfOut').textContent=
      `粒子数 ${particles.length}  均值 ${mean.toFixed(3)}  标准差 ${sd.toFixed(3)}`
      +(showT?`  真值 ${trueX.toFixed(2)}  误差 ${Math.abs(mean-trueX).toFixed(3)}`:'')
      +`\\n标准差大 = 迷失（多峰存活）   标准差小 = 锁定`;
  }
  return {
    measure(){ showT=true; measure(); },
    move(){ move(); },
    reset(){ reset(); },
    auto(){ reset(); showT=true;
      [()=>measure(),()=>move(),()=>measure(),()=>move(),()=>measure()]
        .forEach((f,i)=>setTimeout(f,850*(i+1))); },
    tTruth(){ showT=!showT; draw(); },
    draw(){ draw(); }
  };
})();
PF.reset();
</script>
'''

EXTRA_PAGES.update({
    'ekf.html': ('③ EKF-SLAM', EKF_BODY, EKF_SCRIPT),
    'pf.html': ('④ 粒子滤波', PF_BODY, PF_SCRIPT),
})

# ============================================================
# 模块 5：扫描匹配
# ============================================================
SM_BODY = '''
<div class="card">
  <h2><span class="no">⑤</span>扫描匹配：两帧激光怎么对齐（详细版）</h2>
  <div class="sub">总分 = 匹配代价 + λ·‖偏移 − 里程计预测‖² —— 匹配分负责对齐，里程计先验负责"别乱跳"</div>
  <h3>5.1 直觉</h3>
  <p>"我看到了一排墙点，上一帧地图里也有这排墙点——把新帧<b>平移旋转</b>一下，
  什么时候两组点最重合？"重合度最高的位姿 = "这步实际走了多少"的估计。
  这个相对位姿就是位姿图的一条边（模块 6）——<b>扫描匹配是 SLAM 前端的发动机</b>。</p>
  <h3>5.2 算法（correlative scan matching 伪代码）</h3>
  <div class="formula">for dx in 搜索窗口(步长 0.01m):
  for dy in 搜索窗口:
    for dθ in 角度窗口(±0.35rad, 步长 0.035):
      把激光点按 (dx,dy,dθ) 投影到已有地图，取投影处代价求和
      （墙=大代价；再做高斯模糊 smear 让得分连续可逼近）
      总分 = 匹配代价 + 里程计先验罚
取总分最低的候选 → 匹配结果</div>
  <h3>5.3 交互演示（拖动 dx 滑块，看评分实时变化）</h3>
  <canvas id="smcv" width="980" height="280"></canvas>
  <div class="ctrls">
    <label>偏移 dx=<span id="smxV">0.00</span></label>
    <input type="range" id="smx" min="-1" max="1.5" step="0.25" value="0" oninput="SM.draw()">
    <label>λ=<span id="smlV">2.0</span></label>
    <input type="range" id="sml" min="0" max="10" step="0.5" value="2" oninput="SM.draw()">
  </div>
  <div class="readout" id="smOut"></div>
  <h3>5.4 评分表（doc 21 §6.2 手算）</h3>
  <table>
    <tr><th>偏移</th><th>匹配代价</th><th>里程计罚</th><th>总分</th><th>说明</th></tr>
    <tr><td>dx=0</td><td>0</td><td>0.02</td><td><b style="color:var(--ok)">0.02 ← 最优</b></td><td>完美对齐 + 贴近预测</td></tr>
    <tr><td>dx=−1</td><td>0</td><td>2.42</td><td>2.42</td><td>匹配分打平！<b style="color:var(--wa)">纯匹配分有歧义</b></td></tr>
    <tr><td>dx=+1</td><td>30</td><td>1.62</td><td>31.62</td><td>三点全撞墙——<b style="color:var(--bad)">一票否决</b>（doc 16 同款语义）</td></tr>
  </table>
  <h3>5.5 里程计先验 = 贝叶斯先验</h3>
  <p>λ·‖偏移−预测‖² 这一项在贝叶斯语言里就是<b>先验分布</b>：
  "在看到这帧扫描之前，我最相信的位置在里程计预测附近，服从高斯分布"。
  扫描匹配做的事 = <b>先验 × 似然</b> 的联合最大——和贝叶斯滤波完全同构，
  只不过这里是"连续优化"而不是"离散枚举"。λ 就是先验的强度（= 1/先验方差）。</p>
  <h3>5.6 ICP 与两段式工程解法</h3>
  <p><b>ICP</b>（迭代最近点）：每个新帧点找地图上最近点配对 → 最小化配对距离解
  平移/旋转 → 用新位姿重新配对 → 迭代收敛。优点：亚格精度；缺点：需要好初值，
  否则局部最优。<b>工程解法是两段式</b>：粗搜索（correlative 大窗口粗步长，保证
  全局性）→ 精搜索（ICP 或细步长 correlative）。Cartographer 用多分辨率体素图
  加速粗搜（粗图 O(1) 查询）。对应参数：<span class="kbd">correlation_search_space_*</span>
  （doc 22 §1.2）。</p>
  <h3>5.7 常见误区</h3>
  <p>① 匹配分打平时没有先验可依 → 结果在歧义间跳动（把 λ=0 想象成"完全不信里程计"）；
  ② 窗口开太大 → 慢且易配错（相似结构误配）；开太小 → 纠正不了实际漂移；
  ③ 忘记处理"扫描点投影出界"——出界点要么跳过要么记最大惩罚。</p>
  <details><summary>练习：λ=0 时歧义重现</summary>
    <p>把 λ 设为 0：dx=0 总分 0，dx=−1 总分也是 0——两个候选打平。
    匹配器会选哪个？答案：先枚举到的那个（实现相关）——
    <b>这就是"没有先验就没有唯一解"的最小反例</b>。用上面的交互演示验证。</p>
  </details>
</div>
'''

SM_SCRIPT = '''
<script>
const SM=(()=>{
  const cv=document.getElementById('smcv'), ctx=cv.getContext('2d');
  const wall=new Set(['3,1','3,2','3,3']);
  const S=[[2,1],[2,2],[2,3]];
  const xs=document.getElementById('smx'), ls=document.getElementById('sml');
  function draw(){
    const dx=+xs.value, lam=+ls.value;
    document.getElementById('smxV').textContent=dx.toFixed(2);
    document.getElementById('smlV').textContent=lam.toFixed(1);
    ctx.clearRect(0,0,cv.width,cv.height);
    const cell=64, ox=(cv.width-5*cell)/2-40, oy=40;
    for(let r=0;r<5;r++) for(let c=0;c<5;c++){
      const x=ox+c*cell,y=oy+r*cell;
      const isWall=wall.has(`${c},${r}`);
      ctx.fillStyle=isWall?'#7a2a35':'#141b2c';
      ctx.fillRect(x,y,cell-2,cell-2);
      ctx.strokeStyle='#2a3550'; ctx.strokeRect(x,y,cell-2,cell-2);
      if(isWall){ ctx.fillStyle='#ff6b81'; ctx.font='11px sans-serif'; ctx.textAlign='center';
        ctx.fillText('墙10',x+cell/2-1,y+cell/2+4); }
    }
    let match=0; const det=[];
    S.forEach(([sx,sy],i)=>{
      const cx=Math.round(sx+dx), cy=sy;
      const c=wall.has(`${cx},${cy}`)?10:0; match+=c;
      const x=ox+(sx+dx)*cell+cell/2, y=oy+sy*cell+cell/2;
      ctx.fillStyle='#34d399'; ctx.beginPath(); ctx.arc(x,y,7,0,7); ctx.fill();
      ctx.fillStyle='#34d399'; ctx.fillText('S'+(i+1),x,y-12);
      det.push(`S${i+1}→(${cx},${cy})=${c}`);
    });
    ctx.fillStyle='#8b98b8'; ctx.textAlign='left';
    ctx.fillText('绿点 = 平移后的激光投影（栅格化）',ox,oy-10);
    const pen=lam*((dx-0.1)**2);
    document.getElementById('smOut').textContent=
      `偏移 dx=${dx.toFixed(2)}  匹配代价 ${match}（${det.join('  ')}）`
      +`\\n里程计罚 ${pen.toFixed(2)}（λ=${lam.toFixed(1)}）  总分 ${((match+pen)).toFixed(2)}（越低越好）`
      +`\\n对照：dx=0 最优 0.02   dx=−1 → ${(lam*1.21).toFixed(2)}   dx=+1 撞墙 30+罚=${(30+lam*0.81).toFixed(2)}`;
  }
  return { draw };
})();
SM.draw();
</script>
'''

# ============================================================
# 模块 6：图优化
# ============================================================
GO_BODY = '''
<div class="card">
  <h2><span class="no">⑥</span>图优化：回环把漂移拉回来（详细版）</h2>
  <div class="sub">位姿图 = 节点（位姿）+ 弹簧（约束）—— σ 小 = 硬弹簧；回环是 SLAM 的灵魂</div>
  <h3>6.1 直觉：弹簧网</h3>
  <p>里程计漂移让"按里程计拼的地图"越拼越歪。扫描匹配能纠正相邻两帧，
  但相对运动误差也在累积。出路：机器人回到去过的地方时（<b>回环</b>），
  加一条约束"这里就是你 20 分钟前的位置"，然后<b>全局求解</b>让所有约束尽量
  同时成立——把漂移均摊回整条轨迹。把每条约束想成一根<b>弹簧</b>：
  σ 小 = 硬弹簧（拉力大），σ 大 = 软弹簧。求解 = 让弹簧网达到平衡。</p>
  <h3>6.2 数学形式与完整推导</h3>
  <div class="formula">min J = (x2−x1−1)²/σ_odo² + (x3−x2−1)²/σ_odo² + (x3−x1−1.8)²/σ_loop²
（1D 机器人三位姿，x1 锚定 0；里程计说走 2.0，回环说只走了 1.8）</div>
  <p><b>∂J/∂x2 = 0</b>（x2 参与两条边，各求导后相加）：</p>
  <div class="formula">2(x2−x1−1)/σ_odo² − 2(x3−x2−1)/σ_odo² = 0
→ (x2−1) = (x3−x2−1)   →   2x2 = x3</div>
  <p><b>∂J/∂x3 = 0</b>（x3 参与两条边）：</p>
  <div class="formula">2(x3−x2−1)/0.01 + 2(x3−x1−1.8)/0.04 = 0
→ 200(x3−x2−1) + 50(x3−1.8) = 0  →  5x3 − 4x2 = 5.8</div>
  <p><b>联立</b>（x1=0，x2=x3/2）：<b>x3 = 1.933，x2 = 0.967</b>。</p>
  <p><b>直觉互验</b>：里程计链给出 x3=2.0（精度 1/(0.01+0.01)=50），
  回环给出 x3=1.8（精度 25）→ 精度加权 = (2.0×50+1.8×25)/75 = 1.933 ✓。
  两种算法殊途同归——图优化的本质就是"精度加权的证据合并"的全局版。</p>
  <h3>6.3 交互演示（拖动 σ 滑块，看弹簧网平衡点移动）</h3>
  <canvas id="gocv" width="980" height="240"></canvas>
  <div class="ctrls">
    <label>回环噪声 σ_loop=<span id="golV">0.20</span></label>
    <input type="range" id="gol" min="0.05" max="0.6" step="0.01" value="0.2" oninput="GO.draw()">
    <label>里程计 σ=<span id="gooV">0.10</span></label>
    <input type="range" id="goo" min="0.03" max="0.3" step="0.01" value="0.1" oninput="GO.draw()">
    <button class="ghost" onclick="GO.reset()">重置</button>
  </div>
  <div class="readout" id="goOut"></div>
  <h3>6.4 回环检测：怎么知道"回到原地"？</h3>
  <p>两道门：①位姿距离 &lt; <span class="kbd">loop_search_maximum_distance: 3.0</span>；
  ②扫描匹配响应 ≥ <span class="kbd">loop_match_minimum_response_coarse: 0.35 / fine: 0.45</span>
  ——"位姿近"不够，"扫描长得像"才算（防相似走廊误配）。</p>
  <div class="warnbox"><b>误回环 = 最恶性故障</b>：两张相似的地方被"焊"在一起，
  整张地图折叠。对策：提高响应阈值、缩小搜索距离、增加环境辨识度。
  一旦发生，只能删轨迹重建。</div>
  <h3>6.5 真实求解器</h3>
  <p>节点上千、维度 (x,y,θ) 时用<b>稀疏求解器</b>：位姿图的 Σ 是稀疏的
  （只有相邻/回环节点关联）——g2o、Ceres（slam_toolbox 用）、SPA/SPA-Diff
  （slam_toolbox 的 SpaSolver）。lua 里选 <span class="kbd">solver_plugin:
  solver_plugins::CeresSolver</span> + 线性求解器类型（本机：
  SPARSE_NORMAL_CHOLESKY + LEVENBERG_MARQUARDT）。</p>
  <h3>6.6 常见误区</h3>
  <p>① 忘记锚定一个节点（x1 固定）——整个图可以平移/旋转，解不唯一；
  ② 回环 σ 设得过小（过度信任）——一次误匹配扭曲全图；
  ③ 只优化被回环触碰的节点——正确做法是把"信息"传播到全图（这正是
  "全局"的含义）。</p>
  <details><summary>练习：回环变准了会怎样？</summary>
    <p>σ_loop 从 0.2 提到 0.1（权重 100）：
    ∂/∂x3 = 200(x3−x2−1) + 200(x3−1.8) = 0 → x3−x2−1 = −(x3−1.8) → 2x3−x2 = 2.8；
    ∂/∂x2 不变 → x2 = x3/2 → 1.5x3 = 2.8 → <b>x3 = 1.867</b>（x2 = 0.933）。
    回环越准，解越贴近回环的值（1.8）。σ 是全图质量的总开关。</p>
  </details>
</div>
'''

GO_SCRIPT = '''
<script>
const GO=(()=>{
  const cv=document.getElementById('gocv'), ctx=cv.getContext('2d');
  const gol=document.getElementById('gol'), goo=document.getElementById('goo');
  function draw(){
    const sl=+gol.value, so=+goo.value;
    document.getElementById('golV').textContent=sl.toFixed(2);
    document.getElementById('gooV').textContent=so.toFixed(2);
    const k=so*so/(sl*sl);
    const A=[[2,-1],[-1,1+k]], b=[0,1+1.8*k];
    const det=A[0][0]*A[1][1]-A[0][1]*A[1][0];
    const x2=(b[0]*A[1][1]-A[0][1]*b[1])/det;
    const x3=(A[0][0]*b[1]-b[0]*A[1][0])/det;
    ctx.clearRect(0,0,cv.width,cv.height);
    const x2px=v=>90+v*(cv.width-160)/4.5, y0=110;
    ctx.strokeStyle='#2a3550'; ctx.beginPath(); ctx.moveTo(70,y0); ctx.lineTo(cv.width-70,y0); ctx.stroke();
    ctx.fillStyle='#8b98b8'; ctx.font='12px sans-serif'; ctx.textAlign='center';
    for(let v=0;v<=4;v++) ctx.fillText(v+'m',x2px(v),y0+22);
    ctx.strokeStyle='rgba(139,152,184,0.5)'; ctx.setLineDash([5,5]);
    ctx.beginPath(); ctx.arc(x2px(2.0),y0,12,0,7); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillText('里程计漂移位 2.0',x2px(2.0),y0-44);
    function spring(a,b,tension){
      ctx.strokeStyle=`rgba(79,163,255,${0.3+0.7*tension})`;
      ctx.lineWidth=1+3*tension;
      const x1=x2px(a),x2=x2px(b),n=8,amp=8*(1-tension);
      ctx.beginPath(); ctx.moveTo(x1,y0);
      for(let i=1;i<n;i++){ const t=i/n; ctx.lineTo(x1+(x2-x1)*t,y0+((i%2)?amp:-amp)); }
      ctx.lineTo(x2,y0); ctx.stroke(); ctx.lineWidth=1;
    }
    spring(0,x2,Math.min(1,1/so/10));
    spring(x2,x3,Math.min(1,1/so/10));
    spring(0,x3,Math.min(1,1/sl/5));
    function node(v,rad,color,label,above){
      ctx.fillStyle=color; ctx.beginPath(); ctx.arc(x2px(v),y0,rad,0,7); ctx.fill();
      ctx.strokeStyle='#0b101c'; ctx.stroke();
      ctx.fillText(label+'='+v.toFixed(3),x2px(v),y0+(above?-30:26));
    }
    node(0,8,'#e6edf8','x1=0（锚定）',true);
    node(x2,6+Math.sqrt(so)*10,'#60a5fa','x2',false);
    node(x3,6+Math.sqrt(sl)*10,'#34d399','x3',true);
    ctx.fillStyle='#8b98b8'; ctx.textAlign='left';
    ctx.fillText('圆=位姿节点（大小∝σ）  线=约束弹簧（粗细∝权重1/σ²）  灰虚圈=漂移位',50,30);
    document.getElementById('goOut').textContent=
      `σ_loop=${sl.toFixed(2)}  σ_odo=${so.toFixed(2)}  k=σ_odo²/σ_loop²=${k.toFixed(3)}`
      +`\\n全局解：x2=${x2.toFixed(4)}  x3=${x3.toFixed(4)}`
      +`\\n对照：里程计漂移说 x3=2.0，回环说 x3=1.8 → 全局解偏向可信者`
      +`\\nσ_loop=0.2 / σ_odo=0.1（doc 21 §7.2 手算）：x2=0.9667, x3=1.9333`;
  }
  return { draw, reset(){ gol.value=0.2; goo.value=0.1; draw(); } };
})();
['gol','goo'].forEach(id=>document.getElementById(id).addEventListener('input',e=>{
  document.getElementById(id+'V').textContent=(+e.target.value).toFixed(2); }));
GO.draw();
</script>
'''

# ============================================================
# 工程实战
# ============================================================
ENG_BODY = '''
<div class="card">
  <h2><span class="no">⑦</span>工程实战：参数、标定与抗干扰</h2>
  <div class="sub">建图质量的上限由传感器与标定决定，下限由参数决定；参数救不了标定错误</div>
  <h3>① slam_toolbox 关键参数（本机配置原文，完整版见 doc 22 §1）</h3>
  <table>
    <tr><th>参数</th><th>默认</th><th>语义</th><th>动它会怎样</th></tr>
    <tr><td>minimum_travel_distance</td><td>0.5</td><td>每 0.5m 取一个关键帧（图的节点密度）</td><td>调小→密/精但重；调大→疏/轻但急弯可能丢</td></tr>
    <tr><td>correlation_search_space_dimension</td><td>0.5</td><td>扫描匹配搜索窗（模块 5 的窗口）</td><td>里程计差才加大；越大越慢越易配错</td></tr>
    <tr><td>correlation_search_space_resolution</td><td>0.01</td><td>搜索步长</td><td>亚格精度 vs 速度</td></tr>
    <tr><td>loop_match_minimum_response_coarse/fine</td><td>0.35/0.45</td><td>回环第二道门（扫描长得像才算）</td><td>调低→回环多但可能<b style="color:var(--bad)">误回环折叠地图</b>；调高→漂移留得多</td></tr>
    <tr><td>loop_match_minimum_chain_size</td><td>10</td><td>候选子图至少 10 帧</td><td>与关键帧间距联动：0.5m×10=5m 轨迹才有资格回环</td></tr>
    <tr><td>map_update_interval</td><td>5.0</td><td>/map 发布周期</td><td>调小→RViz 更流畅</td></tr>
    <tr><td>min_pass_through</td><td>2</td><td>格子被穿过 ≥2 次才算已知</td><td>抗单次噪声（证据对冲的输出端）</td></tr>
  </table>
  <h3>② 传感器标定三课（"传感器说的"必须等于"真实几何"）</h3>
  <p><b class="cy">雷达外参</b>（doc 17 实战）：TB3 雷达在中心后方 6.4cm
  （SDF 的 lidar_joint：−0.064, 0, 0.121）。不补偿 → 侧向束 range ≠ 中心距离 →
  CBF/SLAM 系统性误判。标定法：原地转 360° 对固定柱记录 36 点，拟合偏心圆心即偏移。
  <b>补偿前后 CBF 拦截 413 → 16 次</b>。</p>
  <p><b class="vi">里程计双误差标定</b>（手算）：直线走 5.0m 实测 4.85m → 轮径比例 0.97；
  原地转 360° 实测 367° → 轮距比 1.0194。两个实验分别激励 v 和 ω
  ——<b>可观测性设计</b>：轮径误差影响 v，轮距误差影响 ω。</p>
  <p><b class="pk">时间同步/运动畸变</b>：一帧激光 360 点不是同时打的。本机 5Hz →
  帧周期 200ms × 0.2m/s = <b>4cm 帧内畸变（≈一个栅格）</b>。对策：建图减速 /
  de-skew（里程计把每点反推回统一时刻）。<b>"建图要慢"的定量依据。</b></p>
  <h3>③ 抗干扰手册：激光的六个物理弱点</h3>
  <table>
    <tr><th>干扰</th><th>现象</th><th>对策</th></tr>
    <tr><td>玻璃/镜面</td><td>"穿墙点"或凭空短束</td><td>标记玻璃区不建图；强度通道过滤；红外交盲</td></tr>
    <tr><td>黑色吸光物</td><td>障碍"隐身"（回波弱于阈值）</td><td>深度相机/超声波融合；已知静态障碍写进 static layer</td></tr>
    <tr><td>动态物体</td><td>残留"幽灵墙"</td><td>证据对冲（doc 21 §3.3）+ raytrace clearing + 建图选无人时段</td></tr>
    <tr><td>多路径反射</td><td>凭空的远距点</td><td>强度过滤 + 与历史地图比对剔除离群</td></tr>
    <tr><td>灰尘污渍</td><td>全局距离偏差</td><td>定期擦窗；对已知墙的读数统计漂移即报警</td></tr>
    <tr><td>斜面掠射</td><td>丢束或鬼点</td><td>无效值过滤（min/max_laser_range 已兜）</td></tr>
  </table>
  <div class="note">📌 完整版（含故障速查表：双墙/重影/折叠/TF 打架…）见 doc 22。
  每条症状在本系列都有实录。</div>
  <h3>④ 故障速查表</h3>
  <table>
    <tr><th>症状</th><th>最可能根因</th><th>处理</th></tr>
    <tr><td>地图出现"双墙"（平行两道）</td><td>里程计比例误差 → 同一面墙画在两处</td><td>§②里程计标定；或加大扫描匹配搜索窗</td></tr>
    <tr><td>地图重影/错位</td><td>时间戳不同步</td><td>use_sim_time 检查；真机查时钟</td></tr>
    <tr><td>墙变圆弧/斜线</td><td>转向时运动畸变</td><td>§②③de-skew；建图减速</td></tr>
    <tr><td>绕回起点地图"开口"（合不上）</td><td>回环没触发/阈值过高</td><td>查 response 阈值与 chain_size；确认绕行距离够长</td></tr>
    <tr><td>地图"折叠"（两处焊在一起）</td><td><b style="color:var(--bad)">误回环</b>（最恶性）</td><td>提高响应阈值；缩小搜索距离；重建</td></tr>
    <tr><td>TF"map"帧时有时无</td><td>两个 map→odom 发布者打架</td><td>ros2 topic info /map 数发布者；清理孤儿进程</td></tr>
    <tr><td>换环境地图全崩</td><td>过拟合旧环境/特征贫乏</td><td>重新建图；空旷环境加特征物；lifelong 模式</td></tr>
  </table>
</div>
'''

# ============================================================
# 资源页
# ============================================================
RES_BODY = '''
<div class="card">
  <h2>资源与文档</h2>
  <table>
    <tr><th>资源</th><th>说明</th></tr>
    <tr><td class="kbd">docs/21_SLAM建图原理零基础课.md</td><td>本站每个模块的完整手算推导</td></tr>
    <tr><td class="kbd">docs/20_定位与建图SLAM.md</td><td>slam_toolbox 实测管线（真机器人版）</td></tr>
    <tr><td class="kbd">docs/22_SLAM工程实战.md</td><td>参数逐条 + 标定 + 抗干扰完整版</td></tr>
    <tr><td class="kbd">docs/tools/slam_playground.py</td><td>终端版六演示（与本站数字一致）</td></tr>
    <tr><td class="kbd">docs/tools/auto_explorer.py</td><td>真机器人自动探索建图</td></tr>
    <tr><td class="kbd">docs/tools/save_slam_map.py</td><td>抓 /map 存 PGM+YAML</td></tr>
    <tr><td class="kbd">docs/website/build.py</td><td>本站生成器（编辑内容后运行重新生成）</td></tr>
  </table>
  <h3>外部参考</h3>
  <p class="small">Thrun《Probabilistic Robotics》 ·
  <a href="https://docs.ros.org/en/humble/p/slam_toolbox" target="_blank">slam_toolbox 参数文档</a> ·
  <a href="https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml" target="_blank">默认配置</a> ·
  <a href="http://robots.stanford.edu/papers/montemerlo.fastslam-dataassoc03.pdf" target="_blank">FastSLAM 原论文</a> ·
  <a href="https://docs.duckietown.com/daffy/instructor-manual/_assets/slam.pdf" target="_blank">Duckietown SLAM 讲义</a> ·
  <a href="https://fixtherobot.dev" target="_blank">fixtherobot.dev: SLAM Toolbox 工作流</a></p>
  <div class="note">📌 系列文档：doc 01-14（A* 与控制器）、doc 15-18（链路/评分/CBF/工程优化）、
  doc 19（端到端/VLA 路线图）、doc 20-22（定位建图三部曲）。</div>
</div>
'''

EXTRA_PAGES.update({
    'sm.html': ('⑤ 扫描匹配', SM_BODY, SM_SCRIPT),
    'go.html': ('⑥ 图优化回环', GO_BODY, GO_SCRIPT),
    'eng.html': ('⑦ 工程实战', ENG_BODY, ''),
    'res.html': ('资源与文档', RES_BODY, ''),
})
