/* Embedded by visualize_3d.py; no server upload, ROS connection or online dependency. */
(() => {
  'use strict';
  function parseCSV(text) {
    const rows = []; let row = [], cell = '', quoted = false;
    text = text.replace(/^\uFEFF/, '');
    for (let i = 0; i <= text.length; i++) {
      const c = i === text.length ? '\n' : text[i];
      if (c === '"') {
        if (quoted && text[i+1] === '"') { cell += '"'; i++; }
        else quoted = !quoted;
      } else if (!quoted && (c === ',' || c === '\n' || c === '\r')) {
        row.push(cell.trim()); cell = '';
        if (c !== ',') {
          if (row.some(v => v !== '')) rows.push(row);
          row = [];
          if (c === '\r' && text[i+1] === '\n') i++;
        }
      } else cell += c;
    }
    if (quoted) throw Error('CSV 引号未闭合');
    if (rows.length < 2) throw Error('CSV 没有轨迹样本');
    const header = rows.shift();
    const ix = name => header.indexOf(name);
    for (const name of ['x', 'y', 'z']) if (ix(name) < 0) throw Error('CSV 缺少列：' + name);
    const points = rows.map((r, i) => {
      const number = name => {
        const k = ix(name);
        if (k < 0 || !r[k]) return NaN;
        return Number(r[k]);
      };
      const p = {x:number('x'), y:number('y'), z:number('z'), t:number('elapsed_s'),
                 segment:ix('segment') < 0 ? '0' : r[ix('segment')],
                 frame:ix('frame_id') < 0 ? 'world' : r[ix('frame_id')]};
      if (![p.x, p.y, p.z].every(Number.isFinite)) throw Error(`第 ${i+2} 行 XYZ 无效`);
      if (p.frame !== 'world') throw Error(`坐标系 ${p.frame} 不能直接叠加 world；请先转换坐标`);
      return p;
    });
    let distance = 0, distance3d = 0;
    for (let i = 1; i < points.length; i++) {
      const a = points[i-1], b = points[i];
      if (a.segment !== b.segment) continue;
      distance += Math.hypot(b.x-a.x, b.y-a.y);
      distance3d += Math.hypot(b.x-a.x, b.y-a.y, b.z-a.z);
    }
    return {points, distance, distance3d};
  }
  // Also usable in deterministic parser tests without loading Plotly or a browser.
  globalThis.ActualRouteCSV = {parseCSV};
  if (typeof document === 'undefined') return;
  const plot = document.getElementById('{plot_id}');
  const panel = document.createElement('div');
  panel.id = 'actual-route-controls';
  panel.style.cssText = 'position:fixed;right:14px;bottom:45px;z-index:1000;background:#101622ee;color:#e8eefc;padding:14px;border:1px solid #46536b;border-radius:8px;font:13px sans-serif;max-width:420px;line-height:1.7';
  panel.innerHTML = `<strong>GO 实测轨迹 · CSV</strong><br>
    <label>导入记录 <input aria-label="导入实走 CSV" type="file" accept=".csv,text/csv"></label><br>
    <label>同源 CSV 地址 <input aria-label="实走 CSV 地址" placeholder="actual_route.csv" style="width:170px"></label>
    <button type="button" id="load-route-url">加载</button><br>
    <label><input type="checkbox" id="actual-only">仅显示实走（其他世界使用此项）</label>
    <button type="button" id="focus-route">聚焦实走</button><br>
    <span id="actual-route-status" role="status">等待导入；橙色为实走，青色为离线路线。</span><br>
    <small>背景仅适合 Campus 同一 world 坐标；Z 为机身实测高度。<br>绿色覆盖率属于离线路线，不代表实走已覆盖。</small>`;
  document.body.appendChild(panel);
  const status = panel.querySelector('#actual-route-status');
  let actualTrace = null, startTrace = null, endTrace = null;
  const originalCount = plot.data.length;
  async function display(text, label) {
    const {points, distance, distance3d} = parseCSV(text);
    const x=[], y=[], z=[], hover=[];
    const stride = Math.max(1, Math.ceil(points.length/50000));
    for (let i=0; i<points.length; i++) {
      const p=points[i], boundary=i>0 && p.segment!==points[i-1].segment;
      if (boundary) {x.push(null);y.push(null);z.push(null);hover.push('');}
      if (i%stride && i!==points.length-1 && !boundary &&
          !(i+1<points.length && points[i+1].segment!==p.segment)) continue;
      x.push(p.x); y.push(p.y); z.push(p.z);
      hover.push(`样本 ${i+1} · ${Number.isFinite(p.t)?p.t.toFixed(1)+' s':'时间未知'} · 段 ${p.segment}`);
    }
    const trace={x,y,z,text:hover,mode:'lines',type:'scatter3d',name:'GO 实际走过',
                 line:{color:'#ff9b21',width:7},connectgaps:false,
                 hovertemplate:'%{text}<br>X=%{x:.3f} Y=%{y:.3f} Z=%{z:.3f}<extra></extra>'};
    const marker = (p, name, color) => ({type:'scatter3d',mode:'markers',name,
      x:[p.x],y:[p.y],z:[p.z],marker:{size:6,color}});
    if (actualTrace !== null) await Plotly.deleteTraces(plot,[actualTrace,startTrace,endTrace]);
    actualTrace=plot.data.length;startTrace=actualTrace+1;endTrace=actualTrace+2;
    await Plotly.addTraces(plot,[trace,marker(points[0],'实走起点','#ffd34d'),
                               marker(points[points.length-1],'实走终点','#ff6b35')]);
    const t=points[points.length-1].t;
    status.textContent=`${label} · ${points.length} 样本 · 平面 ${distance.toFixed(2)} m · 三维 ${distance3d.toFixed(2)} m`+
      (Number.isFinite(t)?` · 有效记录 ${t.toFixed(1)} s`:'');
    document.querySelector('#focus-route').disabled=false;
  }
  const fail = e => {status.textContent='导入失败：'+e.message;};
  panel.querySelector('input[type=file]').addEventListener('change',async e=>{
    const file=e.target.files[0];if(!file)return;
    try {await display(await file.text(),file.name);} catch(err) {fail(err);}
  });
  async function loadURL(raw) {
    const url=new URL(raw,location.href);
    if (url.origin!==location.origin) throw Error('请使用当前页面同源的 CSV 地址，或直接选取本地文件');
    const response=await fetch(url,{cache:'no-store'});
    if (!response.ok) throw Error('HTTP '+response.status);
    await display(await response.text(),url.pathname.split('/').pop());
  }
  panel.querySelector('#load-route-url').addEventListener('click',async()=>{
    try {await loadURL(panel.querySelector('input[aria-label="实走 CSV 地址"]').value);} catch(err){fail(err);}
  });
  panel.querySelector('#actual-only').addEventListener('change',async e=>{
    await Plotly.restyle(plot,{visible:e.target.checked?false:true},Array.from({length:originalCount},(_,i)=>i));
    await Plotly.relayout(plot,{'scene.xaxis.autorange':true,'scene.yaxis.autorange':true,'scene.zaxis.autorange':true});
  });
  panel.querySelector('#focus-route').disabled=true;
  panel.querySelector('#focus-route').addEventListener('click',()=>{
    if(actualTrace===null)return;
    const tr=plot.data[actualTrace], bounds={};
    for(const axis of ['x','y','z']) {
      const a=tr[axis].filter(Number.isFinite);
      let lo=Infinity,hi=-Infinity;for(const v of a){lo=Math.min(lo,v);hi=Math.max(hi,v);}
      const pad=Math.max(1,(hi-lo)*0.08);bounds[`scene.${axis}axis.range`]=[lo-pad,hi+pad];
      bounds[`scene.${axis}axis.autorange`]=false;
    }
    Plotly.relayout(plot,bounds);
  });
  const initial=new URLSearchParams(location.search).get('actual');
  if(initial)loadURL(initial).catch(fail);
})();
