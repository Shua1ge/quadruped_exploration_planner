# -*- coding: utf-8 -*-
"""HTML template for the offline code-audit page. render(DATA) -> html string."""

import json


def render(D):
    data_json = json.dumps(D, ensure_ascii=False).replace('</', '<\\/')
    html = r'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SCAN-Planner dyn 代码审计索引 · 离线交互</title>
<style>
:root{--bg:#f5f6fa;--panel:#fff;--ink:#1a2233;--muted:#5d6779;--line:#dfe3ec;
--acc:#2456d6;--accbg:#e9eefc;--warn:#b45309;--bad:#b91c1c;--ok:#0f7b4d;
--mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
--shadow:0 6px 24px rgba(20,30,60,.07)}
*{box-sizing:border-box}html,body{margin:0;height:100%}
body{background:var(--bg);color:var(--ink);font:14px/1.6 system-ui,-apple-system,"Segoe UI","Noto Sans SC",sans-serif;display:flex;flex-direction:column;height:100vh}
header{background:var(--panel);border-bottom:1px solid var(--line);padding:8px 14px;display:flex;gap:10px;align-items:center;flex-wrap:wrap;flex-shrink:0}
header h1{font-size:15px;margin:0;font-weight:700}
.chip{font-size:11px;color:var(--muted);border:1px solid var(--line);border-radius:5px;padding:2px 7px;background:var(--bg)}
.chip b{color:var(--ink);font-weight:600}
#gs{border:1px solid var(--line);border-radius:7px;padding:5px 10px;font-size:13px;width:220px;background:var(--bg)}
.tabs{display:flex;gap:4px;margin-left:auto}
.tabs button{border:1px solid var(--line);background:var(--bg);padding:5px 12px;border-radius:7px;cursor:pointer;font-size:13px;color:var(--muted)}
.tabs button[aria-selected="true"]{background:var(--acc);color:#fff;border-color:var(--acc);font-weight:600}
main{flex:1;display:flex;min-height:0}
#side{width:340px;background:var(--panel);border-right:1px solid var(--line);overflow:auto;flex-shrink:0;padding:8px}
#detail{flex:1;overflow:auto;padding:16px 20px}
@media(max-width:860px){#side{display:none}}
.badge{display:inline-block;font-size:10px;padding:1px 6px;border-radius:4px;margin-right:4px;color:#fff;vertical-align:1px;white-space:nowrap}
.b-exact{background:var(--ok)}.b-name{background:var(--warn)}.b-ambig{background:var(--bad)}.b-unres{background:#6b7280}.b-cycle{background:#7c3aed}.b-heur{background:#0891b2}
.tag{font-family:var(--mono);font-size:11px;color:var(--acc)}
a.lnk{color:var(--acc);cursor:pointer;text-decoration:none}
a.lnk:hover{text-decoration:underline}
.kv{font-size:12.5px;color:var(--muted);margin:2px 0}
.sec{font-size:11px;letter-spacing:.08em;color:var(--muted);margin:16px 0 6px;border-bottom:1px dashed var(--line);padding-bottom:3px}
.crumb{display:flex;gap:6px;align-items:center;flex-wrap:wrap;font-size:13px;margin-bottom:10px}
.crumb a{color:var(--acc);cursor:pointer;text-decoration:none}
.crumb .cur{font-weight:700}
.item{display:flex;gap:8px;align-items:baseline;padding:3px 8px;border-radius:6px;cursor:pointer;font-size:13px}
.item:hover{background:var(--accbg)}
.item.sel{background:var(--accbg);box-shadow:inset 0 0 0 1px var(--acc)}
.item .n{font-family:var(--mono);font-size:12.5px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:210px}
#gsres .item .n{max-width:300px}
.item .d{font-size:11px;color:var(--muted);margin-left:auto;flex-shrink:0}
.dirrow{display:flex;gap:8px;padding:3px 8px;cursor:pointer;border-radius:6px;font-size:13px}
.dirrow:hover{background:var(--accbg)}
.dirrow .chev{width:12px;color:var(--muted)}
.dirrow .cnt{color:var(--muted);font-size:11px;margin-left:auto}
.kids{padding-left:16px;border-left:1px dashed var(--line);margin-left:12px}
pre.src{background:#0f172a;color:#dbe4f3;border-radius:10px;padding:12px 14px;overflow:auto;font-family:var(--mono);font-size:12px;line-height:1.55;max-height:480px}
pre.src .ln{color:#4a5a78;user-select:none;display:inline-block;width:44px;text-align:right;margin-right:12px}
pre.src .hit{background:#3b2f10;display:inline-block;width:100%}
pre.src .row:hover{background:#1e293b}
.rowlink{display:flex;gap:10px;align-items:baseline;padding:5px 9px;border:1px solid var(--line);border-radius:8px;margin:4px 0;background:var(--panel);cursor:pointer;font-size:13px}
.rowlink:hover{border-color:var(--acc)}
.rowlink .where{font-family:var(--mono);font-size:11.5px;color:var(--acc);flex-shrink:0}
.rowlink .ctx{font-family:var(--mono);font-size:11.5px;color:var(--muted);word-break:break-all}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;box-shadow:var(--shadow);padding:14px 16px;margin-bottom:14px}
h2.t{margin:0 0 4px;font-size:17px}
.note{font-size:12.5px;color:var(--muted);background:var(--accbg);border:1px solid var(--line);border-radius:8px;padding:8px 12px;margin:8px 0}
input.txt,textarea.txt{border:1px solid var(--line);border-radius:7px;padding:6px 10px;font-size:13px;width:100%;background:var(--panel);font-family:var(--mono)}
button.act{border:1px solid var(--acc);background:var(--acc);color:#fff;border-radius:8px;padding:7px 16px;font-size:13px;cursor:pointer}
button.act.sec{background:var(--panel);color:var(--acc)}
#toast{position:fixed;bottom:18px;left:50%;transform:translateX(-50%);background:#1f2937;color:#fff;border-radius:8px;padding:8px 16px;font-size:13px;display:none;z-index:50;box-shadow:0 8px 24px rgba(0,0,0,.3)}
.sev-INFO{background:#2563eb}.sev-WARN{background:#d97706}.sev-ERROR{background:#b91c1c}.sev-DEBUG{background:#6b7280}.sev-FATAL{background:#7f1d1d}
.k-set{background:var(--ok)}.k-clear{background:var(--bad)}.k-mod{background:#d97706}.k-read{background:#6b7280}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:0 18px}
@media(max-width:1100px){.grid2{grid-template-columns:1fr}}
.empty{color:var(--muted);font-size:13px;padding:12px}
.statgrid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:10px;margin:10px 0}
.stat{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
.stat b{font-size:20px;display:block}
.stat span{font-size:12px;color:var(--muted)}
#gsres{position:fixed;top:52px;right:14px;width:420px;max-height:60vh;overflow:auto;background:var(--panel);border:1px solid var(--line);border-radius:10px;box-shadow:var(--shadow);z-index:40;display:none;padding:6px}
</style>
</head>
<body>
<header>
  <h1>SCAN-Planner · dyn 代码审计索引</h1>
  <span class="chip">分支 <b id="c-br"></b></span>
  <span class="chip">HEAD <b id="c-head"></b></span>
  <span class="chip">未提交改动 <b id="c-dirty"></b></span>
  <span class="chip">生成于 <b id="c-gen"></b></span>
  <input id="gs" type="search" placeholder="全局搜索函数/字段/话题 ( / )" aria-label="搜索">
  <nav class="tabs" role="tablist">
    <button role="tab" aria-selected="true" data-v="tree">文件树</button>
    <button role="tab" aria-selected="false" data-v="st">状态节点</button>
    <button role="tab" aria-selected="false" data-v="msg">消息节点</button>
    <button role="tab" aria-selected="false" data-v="log">日志反查</button>
    <button role="tab" aria-selected="false" data-v="about">说明</button>
  </nav>
</header>
<main>
  <aside id="side"></aside>
  <section id="detail"></section>
</main>
<div id="toast"></div>
<div id="gsres"></div>
<script>
"use strict";
const D = __DATA__;
const $ = s => document.querySelector(s);
const detail = $('#detail'), side = $('#side');
const FUNC = {}; D.funcs.forEach(f => FUNC[f.id] = f);
const byName = {};
D.funcs.forEach(f => (byName[f.name] = byName[f.name] || []).push(f.id));
const inEdges = {};
D.calls.forEach(e => (inEdges[e.b] = inEdges[e.b] || []).push(e));
const outEdges = {};
D.calls.forEach(e => (outEdges[e.a] = outEdges[e.a] || []).push(e));
const FIELD = {}; D.fields.forEach(v => FIELD[v.id] = v);
const siteMap = {};   // "file:line" -> [{v,kind}]
D.fields.forEach(v => (v.sites || []).forEach(s => {
  const k = s.file + ':' + s.line;
  (siteMap[k] = siteMap[k] || []).push({v: v.id, kind: s.kind});
}));
const pubByFunc = {};  // fid -> [{topic, site}]
Object.entries(D.topics).forEach(([t, x]) => (x.publish_sites || []).forEach(ps => {
  if (ps.func) (pubByFunc[ps.func] = pubByFunc[ps.func] || []).push({topic: t, site: ps});
}));
const logByFunc = {};
D.logs.forEach(l => { if (l.func) (logByFunc[l.func] = logByFunc[l.func] || []).push(l); });
const cbTopics = {};   // callback func id or name -> topics
Object.entries(D.topics).forEach(([t, x]) => (x.subs || []).forEach(s => {
  const key = s.func || s.cb;
  if (key) (cbTopics[key] = cbTopics[key] || []).push({topic: t, site: s});
}));
const TOKEN = {};
D.funcs.forEach(f => TOKEN[f.id] = (f.cls ? f.cls + '::' : '') + f.name);
const expanded = new Set();
let curView = 'tree', gsQuery = '';
let fqQuery = '';
let logQuery = '', logHtml = '';
try {
  logQuery = sessionStorage.getItem('audit.logq') || '';
  logHtml = sessionStorage.getItem('audit.loghtml') || '';
} catch (e) { /* file:// or privacy mode */ }

const esc = s => String(s == null ? '' : s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function toast(msg){const t=$('#toast');t.textContent=msg;t.style.display='block';clearTimeout(t._h);t._h=setTimeout(()=>t.style.display='none',2600);}
function sevBadge(s){return '<span class="badge sev-'+esc(s)+'">'+esc(s)+'</span>'}
function confBadge(c){
  const m={exact:['b-exact','精确'],name:['b-name','名称匹配'],ambig:['b-ambig','同名歧义'],unres:['b-unres','未解析']};
  const x=m[c]||['b-unres',c];return '<span class="badge '+x[0]+'" title="'+({
    exact:'限定名或同类命中，唯一目标',
    name:'按函数名在全工作区唯一命中（可能仍是不同类）',
    ambig:'存在多个同名符号，需人工确认',
    unres:'调用目标不在工作区索引中（外部/宏/动态）'})[c]+'">'+x[1]+'</span>';
}
function kindBadge(k){
  const m={set:['k-set','设置'],clear:['k-clear','清除'],mod:['k-mod','修改'],read:['k-read','读取']};
  const x=m[k]||['k-read',k];return '<span class="badge '+x[0]+'">'+x[1]+'</span>';
}
function goto(hash){ if(location.hash === hash){ render(); } else { location.hash = hash; } }

/* ---------------- routing ---------------- */
function parseHash(){
  const h = decodeURIComponent(location.hash.replace(/^#/, ''));
  if(h.startsWith('f='))  return {view:'fn',   path: h.slice(2).split('>')};
  if(h.startsWith('file=')) return {view:'file', path:[h.slice(5)]};
  if(h.startsWith('v='))  return {view:'field', path:[h.slice(2)]};
  if(h.startsWith('m='))  return {view:'msg',   path:[h.slice(2)]};
  if(h.startsWith('tok='))return {view:'tok',   path:[h.slice(4)]};
  if(h === 'log')         return {view:'log',   path:[]};
  if(h === 'about')       return {view:'about', path:[]};
  if(h === 'st')          return {view:'st',    path:[]};
  if(h === 'msg')         return {view:'msglist',path:[]};
  return {view:'tree', path:[]};
}
function viewOf(v){return v==='fn'||v==='file'||v==='field'||v==='msg'||v==='tok' ? 'tree' : v}
window.addEventListener('hashchange', render);

/* ---------------- global search ---------------- */
$('#gs').addEventListener('input', e => {
  gsQuery = e.target.value.trim().toLowerCase();
  const box = $('#gsres');
  if(!gsQuery){box.style.display='none';return;}
  let res = [];
  D.funcs.forEach(f => {
    const label = TOKEN[f.id];
    if(label.toLowerCase().includes(gsQuery) && res.length < 40)
      res.push({k:'函数', label: label, h:'f='+f.id, d:f.file+':'+f.line});
  });
  D.fields.forEach(v => {
    if((v.cls+'.'+v.name).toLowerCase().includes(gsQuery) && res.length < 60)
      res.push({k:'字段', label: v.cls+'.'+v.name, h:'v='+v.id, d:v.file+':'+v.line});
  });
  Object.keys(D.topics).forEach(t => {
    if(t.toLowerCase().includes(gsQuery) && res.length < 70)
      res.push({k:'话题', label: t, h:'msg='+(msgOfTopic(t)||''), d:'pubs '+(D.topics[t].pubs||[]).length+' · subs '+(D.topics[t].subs||[]).length});
  });
  box.innerHTML = res.length ? res.map(r =>
    '<div class="item" onclick="goto(\''+r.h+'\');$(\'#gsres\').style.display=\'none\'">'+
    '<span class="badge b-heur">'+r.k+'</span><span class="n">'+esc(r.label)+'</span><span class="d">'+esc(r.d)+'</span></div>').join('')
    : '<div class="empty">无结果</div>';
  box.style.display='block';
});
document.addEventListener('click', e => {
  if(!e.target.closest('#gsres') && !e.target.closest('#gs')) $('#gsres').style.display='none';
});
document.addEventListener('keydown', e => {
  if(e.key === '/' && document.activeElement !== $('#gs')){e.preventDefault();$('#gs').focus();}
});
function msgOfTopic(t){
  const ty = D.topics[t].type.replace(/^(scan_planner_msgs::msg::|scan_planner_msgs\/msg\/)/,'');
  return D.msgs.find(m => m.name === ty) ? ty : null;
}

/* ---------------- tabs ---------------- */
document.querySelectorAll('.tabs button').forEach(b => b.addEventListener('click', () => {
  const v = b.dataset.v;
  goto(v === 'tree' ? '#tree' : v === 'st' ? '#st' : v === 'msg' ? '#msg' : v === 'log' ? '#log' : '#about');
}));

/* ---------------- render dispatcher ---------------- */
function render(){
  const r = parseHash();
  curView = r.view;
  document.querySelectorAll('.tabs button').forEach(b =>
    b.setAttribute('aria-selected', String(viewOf(r.view) === b.dataset.v)));
  if(r.view === 'tree'){ renderSide(null); detail.innerHTML = overviewHtml(); }
  else if(r.view === 'file'){ const p = r.path[0]; renderSide(p); renderFile(p); }
  else if(r.view === 'fn'){ renderSide(FUNC[r.path[r.path.length-1]]?.file); renderFn(r.path); }
  else if(r.view === 'st'){ renderSide(null); renderFieldList(fqQuery); }
  else if(r.view === 'field'){ renderSide(null); renderField(r.path[0]); }
  else if(r.view === 'msglist'){ renderSide(null); renderMsgList(); }
  else if(r.view === 'msg'){ renderSide(null); renderMsg(r.path[0]); }
  else if(r.view === 'log'){ renderSide(null); renderLog(); }
  else if(r.view === 'tok'){ renderSide(null); renderToken(r.path[0]); }
  else if(r.view === 'about'){ renderSide(null); renderAbout(); }
  const sel = detail.querySelector('.item.sel, .dirrow.sel');
  if(sel) sel.scrollIntoView({block:'nearest'});
}

/* ---------------- overview ---------------- */
function overviewHtml(){
  const m = D.meta;
  return '<div class="card"><h2 class="t">这是一个离线静态审计索引，不是在线 ROS 图</h2>'
    +'<div class="statgrid">'
    +'<div class="stat"><b>'+m.n_funcs+'</b><span>函数（C++/Python）</span></div>'
    +'<div class="stat"><b>'+m.n_calls+'</b><span>调用边（分级）</span></div>'
    +'<div class="stat"><b>'+m.n_fields+'</b><span>状态字段（含读写点）</span></div>'
    +'<div class="stat"><b>'+m.n_msgs+'</b><span>消息定义</span></div>'
    +'<div class="stat"><b>'+m.n_topics+'</b><span>话题（pub/sub 接线）</span></div>'
    +'<div class="stat"><b>'+m.n_logs+'</b><span>日志点（可反查）</span></div>'
    +'<div class="stat"><b>'+m.total_files+'</b><span>文件 / '+m.total_loc.toLocaleString()+' LOC</span></div>'
    +'</div>'
    +'<div class="note">工作区快照：<b>'+esc(m.workspace)+'</b> · 分支 '+esc(m.branch)+' · HEAD '+esc(m.head)
    +' · 未提交改动 '+m.dirty_count+' 个 · 生成时间 '+esc(m.generated_at)+'</div>'
    +'<div class="note"><b>开始使用：</b>左侧文件树展开到 文件 → 类 → 函数；或到「日志反查」粘贴终端报错；或到「状态节点」「消息节点」直接查读写点与生产/消费接线。</div>'
    +'<div class="note"><b>证据分级：</b>'+confBadge('exact')+' 限定名/同类唯一命中 · '+confBadge('name')+' 全工作区唯一同名 · '
    +confBadge('ambig')+' 多个同名符号 · '+confBadge('unres')+' 目标不在索引。后两类不代表确定链路。</div></div>';
}

/* ---------------- tree ---------------- */
['', 'src', 'src/planner', 'src/planner/plan_manage'].forEach(p => expanded.add(p));
function renderSide(selectPath){
  side.innerHTML = '<div style="font-size:12px;color:var(--muted);padding:2px 6px 6px">文件树 · 展开 目录 → 文件 → 类/函数</div>'
    + '<div id="treebox">' + treeHtml(D.tree, 0, selectPath) + '</div>';
}
function treeHtml(node, depth, selectPath){
  if(node.t === 'f'){
    const info = D.files[node.p] || {};
    const n = node.n;
    const sel = selectPath === node.p ? ' sel' : '';
    return '<div class="item'+sel+'" title="'+esc(node.p)+'" onclick="openFile(\''+esc(node.p)+'\')">'
      +'<span class="n">'+esc(n)+'</span>'
      +'<span class="d">'+(info.funcs && info.funcs.length ? info.funcs.length+' fn' : (node.lang==='bin'?'bin':node.loc+' L'))+'</span></div>';
  }
  const isOpen = expanded.has(node.p);
  let kids = '';
  if(isOpen){
    const dirs = Object.values(node.c).filter(c=>c.t==='d');
    const files = Object.values(node.c).filter(c=>c.t==='f');
    kids = dirs.map(c=>treeHtml(c,depth+1,selectPath)).join('') + files.map(c=>treeHtml(c,depth+1,selectPath)).join('');
  }
  return '<div class="dirrow" title="'+esc(node.p || 'SCAN-Planner')+'" onclick="toggleDir(\''+esc(node.p)+'\')">'
    +'<span class="chev">'+(isOpen?'▾':'▸')+'</span><span>'+esc(node.n)+'</span></div>'
    +(isOpen?'<div class="kids">'+kids+'</div>':'');
}
function toggleDir(p){ if(expanded.has(p)) expanded.delete(p); else expanded.add(p); renderSide(null); }
function expandToFile(p){
  const parts = p.split('/');
  for(let i=1;i<parts.length;i++) expanded.add(parts.slice(0,i).join('/'));
}
window.openFile = function(p){
  expandToFile(p);
  goto('#file='+encodeURIComponent(p));
};

/* ---------------- file view ---------------- */
function renderFile(p){
  const info = D.files[p];
  if(!info){ detail.innerHTML = '<div class="empty">文件不在索引中: '+esc(p)+'</div>'; return; }
  const src = D.src[p];
  const cls = info.cls || [];
  const fids = info.funcs || [];
  const vids = info.fields || [];
  let h = '<div class="crumb"><a onclick="goto(\'#tree\')">文件树</a><span>›</span><b class="cur">'+esc(p)+'</b></div>';
  h += '<div class="card"><h2 class="t">'+esc(p.split('/').pop())+'</h2>'
    +'<div class="kv">'+esc(p)+' · '+info.lang+' · '+info.loc+' 行</div>';
  if(cls.length) h += '<div class="sec">类 / 结构体</div>'+cls.map(c =>
      '<div class="rowlink" onclick="goto(\'#file='+encodeURIComponent(p)+'\')"><span class="where">L'+c.line+'</span><span>'+esc(c.n)+'</span></div>').join('');
  if(fids.length) h += '<div class="sec">函数 ('+fids.length+')</div>'+fids.map(id => {
      const f = FUNC[id];
      return '<div class="rowlink" onclick="goto(\'#f='+id+'\')"><span class="where">L'+f.line+'</span>'
        +'<span class="n" style="font-family:var(--mono);font-size:12.5px">'+esc(f.cls?f.cls+'::':'')+esc(f.name)+'</span>'
        +'<span class="ctx" style="margin-left:auto">'+esc((f.sig||'').slice(0,80))+'</span></div>';
    }).join('');
  if(vids.length) h += '<div class="sec">字段 ('+vids.length+') · <a class="lnk" onclick="goto(\'#st\')">到状态节点视图</a></div>'
    +'<div class="note">该文件的字段读写点在「状态节点」视图中按类分组展示。</div>';
  if(src) h += '<div class="sec">源码</div>'+srcHtml(p, src, 1, Math.min(src.length, 60));
  else h += '<div class="note">此文件未嵌入源码（大小/语言限制），仅有索引信息。</div>';
  h += '</div>';
  detail.innerHTML = h;
}
function srcHtml(path, src, from, to){
  let out = '<pre class="src">';
  for(let i=from;i<=Math.min(to, src.length);i++){
    out += '<span class="row" id="L'+i+'"><span class="ln">'+i+'</span>'+esc(src[i-1])+'</span>\n';
  }
  return out+'</pre>';
}

/* ---------------- function view ---------------- */
function renderFn(path){
  const id = path[path.length-1];
  const f = FUNC[id];
  if(!f){ detail.innerHTML = '<div class="empty">函数不存在: '+esc(id)+'</div>'; return; }
  path.forEach(x => { const ff = FUNC[x]; if(ff) expandToFile(ff.file); });
  const isCycle = path.indexOf(id) !== path.length-1;
  let h = '<div class="crumb"><a onclick="goto(\'#tree\')">文件树</a>';
  path.forEach((x,i) => {
    h += '<span>›</span>' + (i===path.length-1
      ? '<b class="cur">'+esc(TOKEN[x]||x)+'</b>'
      : '<a onclick="goto(\'#f='+path.slice(0,i+1).join('>')+'\')">'+esc(TOKEN[x]||x)+'</a>');
  });
  if(isCycle) h += ' <span class="badge b-cycle" title="目标函数已在展开路径上，再次进入即循环">↻ 循环</span>';
  h += '</div>';
  h += '<div class="card"><h2 class="t" style="font-family:var(--mono);font-size:15px">'+esc(f.cls?f.cls+'::':'')+esc(f.name)+'</h2>'
    +'<div class="kv"><a class="lnk" onclick="goto(\'#file='+encodeURIComponent(f.file)+'\')">'+esc(f.file)+'</a>'
    +' · 行 '+f.line+'–'+f.end+' · '+esc(f.lang==='py'?'Python':'C++')+'</div>';
  /* source */
  const src = D.src[f.file];
  if(src){
    const from = Math.max(1, f.line-2), to = Math.min(src.length, f.line+110);
    h += '<div class="sec">源码片段（行 '+from+'–'+to+'，函数体 '+f.line+'–'+f.end+'）</div>'
      +'<pre class="src">'+src.slice(from-1, to).map((ln,i) => {
        const no = from+i;
        const mark = no>=f.line && no<=f.end;
        return '<span class="row'+(mark?' hit':'')+'" id="L'+no+'"><span class="ln">'+no+'</span>'+esc(ln)+'</span>\n';
      }).join('')+'</pre>';
    if(f.end - f.line > 110) h += '<div class="kv">片段截断 · 完整函数体见 <a class="lnk" onclick="goto(\'#file='+encodeURIComponent(f.file)+'\')">文件视图</a></div>';
  } else h += '<div class="note">此文件未嵌入源码。</div>';

  /* calls out */
  const outs = outEdges[id] || [];
  const grouped = {};
  outs.forEach(e => {
    const k = e.b+'|'+e.conf;
    if(!grouped[k] || e.line < grouped[k].line) grouped[k] = {b:e.b, conf:e.conf, line:e.line};
    grouped[k].n = (grouped[k].n||0)+1;
  });
  const outList = Object.values(grouped).sort((a,b)=>a.line-b.line);
  h += '<div class="sec">调用（出边 '+outs.length+' 条 / 去重 '+outList.length+' 个）</div>';
  h += outList.length ? outList.map(e => {
    const t = FUNC[e.b];
    const inPath = path.includes(e.b);
    return '<div class="rowlink" onclick="'+(inPath?'toastCycle()':'gotoFn(\''+e.b+'\')')+'">'
      +'<span class="where">L'+e.line+'</span>'
      +'<span class="n" style="font-family:var(--mono)">'+esc(t?(t.cls?t.cls+'::':'')+t.name:'?')+'</span>'
      +confBadge(e.conf)
      +(e.n>1?'<span class="d">'+e.n+' 处调用</span>':'')
      +(inPath?'<span class="badge b-cycle">↻ 已在路径</span>':'')
      +'<span class="ctx" style="margin-left:auto">'+esc(t?t.file+':'+t.line:'')+'</span></div>';
  }).join('') : '<div class="empty">（无已解析的内部调用）</div>';

  /* unresolved calls */
  const un = D.unresolved.filter(u => u.a === id);
  if(un.length){
    const uniq = [...new Set(un.map(u=>u.name))];
    h += '<div class="sec">未解析调用 '+un.length+' 处（'+uniq.length+' 个不同目标 · 不构成链路）</div>'
      +'<div class="note">'+confBadge('unres')+'以下调用目标不在工作区符号索引内（外部库/宏/动态绑定），'+uniq.slice(0,40).map(esc).join('、')
      +(uniq.length>40?' …等 '+uniq.length+' 个':'')+'</div>';
  }

  /* callers */
  const ins = inEdges[id] || [];
  const groupedIn = {};
  ins.forEach(e => {
    const k = e.a+'|'+e.conf;
    if(!groupedIn[k] || e.line < groupedIn[k].line) groupedIn[k] = {a:e.a, conf:e.conf, line:e.line};
    groupedIn[k].n = (groupedIn[k].n||0)+1;
  });
  const inList = Object.values(groupedIn).sort((a,b)=>a.line-b.line);
  h += '<div class="sec">被调用（入边 '+ins.length+' 条 / 去重 '+inList.length+' 个）</div>';
  h += inList.length ? inList.map(e => {
    const t = FUNC[e.a];
    return '<div class="rowlink" onclick="goto(\'#f='+e.a+'\')">'
      +'<span class="where">L'+e.line+'</span>'
      +'<span class="n" style="font-family:var(--mono)">'+esc(t?(t.cls?t.cls+'::':'')+t.name:'?')+'</span>'
      +confBadge(e.conf)
      +(e.n>1?'<span class="d">'+e.n+' 处</span>':'')
      +'<span class="ctx" style="margin-left:auto">'+esc(t?t.file+':'+t.line:'')+'</span></div>';
  }).join('') : '<div class="empty">（无入边 · 可能仅通过宏/回调引用）</div>';

  /* state read/write inside this function */
  const fieldsHere = {};
  if(src){
    for(let no=f.line; no<=Math.min(f.end||f.line, (D.src[f.file]||[]).length); no++){
      (siteMap[f.file+':'+no]||[]).forEach(x => {
        fieldsHere[x.v] = fieldsHere[x.v] || [];
        fieldsHere[x.v].push({line:no, kind:x.kind});
      });
    }
  }
  const fh = Object.entries(fieldsHere);
  h += '<div class="sec">状态读写（函数体内命中 '+fh.length+' 个字段）</div>';
  h += fh.length ? fh.slice(0,40).map(([vid, sites]) => {
    const v = FIELD[vid];
    return '<div class="rowlink" onclick="goto(\'#v='+vid+'\')">'
      +'<span class="n" style="font-family:var(--mono)">'+esc(v.cls+'.'+v.name)+'</span>'
      +sites.slice(0,4).map(s=>kindBadge(s.kind)+'<span class="where">L'+s.line+'</span>').join(' ')
      +(sites.length>4?'<span class="d">+'+(sites.length-4)+'</span>':'')
      +'</div>';
  }).join('') + (fh.length>40?'<div class="kv">另有 '+(fh.length-40)+' 个字段未列出，见字段视图。</div>':'')
    : '<div class="empty">（无索引字段命中）</div>';

  /* message interactions */
  const pubs = pubByFunc[id] || [];
  h += '<div class="sec">消息发送（publish 命中 '+pubs.length+' 处）</div>';
  h += pubs.length ? pubs.map(x => '<div class="rowlink" onclick="gotoTopic(\''+esc(x.topic)+'\')">'
      +'<span class="where">L'+x.site.line+'</span><span class="tag">'+esc(x.topic)+'</span>'
      +'<span class="ctx" style="margin-left:auto">'+esc(x.site.member)+'</span></div>').join('')
    : '<div class="empty">（无 publish）</div>';

  /* callback wiring */
  const wires = cbTopics[f.name] || [];
  h += '<div class="sec">回调接线</div>';
  h += wires.length ? wires.map(x => '<div class="rowlink" onclick="gotoTopic(\''+esc(x.topic)+'\')">'
      +'<span class="tag">'+esc(x.topic)+'</span><span class="ctx">作为订阅回调注册 · 依据 '+esc(x.site.file)+':'+x.site.line+'</span></div>').join('')
    : '<div class="empty">（未发现该函数被注册为订阅回调；定时器/服务回调未索引）</div>';

  /* log sites */
  const lgs = logByFunc[id] || [];
  h += '<div class="sec">日志点 '+lgs.length+'</div>';
  h += lgs.length ? lgs.slice(0,20).map(l => '<div class="rowlink"><span class="where">L'+l.line+'</span>'
      +sevBadge(l.sev)+'<span class="ctx">'+esc(l.msg.slice(0,140))+'</span></div>').join('')
    + (lgs.length>20?'<div class="kv">另有 '+(lgs.length-20)+' 条…</div>':'')
    : '<div class="empty">（无）</div>';
  h += '</div>';
  detail.innerHTML = h;
}
window.gotoFn = function(id){ goto('#f='+(parseHash().view==='fn'?parseHash().path.concat([id]).join('>'):id)); };
window.toastCycle = function(){ toast('↻ 循环引用：目标函数已在展开路径上，不再重复入栈'); };
window.gotoTopic = function(topic){
  const m = msgOfTopic(topic);
  if(m) goto('#m='+encodeURIComponent(m));
  else { toast('话题 '+topic+' 未关联到 msg 定义'); }
};

/* ---------------- field (state node) views ---------------- */
function renderFieldList(q){
  const groups = {};
  D.fields.forEach(v => {
    if(q){
      const hay = (v.cls+'.'+v.name+' '+v.file).toLowerCase();
      if(!hay.includes(q)) return;
    }
    (groups[v.cls+' @ '+v.file] = groups[v.cls+' @ '+v.file] || []).push(v);
  });
  const keys = Object.keys(groups).sort();
  let h = '<div class="crumb"><b class="cur">状态节点</b><span class="kv">字段按 类@文件 分组 · 读写点为模式匹配启发式</span></div>';
  h += '<div class="card"><input id="fq" class="txt" placeholder="过滤字段（如 lateral / budget / used）" value="'+esc(q)+'">'
    +'<div style="margin-top:8px">';
  h += keys.length ? keys.map(k => '<div class="sec">'+esc(k)+'</div>'
      + groups[k].map(v => '<div class="rowlink" onclick="goto(\'#v='+v.id+'\')">'
        +'<span class="n" style="font-family:var(--mono)">'+esc(v.name)+'</span>'
        +'<span class="d">'+kindCount(v)+'</span></div>').join('')).join('')
    : '<div class="empty">无匹配字段</div>';
  h += '</div></div>';
  detail.innerHTML = h;
  $('#fq').value = q;
  $('#fq').addEventListener('input', e => { fqQuery = e.target.value.trim().toLowerCase();
    renderFieldList(fqQuery);
    const el = $('#fq'); el.focus(); el.setSelectionRange(el.value.length, el.value.length); });
}
function kindCount(v){
  const c = v.counts || {set:0,clear:0,mod:0,read:0};
  const hasClear = (c.clear||0) > 0;
  return c.set+' 设置 · '+c.clear+' 清除 · '+c.mod+' 修改 · '+c.read+' 读取'+(hasClear?'':' · <span title="静态索引未见清除点，视为持续保留（启发式）">保留(未见清除)</span>');
}
function renderField(vid){
  const v = FIELD[vid];
  if(!v){ detail.innerHTML = '<div class="empty">字段不存在</div>'; return; }
  let h = '<div class="crumb"><a onclick="goto(\'#st\')">状态节点</a><span>›</span><b class="cur">'+esc(v.cls+'.'+v.name)+'</b></div>';
  h += '<div class="card"><h2 class="t" style="font-family:var(--mono)">'+esc(v.name)+'</h2>'
    +'<div class="kv">'+esc(v.cls)+' · 声明 <a class="lnk" onclick="goto(\'#file='+encodeURIComponent(v.file)+'\')">'+esc(v.file)+':'+v.line+'</a>'
    +(v.type&&v.type!=='py'?' · 类型 '+esc(v.type):'')+'</div>'
    +'<div class="kv">'+kindCount(v)+'</div>';
  const st = {set:[],clear:[],mod:[],read:[]};
  (v.sites||[]).forEach(s => st[s.kind].push(s));
  ['set','clear','mod','read'].forEach(k => {
    const arr = st[k].sort((a,b)=>a.file<b.file?-1:a.file>b.file?1:a.line-b.line);
    if(!arr.length) return;
    const label = {set:'设置',clear:'清除',mod:'修改',read:'读取'}[k];
    h += '<div class="sec">'+label+' '+arr.length+' 处</div>';
    h += arr.slice(0,60).map(s => {
      const fu = funcAt(s.file, s.line);
      const ctx = s.ctx || ((D.src[s.file]||[])[s.line-1]||'');
      return '<div class="rowlink" onclick="'+(fu?'goto(\'#f='+fu+'\')':'goto(\'#file='+encodeURIComponent(s.file)+'\')')+'">'
        +'<span class="where">'+esc(s.file.split('/').pop())+':'+s.line+'</span>'
        +'<span class="ctx">'+esc(ctx)+'</span></div>';
    }).join('') + (v.counts && v.counts[k] > Math.min(arr.length,60)
      ? '<div class="kv">列表显示前 60 处 · 共 '+v.counts[k]+' 处（计数完整，列表截断）</div>' : '');
  });
  if(!st.clear.length) h += '<div class="note">未发现清除/复位点：该字段一旦设置即持续保留（这是模式匹配结论，不是运行时证明）。</div>';
  h += '</div>';
  detail.innerHTML = h;
}
function funcAt(file, line){
  let best = null;
  D.funcs.forEach(f => { if(f.file===file && f.line<=line && (f.end||f.line)>=line){
    if(!best || f.line>best.line) best = f; } });
  return best ? best.id : null;
}

/* ---------------- message views ---------------- */
function renderMsgList(){
  let h = '<div class="crumb"><b class="cur">消息节点</b><span class="kv">'+D.msgs.length+' 个 msg 定义 · '+Object.keys(D.topics).length+' 个话题</span></div><div class="card">';
  h += '<div class="sec">msg 定义</div>'+D.msgs.map(m =>
    '<div class="rowlink" onclick="goto(\'#m='+encodeURIComponent(m.name)+'\')">'
    +'<span class="n" style="font-family:var(--mono)">'+esc(m.name)+'</span>'
    +'<span class="d">'+m.fields.length+' 字段 · '+(m.consts.length)+' 常量</span></div>').join('');
  h += '<div class="sec">非 msg 话题（标准/可视化消息）</div>';
  const ext = Object.keys(D.topics).filter(t => !msgOfTopic(t));
  h += ext.length ? ext.map(t => '<div class="rowlink"><span class="tag">'+esc(t)+'</span>'
      +'<span class="d">'+esc(D.topics[t].type)+' · pub '+(D.topics[t].pubs||[]).length+' · sub '+(D.topics[t].subs||[]).length+'</span></div>').join('')
    : '<div class="empty">（无）</div>';
  detail.innerHTML = h + '</div>';
}
function renderMsg(name){
  const m = D.msgs.find(x => x.name === name);
  if(!m){ detail.innerHTML = '<div class="empty">消息不存在</div>'; return; }
  let h = '<div class="crumb"><a onclick="goto(\'#msg\')">消息节点</a><span>›</span><b class="cur">'+esc(m.name)+'</b></div>';
  h += '<div class="card"><h2 class="t" style="font-family:var(--mono)">'+esc(m.name)+'.msg</h2>'
    +'<div class="kv"><a class="lnk" onclick="goto(\'#file='+encodeURIComponent(m.file)+'\')">'+esc(m.file)+'</a></div>';
  if(m.consts.length) h += '<div class="sec">常量 / 原因码</div>'+m.consts.map(c =>
    '<div class="rowlink"><span class="where">L'+c.line+'</span><span class="n" style="font-family:var(--mono)">'+esc(c.name)+'='+esc(c.val)+'</span>'
    +(c.comment?'<span class="ctx">'+esc(c.comment.slice(0,120))+'</span>':'')+'</div>').join('');
  h += '<div class="sec">字段</div>'+m.fields.map(f =>
    '<div class="rowlink"><span class="where">L'+f.line+'</span><span class="n" style="font-family:var(--mono)">'+esc(f.type)+' '+esc(f.name)+'</span>'
    +(f.comment?'<span class="ctx">'+esc(f.comment.slice(0,120))+'</span>':'')+'</div>').join('');
  h += '<div class="sec">话题接线（生产者 / 消费者）</div>';
  const topics = Object.entries(m.topics || {});
  h += topics.length ? topics.map(([t, x]) => {
    let r = '<div style="margin:6px 0"><div class="kv"><b class="tag">'+esc(t)+'</b> · 类型 '+esc(x.type)+'</div>';
    r += '<div class="kv">生产者（create_publisher）:</div>' + (x.pubs||[]).map(p =>
      '<div class="rowlink" onclick="'+(funcAt(p.file,p.line)?'goto(\'#f='+funcAt(p.file,p.line)+'\')':'goto(\'#file='+encodeURIComponent(p.file)+'\')')+'">'
      +'<span class="where">'+esc(p.file.split('/').pop())+':'+p.line+'</span>'
      +'<span class="ctx">'+esc(p.member)+' = create_publisher&lt;'+esc(x.type)+'&gt;("'+esc(t+'")')+'</span></div>').join('') || '<div class="empty">（无）</div>';
    r += '<div class="kv">消费者（订阅与回调）:</div>' + (x.subs||[]).map(s => {
      const cbFunc = D.funcs.find(f => f.name === s.cb && f.file === s.file) ||
                     D.funcs.find(f => f.name === s.cb);
      return '<div class="rowlink" onclick="'+(cbFunc?'goto(\'#f='+cbFunc.id+'\')':'goto(\'#file='+encodeURIComponent(s.file)+'\')')+'">'
        +'<span class="where">'+esc(s.file.split('/').pop())+':'+s.line+'</span>'
        +'<span class="ctx">create_subscription → 回调 '+esc(s.cb)+'</span></div>';
    }).join('') || '<div class="empty">（无）</div>';
    r += (x.publish_sites||[]).length ? '<div class="kv">publish 调用点 '+x.publish_sites.length+' 处（含跨文件）:</div>'
      + x.publish_sites.slice(0,30).map(ps =>
        '<div class="rowlink" onclick="'+(ps.func?'goto(\'#f='+ps.func+'\')':'goto(\'#file='+encodeURIComponent(ps.file)+'\')')+'">'
        +'<span class="where">'+esc(ps.file.split('/').pop())+':'+ps.line+'</span>'
        +'<span class="ctx">'+esc(ps.member)+'->publish( … )</span></div>').join('') : '';
    return r+'</div>';
  }).join('') : '<div class="note">未发现该消息类型的 create_publisher / create_subscription 接线（可能仅被转发或未使用）。</div>';
  const src = D.src[m.file];
  if(src) h += '<div class="sec">msg 源文件</div>'+srcHtml(m.file, src, 1, src.length);
  detail.innerHTML = h + '</div>';
}

/* ---------------- log reverse lookup ---------------- */
function renderLog(){
  let h = '<div class="crumb"><b class="cur">日志反查</b><span class="kv">粘贴终端报错 → 定位日志产生位置 → 继续追调用/状态/消息</span></div>';
  h += '<div class="card">'
    +'<textarea id="logq" class="txt" rows="7" placeholder="粘贴终端日志，例如：&#10;[WARN] ... [LATERAL_RECOVERY_STOP] reason=... request=42 ...&#10;[planning_status] VIEWPOINT_LOCAL_REJECTED ..."></textarea>'
    +'<div style="display:flex;gap:8px;margin-top:8px;flex-wrap:wrap">'
    +'<button class="act" onclick="doLog()">分析并定位</button>'
    +'<button class="act sec" onclick="fillDemo()">填入示例（LATERAL_RECOVERY_STOP）</button>'
    +'<span class="kv" style="align-self:center">匹配对象：'+D.logs.length+' 个日志点 + '+Object.keys(D.reasons).length+' 个原因码字符串</span></div>'
    +'<div id="logres"></div></div>';
  detail.innerHTML = h;
  $('#logq').value = logQuery;
  if(logHtml) $('#logres').innerHTML = logHtml;
}
window.fillDemo = function(){
  $('#logq').value = '[scan_planner_node-1] [WARN] [LATERAL_RECOVERY_STOP] reason=HANDOFF_ALIGNMENT_DEVIATED request=8403 trajectory=8402 phase=CAPTURE stop_checks=3 planned=0.412m progress=0.371m overshoot=0.040m longitudinal=0.000m yaw_error=0.310rad speed=0.000m/s pose=(1.200,2.340) start=(1.180,2.310) goal=(1.450,2.500); keeping zero command until new certified candidate';
  doLog();
};
function tokenize(s){
  const tags = [...new Set((s.match(/\[[A-Z][A-Z0-9_]{2,}\]|[A-Z][A-Z0-9_]{5,}/g)||[]).map(x=>x.replace(/[[\]]/g,'')))];
  const kvs = [...new Set((s.match(/\b[a-z_]+=(\S+)/g)||[]).map(x=>x.split('=')[0]))];
  return {tags, kvs};
}
window.doLog = function(){
  const q = $('#logq').value.trim();
  const box = $('#logres');
  if(!q){ box.innerHTML = '<div class="empty">（请先粘贴日志）</div>'; return; }
  const {tags, kvs} = tokenize(q);
  let scored = D.logs.map(l => {
    const lt = tokenize(l.msg || '');
    let sc = 0;
    lt.tags.forEach(t => { if(tags.includes(t)) sc += 3; });
    kvs.forEach(k => { if(l.msg && l.msg.includes(k+'=')) sc += 1; });
    const qWords = q.toLowerCase();
    const hay = ((l.msg||'') + ' ' + l.file).toLowerCase();
    let common = 0;
    (l.msg||'').toLowerCase().split(/[^a-z0-9_]+/).forEach(w => { if(w.length>4 && qWords.includes(w)) common += 1; });
    sc += common * 0.5;
    return {l, sc};
  }).filter(x => x.sc > 0).sort((a,b) => b.sc - a.sc).slice(0, 12);
  let h = '<div class="sec">日志点匹配</div>';
  h += scored.length ? scored.map(x => {
    const fu = x.l.func;
    return '<div class="rowlink" onclick="'+(fu?'goto(\'#f='+fu+'\')':'goto(\'#file='+encodeURIComponent(x.l.file)+'\')')+'">'
      +'<span class="where">'+esc(x.l.file.split('/').pop())+':'+x.l.line+'</span>'
      +sevBadge(x.l.sev)+'<span class="ctx">'+esc(x.l.msg.slice(0,120))+'</span>'
      +'<span class="d">score '+x.sc.toFixed(1)+'</span></div>';
  }).join('') : '<div class="empty">无日志点命中（尝试减少粘贴行数或核对关键字大小写）</div>';
  const tokHits = [];
  Object.keys(D.reasons).forEach(tok => {
    if(tags.includes(tok) || (tok.length>6 && q.includes(tok))) tokHits.push(tok);
  });
  h += '<div class="sec">原因码 / 大写标记命中</div>';
  h += tokHits.length ? tokHits.slice(0,20).map(tok =>
    '<div class="rowlink" onclick="goto(\'#tok='+encodeURIComponent(tok)+'\')">'
    +'<span class="n" style="font-family:var(--mono)">'+esc(tok)+'</span>'
    +'<span class="d">'+D.reasons[tok].length+' 处出现</span></div>').join('')
    : '<div class="empty">（无原因码命中）</div>';
  h += '<div class="note">匹配依据：日志格式串与粘贴文本的 大写标记/键值对/词重叠 打分；点击结果跳到产生函数，可继续看调用、状态读写与消息发送。</div>';
  box.innerHTML = h;
  logQuery = q; logHtml = h;
  try { sessionStorage.setItem('audit.logq', logQuery);
        sessionStorage.setItem('audit.loghtml', logHtml); } catch (e) {}
};

/* ---------------- reason token view ---------------- */
function renderToken(tok){
  const sites = D.reasons[tok] || [];
  let h = '<div class="crumb"><a onclick="goto(\'#log\')">日志反查</a><span>›</span><b class="cur" style="font-family:var(--mono)">'+esc(tok)+'</b></div>';
  h += '<div class="card"><h2 class="t">原因码 / 标记出现在 '+sites.length+' 个位置</h2>';
  h += sites.map(s => {
    const fu = funcAt(s.file, s.line);
    const src = (D.src[s.file]||[])[s.line-1] || '';
    return '<div class="rowlink" onclick="'+(fu?'goto(\'#f='+fu+'\')':'goto(\'#file='+encodeURIComponent(s.file)+'\')')+'">'
      +'<span class="where">'+esc(s.file.split('/').pop())+':'+s.line+'</span>'
      +'<span class="ctx">'+esc(src.trim().slice(0,150))+'</span></div>';
  }).join('') || '<div class="empty">（无）</div>';
  h += '</div>';
  detail.innerHTML = h;
}

/* ---------------- about ---------------- */
function renderAbout(){
  const m = D.meta;
  let h = '<div class="crumb"><b class="cur">说明与限制</b></div><div class="card">';
  h += '<h2 class="t">生成方式（可重复）</h2><pre class="src" style="max-height:none">python3 tools/code_audit/generate.py</pre>'
    +'<div class="kv">产出：本页面 + <span class="tag">tools/code_audit/audit_data.json</span>（索引中间产物，供 diff / 测试）。再生成前请先备份旧页面（脚本首次运行已自动备份为 *.pre-audit.html）。</div>';
  h += '<h2 class="t" style="margin-top:14px">快照</h2><div class="statgrid">'
    +'<div class="stat"><b>'+m.n_funcs+'</b><span>函数</span></div>'
    +'<div class="stat"><b>'+m.n_calls+'</b><span>调用边</span></div>'
    +'<div class="stat"><b>'+m.n_fields+'</b><span>字段</span></div>'
    +'<div class="stat"><b>'+m.n_msgs+'</b><span>消息</span></div>'
    +'<div class="stat"><b>'+m.n_topics+'</b><span>话题</span></div>'
    +'<div class="stat"><b>'+m.n_logs+'</b><span>日志点</span></div>'
    +'<div class="stat"><b>'+Object.keys(D.reasons).length+'</b><span>原因码</span></div>'
    +'<div class="stat"><b>'+m.n_unresolved+'</b><span>未解析调用（已标注，列表 '+m.n_unresolved_listed+' 条）</span></div>'
    +'</div>'
    +'<div class="note">工作区 '+esc(m.workspace)+' · 分支 '+esc(m.branch)+' · HEAD '+esc(m.head)+' · 未提交 '+m.dirty_count+' 个 · '+esc(m.generated_at)+'</div>';
  h += '<h2 class="t" style="margin-top:14px">已知限制（会影响使用）</h2>'
    + m.limits.map(x => '<div class="note">'+esc(x)+'</div>').join('');
  h += '<h2 class="t" style="margin-top:14px">验收映射</h2><div class="kv">'
    +'1 文件树展开 → 文件树页签<br>2 函数调用/状态/消息下钻 → 函数面板各分节<br>'
    +'3 状态设置/读取/清除/保留 → 状态节点页签<br>4 消息定义/生产/消费/接线 → 消息节点页签<br>'
    +'5 终端报错定位 → 日志反查页签<br>6 导航历史/深链接/循环/离线 → 地址栏 hash + 浏览器后退<br>'
    +'7 不确定关系标注 → exact/name/ambig/unres 四级徽标</div>';
  detail.innerHTML = h + '</div>';
}

/* ---------------- boot ---------------- */
$('#c-br').textContent = D.meta.branch;
$('#c-head').textContent = D.meta.head;
$('#c-dirty').textContent = D.meta.dirty_count;
$('#c-gen').textContent = D.meta.generated_at;
render();
</script>
</body>
</html>'''
    return html.replace('__DATA__', data_json)
