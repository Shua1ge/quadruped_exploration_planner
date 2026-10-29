const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(__dirname+'/../coverage_route/actual_route_viewer.js','utf8');
const handlers={},nodes={};
const selectors=['#actual-route-status','input[type=file]','#load-route-url','#actual-only','#focus-route','input[aria-label="实走 CSV 地址"]'];
for(const id of selectors)nodes[id]={addEventListener:(event,callback)=>handlers[id+':'+event]=callback};
const plot={data:Array.from({length:5},()=>({x:[0],y:[0],z:[0]}))};
const panel={style:{},querySelector:id=>nodes[id]};
let added=[],restyles=[],layouts=[];
const sandbox={URL,URLSearchParams,location:{href:'http://localhost/route_3d.html',origin:'http://localhost',search:''},
 document:{getElementById:()=>plot,createElement:()=>panel,body:{appendChild:()=>{}},querySelector:id=>nodes[id]},
 Plotly:{addTraces:async(p,tr)=>{added=tr;p.data.push(...tr);},deleteTraces:async(p,ids)=>{for(const i of [...ids].sort((a,b)=>b-a))p.data.splice(i,1);},
         restyle:async(p,style,indices)=>{restyles.push({style,indices});},relayout:async(p,layout)=>{layouts.push(layout);}},
 fetch:async()=>({ok:true,text:async()=>csv})};
vm.createContext(sandbox);vm.runInContext(source,sandbox);
const parse=sandbox.ActualRouteCSV.parseCSV;
const csv='x,y,z,elapsed_s,segment,frame_id\n0,0,.45,0,0,world\n3,4,.45,1,0,world\n100,100,.45,1,1,world\n';
assert.strictEqual(parse(csv).distance,5);
assert.throws(()=>parse('x,y\n1,2'),/缺少列/);
assert.throws(()=>parse('x,y,z\n1,,3'),/XYZ/);
assert.throws(()=>parse('x,y,z,frame_id\n1,2,3,odom'),/不能直接叠加/);
assert.strictEqual(parse('\ufeff"x","y","z"\r\n"1","2","3"\r\n').points[0].z,3);
assert.strictEqual(parse('x,y,z,note\n1,2,3,"has, comma"\n').points.length,1);
(async()=>{
 const input=handlers['input[type=file]:change'];
 await input({target:{files:[{name:'synthetic.csv',text:async()=>csv}]}});
 assert.strictEqual(plot.data.length,8);assert.strictEqual(added[0].x[2],null);
 assert.match(nodes['#actual-route-status'].textContent,/平面 5.00 m/);
 await input({target:{files:[{name:'repeat.csv',text:async()=>csv}]}});
 assert.strictEqual(plot.data.length,8); // replace, do not accumulate
 await handlers['#actual-only:change']({target:{checked:true}});
 assert.strictEqual(restyles[0].indices.length,5);assert.strictEqual(restyles[0].style.visible,false);
 handlers['#focus-route:click']();assert.ok(layouts.at(-1)['scene.xaxis.range'][1]>100);
 nodes['input[aria-label="实走 CSV 地址"]'].value='actual.csv';
 await handlers['#load-route-url:click']();assert.strictEqual(plot.data.length,8);
 nodes['input[aria-label="实走 CSV 地址"]'].value='https://other.test/data.csv';
 await handlers['#load-route-url:click']();assert.match(nodes['#actual-route-status'].textContent,/同源/);
 console.log('PASS: CSV schema/quotes/frames, segment gaps, file import, replacement, focus, background toggle, URL import, same-origin error');
})().catch(e=>{console.error(e);process.exitCode=1;});
