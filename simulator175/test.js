'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),cp=require('node:child_process'),os=require('node:os'),path=require('node:path');
const E=require('./engine.js');let checks=0;
function test(name,fn){fn();checks++;console.log('PASS',name);}
const ctx={window:{}};vm.runInNewContext(fs.readFileSync(path.join(__dirname,'data.js'),'utf8'),ctx);const D=ctx.window.PENDULUM_DATA;
const near=(a,b,tol=1e-8)=>assert.ok(Math.abs(a-b)<tol,`${a} != ${b} ± ${tol}`);
test('source SHA-256 matches all raw calibration captures and selected control log',()=>{
 const crypto=require('node:crypto');
 for(const s of [...D.captures,D.control_log,...D.sources]){
 const name=s.source||s.path;
 const hash=crypto.createHash('sha256').update(fs.readFileSync(path.join(__dirname,'..',name))).digest('hex');assert.equal(hash,s.sha256);}
 assert.equal(D.captures.length,3);assert.equal(D.fit.total_cycles,33);
});
test('JavaScript motion governor matches current compiled C++ firmware header',()=>{
 const dir=fs.mkdtempSync(path.join(os.tmpdir(),'pendulum-parity-'));
 try{
 const bin=path.join(dir,'reference');cp.execFileSync('clang++',['-std=c++11','-O2','-Wall','-Wextra','-Werror',path.join(__dirname,'firmware_parity.cpp'),'-o',bin]);
 const lines=cp.execFileSync(bin,{encoding:'utf8'}).trim().split('\n');
 for(const line of lines){const [tag,...rest]=line.split(' ');const v=rest.map(Number);
 if(tag==='gain')E.gains(E.defaults).forEach((x,i)=>near(x,v[i],1e-5));
 else if(tag==='capture'){const [q,w,x,velocity,a,expected]=v;assert.equal(Number(E.canCapture(E.defaults,q,w,x,velocity,a)),expected);}
 else {const [x,request,velocity,a,nv,na,brake]=v;
 const s=E.advance({v:velocity,a,brake:0},x,request,{...E.defaults,vmax:1.5,amax_s:6,amax_b:6,jmax:150},6,.001);near(s.v,nv,2e-6);near(s.a,na,2e-6);assert.equal(s.brake,brake);}}
 }finally{fs.rmSync(dir,{recursive:true,force:true});}
});
test('RK4 conserves pendulum energy without friction or cart motion',()=>{
 const p=E.parameters({damping:0}),energy=(q,w)=>.5*p.leff/9.81*w*w+Math.cos(q);let q=2,w=0;const en=energy(q,w);
 for(let i=0;i<20000;i++)[q,w]=E.integrate(q,w,0,p,.001);near(energy(q,w),en,1e-8);
});
test('free-decay small-angle period matches calibrated effective length',()=>{
 const p=E.parameters({damping:0});let q=Math.PI+.01,w=0,cross=[];
 for(let i=0;i<5000;i++){const old=E.wrap(q-Math.PI);[q,w]=E.integrate(q,w,0,p,.001);if(old>0&&E.wrap(q-Math.PI)<=0)cross.push(i*.001);}
 const periods=cross.slice(1).map((x,i)=>x-cross[i]);near(periods.reduce((a,b)=>a+b)/periods.length,2*Math.PI*Math.sqrt(p.leff/9.81),.001);
});
test('plant integration converges at half step over a driven trajectory',()=>{
 let q=2.8,w=1,q2=q,w2=w;const p=E.parameters(),p2=E.parameters({integrationDt:.0005});
 for(let i=0;i<10000;i++){const a=2*Math.sin(i*.001*4);[q,w]=E.integrate(q,w,a,p,.001);[q2,w2]=E.integrate(q2,w2,a,p2,.001);}
 near(q,q2,1e-6);near(w,w2,1e-5);
});
test('all styles enforce acceleration/speed/pulse limits across a limit grid',()=>{
 for(const style of Object.keys(E.styles))for(const amax of [2,6,10])for(const vmax of [.3,1.5]){
 const r=E.simulate({style,scenario:'reversal',amax_s:amax,amax_b:amax,vmax,manualSpeed:1,duration:5});
 assert.ok(r.summary.peakA<=amax+1e-7);assert.ok(r.summary.peakV<=vmax+.00201);assert.ok(r.summary.peakHz<=r.parameters.pulseMax);
 if(style!=='trapezoid')assert.ok(r.summary.peakJ<=r.parameters.jmax+1e-7);
 assert.ok(r.trace.every(s=>Number.isFinite(s.theta)&&Number.isFinite(s.x)));
 }
});
test('balance recovery, rail constraints, and commanded/physical separation',()=>{
 const r=E.simulate({scenario:'balance',initialAngle:5});assert.equal(r.summary.status,'balanced');assert.ok(r.summary.finalStable>15);assert.ok(r.summary.peakX<.14);
 const lag=E.simulate({scenario:'reversal',tracking:.5,lagMs:20,duration:2});assert.ok(lag.trace.some(s=>Math.abs(s.x-s.pulseX)>.01));
 const stuck=E.simulate({tracking:0});assert.equal(stuck.summary.fault,'No pendulum response');assert.equal(stuck.summary.peakX,0);
 const pulse=E.simulate({scenario:'reversal',pulseMax:50});assert.equal(pulse.summary.fault,'STEP pulse-rate ceiling');
});
test('stepper styles have materially different trajectories and jerk behavior',()=>{
 const j=E.simulate({scenario:'reversal'}),t=E.simulate({scenario:'reversal',style:'trapezoid'}),v=E.simulate({scenario:'reversal',style:'velocity'});
 assert.ok(t.summary.peakJ>j.summary.peakJ*10);assert.notDeepEqual(j.trace,v.trace);
});
test('decay overlays and recorded waveform replay remain finite',()=>{
 for(const c of D.captures){const fit=E.decay(c);assert.ok(fit.rmseDeg>0&&fit.rmseDeg<5);assert.ok(fit.points.length>400);}
 const r=E.simulate({scenario:'replay',duration:10},D.control_log);assert.ok(r.trace.length>50);assert.ok(r.trace.every(s=>Number.isFinite(s.theta)));
});
test('invalid values and missing replay input fail clearly',()=>{
 for(const p of [{jmax:0},{amax_s:-1},{vmax:NaN},{duration:Infinity},{leff:0},{style:'bad'},{initialX:.145}])assert.throws(()=>E.simulate(p));
 assert.throws(()=>E.simulate({scenario:'replay'}));
});
test('automatic rail recovery stops, recenters, and resumes within software rail limits',()=>{
 for(const initialX of [-.131,.131]){
  const r=E.simulate({initialX,duration:5});
  assert.equal(r.summary.fault,null);
  assert.ok(r.events.some(e=>e.event==='recenter complete'));
  assert.ok(r.trace.some(s=>s.mode==='recenter'));
  assert.ok(r.summary.peakPulseX<.14);
 }
 const r=new E.RailRecovery();assert.ok(!r.atEdge(.129,.15));assert.ok(r.atEdge(.131,.15));
});
console.log(`${checks} test groups passed.`);
