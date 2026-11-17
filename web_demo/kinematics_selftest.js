/* cad_demo/kinematics_selftest.js — 离线自检（node 运行）
 * node kinematics_selftest.js
 */
'use strict';
const path = require('path');
const FB = require(path.join(__dirname, 'kinematics.js'));
const K = FB.create();

const fmt = (v, n = 3) => Number(v).toFixed(n);

console.log('=== 1) 几何派生量（默认 25/35/35/45/45/13 + offset 45°，仅演示参数）');
const D = K.derived;
console.log('  LEFF =', fmt(D.LEFF), ' GAMMA =', fmt(D.GAMMA_DEG), '°');
console.log('  笔尖相对左轴 A 环: [', fmt(D.RIN), ',', fmt(D.ROUT), ']');
console.log('  腕点相对右轴 B 环: [', fmt(D.R2IN), ',', fmt(D.R2OUT), ']');

console.log('\n=== 2) 正逆解闭环（纯几何，不含电气限位）');
const sc = K.selfCheck({ x0: 0, x1: 30, y0: 35, y1: 95, step: 2 });
console.log(`  采样 ${sc.tested} 点，可解 ${sc.ok} 点，最大回代误差 ${sc.worstRoundTripMm.toExponential(2)} mm`);
if (sc.worstRoundTripMm > 1e-6) { console.error('  !! 闭环误差超限'); process.exitCode = 1; }

console.log('\n=== 3) 字框可达性扫描（20mm 字框整框含四角，纯几何）');
const scan = K.scanRegion({ x0: -5, x1: 35, y0: 35, y1: 95, step: 2.5, half: 10 });
console.log(`  可行中心采样 ${scan.count} 个，bbox x ${fmt(scan.bbox.x0,1)}..${fmt(scan.bbox.x1,1)}  y ${fmt(scan.bbox.y0,1)}..${fmt(scan.bbox.y1,1)}`);

console.log('\n=== 4) 字形方案（20mm 字框 + 2mm 间距，整组居中）');
function report(text, cx, cy) {
  const p = K.plan(text, { center: { x: cx, y: cy }, start: { x: cx, y: cy } });
  const bb = p.bbox;
  const line = `  "${text}" @(${cx},${cy}): ${p.ok ? 'OK' : 'FAIL(' + p.failures.length + ')'}` +
    (bb ? `  bbox x ${fmt(bb.x0,1)}..${fmt(bb.x1,1)} y ${fmt(bb.y0,1)}..${fmt(bb.y1,1)}` : '') +
    `  笔数=${p.strokes.length} 空移段=${p.travel.length} 最大步进=${fmt(p.maxStepDeg,2)}°@${p.maxStepWhere}`;
  console.log(line);
  if (!p.ok) {
    p.failures.slice(0, 3).forEach(f => console.log(`     首失败(${f.kind}): 第${f.stroke + 1}笔 (${fmt(f.x,2)},${fmt(f.y,2)}) ${f.code} ${f.error}`));
  }
  return p;
}
const centers = [[13, 65], [13, 60], [12.5, 60], [15, 67]];
for (const [cx, cy] of centers) {
  for (const t of ['8', '中', '王', 'CQU']) report(t, cx, cy);
  console.log('');
}

console.log('=== 5) 笔画/空移连续性与笔序（固定分支，不得换支或跳点）');
for (const t of ['8', '中', '王', 'CQU']) {
  const p = K.plan(t, { center: { x: 13, y: 65 }, start: { x: 13, y: 65 } });
  const seq = p.strokes.map((s, i) => `${i + 1}:${s.ch}/${s.points.length}点/${fmt(s.maxStepDeg,1)}°`).join('  ');
  console.log(`  "${t}": ${seq}`);
  if (p.travel.length) {
    console.log(`     空移: ${p.travel.map(tr => `${fmt(Math.hypot(tr.to.x - tr.from.x, tr.to.y - tr.from.y),1)}mm/${tr.samples}点/${tr.ok ? 'OK' : 'FAIL'}`).join('  ')}`);
  }
}

console.log('\n=== 6) 参考点角度表（供 HTML 侧比对）');
[[13, 65], [3, 55], [23, 75], [13, 55], [13, 75]].forEach(([x, y]) => {
  const s = K.tryIk(x, y);
  if (!s.ok) console.log(`  (${x},${y}) -> ${s.code}`);
  else {
    const f = K.fk(s.t1, s.t2);
    console.log(`  (${x},${y}) -> θ1=${fmt(s.t1)} θ2=${fmt(s.t2)}  回代(${fmt(f.x)},${fmt(f.y)}) 腕点(${fmt(f.wx,1)},${fmt(f.wy,1)})`);
  }
});
