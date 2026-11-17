'use strict';
const assert = require('node:assert/strict');
const K = require('../web_demo/kinematics.js').create();
const center = {x: 13, y: 65};
let total = 0;
let worst = 0;
function checkLine(a, b) {
  const count = Math.max(1, Math.ceil(Math.hypot(b.x-a.x, b.y-a.y) / .35));
  for (let i=0; i<=count; i++) {
    const x=a.x+(b.x-a.x)*i/count, y=a.y+(b.y-a.y)*i/count;
    const angle=K.ik(x,y), p=K.fk(angle.t1,angle.t2);
    const error=Math.hypot(p.x-x,p.y-y);
    assert.ok(error<1e-6, `${x},${y}: ${error}`);
    worst=Math.max(worst,error); total++;
  }
}
for (const text of ['8','中','王','CQU']) {
  const p=K.plan(text,{center,start:center}); assert.ok(p.ok,text);
  let cursor=center;
  for (const stroke of p.strokes) {
    checkLine(cursor,stroke.points[0]);
    for (let i=1;i<stroke.points.length;i++) checkLine(stroke.points[i-1],stroke.points[i]);
    cursor=stroke.points.at(-1);
  }
  console.log(`PASS ${text}: ${p.strokes.length} strokes including pen-up travel`);
}
console.log(`PASS 4 trajectories, ${total} samples; worst FK/IK error ${worst} mm (geometry only)`);
