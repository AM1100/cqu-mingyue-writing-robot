/*!
 * kinematics.js — 五连杆写字机构「纯几何」正/逆解统一模块（CAD 演示用）
 * ---------------------------------------------------------------------------
 * 只用几何：连杆长度 + 装配分支。**不含任何舵机脉宽 / 电气角 / 增益映射**，
 * 也不代表真机可达性——真机还受电气限位与标定影响，不在本模块职责内。
 *
 * 尺寸默认取自用户实机基线（待 CAD 复核，不是实测）：
 *   L1=25 (**两轴全距**：左轴 A=(0,0)，右轴 B=(L1,0)=(25,0))
 *   L2=L3=35 (主动臂) L4=L5=45 (连杆) LT=13 (笔偏移) OFFSET=45°
 *   分支 LEFT_BRANCH=+1, RIGHT_BRANCH=-1, WRIST_BRANCH=+1
 * 角度单位统一 **度**，坐标单位 **mm**，与固件一致。
 *
 * 插补范围（重要）：本模块**只解顶点**。笔画内部各线段之间**不做插补**，
 *   plan() 只对笔间抬笔空移按 travelStep 采样。若需要连续轨迹，调用方自行
 *   沿线段取点后交给 solvePath()；不要把 plan() 当成"全程轨迹已逐帧校验"。
 *
 * 用法：
 *   浏览器 <script src="kinematics.js"></script>  -> window.FiveBarK
 *   Node    const FiveBarK = require('./kinematics.js');
 *   const K = FiveBarK.create();              // 默认参数
 *   const s = K.ik(13, 65);                   // {ok,t1,t2}
 *   const p = K.fk(130.95, 44.88);            // {x,y,...}
 *
 * 约定：任何"不可解/奇异/分支不符"都**抛错**（KinematicsError，带 .code），
 *       调用方必须显式处理，绝不允许静默换支或跳点。
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else if (typeof define === 'function' && define.amd) define([], factory);
  else root.FiveBarK = factory();
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  var DEG = 180 / Math.PI;
  var RAD = Math.PI / 180;

  // ---------------------------------------------------------------- 默认参数
  var DEFAULTS = {
    L1: 25.0, L2: 35.0, L3: 35.0, L4: 45.0, L5: 45.0, LT: 13.0,
    OFFSET_DEG: 45.0,            // CAD 偏置幅值45°；演示选+45°观察侧，实机符号待核对
    LEFT_BRANCH: 1,              // 逆解取支符号（固定，不允许逐点切换）
    RIGHT_BRANCH: -1,
    WRIST_BRANCH: 1,             // 正解取腕点符号（与装配一致）
    CHAR_BOX_MM: 20.0,           // 单字 20x20 字框
    CHAR_GAP_MM: 2.0,            // 字间距
    ROUND_TRIP_TOL: 1e-6,        // 逆解回代容差 (mm²)
    DISCONTINUITY_DEG: 90.0      // 相邻点角度跳变告警阈值
  };

  // ------------------------------------------------------------- 字符笔画库
  // 本地坐标固定 0..20（对应 20x20 字框），与固件 _GLYPH_STROKES 完全一致
  var GLYPHS = {
    '王': [
      [[3.0, 18.0], [17.0, 18.0]],
      [[4.5, 10.0], [15.5, 10.0]],
      [[10.0, 18.0], [10.0, 2.0]],
      [[2.0, 2.0], [18.0, 2.0]]
    ],
    '中': [
      [[3.5, 14.5], [3.5, 6.5]],
      [[3.5, 14.5], [16.5, 14.5], [16.5, 6.5]],
      [[3.5, 6.5], [16.5, 6.5]],
      [[10.0, 19.0], [10.0, 1.0]]
    ],
    'C': [
      [[15.5, 16.5], [12.5, 19.0], [7.5, 19.0], [4.5, 16.0], [3.5, 10.0],
       [4.5, 4.0], [7.5, 1.0], [12.5, 1.0], [15.5, 3.5]]
    ],
    'Q': [
      [[10.0, 19.0], [14.0, 17.5], [16.0, 14.0], [16.0, 6.0], [14.0, 2.5],
       [10.0, 1.0], [6.0, 2.5], [4.0, 6.0], [4.0, 14.0], [6.0, 17.5], [10.0, 19.0]],
      [[11.5, 5.5], [16.5, 1.0]]
    ],
    'U': [
      [[4.0, 19.0], [4.0, 5.5], [6.0, 2.0], [10.0, 1.0], [14.0, 2.0],
       [16.0, 5.5], [16.0, 19.0]]
    ],
    '0': [
      [[10.0, 19.0], [14.0, 17.0], [15.5, 10.0], [14.0, 3.0], [10.0, 1.0],
       [6.0, 3.0], [4.5, 10.0], [6.0, 17.0], [10.0, 19.0]]
    ],
    '1': [[[7.5, 16.5], [10.0, 19.0], [10.0, 1.0]]],
    '2': [
      [[4.5, 15.5], [6.5, 18.5], [10.0, 19.0], [13.5, 18.5], [15.5, 15.0],
       [14.5, 11.5], [4.5, 1.0], [15.5, 1.0]]
    ],
    '3': [
      [[4.5, 16.5], [7.0, 19.0], [13.0, 19.0], [15.5, 16.0], [15.0, 12.0],
       [11.5, 10.0], [8.0, 10.0]],
      [[11.5, 10.0], [15.5, 8.0], [15.5, 3.5], [13.0, 1.0], [6.5, 1.0], [4.5, 3.5]]
    ],
    '4': [
      [[12.5, 19.0], [4.5, 6.5], [16.0, 6.5]],
      [[12.5, 19.0], [12.5, 1.0]]
    ],
    '5': [
      [[15.5, 19.0], [5.5, 19.0], [5.0, 11.5], [11.5, 12.0], [15.0, 9.5],
       [15.5, 5.0], [13.0, 1.5], [7.5, 1.0], [4.5, 2.5]]
    ],
    '6': [
      [[14.0, 18.0], [10.5, 19.0], [6.5, 17.0], [4.5, 10.0], [5.0, 3.5],
       [7.5, 1.0], [12.5, 1.0], [15.5, 4.0], [15.0, 8.5], [12.0, 10.5],
       [6.0, 10.0], [4.5, 7.5]]
    ],
    '7': [[[4.5, 19.0], [15.5, 19.0], [8.0, 1.0]]],
    '8': [
      [[10.0, 10.0], [6.0, 11.5], [5.0, 15.0], [6.5, 18.0], [10.0, 19.0],
       [13.5, 18.0], [15.0, 15.0], [14.0, 11.5], [10.0, 10.0], [5.5, 8.0],
       [4.5, 4.0], [6.5, 1.5], [10.0, 1.0], [13.5, 1.5], [15.5, 4.0],
       [14.5, 8.0], [10.0, 10.0]]
    ],
    '9': [
      [[15.5, 12.5], [13.5, 9.5], [8.5, 9.0], [5.0, 11.5], [4.5, 15.5],
       [7.0, 18.5], [11.5, 19.0], [15.0, 16.5], [15.5, 10.0], [13.5, 3.0],
       [9.5, 1.0], [6.0, 2.0]]
    ]
  };

  // ------------------------------------------------------------------- 错误
  function KinematicsError(code, message, detail) {
    this.name = 'KinematicsError';
    this.code = code;
    this.message = message;
    this.detail = detail || null;
  }
  KinematicsError.prototype = Object.create(Error.prototype);
  KinematicsError.prototype.constructor = KinematicsError;

  function isFiniteNum(v) { return typeof v === 'number' && isFinite(v); }
  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

  // ------------------------------------------------------------ 实例化工厂
  function create(userCfg) {
    var cfg = {};
    for (var k in DEFAULTS) if (Object.prototype.hasOwnProperty.call(DEFAULTS, k)) cfg[k] = DEFAULTS[k];
    if (userCfg) for (var j in userCfg) if (Object.prototype.hasOwnProperty.call(userCfg, j)) cfg[j] = userCfg[j];

    var L1 = cfg.L1, L2 = cfg.L2, L3 = cfg.L3, L4 = cfg.L4, L5 = cfg.L5, LT = cfg.LT;
    var offset = clamp(cfg.OFFSET_DEG, 0, 180) * RAD;
    var LB = cfg.LEFT_BRANCH >= 0 ? 1 : -1;
    var RB = cfg.RIGHT_BRANCH >= 0 ? 1 : -1;
    var WB = cfg.WRIST_BRANCH >= 0 ? 1 : -1;

    // 等效笔长与偏角（与固件同式）
    var LEFF = Math.sqrt(L4 * L4 + LT * LT + 2 * L4 * LT * Math.cos(offset));
    var GAMMA = Math.atan2(LT * Math.sin(offset), L4 + LT * Math.cos(offset));

    // 纯几何可达环（不含电气限位）
    var RIN = Math.abs(L2 - LEFF), ROUT = L2 + LEFF;        // 笔尖相对左轴 A 的半径范围
    var R2IN = Math.abs(L3 - L5), R2OUT = L3 + L5;          // 腕点相对右轴 B 的半径范围

    // ---------------------------------------------------------------- 正解
    function fk(t1deg, t2deg) {
      if (!isFiniteNum(t1deg) || !isFiniteNum(t2deg)) {
        throw new KinematicsError('NON_FINITE', '正解角度必须为有限数: ' + t1deg + ', ' + t2deg);
      }
      var t1 = t1deg * RAD, t2 = t2deg * RAD;
      var ex = L2 * Math.cos(t1), ey = L2 * Math.sin(t1);
      var fx = L1 + L3 * Math.cos(t2), fy = L3 * Math.sin(t2);
      var dx = fx - ex, dy = fy - ey;
      var d = Math.sqrt(dx * dx + dy * dy);
      if (d < 1e-9) throw new KinematicsError('SINGULAR', '两肘重合，机构奇异', { t1: t1deg, t2: t2deg });
      var along = (L4 * L4 - L5 * L5 + d * d) / (2 * d);
      var h2 = L4 * L4 - along * along;
      if (h2 <= 1e-9) {
        throw new KinematicsError('UNCLOSABLE', '从动臂不可闭合（两肘间距 ' + d.toFixed(3) + ' mm 超出连杆可及）',
          { d: d, t1: t1deg, t2: t2deg });
      }
      var h = Math.sqrt(h2) * WB;
      var wx = ex + along * dx / d - h * dy / d;
      var wy = ey + along * dy / d + h * dx / d;
      var phi = Math.atan2(wy - ey, wx - ex);
      return {
        x: wx + LT * Math.cos(phi + offset),
        y: wy + LT * Math.sin(phi + offset),
        wx: wx, wy: wy,
        ex: ex, ey: ey, fx: fx, fy: fy,
        phiDeg: phi * DEG,
        t1: t1deg, t2: t2deg, d: d
      };
    }

    // ---------------------------------------------------------------- 逆解
    function ik(x, y) {
      if (!isFiniteNum(x) || !isFiniteNum(y)) {
        throw new KinematicsError('NON_FINITE', '目标坐标必须为有限数: ' + x + ', ' + y);
      }
      var r = Math.sqrt(x * x + y * y);
      if (r < 1e-9) throw new KinematicsError('DEGENERATE', '目标位于左轴中心 A(0,0)', { x: x, y: y });
      if (r < RIN - 1e-6 || r > ROUT + 1e-6) {
        throw new KinematicsError('UNREACHABLE',
          '笔尖到左轴距离 r=' + r.toFixed(3) + ' mm 超出 [' + RIN.toFixed(3) + ', ' + ROUT.toFixed(3) + ']',
          { x: x, y: y, r: r });
      }
      var arg = clamp((L2 * L2 + r * r - LEFF * LEFF) / (2 * L2 * r), -1, 1);
      var t1 = Math.atan2(y, x) + LB * Math.acos(arg);
      var ex = L2 * Math.cos(t1), ey = L2 * Math.sin(t1);

      var dxp = x - ex, dyp = y - ey;
      if (Math.sqrt(dxp * dxp + dyp * dyp) < 1e-9) {
        throw new KinematicsError('SINGULAR', '笔尖与肘点重合，机构奇异', { x: x, y: y });
      }
      var phi = Math.atan2(dyp, dxp) - GAMMA;
      var wx = ex + L4 * Math.cos(phi), wy = ey + L4 * Math.sin(phi);

      var bx = wx - L1, by = wy;
      var r2 = Math.sqrt(bx * bx + by * by);
      if (r2 < 1e-9) throw new KinematicsError('DEGENERATE', '腕点位于右轴中心 B', { x: x, y: y });
      if (r2 < R2IN - 1e-6 || r2 > R2OUT + 1e-6) {
        throw new KinematicsError('UNREACHABLE',
          '腕点到右轴距离 ' + r2.toFixed(3) + ' mm 超出 [' + R2IN.toFixed(3) + ', ' + R2OUT.toFixed(3) + ']',
          { x: x, y: y, r2: r2 });
      }
      var arg2 = clamp((L3 * L3 + r2 * r2 - L5 * L5) / (2 * L3 * r2), -1, 1);
      var t2 = Math.atan2(by, bx) + RB * Math.acos(arg2);

      var a1 = t1 * DEG, a2 = t2 * DEG;

      // 回代校验：解必须属于当前装配分支，否则明确报错（绝不换支迁就）
      var back = fk(a1, a2);
      var err = (back.x - x) * (back.x - x) + (back.y - y) * (back.y - y);
      if (err > cfg.ROUND_TRIP_TOL) {
        throw new KinematicsError('BRANCH_MISMATCH',
          '目标不属于当前装配分支（回代误差 ' + Math.sqrt(err).toFixed(4) + ' mm）',
          { x: x, y: y, t1: a1, t2: a2, err: Math.sqrt(err) });
      }
      return { ok: true, t1: a1, t2: a2, x: x, y: y };
    }

    function tryIk(x, y) {
      try { return ik(x, y); }
      catch (e) {
        if (e instanceof KinematicsError) return { ok: false, code: e.code, error: e.message, x: x, y: y };
        throw e;
      }
    }

    // ------------------------------------------------------- 角度展开与连续性
    function wrap180(a) {
      var r = ((a + 180) % 360 + 360) % 360 - 180;
      return r === -180 ? 180 : r;
    }
    function unwrapTo(a, ref) { return a + 360 * Math.round((ref - a) / 360); }
    function degDiff(a, b) { return wrap180(a - b); }

    /**
     * 沿一串点求解，保持角度连续（只在 ±360k 上展开，不换装配分支）。
     * @param {Array<{x:number,y:number}>|Array<[number,number]>} points
     * @param {{startRef?:{t1:number,t2:number}}} [opts]
     * @returns {{ok:boolean, solutions:Array, failures:Array, maxStepDeg:number,
     *            maxStepAt:number, resolved:boolean}}
     */
    function solvePath(points, opts) {
      opts = opts || {};
      var sols = [], fails = [];
      var ref = opts.startRef || null;
      var maxStep = 0, maxAt = -1;
      for (var i = 0; i < points.length; i++) {
        var p = points[i];
        var x = Array.isArray(p) ? p[0] : p.x;
        var y = Array.isArray(p) ? p[1] : p.y;
        var s;
        try { s = ik(x, y); }
        catch (e) {
          if (!(e instanceof KinematicsError)) throw e;
          fails.push({ index: i, x: x, y: y, code: e.code, error: e.message });
          sols.push(null);
          continue;
        }
        if (ref) {
          s.t1 = unwrapTo(s.t1, ref.t1);
          s.t2 = unwrapTo(s.t2, ref.t2);
          var d1 = Math.abs(s.t1 - ref.t1), d2 = Math.abs(s.t2 - ref.t2);
          var dmax = Math.max(d1, d2);
          if (dmax > maxStep) { maxStep = dmax; maxAt = i; }
        }
        ref = { t1: s.t1, t2: s.t2 };
        sols.push(s);
      }
      var resolved = true;
      if (ref && opts.endRef) {
        // 检验末点是否与调用方给定参考一致（用于多段拼接）
        resolved = Math.abs(unwrapTo(ref.t1, opts.endRef.t1) - opts.endRef.t1) < 1e-6;
      }
      return {
        ok: fails.length === 0,
        solutions: sols,
        failures: fails,
        maxStepDeg: maxStep,
        maxStepAt: maxAt,
        discontinuity: maxStep > cfg.DISCONTINUITY_DEG,
        last: ref,
        resolved: resolved
      };
    }

    // ------------------------------------------------------------- 文字排版
    function normalizeText(text) {
      if (typeof text === 'number') text = String(text);
      if (typeof text !== 'string' || !text.trim()) {
        throw new KinematicsError('BAD_TEXT', '文字必须是非空字符串或数字: ' + text);
      }
      var out = [];
      for (var i = 0; i < text.length; i++) {
        var ch = text[i];
        var key = /[a-z]/.test(ch) ? ch.toUpperCase() : ch;
        if (ch === ' ') continue;
        if (!GLYPHS[key]) {
          throw new KinematicsError('BAD_TEXT', '不支持的字符 "' + ch + '"（支持 王/中/0-9/C/Q/U）');
        }
        out.push(key);
      }
      if (!out.length) throw new KinematicsError('BAD_TEXT', '文字不能全为空格');
      return out;
    }

    /** 每字 20x20 字框（整体居中），返回 [{ch, box:{x0,y0,x1,y1}}] */
    function layoutText(text, options) {
      options = options || {};
      var chars = normalizeText(text);
      var box = options.box != null ? options.box : cfg.CHAR_BOX_MM;
      var gap = options.gap != null ? options.gap : cfg.CHAR_GAP_MM;
      var c = options.center || { x: 0, y: 0 };
      var n = chars.length;
      var totalW = n * box + (n - 1) * gap;
      var startX = c.x - totalW / 2;
      var y0 = c.y - box / 2, y1 = c.y + box / 2;
      var out = [];
      for (var i = 0; i < n; i++) {
        var x0 = startX + i * (box + gap);
        out.push({ ch: chars[i], box: { x0: x0, y0: y0, x1: x0 + box, y1: y1 } });
      }
      return out;
    }

    /** 文字 → 笔画点列（mm，绝对坐标） */
    function textStrokes(text, options) {
      options = options || {};
      var box = options.box != null ? options.box : cfg.CHAR_BOX_MM;
      var scale = box / 20.0;
      var strokes = [];
      var boxes = layoutText(text, options);
      for (var i = 0; i < boxes.length; i++) {
        var b = boxes[i], glyph = GLYPHS[b.ch];
        for (var s = 0; s < glyph.length; s++) {
          var pts = [];
          for (var p = 0; p < glyph[s].length; p++) {
            pts.push({ x: b.box.x0 + glyph[s][p][0] * scale, y: b.box.y0 + glyph[s][p][1] * scale });
          }
          strokes.push({ ch: b.ch, index: s, points: pts });
        }
      }
      return strokes;
    }

    function sampleLine(a, b, stepMm) {
      var d = Math.hypot(b.x - a.x, b.y - a.y);
      var n = Math.max(1, Math.ceil(d / (stepMm || 1.0)));
      var out = [];
      for (var i = 0; i <= n; i++) {
        out.push({ x: a.x + (b.x - a.x) * i / n, y: a.y + (b.y - a.y) * i / n });
      }
      return out;
    }

    /**
     * 方案装配：排版 + 每笔**顶点**逆解 + 笔间抬笔空移（按 travelStep 采样校验）。
     * 注意：**笔画内部不插补**，每笔只解字形的控制点；需要逐帧连续轨迹请自行
     * 沿线段取点后调用 solvePath()。
     * @param {string|number} text
     * @param {{center?:{x,y}, box?:number, gap?:number, start?:{x,y}, travelStep?:number}} [options]
     * @returns {{ok, boxes, strokes:Array, travel:Array, failures:Array, maxStepDeg:number,
     *            maxStepWhere:string, bbox:{x0,y0,x1,y1}}}
     */
    function plan(text, options) {
      options = options || {};
      var boxes = layoutText(text, options);
      var strokes = textStrokes(text, options);
      var travelStep = options.travelStep || 1.0;
      var ref = null;
      var allFail = [];
      var outStrokes = [], travel = [];
      var minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      var maxStep = 0, maxWhere = '';
      var cursor = options.start ? { x: options.start.x, y: options.start.y } : null;

      for (var i = 0; i < strokes.length; i++) {
        var st = strokes[i];

        // ---- 抬笔空移（上一笔末点 → 本笔首点）
        if (cursor) {
          var a = cursor, b = st.points[0];
          if (Math.hypot(b.x - a.x, b.y - a.y) > 1e-9) {
            var seg = sampleLine(a, b, travelStep);
            var tr = solvePath(seg, { startRef: ref });
            for (var tf = 0; tf < tr.failures.length; tf++) {
              allFail.push({
                kind: 'travel', stroke: i, from: a, to: b,
                x: tr.failures[tf].x, y: tr.failures[tf].y,
                code: tr.failures[tf].code, error: tr.failures[tf].error
              });
            }
            if (tr.maxStepDeg > maxStep) { maxStep = tr.maxStepDeg; maxWhere = 'travel#' + i; }
            travel.push({ from: a, to: b, samples: seg.length, solutions: tr.solutions, ok: tr.ok, failures: tr.failures });
            ref = tr.last || ref;
          }
        }

        // ---- 落笔走本笔
        var r = solvePath(st.points, { startRef: ref });
        for (var f = 0; f < r.failures.length; f++) {
          allFail.push({
            kind: 'stroke', stroke: i, ch: st.ch, index: r.failures[f].index,
            x: r.failures[f].x, y: r.failures[f].y,
            code: r.failures[f].code, error: r.failures[f].error
          });
        }
        // 笔内最大步进（不含与上一笔的衔接）
        var inner = 0;
        for (var k = 1; k < r.solutions.length; k++) {
          if (!r.solutions[k] || !r.solutions[k - 1]) continue;
          inner = Math.max(inner,
            Math.abs(r.solutions[k].t1 - r.solutions[k - 1].t1),
            Math.abs(r.solutions[k].t2 - r.solutions[k - 1].t2));
        }
        if (inner > maxStep) { maxStep = inner; maxWhere = 'stroke#' + i; }

        outStrokes.push({
          ch: st.ch, index: st.index, points: st.points,
          solutions: r.solutions, ok: r.ok, failures: r.failures,
          maxStepDeg: inner
        });

        for (var q = 0; q < st.points.length; q++) {
          var pt = st.points[q];
          if (pt.x < minX) minX = pt.x;
          if (pt.x > maxX) maxX = pt.x;
          if (pt.y < minY) minY = pt.y;
          if (pt.y > maxY) maxY = pt.y;
        }
        ref = r.last || ref;
        cursor = st.points[st.points.length - 1];
      }
      return {
        ok: allFail.length === 0,
        boxes: boxes,
        strokes: outStrokes,
        travel: travel,
        failures: allFail,
        maxStepDeg: maxStep,
        maxStepWhere: maxWhere,
        discontinuity: maxStep > cfg.DISCONTINUITY_DEG,
        bbox: isFinite(minX) ? { x0: minX, y0: minY, x1: maxX, y1: maxY } : null
      };
    }

    // ---------------------------------------------------- 可达性 / 演示字区
    function reachable(x, y) { return tryIk(x, y).ok; }

    /** 以 (cx,cy) 为中心的方形字框是否全可达（含四角与中心） */
    function boxOk(cx, cy, half) {
      var pts = [[cx - half, cy - half], [cx + half, cy - half], [cx + half, cy + half],
                 [cx - half, cy + half], [cx, cy]];
      for (var i = 0; i < pts.length; i++) if (!reachable(pts[i][0], pts[i][1])) return false;
      return true;
    }

    /** 扫描一片区域，返回全可达的 (cx,cy) 采样点 */
    function scanRegion(opts) {
      opts = opts || {};
      var x0 = opts.x0, x1 = opts.x1, y0 = opts.y0, y1 = opts.y1;
      var step = opts.step || 2.5, half = opts.half != null ? opts.half : cfg.CHAR_BOX_MM / 2;
      var good = [];
      for (var y = y0; y <= y1 + 1e-9; y += step) {
        for (var x = x0; x <= x1 + 1e-9; x += step) {
          if (boxOk(x, y, half)) good.push({ x: x, y: y });
        }
      }
      var bx = null;
      if (good.length) {
        var mnx = Infinity, mxx = -Infinity, mny = Infinity, mxy = -Infinity;
        for (var i = 0; i < good.length; i++) {
          mnx = Math.min(mnx, good[i].x); mxx = Math.max(mxx, good[i].x);
          mny = Math.min(mny, good[i].y); mxy = Math.max(mxy, good[i].y);
        }
        bx = { x0: mnx, y0: mny, x1: mxx, y1: mxy };
      }
      return { good: good, count: good.length, bbox: bx };
    }

    /** 半径 r 处沿角度扫描（画可达包络用） */
    function reachAt(r) {
      var out = [];
      for (var a = -180; a < 180; a += 2) {
        var x = r * Math.cos(a * RAD), y = r * Math.sin(a * RAD);
        out.push({ deg: a, x: x, y: y, ok: reachable(x, y) });
      }
      return out;
    }

    // --------------------------------------------------------------- 自检
    function selfCheck(opts) {
      opts = opts || {};
      var x0 = opts.x0 != null ? opts.x0 : 0, x1 = opts.x1 != null ? opts.x1 : 30;
      var y0 = opts.y0 != null ? opts.y0 : 40, y1 = opts.y1 != null ? opts.y1 : 90;
      var step = opts.step || 2.0;
      var tested = 0, okCount = 0, worst = 0, worstAt = null;
      for (var y = y0; y <= y1; y += step) {
        for (var x = x0; x <= x1; x += step) {
          tested++;
          var s = tryIk(x, y);
          if (!s.ok) continue;
          okCount++;
          var p = fk(s.t1, s.t2);
          var err = Math.hypot(p.x - x, p.y - y);
          if (err > worst) { worst = err; worstAt = { x: x, y: y }; }
        }
      }
      return { tested: tested, ok: okCount, worstRoundTripMm: worst, worstAt: worstAt };
    }

    return {
      cfg: cfg,
      derived: {
        LEFF: LEFF, GAMMA_DEG: GAMMA * DEG, RIN: RIN, ROUT: ROUT,
        R2IN: R2IN, R2OUT: R2OUT, OFFSET_DEG: cfg.OFFSET_DEG
      },
      fk: fk,
      ik: ik,
      tryIk: tryIk,
      solvePath: solvePath,
      layoutText: layoutText,
      textStrokes: textStrokes,
      plan: plan,
      reachable: reachable,
      boxOk: boxOk,
      scanRegion: scanRegion,
      reachAt: reachAt,
      selfCheck: selfCheck,
      wrap180: wrap180,
      unwrapTo: unwrapTo,
      degDiff: degDiff
    };
  }

  return {
    create: create,
    DEFAULTS: DEFAULTS,
    GLYPHS: GLYPHS,
    KinematicsError: KinematicsError
  };
});
