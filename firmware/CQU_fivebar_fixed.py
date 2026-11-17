"""ESP32 MicroPython 三舵机五连杆逐笔书写控制。

基于用户最新中心(13,65)、右反向、零微调配置整理。
分发默认 CALIBRATED=False；完成实物标定后才可启用。
原控制偏置60°保留；网页CAD演示采用45°幅值，两者未作实机统一标定。
import 本模块不创建PWM。
"""
import math
import time

VERSION = "v7.1-user-baseline"

# 信号引脚（保留用户配置；GPIO0 为 BOOT，不可冲突）
SERVO_LEFT_PIN = 13
SERVO_RIGHT_PIN = 14
PEN_SERVO_PIN = 26
CALIBRATED = False

# 书写板中心（保留用户配置，相对左轴中心 A(0,0) 的物理坐标 (x, y) mm）
BOARD_CENTER = (13.0, 65.0)

# 两平面舵机安装与增益映射参数（均需在 setup.py 中通过两点小角度命令实测确认，勿猜值）：
# pulse = _CENTER_US + us_per_deg * (direction * (geom_deg - geom_center_deg) + trim_deg)
# 注：5.556 (1000/180) 为原旧配置假设（待实测）；若实测转角只有预期一半，才可能是 ~11.111。
LEFT_GEOM_CENTER_DEG = 90.0
RIGHT_GEOM_CENTER_DEG = 90.0
LEFT_DIRECTION = 1
RIGHT_DIRECTION = -1
LEFT_TRIM_DEG = 0.0
RIGHT_TRIM_DEG = 0.0
LEFT_US_PER_DEG = 1000.0 / 180.0    # 旧配置默认值 (~5.556 us/deg，待实测)
RIGHT_US_PER_DEG = 1000.0 / 180.0   # 旧配置默认值 (~5.556 us/deg，待实测)

# 第三路抬笔舵机配置（固定独立 1500us 中位 + 5.556us/deg + 1000..2000us，绝不继承 XY 增益）
PEN_UP_DEG = 40.0
PEN_DOWN_DEG = 5.0
PEN_DIRECTION = 1
PEN_TRIM_DEG = 0.0
PEN_ANGLE_MIN = 5.0
PEN_ANGLE_MAX = 175.0
PEN_PULSE_MIN = 1000
PEN_PULSE_MAX = 2000
PEN_STEP_DEG = 3.0
PEN_SETTLE_MS = 200
_PEN_CENTER_US = 1500.0
_PEN_US_PER_DEG = 1000.0 / 180.0

# 平面舵机名义中位与默认硬件安全脉宽门限（保持 1000..2000，不随 gain 自动扩大）
_CENTER_US = 1500.0
_US_PER_DEG = 1000.0 / 180.0
_ELEC_CENTER_DEG = 90.0

L1, L2, L3, L4, L5, LT = 25.0, 35.0, 35.0, 45.0, 45.0, 13.0
OFFSET_DEG = 60.0             # 保留用户实机设置
LEFT_BRANCH, RIGHT_BRANCH = 1, -1
WRIST_BRANCH = 1

ELEC_ANGLE_MIN, ELEC_ANGLE_MAX = 5.0, 175.0   # 仅校核电气角，不用作通用几何角截断
PULSE_MIN, PULSE_MAX = 1000, 2000             # 独立于 gain；无硬件实测依据禁止擅自放宽

FRAME_MS = 20
MAX_STEP_MM = 0.8     # 50Hz(20ms) 下最大单帧步长，对应 40.0 mm/s；超限直接报错
DRAW_SPEED = 20.0     # mm/s 落笔书写名义速度
TRAVEL_SPEED = 30.0   # mm/s 抬笔空移名义速度
MAX_PULSE_STEP = 30.0

# 默认字内容与固定 20x20 mm 字框（留 1 mm 内边距；'1' 等窄字符居中摆放，不横向拉满）
DRAW_TEXT = "中"
CHAR_BOX_MM = 20.0
CHAR_PAD_MM = 1.0
CHAR_GAP_MM = 2.0

_offset = math.radians(OFFSET_DEG)
LEFF = math.sqrt(L4*L4 + LT*LT + 2*L4*LT*math.cos(_offset))
GAMMA = math.atan2(LT*math.sin(_offset), L4 + LT*math.cos(_offset))


def _is_finite_number(value):
    return (not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value))


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    if hasattr(time, "monotonic_ns"):
        return time.monotonic_ns() // 1000000
    return int(time.monotonic() * 1000)


def _ticks_diff(new_ticks, old_ticks):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_ticks, old_ticks)
    return new_ticks - old_ticks


def _sleep_ms(ms):
    delay = int(round(ms))
    if delay <= 0:
        return
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(delay)
    else:
        time.sleep(delay / 1000.0)


def _validate_gain(us_per_deg, label="US_PER_DEG"):
    if not _is_finite_number(us_per_deg) or float(us_per_deg) <= 0.0:
        raise ValueError("%s 必须为大于 0 的有限数字: %r" % (label, us_per_deg))
    return float(us_per_deg)


def board_center_from_axis_midpoint(dx=0.0, fwd=0.0):
    """由两轴中点 (L1/2, 0) 偏移 (dx, fwd) 换算为相对左轴 A(0, 0) 的坐标。"""
    if not _is_finite_number(dx) or not _is_finite_number(fwd):
        raise ValueError("偏移量 dx/fwd 必须为有限数字: (%r, %r)" % (dx, fwd))
    return (L1 / 2.0 + float(dx), float(fwd))


def resolve_board_center(board_center=None):
    """校验并返回已确认的 BOARD_CENTER；未配置或非法时拒绝动作。"""
    if board_center is None:
        board_center = BOARD_CENTER
    if board_center is None:
        raise ValueError(
            "未配置 BOARD_CENTER：请先实测书写板中心相对左轴 A(0,0) 的坐标 (x, y) mm"
            "（或调用 board_center_from_axis_midpoint(dx, fwd) 换算）"
        )
    if (not isinstance(board_center, (tuple, list))
            or len(board_center) != 2
            or not _is_finite_number(board_center[0])
            or not _is_finite_number(board_center[1])):
        raise ValueError("BOARD_CENTER 必须为两个有限数字 (x, y): %r" % (board_center,))
    return (float(board_center[0]), float(board_center[1]))


def acos_checked(c):
    if c < -1.0 - 1e-9 or c > 1.0 + 1e-9:
        raise ValueError("目标不可达")
    return math.acos(max(-1.0, min(1.0, c)))


def unwrap_angle_deg(radians, ref_deg):
    """按参考几何角就近展开角度，支持左>180°/右<0° 而不发生 ±180° 跳变。"""
    if not _is_finite_number(ref_deg):
        raise ValueError("GEOM_CENTER_DEG 必须为有限数字: %r" % (ref_deg,))
    diff = (math.degrees(radians) - float(ref_deg) + 180.0) % 360.0 - 180.0
    return float(ref_deg) + diff


def forward(a1, a2):
    """正解：锁定同一腕点分支，用于逆解回代校验。"""
    t1, t2 = math.radians(a1), math.radians(a2)
    ex, ey = L2*math.cos(t1), L2*math.sin(t1)
    fx, fy = L1 + L3*math.cos(t2), L3*math.sin(t2)
    dx, dy = fx - ex, fy - ey
    d = math.sqrt(dx*dx + dy*dy)
    if d < 1e-6:
        raise ValueError("两肘重合，机构奇异")
    along = (L4*L4 - L5*L5 + d*d) / (2*d)
    h2 = L4*L4 - along*along
    if h2 <= 1e-6:
        raise ValueError("从动臂不可闭合或接近奇异")
    h = math.sqrt(h2) * WRIST_BRANCH
    wx = ex + along*dx/d - h*dy/d
    wy = ey + along*dy/d + h*dx/d
    phi = math.atan2(wy - ey, wx - ex)
    return wx + LT*math.cos(phi + _offset), wy + LT*math.sin(phi + _offset)


def geom_to_elec_angle(geom_angle, direction=1, trim_deg=0.0, geom_center_deg=90.0):
    """分离几何角与电气舵机角：elec = 90 + direction * (geom - geom_center_deg) + trim。"""
    if isinstance(direction, bool) or direction not in (-1, 1):
        raise ValueError("DIRECTION 只能为 +1 或 -1: %r" % (direction,))
    if not _is_finite_number(geom_angle):
        raise ValueError("几何角必须为有限数字: %r" % (geom_angle,))
    if not _is_finite_number(trim_deg):
        raise ValueError("TRIM_DEG 必须为有限数字: %r" % (trim_deg,))
    if not _is_finite_number(geom_center_deg):
        raise ValueError("GEOM_CENTER_DEG 必须为有限数字: %r" % (geom_center_deg,))
    return _ELEC_CENTER_DEG + direction * (float(geom_angle) - float(geom_center_deg)) + float(trim_deg)


def elec_angle_to_pulse(elec_angle, angle_min=None, angle_max=None,
                        pulse_min=None, pulse_max=None,
                        center_us=None, us_per_deg=None):
    """校验电气舵机角与脉宽安全范围并换算微秒（默认保持 1000..2000us，不随 gain 自动放宽）。"""
    if angle_min is None:
        angle_min = ELEC_ANGLE_MIN
    if angle_max is None:
        angle_max = ELEC_ANGLE_MAX
    if pulse_min is None:
        pulse_min = PULSE_MIN
    if pulse_max is None:
        pulse_max = PULSE_MAX
    if center_us is None:
        center_us = _CENTER_US
    if us_per_deg is None:
        us_per_deg = _US_PER_DEG
    us_per_deg = _validate_gain(us_per_deg)
    if not _is_finite_number(elec_angle):
        raise ValueError("电气角必须为有限数字: %r" % (elec_angle,))
    if not (angle_min <= elec_angle <= angle_max):
        raise ValueError("电气舵机角超限 %.1f..%.1f: %.2f" % (angle_min, angle_max, elec_angle))
    pulse = float(center_us) + us_per_deg * (float(elec_angle) - _ELEC_CENTER_DEG)
    if not (pulse_min <= pulse <= pulse_max):
        raise ValueError("舵机脉宽超限 %.1f (允许 %d..%d)" % (pulse, pulse_min, pulse_max))
    return pulse


def angle_to_pulse(geom_angle, direction=1, trim_deg=0.0, geom_center_deg=90.0, us_per_deg=None):
    """按参考角、方向、微调与每侧增益将几何角换算为脉宽。"""
    elec_deg = geom_to_elec_angle(geom_angle, direction, trim_deg, geom_center_deg)
    return elec_angle_to_pulse(elec_deg, us_per_deg=us_per_deg)


def calibrate_side_two_point(elec1, geom1, elec2, geom2,
                             current_us_per_deg=None, trim_deg=0.0):
    """根据两次小角度电气命令（如 eleft 90 与 110）的实测几何角反算该侧映射参数。

    说明：
    - eleft/eright 驱动时实际输出 pulse_k = 1500 + current_us_per_deg * (elec_k - 90)（不含 TRIM）。
    - 默认 trim_deg=0.0，由算出的 geom_center_deg 直接吸收机械零偏，避免保留旧 TRIM 造成双算。
      若调用方显式保留非零 trim_deg，本函数会将其扣算到 geom_center_deg 内，确保两点脉宽精确一致。
    """
    if current_us_per_deg is None:
        current_us_per_deg = _US_PER_DEG
    curr_gain = _validate_gain(current_us_per_deg, "current_us_per_deg")
    for name, val in (("elec1", elec1), ("geom1", geom1), ("elec2", elec2),
                      ("geom2", geom2), ("trim_deg", trim_deg)):
        if not _is_finite_number(val):
            raise ValueError("%s 必须为有限数字: %r" % (name, val))
    d_elec = float(elec2) - float(elec1)
    d_geom = float(geom2) - float(geom1)
    if abs(d_elec) < 1e-6:
        raise ValueError("两次电气角不能相同")
    if abs(d_geom) < 1e-6:
        raise ValueError("两次实测几何角无变化，无法标定增益与方向")
    direction = 1 if (d_elec * d_geom) > 0.0 else -1
    us_per_deg = curr_gain * abs(d_elec) / abs(d_geom)
    # 令 angle_to_pulse(geom1, direction, trim_deg, geom_center_deg, us_per_deg) == pulse(elec1)
    geom_center_deg = (
        float(geom1)
        + direction * (float(trim_deg) - (curr_gain / us_per_deg) * (float(elec1) - _ELEC_CENTER_DEG))
    )
    return {
        "us_per_deg": us_per_deg,
        "geom_center_deg": geom_center_deg,
        "direction": direction,
        "trim_deg": float(trim_deg),
    }


def solve_pure_geom(x, y, left_ref=None, right_ref=None):
    """纯几何逆解：固定五连杆分支并按参考角就近展开，仅校核奇异与正解回代。"""
    if left_ref is None:
        left_ref = LEFT_GEOM_CENTER_DEG
    if right_ref is None:
        right_ref = RIGHT_GEOM_CENTER_DEG
    if not _is_finite_number(x) or not _is_finite_number(y):
        raise ValueError("坐标必须为有限数")
    r = math.sqrt(x*x + y*y)
    if r < 1e-6:
        raise ValueError("目标位于左轴中心")
    t1 = math.atan2(y, x) + LEFT_BRANCH * acos_checked(
        (L2*L2 + r*r - LEFF*LEFF) / (2*L2*r))
    ex, ey = L2*math.cos(t1), L2*math.sin(t1)
    phi = math.atan2(y - ey, x - ex) - GAMMA
    wx, wy = ex + L4*math.cos(phi), ey + L4*math.sin(phi)
    dx, dy = wx - L1, wy
    r = math.sqrt(dx*dx + dy*dy)
    if r < 1e-6:
        raise ValueError("腕点位于右轴中心")
    t2 = math.atan2(dy, dx) + RIGHT_BRANCH * acos_checked(
        (L3*L3 + r*r - L5*L5) / (2*L3*r))

    a1 = unwrap_angle_deg(t1, left_ref)
    a2 = unwrap_angle_deg(t2, right_ref)
    px, py = forward(a1, a2)
    if (px - x)**2 + (py - y)**2 > 1e-6:
        raise ValueError("目标不属于当前装配分支")
    return a1, a2


def solve(x, y):
    """纯逆解：固定五连杆分支并按 GEOM_CENTER_DEG 就近展开，校核奇异、回代与电气限位。"""
    a1, a2 = solve_pure_geom(x, y)
    e1 = geom_to_elec_angle(a1, LEFT_DIRECTION, LEFT_TRIM_DEG, LEFT_GEOM_CENTER_DEG)
    e2 = geom_to_elec_angle(a2, RIGHT_DIRECTION, RIGHT_TRIM_DEG, RIGHT_GEOM_CENTER_DEG)
    if not (ELEC_ANGLE_MIN <= e1 <= ELEC_ANGLE_MAX and ELEC_ANGLE_MIN <= e2 <= ELEC_ANGLE_MAX):
        raise ValueError(
            "电气舵机角超限 %.1f..%.1f: 电气(%.2f, %.2f) / 几何(%.2f, %.2f)"
            % (ELEC_ANGLE_MIN, ELEC_ANGLE_MAX, e1, e2, a1, a2)
        )
    return a1, a2


def angles_to_pulses(a1, a2):
    """按当前左右参考角、方向、微调与独立增益计算两路脉宽。"""
    return (
        angle_to_pulse(a1, LEFT_DIRECTION, LEFT_TRIM_DEG, LEFT_GEOM_CENTER_DEG,
                       us_per_deg=LEFT_US_PER_DEG),
        angle_to_pulse(a2, RIGHT_DIRECTION, RIGHT_TRIM_DEG, RIGHT_GEOM_CENTER_DEG,
                       us_per_deg=RIGHT_US_PER_DEG),
    )


def pulses(x, y):
    a1, a2 = solve(x, y)
    return angles_to_pulses(a1, a2)


def pen_angle_to_pulse(pen_deg, direction=None, trim_deg=None):
    """抬笔独立映射：固定 1500us 中位 + 5.556us/deg + 1000..2000us，绝不随 XY 增益改变。"""
    if direction is None:
        direction = PEN_DIRECTION
    if trim_deg is None:
        trim_deg = PEN_TRIM_DEG
    if isinstance(direction, bool) or direction not in (-1, 1):
        raise ValueError("PEN_DIRECTION 只能为 +1 或 -1: %r" % (direction,))
    if not _is_finite_number(pen_deg):
        raise ValueError("抬笔角度必须为有限数字: %r" % (pen_deg,))
    if not _is_finite_number(trim_deg):
        raise ValueError("PEN_TRIM_DEG 必须为有限数字: %r" % (trim_deg,))
    elec_deg = _ELEC_CENTER_DEG + direction * (float(pen_deg) - _ELEC_CENTER_DEG) + float(trim_deg)
    return elec_angle_to_pulse(
        elec_deg,
        angle_min=PEN_ANGLE_MIN,
        angle_max=PEN_ANGLE_MAX,
        pulse_min=PEN_PULSE_MIN,
        pulse_max=PEN_PULSE_MAX,
        center_us=_PEN_CENTER_US,
        us_per_deg=_PEN_US_PER_DEG,
    )


def validate_pen_config():
    """校验抬落笔角度已配置且各自对应的电气角/脉宽在独立限位内。"""
    if PEN_UP_DEG is None or PEN_DOWN_DEG is None:
        raise ValueError("抬落笔角度未标定：请先实测并设置 PEN_UP_DEG 与 PEN_DOWN_DEG")
    up_us = pen_angle_to_pulse(PEN_UP_DEG)
    down_us = pen_angle_to_pulse(PEN_DOWN_DEG)
    if abs(float(PEN_UP_DEG) - float(PEN_DOWN_DEG)) < 1e-6:
        raise ValueError("PEN_UP_DEG 与 PEN_DOWN_DEG 不能相同: %r" % (PEN_UP_DEG,))
    return up_us, down_us


def validate_pins(boot_pin=0):
    """校验三路舵机信号引脚均已配置、有效、互不重复且不与 BOOT 冲突。"""
    pins = (
        ("SERVO_LEFT_PIN", SERVO_LEFT_PIN),
        ("SERVO_RIGHT_PIN", SERVO_RIGHT_PIN),
        ("PEN_SERVO_PIN", PEN_SERVO_PIN),
    )
    for name, pin in pins:
        if pin is None:
            raise ValueError("缺少舵机引脚配置: %s" % name)
        if isinstance(pin, bool) or not isinstance(pin, int) or pin < 0:
            raise ValueError("舵机引脚 %s 无效: %r" % (name, pin))
    values = [pin for _, pin in pins]
    if len(set(values)) != 3:
        raise ValueError("三路舵机引脚不能重复: %r" % (values,))
    if boot_pin is not None and boot_pin in values:
        raise ValueError("舵机引脚不能与 BOOT 引脚 GPIO%d 冲突" % boot_pin)
    return tuple(values)


def resolve_home_point(board_center=None):
    """home 不再硬编码 60/120°，而是已确认的板中心/字组中心。"""
    center = resolve_board_center(board_center)
    pulses(*center)
    return center


def frame_step_mm(speed=None):
    """根据速度与 20ms 帧周期计算单帧步长；超限报错，绝不静默降速。"""
    if speed is None:
        speed = DRAW_SPEED
    if not _is_finite_number(speed) or speed <= 0:
        raise ValueError("速度必须为大于 0 的有限数字: %r" % (speed,))
    if not _is_finite_number(FRAME_MS) or FRAME_MS <= 0:
        raise ValueError("FRAME_MS 必须为大于 0 的有限数字: %r" % (FRAME_MS,))
    if not _is_finite_number(MAX_STEP_MM) or MAX_STEP_MM <= 0:
        raise ValueError("MAX_STEP_MM 必须为大于 0 的有限数字: %r" % (MAX_STEP_MM,))
    step = float(speed) * float(FRAME_MS) / 1000.0
    if step > MAX_STEP_MM + 1e-12:
        raise ValueError(
            "单帧步长 %.3f mm 超过上限 MAX_STEP_MM=%.3f mm（请降低速度 <= %.1f mm/s）"
            % (step, MAX_STEP_MM, MAX_STEP_MM * 1000.0 / FRAME_MS)
        )
    return step


def segment(start, end, speed=None):
    dx, dy = end[0] - start[0], end[1] - start[1]
    distance = math.sqrt(dx*dx + dy*dy)
    step = frame_step_mm(speed)
    count = max(1, int(math.ceil(distance / step)))
    for i in range(1, count + 1):
        yield start[0] + dx*i/count, start[1] + dy*i/count


def check_line(start, end, speed=None):
    previous = pulses(*start)
    for point in segment(start, end, speed):
        current = pulses(*point)
        if max(abs(current[0] - previous[0]), abs(current[1] - previous[1])) > MAX_PULSE_STEP:
            raise ValueError("关节变化过快，请降速或调整字区/中心位置")
        previous = current


def check_strokes(strokes, start=None, draw_speed=None, travel_speed=None):
    """整幅逐笔预检（含 home->首点及全部笔画间空移，落笔@DRAW_SPEED、空移@TRAVEL_SPEED）。"""
    if draw_speed is None:
        draw_speed = DRAW_SPEED
    if travel_speed is None:
        travel_speed = TRAVEL_SPEED
    _validate_gain(LEFT_US_PER_DEG, "LEFT_US_PER_DEG")
    _validate_gain(RIGHT_US_PER_DEG, "RIGHT_US_PER_DEG")
    frame_step_mm(draw_speed)
    frame_step_mm(travel_speed)
    validate_pen_config()
    if start is None:
        start = resolve_home_point()
    if not strokes or not any(strokes):
        raise ValueError("笔画列表不能为空")
    pulses(*start)
    cursor = start
    for stroke in strokes:
        if not stroke:
            raise ValueError("不允许包含空笔画")
        for i, point in enumerate(stroke):
            if (not isinstance(point, (tuple, list))
                    or len(point) != 2
                    or not _is_finite_number(point[0])
                    or not _is_finite_number(point[1])):
                raise ValueError("轨迹点必须为二维有限坐标: %r" % (point,))
            speed = travel_speed if i == 0 else draw_speed
            check_line(cursor, point, speed)
            cursor = (float(point[0]), float(point[1]))


def diagnose_text(text=None, board_center=None):
    """精简诊断：常量空间在线统计当前轨迹的纯 IK/电气/脉宽范围与首拒点，避免嵌入式堆耗尽。"""
    try:
        import gc
        gc.collect()
    except Exception:
        pass
    center = resolve_board_center(board_center)
    draw_text = normalize_text(text)
    boxes = text_boxes(draw_text, board_center=center)
    strokes = text_strokes(draw_text, board_center=center)
    frame_step_mm(DRAW_SPEED)
    frame_step_mm(TRAVEL_SPEED)
    _validate_gain(LEFT_US_PER_DEG, "LEFT_US_PER_DEG")
    _validate_gain(RIGHT_US_PER_DEG, "RIGHT_US_PER_DEG")

    a1_min = a1_max = a2_min = a2_max = None
    e1_min = e1_max = e2_min = e2_max = None
    u_min = u_max = v_min = v_max = None
    prev_u = prev_v = None
    first_reject = None

    def _record_point(stage, pt):
        nonlocal first_reject, prev_u, prev_v
        nonlocal a1_min, a1_max, a2_min, a2_max
        nonlocal e1_min, e1_max, e2_min, e2_max
        nonlocal u_min, u_max, v_min, v_max
        try:
            a1, a2 = solve_pure_geom(*pt)
        except ValueError as err:
            if first_reject is None:
                first_reject = {"stage": stage, "point": pt, "reason": str(err)}
            return
        if a1_min is None:
            a1_min = a1_max = a1
            a2_min = a2_max = a2
        else:
            if a1 < a1_min: a1_min = a1
            if a1 > a1_max: a1_max = a1
            if a2 < a2_min: a2_min = a2
            if a2 > a2_max: a2_max = a2

        e1 = geom_to_elec_angle(a1, LEFT_DIRECTION, LEFT_TRIM_DEG, LEFT_GEOM_CENTER_DEG)
        e2 = geom_to_elec_angle(a2, RIGHT_DIRECTION, RIGHT_TRIM_DEG, RIGHT_GEOM_CENTER_DEG)
        if e1_min is None:
            e1_min = e1_max = e1
            e2_min = e2_max = e2
        else:
            if e1 < e1_min: e1_min = e1
            if e1 > e1_max: e1_max = e1
            if e2 < e2_min: e2_min = e2
            if e2 > e2_max: e2_max = e2

        if not (ELEC_ANGLE_MIN <= e1 <= ELEC_ANGLE_MAX and ELEC_ANGLE_MIN <= e2 <= ELEC_ANGLE_MAX):
            if first_reject is None:
                first_reject = {
                    "stage": stage,
                    "point": pt,
                    "reason": "电气舵机角超限 %.1f..%.1f: 电气(%.2f, %.2f) / 几何(%.2f, %.2f)"
                    % (ELEC_ANGLE_MIN, ELEC_ANGLE_MAX, e1, e2, a1, a2),
                }
            return
        try:
            u, v = angles_to_pulses(a1, a2)
        except ValueError as err:
            if first_reject is None:
                first_reject = {"stage": stage, "point": pt, "reason": str(err)}
            return

        if u_min is None:
            u_min = u_max = u
            v_min = v_max = v
        else:
            if u < u_min: u_min = u
            if u > u_max: u_max = u
            if v < v_min: v_min = v
            if v > v_max: v_max = v

        if prev_u is not None:
            if max(abs(u - prev_u), abs(v - prev_v)) > MAX_PULSE_STEP:
                if first_reject is None:
                    first_reject = {"stage": stage, "point": pt, "reason": "关节变化过快"}
        prev_u, prev_v = u, v

    _record_point("home", center)
    cursor = center
    for s_idx, stroke in enumerate(strokes):
        for i, pt in enumerate(stroke):
            spd = TRAVEL_SPEED if i == 0 else DRAW_SPEED
            tag = "stroke#%d[%d]" % (s_idx, i)
            for sp in segment(cursor, pt, spd):
                _record_point(tag, sp)
            cursor = pt

    return {
        "center": center,
        "text": draw_text,
        "boxes": boxes,
        "strokes_count": len(strokes),
        "ok": first_reject is None,
        "first_reject": first_reject,
        "geom_range_left": (a1_min, a1_max) if a1_min is not None else None,
        "geom_range_right": (a2_min, a2_max) if a2_min is not None else None,
        "elec_range_left": (e1_min, e1_max) if e1_min is not None else None,
        "elec_range_right": (e2_min, e2_max) if e2_min is not None else None,
        "pulse_range_left": (u_min, u_max) if u_min is not None else None,
        "pulse_range_right": (v_min, v_max) if v_min is not None else None,
    }


def make_servo(pin):
    from machine import Pin, PWM
    return PWM(Pin(pin, Pin.OUT), freq=50, duty=0)


def set_pulse_us(pwm, us, limits=None):
    minimum, maximum = (PULSE_MIN, PULSE_MAX) if limits is None else limits
    if not (minimum <= us <= maximum):
        raise ValueError("脉宽超限 %.1f (允许 %d..%d)，拒绝截断" % (us, minimum, maximum))
    if hasattr(pwm, "duty_ns"):
        pwm.duty_ns(int(round(us * 1000)))
    elif hasattr(pwm, "duty_u16"):
        pwm.duty_u16(round(us * 65535 / 20000))
    else:
        pwm.duty(round(us * 1023 / 20000))


def _ramp_sleep(frame_start):
    elapsed = _ticks_diff(_ticks_ms(), frame_start)
    if elapsed < 0:
        elapsed = 0
    remaining = FRAME_MS - elapsed
    if remaining > 0:
        _sleep_ms(remaining)


def ramp_pen_to(pen_pwm, current_deg, target_deg, direction=None, trim_deg=None, on_step=None):
    """按步进斜坡控制抬笔舵机；位置未知时单步定位（仅允许用于抬笔）。"""
    limits = (PEN_PULSE_MIN, PEN_PULSE_MAX)
    target_deg = float(target_deg)
    if current_deg is None:
        pulse = pen_angle_to_pulse(target_deg, direction=direction, trim_deg=trim_deg)
        set_pulse_us(pen_pwm, pulse, limits)
        if on_step is not None:
            on_step(target_deg)
        return target_deg, True
    start_deg = float(current_deg)
    delta = abs(target_deg - start_deg)
    if delta < 1e-6:
        return start_deg, False
    if not _is_finite_number(PEN_STEP_DEG) or PEN_STEP_DEG <= 0:
        raise ValueError("PEN_STEP_DEG 必须为大于 0 的有限数字: %r" % (PEN_STEP_DEG,))
    count = max(1, int(math.ceil(delta / PEN_STEP_DEG)))
    for i in range(1, count + 1):
        frame_start = _ticks_ms()
        deg = start_deg + (target_deg - start_deg) * i / count
        pulse = pen_angle_to_pulse(deg, direction=direction, trim_deg=trim_deg)
        set_pulse_us(pen_pwm, pulse, limits)
        if on_step is not None:
            on_step(deg)
        _ramp_sleep(frame_start)
    return target_deg, True


class Plotter:
    def __init__(self):
        left_pin, right_pin, pen_pin = validate_pins(boot_pin=None)
        self.left = self.right = self.pen = None
        self.position = None  # 估计 XY 位置（无传感器反馈）
        self.pen_deg = None   # 已知抬笔角度（度）；未知时禁止 direct down
        try:
            self.left = make_servo(left_pin)
            self.right = make_servo(right_pin)
            self.pen = make_servo(pen_pin)
        except BaseException:
            self.close()
            raise

    def lift(self, up=True):
        """按角度与斜坡抬/落笔；位置未知时拒绝直接落笔防砸板。"""
        if self.pen is None:
            raise ValueError("抬笔舵机未就绪")
        validate_pen_config()
        target_deg = float(PEN_UP_DEG if up else PEN_DOWN_DEG)
        if self.pen_deg is None and not up:
            raise ValueError("抬笔当前位置未知，必须先执行抬笔")
        _, moved = ramp_pen_to(
            self.pen,
            self.pen_deg,
            target_deg,
            on_step=lambda deg: setattr(self, "pen_deg", deg),
        )
        if moved:
            _sleep_ms(PEN_SETTLE_MS)

    def home(self, home_point=None):
        """先抬笔再归位到已确认的板中心/字组中心；首次无位置反馈不保证平滑。"""
        if CALIBRATED is not True:
            raise ValueError("首次使用请先完成标定并设 CALIBRATED=True")
        validate_pen_config()
        target = resolve_home_point(home_point)
        self.lift(True)
        if self.position is not None:
            self.move(*target, speed=TRAVEL_SPEED)
            return
        u, v = pulses(*target)
        set_pulse_us(self.left, u)
        set_pulse_us(self.right, v)
        _sleep_ms(800)
        self.position = target

    def move(self, x, y, speed=None):
        if self.position is None:
            raise ValueError("当前位置未知，请先执行 home")
        if speed is None:
            speed = TRAVEL_SPEED
        start = self.position
        check_line(start, (x, y), speed)
        for point in segment(start, (x, y), speed):
            frame_start = _ticks_ms()
            self.position = None
            u, v = pulses(*point)
            set_pulse_us(self.left, u)
            set_pulse_us(self.right, v)
            _ramp_sleep(frame_start)
            self.position = point

    def draw(self, strokes, draw_speed=None, travel_speed=None):
        """整幅预检通过后逐笔书写：抬笔->空移首点->缓降->写笔画->抬笔，写完抬笔待机。"""
        if CALIBRATED is not True or self.position is None or self.pen is None:
            raise ValueError("书写需要完成标定、配置三路舵机并先执行 home")
        if draw_speed is None:
            draw_speed = DRAW_SPEED
        if travel_speed is None:
            travel_speed = TRAVEL_SPEED
        check_strokes(strokes, start=self.position,
                      draw_speed=draw_speed, travel_speed=travel_speed)
        orig_exc = None
        try:
            for stroke in strokes:
                self.lift(True)
                self.move(*stroke[0], speed=travel_speed)
                self.lift(False)
                for x, y in stroke[1:]:
                    self.move(x, y, speed=draw_speed)
                self.lift(True)
        except BaseException as exc:
            self.position = None
            orig_exc = exc
            raise
        finally:
            try:
                if self.pen is not None and PEN_UP_DEG is not None:
                    self.lift(True)
            except BaseException as lift_err:
                if orig_exc is None:
                    raise
                print("异常清理时抬笔失败:", lift_err)

    def close(self):
        self.position = None
        self.pen_deg = None
        for pwm in (self.pen, self.left, self.right):
            if pwm is not None:
                try:
                    pwm.deinit()
                except BaseException:
                    pass


# 单字标准 20x20 mm 方框（本地坐标 0..20, 0..20），全部留 >= 1mm 内边距（位于 [1..19]）
_GLYPH_STROKES = {
    "王": [
        [(3.0, 18.0), (17.0, 18.0)],
        [(4.5, 10.0), (15.5, 10.0)],
        [(10.0, 18.0), (10.0, 2.0)],
        [(2.0, 2.0), (18.0, 2.0)],
    ],
    "中": [
        [(3.5, 14.5), (3.5, 6.5)],
        [(3.5, 14.5), (16.5, 14.5), (16.5, 6.5)],
        [(3.5, 6.5), (16.5, 6.5)],
        [(10.0, 19.0), (10.0, 1.0)],
    ],
    "C": [
        [(15.5, 16.5), (12.5, 19.0), (7.5, 19.0), (4.5, 16.0), (3.5, 10.0), (4.5, 4.0), (7.5, 1.0), (12.5, 1.0), (15.5, 3.5)],
    ],
    "Q": [
        [(10.0, 19.0), (14.0, 17.5), (16.0, 14.0), (16.0, 6.0), (14.0, 2.5), (10.0, 1.0), (6.0, 2.5), (4.0, 6.0), (4.0, 14.0), (6.0, 17.5), (10.0, 19.0)],
        [(11.5, 5.5), (16.5, 1.0)],
    ],
    "U": [
        [(4.0, 19.0), (4.0, 5.5), (6.0, 2.0), (10.0, 1.0), (14.0, 2.0), (16.0, 5.5), (16.0, 19.0)],
    ],
    "0": [
        [(10.0, 19.0), (14.0, 17.0), (15.5, 10.0), (14.0, 3.0), (10.0, 1.0), (6.0, 3.0), (4.5, 10.0), (6.0, 17.0), (10.0, 19.0)],
    ],
    "1": [
        [(7.5, 16.5), (10.0, 19.0), (10.0, 1.0)],
    ],
    "2": [
        [(4.5, 15.5), (6.5, 18.5), (10.0, 19.0), (13.5, 18.5), (15.5, 15.0), (14.5, 11.5), (4.5, 1.0), (15.5, 1.0)],
    ],
    "3": [
        [(4.5, 16.5), (7.0, 19.0), (13.0, 19.0), (15.5, 16.0), (15.0, 12.0), (11.5, 10.0), (8.0, 10.0)],
        [(11.5, 10.0), (15.5, 8.0), (15.5, 3.5), (13.0, 1.0), (6.5, 1.0), (4.5, 3.5)],
    ],
    "4": [
        [(12.5, 19.0), (4.5, 6.5), (16.0, 6.5)],
        [(12.5, 19.0), (12.5, 1.0)],
    ],
    "5": [
        [(15.5, 19.0), (5.5, 19.0), (5.0, 11.5), (11.5, 12.0), (15.0, 9.5), (15.5, 5.0), (13.0, 1.5), (7.5, 1.0), (4.5, 2.5)],
    ],
    "6": [
        [(14.0, 18.0), (10.5, 19.0), (6.5, 17.0), (4.5, 10.0), (5.0, 3.5), (7.5, 1.0), (12.5, 1.0), (15.5, 4.0), (15.0, 8.5), (12.0, 10.5), (6.0, 10.0), (4.5, 7.5)],
    ],
    "7": [
        [(4.5, 19.0), (15.5, 19.0), (8.0, 1.0)],
    ],
    "8": [
        [(10.0, 10.0), (6.0, 11.5), (5.0, 15.0), (6.5, 18.0), (10.0, 19.0), (13.5, 18.0), (15.0, 15.0), (14.0, 11.5), (10.0, 10.0), (5.5, 8.0), (4.5, 4.0), (6.5, 1.5), (10.0, 1.0), (13.5, 1.5), (15.5, 4.0), (14.5, 8.0), (10.0, 10.0)],
    ],
    "9": [
        [(15.5, 12.5), (13.5, 9.5), (8.5, 9.0), (5.0, 11.5), (4.5, 15.5), (7.0, 18.5), (11.5, 19.0), (15.0, 16.5), (15.5, 10.0), (13.5, 3.0), (9.5, 1.0), (6.0, 2.0)],
    ],
}


def normalize_text(text=None):
    """校验并规范化待写字符/字符串（支持 '王'、'中'、'0'..'9'、'C'/'Q'/'U'）。"""
    if text is None:
        text = DRAW_TEXT
    if isinstance(text, bool):
        raise ValueError("DRAW_TEXT 不支持布尔值")
    if isinstance(text, int) and 0 <= text <= 9:
        return str(text)
    if not isinstance(text, str):
        raise ValueError("DRAW_TEXT 必须为字符串或 0..9 整数: %r" % (text,))
    cleaned = text.strip()
    if not cleaned:
        raise ValueError("DRAW_TEXT 不能为空")
    chars = []
    for ch in cleaned:
        key = ch.upper() if ("a" <= ch <= "z") else ch
        if key not in _GLYPH_STROKES:
            raise ValueError("不支持的字符 %r（支持 '王'/'中'/'0'..'9'/'C'/'Q'/'U'）" % (ch,))
        chars.append(key)
    return "".join(chars)


def text_boxes(text=None, board_center=None):
    """返回每字固定 20x20 mm 字框列表 [(ch, (x0, y0, x1, y1)), ...]，整组居中于 BOARD_CENTER。"""
    cx, cy = resolve_board_center(board_center)
    chars = normalize_text(text)
    if not _is_finite_number(CHAR_BOX_MM) or CHAR_BOX_MM <= 0:
        raise ValueError("CHAR_BOX_MM 无效: %r" % (CHAR_BOX_MM,))
    if not _is_finite_number(CHAR_GAP_MM) or CHAR_GAP_MM < 0:
        raise ValueError("CHAR_GAP_MM 无效: %r" % (CHAR_GAP_MM,))
    n = len(chars)
    total_w = n * CHAR_BOX_MM + (n - 1) * CHAR_GAP_MM
    start_x = cx - total_w / 2.0
    y0 = cy - CHAR_BOX_MM / 2.0
    y1 = cy + CHAR_BOX_MM / 2.0
    boxes = []
    for idx, ch in enumerate(chars):
        x0 = start_x + idx * (CHAR_BOX_MM + CHAR_GAP_MM)
        boxes.append((ch, (x0, y0, x0 + CHAR_BOX_MM, y1)))
    return boxes


def text_strokes(text=None, board_center=None):
    """按固定 20x20 mm 字框排布并居中返回独立笔画列表（绝不静默缩小或移边）。"""
    scale = CHAR_BOX_MM / 20.0
    strokes = []
    for ch, (x0, y0, _x1, _y1) in text_boxes(text=text, board_center=board_center):
        for local_stroke in _GLYPH_STROKES[ch]:
            strokes.append([(x0 + lx * scale, y0 + ly * scale) for lx, ly in local_stroke])
    return strokes


def get_strokes(text=None, board_center=None):
    return text_strokes(text=text, board_center=board_center)
