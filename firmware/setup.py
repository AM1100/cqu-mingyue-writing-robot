"""ESP32 MicroPython 五连杆三舵机角度调试、两点校准与轨迹预检入口（v7）。

仅在拆下笔并解除主动臂与舵机机械耦合后使用驱动舵机命令！
- 角度调试：center / left <几何角> / right <几何角> / eleft <电气角> / eright <电气角> / pen <角度|up|down>
- 两点标定（纯计算，先用 eleft 90 / eleft 110 实测几何角后输入）：
  calleft <90°测得几何角> <110°测得几何角> 或 calleft <e1> <g1> <e2> <g2>
  calright <90°测得几何角> <110°测得几何角> 或 calright <e1> <g1> <e2> <g2>
- 配置预检（纯计算，不建 PWM、不改参数）：
  check [文本] [x y | midpoint dx fwd]
import 本模块不会创建 PWM 或驱动舵机。
"""
import CQU_fivebar_fixed as f


def _open_servos():
    left_pin, right_pin, pen_pin = f.validate_pins(boot_pin=None)
    left = right = pen = None
    try:
        left = f.make_servo(left_pin)
        right = f.make_servo(right_pin)
        pen = f.make_servo(pen_pin)
        return left, right, pen
    except BaseException:
        _close_servos(left, right, pen)
        raise


def _close_servos(*pwms):
    for pwm in pwms:
        if pwm is not None:
            try:
                pwm.deinit()
            except BaseException:
                pass


def _parse_float(token, label="数值"):
    try:
        return float(token)
    except ValueError:
        raise ValueError("%s必须为数字: %r" % (label, token))


def check_center_config(board_center=None, text=None):
    """纯计算诊断：报告当前配置下的轨迹角度/脉宽范围与首拒点，不创建 PWM。"""
    diag = f.diagnose_text(text=text, board_center=board_center)
    pen_ready = True
    try:
        pen_u = f.validate_pen_config()
    except ValueError as err:
        pen_u = str(err)
        pen_ready = False
    diag["pen_ready"] = pen_ready
    diag["pen_info"] = pen_u
    return diag


def _parse_check_args(args):
    text = None
    idx = 0
    if idx < len(args):
        first = args[idx]
        if first.lower() != "midpoint":
            try:
                float(first)
                if len(args) == 1:
                    text = first
                    idx = 1
            except ValueError:
                text = first
                idx = 1

    rest = args[idx:]
    if not rest:
        return check_center_config(text=text)
    if len(rest) == 2:
        cx = _parse_float(rest[0], "板中心 X")
        cy = _parse_float(rest[1], "板中心 Y")
        return check_center_config(board_center=(cx, cy), text=text)
    if len(rest) == 3 and rest[0].lower() == "midpoint":
        dx = _parse_float(rest[1], "中点偏移 dx")
        fwd = _parse_float(rest[2], "前伸 fwd")
        return check_center_config(
            board_center=f.board_center_from_axis_midpoint(dx, fwd),
            text=text,
        )
    raise ValueError("check 参数错误；用法: check [文本] [x y | midpoint dx fwd]")


def _parse_cal_args(side, parts):
    if len(parts) == 3:
        e1, e2 = 90.0, 110.0
        g1 = _parse_float(parts[1], "90°测得几何角")
        g2 = _parse_float(parts[2], "110°测得几何角")
    elif len(parts) == 5:
        e1 = _parse_float(parts[1], "第1次电气角")
        g1 = _parse_float(parts[2], "第1次实测几何角")
        e2 = _parse_float(parts[3], "第2次电气角")
        g2 = _parse_float(parts[4], "第2次实测几何角")
    else:
        raise ValueError("用法: %s <90°测得几何角> <110°测得几何角> 或 %s <e1> <g1> <e2> <g2>" % (parts[0], parts[0]))
    curr_gain = f.LEFT_US_PER_DEG if side == "LEFT" else f.RIGHT_US_PER_DEG
    res = f.calibrate_side_two_point(e1, g1, e2, g2, current_us_per_deg=curr_gain, trim_deg=0.0)
    return ("cal", side, e1, g1, e2, g2, res)


def parse_command(command):
    """解析并校验单条调试/标定/预检命令，仅做纯计算，不直接输出 PWM。"""
    parts = command.strip().split()
    if not parts:
        return None
    op = parts[0].lower()
    if op == "quit" and len(parts) == 1:
        return ("quit",)
    if op == "center" and len(parts) == 1:
        u = f.angle_to_pulse(f.LEFT_GEOM_CENTER_DEG, f.LEFT_DIRECTION, f.LEFT_TRIM_DEG,
                             f.LEFT_GEOM_CENTER_DEG, us_per_deg=f.LEFT_US_PER_DEG)
        v = f.angle_to_pulse(f.RIGHT_GEOM_CENTER_DEG, f.RIGHT_DIRECTION, f.RIGHT_TRIM_DEG,
                             f.RIGHT_GEOM_CENTER_DEG, us_per_deg=f.RIGHT_US_PER_DEG)
        return ("both", f.LEFT_GEOM_CENTER_DEG, f.RIGHT_GEOM_CENTER_DEG, u, v, "电气中位 (ref)")
    if op == "left" and len(parts) == 2:
        geom = _parse_float(parts[1], "几何角")
        elec = f.geom_to_elec_angle(geom, f.LEFT_DIRECTION, f.LEFT_TRIM_DEG, f.LEFT_GEOM_CENTER_DEG)
        u = f.elec_angle_to_pulse(elec, us_per_deg=f.LEFT_US_PER_DEG)
        return ("planar", "左", geom, elec, u, 0)
    if op == "right" and len(parts) == 2:
        geom = _parse_float(parts[1], "几何角")
        elec = f.geom_to_elec_angle(geom, f.RIGHT_DIRECTION, f.RIGHT_TRIM_DEG, f.RIGHT_GEOM_CENTER_DEG)
        v = f.elec_angle_to_pulse(elec, us_per_deg=f.RIGHT_US_PER_DEG)
        return ("planar", "右", geom, elec, v, 1)
    if op == "eleft" and len(parts) == 2:
        elec = _parse_float(parts[1], "电气角")
        u = f.elec_angle_to_pulse(elec, us_per_deg=f.LEFT_US_PER_DEG)
        return ("elec", "左", elec, u, 0)
    if op == "eright" and len(parts) == 2:
        elec = _parse_float(parts[1], "电气角")
        v = f.elec_angle_to_pulse(elec, us_per_deg=f.RIGHT_US_PER_DEG)
        return ("elec", "右", elec, v, 1)
    if op == "calleft":
        return _parse_cal_args("LEFT", parts)
    if op == "calright":
        return _parse_cal_args("RIGHT", parts)
    if op == "pen" and len(parts) == 2:
        arg = parts[1].lower()
        if arg == "up":
            if f.PEN_UP_DEG is None:
                raise ValueError("PEN_UP_DEG 尚未配置")
            deg = float(f.PEN_UP_DEG)
        elif arg == "down":
            if f.PEN_DOWN_DEG is None:
                raise ValueError("PEN_DOWN_DEG 尚未配置")
            deg = float(f.PEN_DOWN_DEG)
        else:
            deg = _parse_float(parts[1], "抬笔角度")
        p_us = f.pen_angle_to_pulse(deg)
        return ("pen", deg, p_us)
    if op == "check":
        return ("check", _parse_check_args(parts[1:]))
    raise ValueError(
        "未知命令或格式错误；可用: center / left <几何角> / right <几何角> / "
        "eleft <电气角> / eright <电气角> / calleft <g90> <g110> / calright <g90> <g110> / "
        "pen <角度|up|down> / check [文本] [x y | midpoint dx fwd] / quit"
    )


def _format_check_report(info):
    gl = "无" if info["geom_range_left"] is None else "[%.1f,%.1f]°" % info["geom_range_left"]
    gr = "无" if info["geom_range_right"] is None else "[%.1f,%.1f]°" % info["geom_range_right"]
    el = "无" if info["elec_range_left"] is None else "[%.1f,%.1f]°" % info["elec_range_left"]
    er = "无" if info["elec_range_right"] is None else "[%.1f,%.1f]°" % info["elec_range_right"]
    pl = "无" if info["pulse_range_left"] is None else "[%.0f,%.0f]us" % info["pulse_range_left"]
    pr = "无" if info["pulse_range_right"] is None else "[%.0f,%.0f]us" % info["pulse_range_right"]
    pen_str = "就绪" if info["pen_ready"] else info["pen_info"]
    if info["ok"]:
        return (
            "预检通过: 板中心=(%.2f, %.2f) 文本='%s' (%d笔) "
            "几何左=%s 右=%s 电气左=%s 右=%s 脉宽左=%s 右=%s 抬笔=%s"
            % (info["center"][0], info["center"][1], info["text"], info["strokes_count"],
               gl, gr, el, er, pl, pr, pen_str)
        )
    rej = info["first_reject"]
    return (
        "预检拒绝: 板中心=(%.2f, %.2f) 文本='%s' 首拒点[%s @(%.2f, %.2f)]: %s\n"
        "  统计范围: 纯IK几何左=%s 右=%s | 映射电气左=%s 右=%s | 合法脉宽左=%s 右=%s"
        % (info["center"][0], info["center"][1], info["text"],
           rej["stage"], rej["point"][0], rej["point"][1], rej["reason"],
           gl, gr, el, er, pl, pr)
    )


def _move_pen_ramped(pen_pwm, current_deg, target_deg):
    new_deg, _ = f.ramp_pen_to(pen_pwm, current_deg, target_deg)
    return new_deg


def main(input_fn=input):
    print("警告：仅在拆下笔并解除主动臂与舵机机械耦合后使用驱动舵机命令！")
    print("命令: center | left <角> | right <角> | eleft <角> | eright <角> | "
          "calleft <g90> <g110> | calright <g90> <g110> | pen <角|up|down> | check [文本] [x y|midpoint dx fwd] | quit")
    left = right = pen = None
    pen_deg = None
    try:
        while True:
            try:
                raw = input_fn("setup> ")
            except EOFError:
                break
            try:
                action = parse_command(raw)
            except ValueError as error:
                print("拒绝:", error)
                continue
            if action is None:
                continue
            kind = action[0]
            if kind == "quit":
                break
            if kind == "check":
                print(_format_check_report(action[1]))
                continue
            if kind == "cal":
                _, side, e1, g1, e2, g2, res = action
                print(
                    "%s 标定结果（由 %.1f°->%.1f° 实测几何 %.1f°->%.1f°）：\n"
                    "  %s_US_PER_DEG = %.6f\n"
                    "  %s_GEOM_CENTER_DEG = %.2f\n"
                    "  %s_DIRECTION = %d\n"
                    "  %s_TRIM_DEG = 0.0  # 重要：须置 0.0，勿保留旧 TRIM=1.0 以免双算！"
                    % (side, e1, e2, g1, g2,
                       side, res["us_per_deg"],
                       side, res["geom_center_deg"],
                       side, res["direction"],
                       side)
                )
                continue

            if left is None:
                left, right, pen = _open_servos()
            if kind == "both":
                _, g1, g2, u, v, label = action
                f.set_pulse_us(left, u)
                f.set_pulse_us(right, v)
                print("已定位 %s -> 几何 %.1f° / %.1f° (电气中位 90°)" % (label, g1, g2))
            elif kind == "planar":
                _, side, geom, elec, pulse, idx = action
                f.set_pulse_us(left if idx == 0 else right, pulse)
                print("已定位%s舵机 -> 几何 %.1f° (电气 %.1f°)" % (side, geom, elec))
            elif kind == "elec":
                _, side, elec, pulse, idx = action
                f.set_pulse_us(left if idx == 0 else right, pulse)
                print("已定位%s舵机 -> 电气 %.1f°" % (side, elec))
            elif kind == "pen":
                _, deg, _pulse = action
                pen_deg = _move_pen_ramped(pen, pen_deg, deg)
                print("已定位抬笔舵机 -> %.1f°" % deg)
    except KeyboardInterrupt:
        print("收到 Ctrl+C，退出角度调试并关闭三路 PWM。")
    finally:
        _close_servos(left, right, pen)


if __name__ == "__main__":
    main()
