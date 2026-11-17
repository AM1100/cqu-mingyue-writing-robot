"""ESP32 MicroPython 五连杆三舵机居中逐笔版上电单次书写入口（v7）。

启动即打印 VERSION 与关键映射参数，便于核对 ESP32 上实际运行文件；
支持单字及整组一次书写（如 main.run(text='CQU') 或 main.write('CQU')），
整图预检不通过时零 PWM、零部分绘制。import 本模块不会创建 PWM。
"""
import math
import CQU_fivebar_fixed as f

AUTOSTART = False        # 默认关闭上电自启，避免上传复位即按未实测旧增益重画；完成两点校准与 check 后改为 True
STARTUP_DELAY_MS = 5000  # 开机 5 秒窗口持续检测 BOOT(GPIO0) 取消
DRAW_TEXT = None         # 为 None 时读取 CQU_fivebar_fixed.DRAW_TEXT（默认 '8'）

BOOT_PIN = 0
BOOT_POLL_MS = 50
IDLE_POLL_MS = 100

ACTIVE_PLOTTER = None


def format_runtime_summary():
    """返回关键版本与映射参数摘要，便于核对板上实际运行代码与配置。"""
    return (
        "[%s] center=%r offset=%.1f L(ref=%.1f dir=%+d trim=%.1f gain=%.3f) "
        "R(ref=%.1f dir=%+d trim=%.1f gain=%.3f) pulse=%d..%d "
        "pen(up=%r down=%r gain=%.3f pulse=%d..%d)"
        % (
            f.VERSION, f.BOARD_CENTER, f.OFFSET_DEG,
            f.LEFT_GEOM_CENTER_DEG, f.LEFT_DIRECTION, f.LEFT_TRIM_DEG, f.LEFT_US_PER_DEG,
            f.RIGHT_GEOM_CENTER_DEG, f.RIGHT_DIRECTION, f.RIGHT_TRIM_DEG, f.RIGHT_US_PER_DEG,
            f.PULSE_MIN, f.PULSE_MAX,
            f.PEN_UP_DEG, f.PEN_DOWN_DEG, f._PEN_US_PER_DEG, f.PEN_PULSE_MIN, f.PEN_PULSE_MAX,
        )
    )


def make_boot_pin(pin=BOOT_PIN):
    """创建 BOOT 按键输入对象（内部上拉，低电平有效）。"""
    from machine import Pin
    return Pin(pin, Pin.IN, Pin.PULL_UP)


def validate_ready(text=None, startup_delay_ms=None, boot_pin=BOOT_PIN, board_center=None):
    """创建三路 PWM 前的门禁校验与整幅逐笔预检（含全部空移与抬落笔角度校验）。"""
    if f.CALIBRATED is not True:
        raise ValueError("首次门禁未放行：请先按 README 完成实测标定并设 CALIBRATED=True")

    if boot_pin is not None:
        if isinstance(boot_pin, bool) or not isinstance(boot_pin, int) or boot_pin < 0:
            raise ValueError("BOOT 引脚配置无效: %r" % (boot_pin,))

    f.validate_pins(boot_pin=boot_pin)
    f.validate_pen_config()

    if startup_delay_ms is None:
        startup_delay_ms = STARTUP_DELAY_MS
    if (isinstance(startup_delay_ms, bool)
            or not isinstance(startup_delay_ms, (int, float))
            or not math.isfinite(startup_delay_ms)
            or startup_delay_ms < 0):
        raise ValueError("STARTUP_DELAY_MS 无效: %r" % (startup_delay_ms,))

    if text is None:
        text = DRAW_TEXT
    normalized_text = f.normalize_text(text)
    home_point = f.resolve_home_point(board_center=board_center)
    strokes = f.text_strokes(normalized_text, board_center=home_point)
    f.check_strokes(strokes, start=home_point,
                    draw_speed=f.DRAW_SPEED, travel_speed=f.TRAVEL_SPEED)
    return normalized_text, int(round(startup_delay_ms)), home_point, strokes


def wait_startup_cancelled(delay_ms=None, poll_ms=BOOT_POLL_MS, boot_pin=BOOT_PIN):
    """开机窗口内持续检测 GPIO0 上拉按键；按住返回 True 并阻止创建 PWM。

    说明：BOOT(GPIO0) 仅作开机启动取消，非运动急停；复位瞬间按住会进入下载模式。
    """
    if delay_ms is None:
        delay_ms = STARTUP_DELAY_MS
    if boot_pin is None:
        f._sleep_ms(delay_ms)
        return False
    pin = make_boot_pin(boot_pin)
    if pin.value() == 0:
        return True
    remaining = int(round(delay_ms))
    while remaining > 0:
        step = poll_ms if remaining > poll_ms else remaining
        f._sleep_ms(step)
        remaining -= step
        if pin.value() == 0:
            return True
    return False


def _safe_shutdown(plotter):
    global ACTIVE_PLOTTER
    if plotter is None:
        return
    try:
        if getattr(plotter, "pen", None) is not None and f.PEN_UP_DEG is not None:
            plotter.lift(True)
    except BaseException as error:
        print("异常/停止清理时抬笔失败:", error)
    finally:
        try:
            plotter.close()
        finally:
            if ACTIVE_PLOTTER is plotter:
                ACTIVE_PLOTTER = None


def run(autostart=None, startup_delay_ms=None, text=None,
        boot_pin=BOOT_PIN, standby=True, board_center=None):
    """上电单次逐笔书写入口：打印版本参数 -> 全图预检 -> 5s BOOT 窗口 -> 抬笔归位 -> 逐笔书写。"""
    global ACTIVE_PLOTTER

    print(format_runtime_summary())

    if autostart is None:
        autostart = AUTOSTART
    if not isinstance(autostart, bool):
        print("拒绝自启动（未创建 PWM）: AUTOSTART 必须为布尔值")
        return None
    if not autostart:
        print("AUTOSTART 未启用，跳过自动书写。")
        return None

    try:
        draw_text, delay_ms, home_point, strokes = validate_ready(
            text=text,
            startup_delay_ms=startup_delay_ms,
            boot_pin=boot_pin,
            board_center=board_center,
        )
    except ValueError as error:
        print("拒绝自启动（未创建 PWM，无部分绘制）:", error)
        return None

    print("将在 %d ms 后于板中心 (%.2f, %.2f) 逐笔书写 '%s'；开机窗口按住 BOOT(GPIO%s) 可取消。"
          % (delay_ms, home_point[0], home_point[1], draw_text, boot_pin))
    try:
        if wait_startup_cancelled(delay_ms=delay_ms, boot_pin=boot_pin):
            print("检测到 BOOT(GPIO%s) 按下，已取消自启动（未创建 PWM）。" % boot_pin)
            return None
    except KeyboardInterrupt:
        print("启动窗口被 Ctrl+C 中断（未创建 PWM）。")
        return None

    plotter = f.Plotter()
    ACTIVE_PLOTTER = plotter
    keep_holding = False
    try:
        plotter.home(home_point)
        plotter.draw(strokes)
        print("单次逐笔书写 '%s' 完成，已抬笔并保持当前姿态待机（不会重复书写）。" % draw_text)
        print("说明：BOOT 仅作启动取消，非运动急停；需停机请按 Ctrl+C 尝试抬笔并关闭三路 PWM。")
        if not standby:
            keep_holding = True
            return plotter
        while True:
            f._sleep_ms(IDLE_POLL_MS)
    except KeyboardInterrupt:
        print("收到 Ctrl+C，尝试抬笔并关闭三路 PWM。")
        return None
    finally:
        if not keep_holding:
            _safe_shutdown(plotter)


def write(text="CQU", startup_delay_ms=0, standby=False, board_center=None, boot_pin=BOOT_PIN):
    """手动单次书写快捷入口（例如 main.write('CQU')）；同样执行全图预检，不通过则零动作。"""
    return run(
        autostart=True,
        startup_delay_ms=startup_delay_ms,
        text=text,
        boot_pin=boot_pin,
        standby=standby,
        board_center=board_center,
    )


if __name__ == "__main__":
    run()
