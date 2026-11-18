"""
Car control GUI (pygame) for the STM32 / FreeRTOS car.

Protocol (taken from main.c):
  PC -> car   "M x y\r\n"  drive vector, left = x - y, right = -x - y   (-100..100, forward = negative y)
              'H' horn | 'G' toggle left LED | 'F' toggle right LED
              'J' blink left LED | 'K' blink right LED
              'I' toggle autonomous mode | 'Q' toggle sonar scan
  car -> PC   CSV line, 12 fields:
              rpm_R,rpm_L,pwm_R,pwm_L,ticks_L,ticks_R,angle,distance,led_R,led_L,scan,auto\r\n
              (rpm and pwm are sent multiplied by 10)

Run:
  pip install pygame bleak
  python car_gui.py              # connect to the BLE module
  python car_gui.py --demo       # fake car, to test the GUI without hardware
  python car_gui.py --mac AA:BB:CC:DD:EE:FF
"""
import argparse
import asyncio
import math
import re
import threading
import time

import pygame

# ----------------------------------------------------------------- config --
HC05_MAC = "61:18:05:7D:15:A8"
UART_CHAR = "0000ffe1-0000-1000-8000-00805f9b34fb"

SEND_INTERVAL = 0.1          # seconds between "M x y" packets (firmware loop is 100 ms)
DEFAULT_POWER = 60           # joystick full-deflection power in %
MAX_RPM_DISPLAY = 250.0      # full scale of the RPM bars (set to your MAX_RPM)
OBSTACLE_CM = 15             # firmware brakes at Distance <= 15
RADAR_RANGE_CM = 100
TELEM_TIMEOUT = 1.5          # s without telemetry -> "stale"
N_FIELDS = 12
LOCAL_HOLD = 0.5             # s: after a button press, trust the local guess until telemetry catches up

# ------------------------------------------------------------- telemetry --
class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.buf = b""
        self.rpm_L = self.rpm_R = 0.0
        self.pwm_L = self.pwm_R = 0.0
        self.ticks_L = self.ticks_R = 0
        self.angle = 90
        self.distance = 400
        self.led_right = self.led_left = False
        self.scan = self.auto = False
        self.last_update = 0.0

    def feed(self, data: bytes):
        with self.lock:
            self.buf += data
            if len(self.buf) > 2048:
                self.buf = b""
            while b"\n" in self.buf:           # BLE packets are ~20 bytes: reassemble lines
                line, self.buf = self.buf.split(b"\n", 1)
                self._parse(line.decode("utf-8", errors="ignore"))

    def _parse(self, line):
        parts = line.strip().split(",")
        if len(parts) != N_FIELDS:
            return
        try:
            rR, rL, pR, pL, tL, tR, ang, dist, lR, lL, scan, auto = (int(p) for p in parts)
        except ValueError:
            return                                   # partial / corrupted line
        self.rpm_R, self.rpm_L = rR / 10.0, rL / 10.0
        self.pwm_R, self.pwm_L = pR / 10.0, pL / 10.0
        self.ticks_L, self.ticks_R = tL, tR
        self.angle = max(0, min(180, ang))
        self.distance = dist
        self.led_right, self.led_left = bool(lR), bool(lL)
        self.scan, self.auto = bool(scan), bool(auto)
        self.last_update = time.monotonic()

    def snapshot(self):
        with self.lock:
            return dict(rpm_L=self.rpm_L, rpm_R=self.rpm_R, pwm_L=self.pwm_L, pwm_R=self.pwm_R,
                        ticks_L=self.ticks_L, ticks_R=self.ticks_R, angle=self.angle,
                        distance=self.distance, led_left=self.led_left, led_right=self.led_right,
                        scan=self.scan, auto=self.auto, last_update=self.last_update)

# ----------------------------------------------------------------- links --
class BleLink:
    def __init__(self, telem, mac):
        self.telem, self.mac = telem, mac
        self.client = None
        self.loop = None
        self.status = "connecting..."

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._main())

    async def _main(self):
        try:
            from bleak import BleakClient
        except ImportError:
            self.status = "bleak not installed"
            return
        while True:
            try:
                self.status = "connecting..."
                async with BleakClient(self.mac) as client:
                    self.client = client
                    self.status = "connected"
                    await client.start_notify(UART_CHAR, lambda _, d: self.telem.feed(bytes(d)))
                    while client.is_connected:
                        await asyncio.sleep(0.5)
            except Exception as e:                      # noqa: BLE001
                self.status = f"error: {str(e)[:40]}"
            self.client = None
            await asyncio.sleep(2)

    @property
    def connected(self):
        return self.client is not None and self.client.is_connected

    def send(self, data: bytes):
        if not self.connected or self.loop is None:
            return False
        fut = asyncio.run_coroutine_threadsafe(
            self.client.write_gatt_char(UART_CHAR, data, response=True), self.loop)
        fut.add_done_callback(lambda f: f.exception() and print("write failed:", f.exception()))
        return True


class DemoLink:
    """Fake car so the GUI can be tried without hardware."""
    connected = True
    status = "demo mode"

    def __init__(self, telem):
        self.telem = telem
        self.x = self.y = 0
        self.scan = False
        self.auto = False
        self.led_l = self.led_r = False
        self.blink_l = self.blink_r = 0.0

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def send(self, data: bytes):
        s = data.decode("ascii", errors="ignore").strip()
        if s.startswith("M"):
            try:
                _, x, y = s.split()
                self.x, self.y = int(x), int(y)
            except ValueError:
                pass
        elif s == "Q":
            self.scan = not self.scan
        elif s == "I":
            self.auto = not self.auto
        elif s == "G":
            self.led_l = not self.led_l
        elif s == "F":
            self.led_r = not self.led_r
        elif s == "J":
            self.blink_l = time.monotonic() + 1.0
        elif s == "K":
            self.blink_r = time.monotonic() + 1.0
        return True

    def _run(self):
        rL = rR = 0.0
        ticksL = ticksR = 0.0
        ang, inc, t = 90, 3, 0.0
        while True:
            tgtL = (0.6 if self.auto else (self.x - self.y) / 100.0) * 200
            tgtR = (0.6 if self.auto else (-self.x - self.y) / 100.0) * 200
            rL += (abs(tgtL) - rL) * 0.3
            rR += (abs(tgtR) - rR) * 0.3
            pL = 0 if abs(tgtL) < 1 else 25 + rL / 200 * 75
            pR = 0 if abs(tgtR) < 1 else 25 + rR / 200 * 75
            ticksL += rL / 60 * 0.1 * 20
            ticksR += rR / 60 * 0.1 * 20
            if self.scan:
                ang += inc
                if ang >= 180 or ang <= 0:
                    inc = -inc
            t += 0.1
            dist = int(45 + 40 * math.sin(ang / 25.0 + t * 0.5))
            now = time.monotonic()
            led_l = self.led_l or (now < self.blink_l and int(now * 5) % 2 == 0)
            led_r = self.led_r or (now < self.blink_r and int(now * 5) % 2 == 0)
            line = (f"{int(rR*10)},{int(rL*10)},{int(pR*10)},{int(pL*10)},{int(ticksL)},{int(ticksR)},"
                    f"{ang},{dist},{int(led_r)},{int(led_l)},{int(self.scan)},{int(self.auto)}\r\n")
            self.telem.feed(line.encode())
            time.sleep(0.1)

# --------------------------------------------------------------- drawing --
BG = (22, 24, 32)
PANEL = (34, 37, 49)
PANEL_EDGE = (62, 67, 88)
TXT = (225, 228, 238)
DIM = (140, 146, 165)
GREEN = (70, 220, 120)
RED = (235, 80, 80)
AMBER = (240, 180, 60)
BLUE = (80, 160, 255)

_fonts = {}


def font(size, bold=False):
    key = (size, bold)
    if key not in _fonts:
        _fonts[key] = pygame.font.SysFont("dejavusans,arial,helvetica", size, bold=bold)
    return _fonts[key]


def text(surf, s, size, color, pos, anchor="topleft", bold=False):
    img = font(size, bold).render(str(s), True, color)
    r = img.get_rect(**{anchor: pos})
    surf.blit(img, r)
    return r


def panel(surf, rect, title):
    pygame.draw.rect(surf, PANEL, rect, border_radius=12)
    pygame.draw.rect(surf, PANEL_EDGE, rect, 2, border_radius=12)
    text(surf, title, 16, DIM, (rect[0] + 16, rect[1] + 10), bold=True)


def bar(surf, rect, value, vmax, color, label, value_text):
    x, y, w, h = rect
    text(surf, label, 14, DIM, (x, y - 18))
    text(surf, value_text, 14, TXT, (x + w, y - 18), anchor="topright")
    pygame.draw.rect(surf, (20, 22, 30), rect, border_radius=6)
    frac = max(0.0, min(1.0, abs(value) / vmax))
    if frac > 0:
        pygame.draw.rect(surf, color, (x, y, max(6, int(w * frac)), h), border_radius=6)
    pygame.draw.rect(surf, PANEL_EDGE, rect, 1, border_radius=6)


def pill(surf, rect, label, on, color=GREEN):
    pygame.draw.rect(surf, (28, 30, 40), rect, border_radius=15)
    pygame.draw.rect(surf, color if on else PANEL_EDGE, rect, 2, border_radius=15)
    pygame.draw.circle(surf, color if on else (70, 74, 92), (rect[0] + 16, rect[1] + rect[3] // 2), 6)
    text(surf, label, 13, TXT if on else DIM, (rect[0] + 28, rect[1] + rect[3] // 2), anchor="midleft", bold=True)


def pointer_pos(event):
    if event.type in (pygame.FINGERDOWN, pygame.FINGERMOTION, pygame.FINGERUP):
        w, h = pygame.display.get_surface().get_size()
        return pygame.Vector2(event.x * w, event.y * h)
    return pygame.Vector2(event.pos)

# --------------------------------------------------------------- widgets --
class VirtualJoystick:
    def __init__(self, center, radius, deadzone=0.12):
        self.center = pygame.Vector2(center)
        self.radius = radius
        self.knob_radius = max(8, radius // 3)
        self.knob_pos = pygame.Vector2(center)
        self.deadzone = deadzone
        self.active_pointer = None
        self.active = False

    def _inside(self, pos):
        return pos.distance_to(self.center) <= self.radius + 10

    def _set_knob(self, pos):
        delta = pos - self.center
        if delta.length_squared() > self.radius ** 2:
            delta.scale_to_length(self.radius)
        self.knob_pos = self.center + delta

    def reset(self):
        self.active_pointer = None
        self.active = False
        self.knob_pos = self.center.copy()

    def handle_event(self, e):
        if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
            p = pointer_pos(e)
            if self.active_pointer is None and self._inside(p):
                self.active_pointer, self.active = "mouse", True
                self._set_knob(p)
        elif e.type == pygame.MOUSEMOTION and self.active_pointer == "mouse":
            if e.buttons[0]:
                self._set_knob(pointer_pos(e))
            else:
                self.reset()
        elif e.type == pygame.MOUSEBUTTONUP and e.button == 1 and self.active_pointer == "mouse":
            self.reset()
        elif e.type == pygame.FINGERDOWN:
            p = pointer_pos(e)
            if self.active_pointer is None and self._inside(p):
                self.active_pointer, self.active = e.finger_id, True
                self._set_knob(p)
        elif e.type == pygame.FINGERMOTION and self.active_pointer == e.finger_id:
            self._set_knob(pointer_pos(e))
        elif e.type == pygame.FINGERUP and self.active_pointer == e.finger_id:
            self.reset()

    def get_xy(self, power):
        """Integer (x, y) for the 'M x y' command. Forward (up) = negative y, as the firmware expects."""
        if not self.active:
            return 0, 0
        delta = self.knob_pos - self.center
        if delta.length() < self.deadzone * self.radius:
            return 0, 0
        v = delta / self.radius * power
        x, y = v.x, v.y
        s = abs(x) + abs(y)
        if s > 100:                       # keep both wheel targets within +-100 %
            x, y = x * 100 / s, y * 100 / s
        return round(x), round(y)

    def draw(self, surf, xy):
        c = (int(self.center.x), int(self.center.y))
        k = (int(self.knob_pos.x), int(self.knob_pos.y))
        pygame.draw.circle(surf, (48, 52, 68), c, self.radius)
        pygame.draw.circle(surf, (120, 126, 150), c, self.radius, 3)
        pygame.draw.line(surf, (90, 96, 120), (c[0] - self.radius, c[1]), (c[0] + self.radius, c[1]), 1)
        pygame.draw.line(surf, (90, 96, 120), (c[0], c[1] - self.radius), (c[0], c[1] + self.radius), 1)
        pygame.draw.line(surf, (130, 136, 160), c, k, 4)
        pygame.draw.circle(surf, (255, 110, 110) if self.active or xy != (0, 0) else (200, 80, 80), k, self.knob_radius)
        pygame.draw.circle(surf, (255, 215, 215), k, self.knob_radius, 2)


class Slider:
    def __init__(self, rect, lo, hi, value):
        self.rect = pygame.Rect(rect)
        self.lo, self.hi, self.value = lo, hi, value
        self.drag = False

    def _set_from_x(self, x):
        f = (x - self.rect.x) / self.rect.w
        self.value = int(round(self.lo + max(0.0, min(1.0, f)) * (self.hi - self.lo)))

    def handle_event(self, e):
        if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1 and self.rect.inflate(20, 30).collidepoint(e.pos):
            self.drag = True
            self._set_from_x(e.pos[0])
        elif e.type == pygame.MOUSEMOTION and self.drag:
            self._set_from_x(e.pos[0])
        elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
            self.drag = False

    def draw(self, surf):
        r = self.rect
        pygame.draw.rect(surf, (20, 22, 30), r, border_radius=r.h // 2)
        f = (self.value - self.lo) / (self.hi - self.lo)
        pygame.draw.rect(surf, BLUE, (r.x, r.y, int(r.w * f), r.h), border_radius=r.h // 2)
        pygame.draw.circle(surf, TXT, (int(r.x + r.w * f), r.centery), r.h + 3)


class Button:
    def __init__(self, rect, label, sub, on_click, is_on=None, color=GREEN):
        self.rect = pygame.Rect(rect)
        self.label, self.sub = label, sub
        self.on_click, self.is_on, self.color = on_click, is_on, color
        self.flash_until = 0.0

    def handle_event(self, e):
        if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1 and self.rect.collidepoint(e.pos):
            self.flash_until = time.monotonic() + 0.15
            self.on_click()

    def draw(self, surf):
        on = (self.is_on() if self.is_on else False) or time.monotonic() < self.flash_until
        hover = self.rect.collidepoint(pygame.mouse.get_pos())
        fill = tuple(int(c * 0.35) for c in self.color) if on else ((58, 63, 82) if hover else (46, 50, 66))
        pygame.draw.rect(surf, fill, self.rect, border_radius=10)
        pygame.draw.rect(surf, self.color if on else PANEL_EDGE, self.rect, 2, border_radius=10)
        text(surf, self.label, 17, TXT, (self.rect.centerx, self.rect.centery - 8), anchor="center", bold=True)
        text(surf, self.sub, 12, DIM, (self.rect.centerx, self.rect.centery + 14), anchor="center")


def draw_radar(surf, cx, cy, R, hist, cur_angle, now):
    def pt(a, r):
        return (cx + r * math.cos(math.radians(a)), cy - r * math.sin(math.radians(a)))

    for f in (0.25, 0.5, 0.75, 1.0):
        pygame.draw.lines(surf, (40, 110, 70), False, [pt(a, R * f) for a in range(0, 181, 3)], 2)
        text(surf, int(RADAR_RANGE_CM * f), 11, DIM, (cx + R * f, cy + 4), anchor="midtop")
    pygame.draw.line(surf, (40, 110, 70), (cx - R, cy), (cx + R, cy), 2)
    for a in range(30, 151, 30):
        pygame.draw.line(surf, (40, 110, 70), (cx, cy), pt(a, R), 1)
        text(surf, f"{a}\u00b0", 12, GREEN, pt(a, R + 14), anchor="center")
    for a, (d, t) in list(hist.items()):             # echoes, fading over 4 s
        age = now - t
        if age > 4:
            del hist[a]
            continue
        k = 1 - age / 4
        pygame.draw.line(surf, (int(255 * k), 30, 30), pt(a, R * d / RADAR_RANGE_CM), pt(a, R), 4)
    pygame.draw.line(surf, GREEN, (cx, cy), pt(cur_angle, R), 3)


class CarState:
    """LED / scan / auto flags. Taken from telemetry; a button press sets them optimistically
    and telemetry is ignored for LOCAL_HOLD seconds so the indicator doesn't flicker back."""
    led_left = led_right = auto = scan = False
    horn_until = 0.0

    def __init__(self):
        self.touched = {}

    def toggle(self, name):
        setattr(self, name, not getattr(self, name))
        self.touched[name] = time.monotonic()

    def sync(self, tm, now):
        for name in ("led_left", "led_right", "auto", "scan"):
            if now - self.touched.get(name, 0.0) > LOCAL_HOLD:
                setattr(self, name, tm[name])

# ------------------------------------------------------------------ main --
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true", help="simulate the car (no Bluetooth)")
    ap.add_argument("--mac", default=HC05_MAC)
    ap.add_argument("--frames", type=int, default=0, help=argparse.SUPPRESS)  # for headless tests
    args = ap.parse_args()

    telem = Telemetry()
    link = DemoLink(telem) if args.demo else BleLink(telem, args.mac)
    link.start()

    pygame.init()
    W, H = 1100, 700
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Car Control")
    clock = pygame.time.Clock()

    joystick = VirtualJoystick((210, 410), 125)
    power = Slider((50, 625, 320, 8), 10, 100, DEFAULT_POWER)
    st = CarState()
    radar_hist, last_seen = {}, 0.0

    def cmd(ch):
        link.send(ch)

    def toggle_auto():
        cmd(b"I")
        st.toggle("auto")

    def act_horn():
        cmd(b"H"); st.horn_until = time.monotonic() + 0.5

    def act_led_l():
        cmd(b"G"); st.toggle("led_left")

    def act_led_r():
        cmd(b"F"); st.toggle("led_right")

    def act_scan():
        cmd(b"Q"); st.toggle("scan")

    def act_stop():
        joystick.reset()
        link.send(b"M 0 0\r\n")
        if st.auto:
            toggle_auto()

    bw, bh, gap, x0 = 150, 70, 12, 435
    defs = [
        ("HORN", "H", act_horn, lambda: time.monotonic() < st.horn_until, AMBER),
        ("LEFT LED", "G  toggle", act_led_l, lambda: st.led_left, AMBER),
        ("RIGHT LED", "F  toggle", act_led_r, lambda: st.led_right, AMBER),
        ("AUTONOMOUS", "I  toggle", toggle_auto, lambda: st.auto, GREEN),
        ("BLINK LEFT", "J", lambda: cmd(b"J"), None, AMBER),
        ("BLINK RIGHT", "K", lambda: cmd(b"K"), None, AMBER),
        ("SONAR SCAN", "Q  toggle", act_scan, lambda: st.scan, BLUE),
        ("STOP", "M 0 0 (+ auto off)", act_stop, None, RED),
    ]
    buttons = []
    for i, (label, sub, fn, on, col) in enumerate(defs):
        r = (x0 + (i % 4) * (bw + gap), 430 + (i // 4) * (bh + gap), bw, bh)
        buttons.append(Button(r, label, sub, fn, on, col))

    last_send, frame, running = 0.0, 0, True
    while running:
        clock.tick(60)
        frame += 1
        if args.frames and frame > args.frames:
            break
        now = time.monotonic()

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                elif e.key == pygame.K_SPACE:
                    act_horn()
            joystick.handle_event(e)
            power.handle_event(e)
            for b in buttons:
                b.handle_event(e)

        # ---- drive vector (joystick, or keyboard when the joystick is idle)
        x, y = joystick.get_xy(power.value)
        if (x, y) == (0, 0):
            k = pygame.key.get_pressed()
            kx = (k[pygame.K_d] or k[pygame.K_RIGHT]) - (k[pygame.K_a] or k[pygame.K_LEFT])
            ky = (k[pygame.K_s] or k[pygame.K_DOWN]) - (k[pygame.K_w] or k[pygame.K_UP])
            x, y = int(kx * power.value), int(ky * power.value)
            s = abs(x) + abs(y)
            if s > 100:
                x, y = round(x * 100 / s), round(y * 100 / s)

        # ---- stream "M x y" (the firmware ignores it in autonomous mode, so don't send then)
        if not st.auto and now - last_send >= SEND_INTERVAL:
            last_send = now
            link.send(f"M {x} {y}\r\n".encode("ascii"))

        tm = telem.snapshot()
        fresh = now - tm["last_update"] < TELEM_TIMEOUT
        if fresh:
            st.sync(tm, now)
        if tm["last_update"] > last_seen:
            last_seen = tm["last_update"]
            if tm["distance"] <= RADAR_RANGE_CM:
                radar_hist[tm["angle"]] = (tm["distance"], now)
            else:
                radar_hist.pop(tm["angle"], None)
        obstacle = fresh and tm["distance"] <= OBSTACLE_CM

        # ================================================================ draw
        screen.fill(BG)
        text(screen, "CAR CONTROL", 26, TXT, (20, 14), bold=True)
        linked = link.connected
        pygame.draw.circle(screen, GREEN if linked else RED, (W - 330, 32), 8)
        text(screen, f"Link: {link.status}", 15, TXT, (W - 315, 32), anchor="midleft")
        mode = "AUTONOMOUS" if st.auto else "MANUAL"
        text(screen, f"Mode: {mode}", 18, GREEN if st.auto else AMBER, (W - 20, 32), anchor="midright", bold=True)

        # ---- drive panel
        panel(screen, (20, 70, 380, 610), "DRIVE")
        tl, tr = x - y, -x - y
        text(screen, f"x {x:+d}   y {y:+d}", 20, TXT, (210, 118), anchor="center", bold=True)
        text(screen, f"target  L {tl:+d}%   R {tr:+d}%", 15, DIM, (210, 146), anchor="center")
        if st.auto:
            text(screen, "joystick ignored in autonomous mode", 13, AMBER, (210, 168), anchor="center")
        joystick.draw(screen, (x, y))
        text(screen, f"Power limit: {power.value}%", 15, TXT, (50, 598))
        power.draw(screen)

        # ---- motors panel
        panel(screen, (420, 70, 330, 300), "MOTORS")
        for i, (name, rpm, pwm, ticks) in enumerate((
                ("LEFT", tm["rpm_L"], tm["pwm_L"], tm["ticks_L"]),
                ("RIGHT", tm["rpm_R"], tm["pwm_R"], tm["ticks_R"]))):
            y0 = 100 + i * 135
            text(screen, name, 18, TXT, (440, y0), bold=True)
            text(screen, f"ticks {ticks}", 14, DIM, (730, y0 + 3), anchor="topright")
            bar(screen, (440, y0 + 48, 290, 20), rpm, MAX_RPM_DISPLAY, BLUE, "Speed", f"{rpm:.1f} RPM")
            bar(screen, (440, y0 + 94, 290, 20), pwm, 100.0, AMBER, "PWM", f"{pwm:.1f} %")
        if not fresh:
            text(screen, "no telemetry", 13, RED, (730, 12 + 70), anchor="topright", bold=True)

        # ---- radar panel
        panel(screen, (770, 70, 310, 300), "SONAR")
        dist_txt = f"{tm['distance']} cm" if tm["distance"] < 400 else "-- cm"
        text(screen, f"Angle {tm['angle']}\u00b0", 18, TXT, (790, 100), bold=True)
        text(screen, f"Distance {dist_txt}", 18, RED if obstacle else TXT, (790, 126), bold=True)
        draw_radar(screen, 925, 335, 130, radar_hist, tm["angle"], now)

        # ---- controls panel
        panel(screen, (420, 390, 660, 290), "CONTROLS")
        for b in buttons:
            b.draw(screen)
        pw = 98
        pills = [("AUTO", st.auto, GREEN), ("SCAN", st.scan, BLUE), ("L LED", st.led_left, AMBER),
                 ("R LED", st.led_right, AMBER), ("OBSTACLE", obstacle, RED), ("TELEM", fresh, GREEN)]
        for i, (lbl, on, col) in enumerate(pills):
            pill(screen, (x0 + i * (pw + 9), 610, pw, 30), lbl, on, col)
        text(screen, "WASD / arrows drive  |  Space horn  |  Esc quit   (LED, scan and mode states come from telemetry)",
             12, DIM, (435, 656))

        if obstacle:
            text(screen, f"OBSTACLE {tm['distance']} cm", 20, RED, (W // 2, 32), anchor="center", bold=True)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
