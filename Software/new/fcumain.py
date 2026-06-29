# =============================================================================
# FCU_main.py  –  Flight Computer Unit (FCU)
# RPi Pico 2
#
# Responsibilities:
#   - Read BNO055 (IMU), BMP280 (barometer), KX134 (high-G accel)
#   - Run state machine, pyro logic
#   - Log flight data to onboard flash
#   - Send packed telemetry frames to TDU over UART (B2B connector)
#   - Receive ACK from TDU over the same UART (handshake only)
#
# Nav algorithm moved to TDU. FCU no longer receives GPS packets.
# TDU owns: GPS (Quectel L89HA), nav Kalman filter, lat/lon/track.
#
# Pin assignments (from FCU schematic Ember_v0.2):
#   I2C-0  : SDA=GP20  SCL=GP21  (BNO055 0x28, BMP280 0x76, KX134 0x1F,
#                                  ADS1115 0x48)
#   BNO_INT: GP18  (interrupt, not used in firmware yet)
#   KX_INT : GP19  (interrupt, not used in firmware yet)
#   Buzzer : GP2   (via MOSFET, passive PWM)
#   LED    : GP25  (onboard Pico LED)
#   LEDs   : GP0 (nav light A), GP1 (nav light B)
#   Pyros  : DROGUE=GP6  DROGUE2=GP7  MAIN=GP16  MAIN2=GP17
#   B2B    : UART-0  TX=GP4  RX=GP5  (to TDU)
#   ADC_REF: GP35
#   PYRO CONTINUITY:
#     GP26 (ADC0) = PYRO_DROGUE  (primary)
#     GP27 (ADC1) = PYRO_MAIN    (primary)
#   ADS1115 (U5, I2C-0, 0x48):
#     AIN0 = differential pressure probe  →  volt_dp
#     AIN1 = not connected
#     AIN2 = PYRO_MAIN2   (redundant)     →  volt_main2
#     AIN3 = PYRO_DROGUE2 (redundant)     →  volt_drogue2
#
# Packet formats:
#
#   B2B_FMT  '<si18f'  77 bytes  — sent to TDU over UART every 200 ms
#     'F' | t_ms(i) | state | temp
#     | bno ax ay az | gx gy gz | kx ax ay az
#     | alt | vel
#     | volt_drogue_primary | volt_main_primary
#     | volt_main2 | volt_drogue2 | dp
#
#   LOG_FMT  '<si19f'  81 bytes  — written to flash during flight
#     'L' | t_ms(i) | state | temp
#     | bno ax ay az | gx gy gz | kx ax ay az
#     | alt | vel
#     | volt_drogue_primary | volt_main_primary
#     | volt_dp | volt_main2 | volt_drogue2 | dp
#
# State machine:
#   UNARMED → FLIGHT_READY → BOOST → COAST → DROGUE → MAIN → LANDED
#
# Startup handshake (blocking, before async loop):
#   FCU → TDU : b'FCU_INIT_OK\n'                 every 100 ms until ACK
#   TDU → FCU : b'TDU_ACK\n'                     unblocks FCU
#   FCU runs  : _calib_bmp(), then sets calib_time
#   FCU → TDU : b'C' + calib_alt(f) + calib_temp(f)   9 bytes
# =============================================================================

from machine import I2C, Pin, UART, PWM, ADC, freq
from math   import sqrt, pi
from time   import ticks_ms, sleep_ms
from ulab   import numpy as np
from bno055_base import BNO055_BASE
from bno055 import *
import ustruct
import uasyncio as asyncio
import uos
import gc
import micropython

# ── optional high-G accel ────────────────────────────────────────────────────
try:
    from kx132 import KX132, ACC_RANGE_16
    KX_AVAILABLE = True
except ImportError:
    KX_AVAILABLE = False

# ── optional barometer ───────────────────────────────────────────────────────
try:
    from bmp280 import BMP280
    BMP_AVAILABLE = True
except ImportError:
    BMP_AVAILABLE = False

# ── optional ADS1115 ─────────────────────────────────────────────────────────
try:
    import ads1x15
    ADS_AVAILABLE = True
except ImportError:
    ADS_AVAILABLE = False

micropython.alloc_emergency_exception_buf(100)
gc.collect()
freq(150_000_000)


# =============================================================================
# Clash Royale fanfare
# =============================================================================
def play_clash_royale(buzzer: PWM):
    buzzer.duty_u16(0)
    C5      = 523;  D5  = 587;  F5  = 698
    Fsharp5 = 740;  G5  = 784;  Gsharp5 = 831
    A5      = 880;  Asharp5 = 932;  Csharp5 = 554
    C6      = 1047; F6  = 1397

    melody  = [Csharp5, Fsharp5, Gsharp5, C6, F6]
    tempo   = [14,      14,      14,      14,  6]
    melody1 = [Asharp5, A5, F5, G5, D5, C5]
    tempo1  = [4,       10, 12, 12,  8,  6]

    def play(mel, tmp):
        for i in range(len(mel)):
            buzzer.freq(mel[i])
            buzzer.duty_u16(32000)
            sleep_ms(1000 // tmp[i])
            buzzer.duty_u16(0)
            sleep_ms(20)

    play(melody, tempo)
    sleep_ms(1000)
    play(melody1, tempo1)
    buzzer.duty_u16(0)


# =============================================================================
# BNO055
# =============================================================================
class BNO055(BNO055_BASE):
    pass


# =============================================================================
# FCU
# =============================================================================
class FCU:

    G = 9.80665

    # ── States ────────────────────────────────────────────────────────────────
    STATE_UNARMED      = 0
    STATE_FLIGHT_READY = 1
    STATE_BOOST        = 2
    STATE_COAST        = 3
    STATE_DROGUE       = 4
    STATE_MAIN         = 5
    STATE_LANDED       = 6

    # ── Board-to-board telemetry (UART → TDU) ─────────────────────────────────
    # 'F' | t_ms(i) | state(f) | temp(f)
    # | bno ax ay az(3f) | gx gy gz(3f) | kx ax ay az(3f)
    # | alt(f) | vel(f)
    # | volt_drogue_primary(f) | volt_main_primary(f)
    # | volt_main2(f) | volt_drogue2(f) | dp(f)
    # = 1s + 1i + 18f = 77 bytes
    B2B_FMT = '<si18f'
    B2B_LEN = ustruct.calcsize(B2B_FMT)   # 77 bytes

    # ── Flash log format ──────────────────────────────────────────────────────
    # 'L' | t_ms(i) | state(f) | temp(f)
    # | bno ax ay az(3f) | gx gy gz(3f) | kx ax ay az(3f)
    # | alt(f) | vel(f)
    # | volt_drogue_primary(f) | volt_main_primary(f)
    # | volt_dp(f) | volt_main2(f) | volt_drogue2(f)
    # | dp(f) | airspeed(f)
    # = 1s + 1i + 20f = 85 bytes
    LOG_FMT = '<si19f'
    LOG_LEN = ustruct.calcsize(LOG_FMT)   # 81 bytes

    # ── Calibration packet ────────────────────────────────────────────────────
    # 'C' | calib_alt(f) | calib_temp(f)   → 9 bytes
    CALIB_FMT = '<s2f'
    CALIB_LEN = ustruct.calcsize(CALIB_FMT)

    # ── Pyro continuity thresholds ────────────────────────────────────────────
    PYRO_CONT_THRESHOLD = int(1.2 / 3.3 * 65535)   # ~23831 counts  (Pico ADC)
    ADS_CONT_THRESHOLD  = 1.2                        # volts          (ADS1115)
    PYRO_CONT_DURATION  = 3500                       # ms

    def __init__(self):
        self.state      = self.STATE_UNARMED
        self.t_log      = 0
        self.last_t_log = 0
        self.async_loop = None
        self.calib_time = 0

        self.bno = None
        self.bmp = None
        self.kx  = None
        self.ads = None
        self.b2b = None

        # ── Primary pyro continuity (Pico ADC) ───────────────────────────────
        # GP26 = PYRO_DROGUE   GP27 = PYRO_MAIN
        self.adc_pyro_drogue = None
        self.adc_pyro_main   = None

        self.volt_drogue_primary = 0.0
        self.volt_main_primary   = 0.0

        self.cont_ch1 = False   # drogue (GP26)
        self.cont_ch2 = False   # main   (GP27)

        # ── Redundant pyro continuity + airspeed (ADS1115) ───────────────────
        # AIN0 = diff pressure   AIN2 = PYRO_MAIN2   AIN3 = PYRO_DROGUE2
        self.volt_dp = 0.0
        self.volt_main2    = 0.0
        self.volt_drogue2  = 0.0

        self.cont_main2   = False
        self.cont_drogue2 = False

        self.ADS_ADDR    = 0x48
        self.ADS_RATE    = 7      # ads1x15 rate for set_conv — 860 SPS
        self.ads_working = False

        self.both_cont_since = 0

        # Beep queue: push (n, t_ms, freq) from any async context.
        # beep_task drains it without blocking the loop.
        self._beep_queue = []

        self.drogue_pin  = None
        self.drogue2_pin = None
        self.main_pin    = None
        self.main2_pin   = None

        self.buzzer = None
        self.led    = None
        self.led0   = None
        self.led1   = None

        self.bno_scale = 1.0
        self.kx_scale  = 1.0
        self.temp      = 0.0
        self.alt       = 0.0
        self.density   = 1.15

        # ── Rocket-frame sensor scalars ───────────────────────────────────────
        # BNO055 (Ember V0.2):
        #   axial (+up) = -BNO Y   lateral Y = BNO Z   lateral Z = BNO X
        # Gyro (rad/s):
        #   roll = -BNO gyro Y   pitch = BNO gyro Z   yaw = BNO gyro X
        # KX134:
        #   axial (+up) = KX X   lateral Y = KX Z   lateral Z = KX Y
        self.ax    = 0.0;  self.ay    = 0.0;  self.az    = 0.0
        self.gx    = 0.0;  self.gy    = 0.0;  self.gz    = 0.0
        self.kx_ax = 0.0;  self.kx_ay = 0.0;  self.kx_az = 0.0

        self.calib_altitude = 0.0
        self.calib_temp     = 0.0
        self.calib_count    = 0

        self.buf_len = 10
        self.alt_buf = np.zeros((self.buf_len,))
        self.vel_buf = np.zeros((self.buf_len,))
        self.acc_buf = np.zeros((self.buf_len,))

        self.t_events = [0.0] * 6

        # Two separate buffers — one for B2B, one for flash log
        self.data_b2b    = bytearray(self.B2B_LEN)
        self.data_log    = bytearray(self.LOG_LEN)
        self.data_buffer = bytearray(0)   # pre-launch rolling buffer (LOG frames)
        self.data_file   = None
        self.index       = 0
        self.size        = 0

        storage         = uos.statvfs('/')
        self.free_bytes = storage[0] * storage[3]
        self.logging_done = False

        # ── Flight parameters ─────────────────────────────────────────────────
        self.liftoff_accel             = 2.0
        self.min_liftoff_alt           = 10
        self.force_burnout_time        = 5000
        self.lockout_drogue_time       = 16000
        self.force_drogue_time         = 26000
        self.main_alt                  = 400
        self.drogue_to_main_lockout    = 60000
        self.ballistic_lockout_time    = 6000
        self.max_re_entry_speed        = -60.0
        self.touchdown_alt             = 50
        self.touchdown_vel_limit       = 0.1
        self.main_to_touchdown_lockout = 10000
        self.runtime                   = 36000000

        # ── Sensor calibration ───────────────────────────────────────────────
        # Differential pressure sensor supply voltage.
        # Datasheet: V_OUT = V_S * (0.009 * P_kPa + 0.04)
        # Tune if your 5V rail reads slightly off (±0.1V typical).
        self.VS_SUPPLY                 = 5.0   # V

        self.bno_working = True
        self.bmp_working = True
        self.kx_working  = True
        self.tdu_ready   = False
        self._flicker_event = False

    # ── altitude ──────────────────────────────────────────────────────────────
    def altitude(self):
        return 4947.19 * (8.9611 - pow(self.bmp.pressure, 0.190255))

    def rho(self, y):
        return (
            pow(8.9611 - (y + self.calib_altitude) / 4947.19, 5.2479)
            / (78410.439 + 287.06 * self.calib_temp - 1.86589 * (y + self.calib_altitude))
        )

    # ── failure logger ────────────────────────────────────────────────────────
    def failure(self, e):
        try:
            if self.free_bytes - self.size > 1000:
                with open('fcu_fail.txt', 'a') as f:
                    f.write(str(self.t_log) + ',' + str(e) + '\n')
        except Exception:
            pass

    # ── beeper (blocking — init context only) ─────────────────────────────────
    def beep(self, n=1, t=75, f=2000):
        self.buzzer.freq(f)
        for _ in range(n):
            self.buzzer.duty_u16(30000)
            sleep_ms(t)
            self.buzzer.duty_u16(0)
            if n > 1:
                sleep_ms(t)

    def beep_flight_ready(self):
        for frq in (2000, 3000, 4000):
            self.buzzer.freq(frq)
            self.buzzer.duty_u16(30000)
            sleep_ms(80)
            self.buzzer.duty_u16(0)
            sleep_ms(40)

    # ── async beep consumer ───────────────────────────────────────────────────
    async def beep_task(self):
        while True:
            if self._beep_queue:
                n, t, f = self._beep_queue.pop(0)
                self.buzzer.freq(f)
                for i in range(n):
                    self.buzzer.duty_u16(30000)
                    await asyncio.sleep_ms(t)
                    self.buzzer.duty_u16(0)
                    if i < n - 1:
                        await asyncio.sleep_ms(t)
            else:
                await asyncio.sleep_ms(20)

    # =========================================================================
    # Hardware init
    # =========================================================================
    def init_hardware(self):
        print('[FCU] Hardware init start')

        # ── GPIO ─────────────────────────────────────────────────────────────
        self.led    = Pin(25, Pin.OUT)
        self.led0   = Pin(0,  Pin.OUT);  self.led0.value(0)
        self.led1   = Pin(1,  Pin.OUT);  self.led1.value(0)
        self.buzzer = PWM(Pin(2));       self.buzzer.duty_u16(0)

        self.drogue_pin  = Pin(6,  Pin.OUT);  self.drogue_pin.value(0)
        self.drogue2_pin = Pin(7,  Pin.OUT);  self.drogue2_pin.value(0)
        self.main_pin    = Pin(16, Pin.OUT);  self.main_pin.value(0)
        self.main2_pin   = Pin(17, Pin.OUT);  self.main2_pin.value(0)

        # ── Primary pyro continuity ADC ───────────────────────────────────────
        self.adc_pyro_drogue = ADC(Pin(26))   # GP26 = PYRO_DROGUE
        self.adc_pyro_main   = ADC(Pin(27))   # GP27 = PYRO_MAIN
        print('[FCU] Pyro ADC ready (GP26=DROGUE, GP27=MAIN)')

        # ── UART to TDU ───────────────────────────────────────────────────────
        self.b2b = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
        print('[FCU] B2B UART ready')

        # ── I2C-0 ─────────────────────────────────────────────────────────────
        sleep_ms(200)
        i2c0 = I2C(0, scl=Pin(21), sda=Pin(20), freq=400_000)
        _devs = []
        for _attempt in range(3):
            _devs = i2c0.scan()
            if _devs:
                break
            print('[FCU] I2C scan empty, retrying ({})...'.format(_attempt + 1))
            sleep_ms(200)
        print('[FCU] I2C devices: {}'.format([hex(d) for d in _devs]))

        # ── BMP280 ────────────────────────────────────────────────────────────
        if BMP_AVAILABLE:
            try:
                self.bmp = BMP280(i2c0)
                print('[FCU] BMP280 OK')
            except Exception as e:
                print('[FCU] BMP280 FAIL:', e)
                self.failure('BMP280 init: ' + str(e))

        # ── BNO055 ────────────────────────────────────────────────────────────
        try:
            self.bno = BNO055(i2c0)
            self.bno.mode(0)
            sleep_ms(25)
            bno_offsets = bytearray(
                b'\xef\xff\t\x00\xef\xff\xae\x03\xa2\xff\x98\x04'
                b'\xff\xff\x02\x00\xff\xff\xe8\x03\x02\x03'
            )
            ACCEL_OFFSET_X_LSB = 0x55
            for i in range(6):
                self.bno._write(ACCEL_OFFSET_X_LSB + i, bno_offsets[i])
            self.bno._write(0x67, bno_offsets[18])
            self.bno._write(0x68, bno_offsets[19])
            sleep_ms(25)
            self.bno.mode(7)
            sleep_ms(20)
            print('[FCU] Place IMU at rest for scale cal')
            self.bno_scale = 0.0
            for _ in range(10):
                ba = (0.0, 0.0, 0.0)
                while ba == (0.0, 0.0, 0.0):
                    ba = self.bno.accel()
                    sleep_ms(1)
                self.bno_scale += self.G / sqrt(ba[0]**2 + ba[1]**2 + ba[2]**2) / 10
            print('[FCU] BNO055 OK  scale={:.4f}'.format(self.bno_scale))
        except Exception as e:
            print('[FCU] BNO055 FAIL:', e)
            self.failure('BNO055 init: ' + str(e))
            self.bno_working = False

        # ── KX134 ─────────────────────────────────────────────────────────────
        if KX_AVAILABLE:
            try:
                self.kx = KX132(i2c0, address=0x1F)
                self.kx.acc_range = ACC_RANGE_16
                self.kx_scale = 0.0
                for _ in range(10):
                    ka = (0.0, 0.0, 0.0)
                    while ka == (0.0, 0.0, 0.0):
                        ka = self.kx.acceleration
                        sleep_ms(1)
                    self.kx_scale += self.G / sqrt(ka[0]**2 + ka[1]**2 + ka[2]**2) / 10
                print('[FCU] KX134 OK  scale={:.4f}'.format(self.kx_scale))
            except Exception as e:
                print('[FCU] KX134 FAIL:', e)
                self.failure('KX134 init: ' + str(e))
                self.kx_working = False

        # ── ADS1115 ───────────────────────────────────────────────────────────
        # AIN0=diff_pressure  AIN2=PYRO_MAIN2  AIN3=PYRO_DROGUE2
        if ADS_AVAILABLE:
            try:
                self.ads = ads1x15.ADS1115(i2c0, self.ADS_ADDR)
                self.ads_working = True
                print('[FCU] ADS1115 OK (0x{:02X})'.format(self.ADS_ADDR))
            except Exception as e:
                print('[FCU] ADS1115 FAIL:', e)
                self.failure('ADS1115 init: ' + str(e))
                self.ads_working = False

        # ── Index file ────────────────────────────────────────────────────────
        try:
            with open('/index.txt', 'r') as f:
                self.index = int(f.read())
        except Exception:
            self.index = 0
        with open('/index.txt', 'w') as f:
            f.write(str(self.index + 1))

        print('[FCU] Hardware init complete')
        play_clash_royale(self.buzzer)

        # ── Wait for TDU ACK ──────────────────────────────────────────────────
        print('[FCU] Waiting for TDU ACK...')
        while True:
            self.b2b.write(b'FCU_INIT_OK\n')
            sleep_ms(100)
            rx = self.b2b.read(64)
            if rx and b'TDU_ACK' in rx:
                print('[FCU] TDU ACK received')
                self.tdu_ready = True
                break

        # ── BMP calibration ───────────────────────────────────────────────────
        self._calib_bmp()
        self.calib_time = ticks_ms()
        print('[FCU] calib_time set: {} ms from boot'.format(self.calib_time))

        # ── Send calibration data to TDU ──────────────────────────────────────
        calib_pkt = ustruct.pack(self.CALIB_FMT, b'C',
                                 self.calib_altitude, self.calib_temp)
        self.b2b.write(calib_pkt)
        print('[FCU] Sent calib: alt={:.1f}m  T={:.1f}°C'.format(
            self.calib_altitude, self.calib_temp))

    # ── BMP calibration ───────────────────────────────────────────────────────
    def _calib_bmp(self, n=10):
        if self.bmp is None:
            return
        try:
            self.calib_temp = self.bmp.temperature
            avg = 0.0
            for _ in range(n):
                avg += self.altitude()
                sleep_ms(10)
            self.calib_altitude = avg / n
            self.calib_count += 1
            print('[FCU] Calib alt={:.1f}m  T={:.1f}°C'.format(
                self.calib_altitude, self.calib_temp))
        except Exception as e:
            self.calib_altitude = 0.0
            self.failure('BMP calib: ' + str(e))

    # =========================================================================
    # Async tasks
    # =========================================================================

    async def blink(self, t):
        while True:
            self.led.value(0)
            await asyncio.sleep_ms(t)
            self.led.value(1)
            await asyncio.sleep_ms(t)

    async def led_flicker(self):
        async def strobe_normal(led, offset_ms):
            await asyncio.sleep_ms(offset_ms)
            while True:
                if self.state >= self.STATE_DROGUE:
                    await asyncio.sleep_ms(100)
                    continue
                led.value(1); await asyncio.sleep_ms(70)
                led.value(0); await asyncio.sleep_ms(100)
                led.value(1); await asyncio.sleep_ms(70)
                led.value(0); await asyncio.sleep_ms(1760)

        async def strobe_slow():
            while True:
                if self.state == self.STATE_DROGUE:
                    self.led0.value(1); self.led1.value(0)
                    await asyncio.sleep_ms(500)
                    self.led0.value(0)
                    await asyncio.sleep_ms(1500)
                elif self.state >= self.STATE_MAIN:
                    self.led0.value(1); self.led1.value(1)
                    await asyncio.sleep_ms(500)
                    self.led0.value(0); self.led1.value(0)
                    await asyncio.sleep_ms(1500)
                else:
                    await asyncio.sleep_ms(100)

        async def event_watcher():
            while True:
                if self._flicker_event:
                    self._flicker_event = False
                    for _ in range(6):
                        self.led0.value(1); self.led1.value(1)
                        await asyncio.sleep_ms(40)
                        self.led0.value(0); self.led1.value(0)
                        await asyncio.sleep_ms(40)
                await asyncio.sleep_ms(10)

        self.async_loop.create_task(strobe_normal(self.led0, 0))
        self.async_loop.create_task(strobe_normal(self.led1, 1000))
        self.async_loop.create_task(strobe_slow())
        self.async_loop.create_task(event_watcher())
        await asyncio.sleep(0)

    # ── sensor read loop ──────────────────────────────────────────────────────
    async def get_data(self):
        last_beep = 0
        sleep_ms(10)

        while True:
            self.t_log = ticks_ms() - self.calib_time

            # ── BNO055 ────────────────────────────────────────────────────────
            if self.bno is not None:
                try:
                    _ba = self.bno.accel()
                    _s  = self.bno_scale
                    self.ax = -float(_ba[1]) * _s
                    self.ay =  float(_ba[2]) * _s
                    self.az =  float(_ba[0]) * _s
                    _gy = self.bno.gyro()
                    self.gx = -float(_gy[1]) * pi / 180
                    self.gy =  float(_gy[2]) * pi / 180
                    self.gz =  float(_gy[0]) * pi / 180
                    self.bno_working = True
                except Exception as e:
                    self.bno_working = False
                    self.failure('BNO read: ' + str(e))

            # ── KX134 ─────────────────────────────────────────────────────────
            if self.kx is not None:
                try:
                    _ka = self.kx.acceleration
                    _ks = self.kx_scale
                    self.kx_ax =  float(_ka[0]) * _ks
                    self.kx_ay =  float(_ka[2]) * _ks
                    self.kx_az =  float(_ka[1]) * _ks
                    self.kx_working = True
                except Exception as e:
                    self.kx_working = False
                    self.failure('KX read: ' + str(e))

            # ── BMP280 ────────────────────────────────────────────────────────
            if self.bmp is not None:
                try:
                    self.temp    = self.bmp.temperature
                    self.alt     = self.altitude() - self.calib_altitude
                    self.density = self.rho(self.alt)
                    self.bmp_working = True
                except Exception as e:
                    self.bmp_working = False
                    self.failure('BMP read: ' + str(e))

            # ── Primary pyro continuity (Pico ADC) ───────────────────────────
            try:
                raw_drogue = self.adc_pyro_drogue.read_u16()
                raw_main   = self.adc_pyro_main.read_u16()
                self.volt_drogue_primary = raw_drogue * 3.3 / 65535
                self.volt_main_primary   = raw_main   * 3.3 / 65535
                self.cont_ch1 = raw_drogue > self.PYRO_CONT_THRESHOLD
                self.cont_ch2 = raw_main   > self.PYRO_CONT_THRESHOLD
            except Exception as e:
                self.failure('Pico ADC read: ' + str(e))

            # ── ADS1115 ───────────────────────────────────────────────────────
            # AIN0 = diff pressure probe   AIN1 = skip
            # AIN2 = PYRO_MAIN2            AIN3 = PYRO_DROGUE2
            # Using set_conv + read_rev pattern (pipeline: starts next
            # conversion and reads the previous result — requires sleep_ms(10)
            # between set_conv and read_rev to let conversion complete at
            # default 128 SPS; ADS_RATE=7 gives 860 SPS so 2 ms is enough).
            if self.ads_working and self.ads is not None:
                try:
                    self.ads.set_conv(self.ADS_RATE, 0)
                    sleep_ms(2)
                    self.volt_dp = self.ads.raw_to_v(self.ads.read_rev())

                    self.ads.set_conv(self.ADS_RATE, 2)
                    sleep_ms(2)
                    self.volt_main2    = self.ads.raw_to_v(self.ads.read_rev())

                    self.ads.set_conv(self.ADS_RATE, 3)
                    sleep_ms(2)
                    self.volt_drogue2  = self.ads.raw_to_v(self.ads.read_rev())

                    self.cont_main2   = self.volt_main2   > self.ADS_CONT_THRESHOLD
                    self.cont_drogue2 = self.volt_drogue2 > self.ADS_CONT_THRESHOLD

                    # ── Differential pressure ────────────────────────────────
                    # Datasheet: V_OUT = V_S * (0.009 * P_kPa + 0.04)
                    # Inverted:  P_kPa = (V_OUT / V_S - 0.04) / 0.009
                    # dp in Pa  = P_kPa * 1000
                    p_kpa    = (self.volt_dp / self.VS_SUPPLY - 0.04) / 0.009
                    self.dp  = max(0.0, p_kpa * 1000.0)   # Pa, clamp negatives

                except Exception as e:
                    self.ads_working = False
                    self.failure('ADS read: ' + str(e))

            # ── Buzzer heartbeat ──────────────────────────────────────────────
            now = ticks_ms()
            if self.state == self.STATE_UNARMED:
                both = self.cont_ch1 and self.cont_ch2
                one  = self.cont_ch1 or  self.cont_ch2
                if both:
                    if now - last_beep > 1000 and not self._beep_queue:
                        self._beep_queue.append((3, 50, 3000))
                        last_beep = now
                elif one:
                    if now - last_beep > 2000 and not self._beep_queue:
                        self._beep_queue.append((2, 60, 2000))
                        last_beep = now
                else:
                    if now - last_beep > 3000 and not self._beep_queue:
                        self._beep_queue.append((1, 75, 1000))
                        last_beep = now
            elif self.state == self.STATE_FLIGHT_READY:
                if abs(self.ax - self.G) / self.G < 0.15 and \
                        now - last_beep > 2000 and not self._beep_queue:
                    self._beep_queue.append((1, 50, 3000))
                    last_beep = now

            await asyncio.sleep(0)

    # ── state machine ─────────────────────────────────────────────────────────
    async def state_machine(self):
        while True:
            try:
                self.alt_buf = np.roll(self.alt_buf, -1)
                self.vel_buf = np.roll(self.vel_buf, -1)
                self.acc_buf = np.roll(self.acc_buf, -1)

                self.alt_buf[-1] = self.alt

                if self.t_log - self.last_t_log > 0:
                    self.vel_buf[-1] = 1000.0 * (
                        self.alt_buf[-1] - self.alt_buf[-2]
                    ) / (self.t_log - self.last_t_log)

                if self.bno_working and self.kx_working:
                    self.acc_buf[-1] = (self.ax + self.kx_ax) / 2
                elif self.bno_working:
                    self.acc_buf[-1] = self.ax
                elif self.kx_working:
                    self.acc_buf[-1] = self.kx_ax

                if self.t_log > self.runtime or self.logging_done:
                    self.state = self.STATE_LANDED
                    self.async_loop.stop()

                # ── UNARMED ───────────────────────────────────────────────────
                if self.state == self.STATE_UNARMED:
                    both_live = self.cont_ch1 and self.cont_ch2
                    now = ticks_ms()
                    if both_live:
                        if self.both_cont_since == 0:
                            self.both_cont_since = now
                        elif now - self.both_cont_since >= self.PYRO_CONT_DURATION:
                            self.state = self.STATE_FLIGHT_READY
                            self.both_cont_since = 0
                            self._flicker_event = True
                            self._beep_queue.append((1, 80, 2000))
                            self._beep_queue.append((1, 80, 3000))
                            self._beep_queue.append((1, 80, 4000))
                            print('[FCU] → FLIGHT_READY')
                            self.failure('FLIGHT_READY at ' + str(self.t_log))
                    else:
                        self.both_cont_since = 0

                # ── FLIGHT_READY ──────────────────────────────────────────────
                elif self.state == self.STATE_FLIGHT_READY:
                    if (np.all(self.alt_buf > self.min_liftoff_alt) and
                            np.all(self.acc_buf > self.liftoff_accel * self.G)):
                        self.state = self.STATE_BOOST
                        self.t_events[0] = self.t_log
                        self._flicker_event = True
                        self._beep_queue.append((3, 50, 2500))
                        self.failure('BOOST at ' + str(self.t_events[0]))

                # ── BOOST ─────────────────────────────────────────────────────
                elif self.state == self.STATE_BOOST:
                    if ((np.all(self.acc_buf < 0) and
                             float(self.acc_buf[-1]) > float(self.acc_buf[-2])) or
                            self.t_log - self.t_events[0] > self.force_burnout_time):
                        self.state = self.STATE_COAST
                        self.t_events[1] = self.t_log
                        self._flicker_event = True
                        self.failure('COAST at ' + str(self.t_events[1]))

                # ── COAST ─────────────────────────────────────────────────────
                elif self.state == self.STATE_COAST:
                    if ((self.t_log - self.t_events[0] > self.lockout_drogue_time) and
                            (np.all(self.vel_buf <= 0) or
                             self.t_log - self.t_events[0] > self.force_drogue_time)):
                        self.state = self.STATE_DROGUE
                        self.t_events[2] = self.t_log
                        self._flicker_event = True
                        self.failure('DROGUE at ' + str(self.t_events[2]))
                        for _ in range(3):
                            self.drogue_pin.value(1)
                            self.drogue2_pin.value(1)
                            sleep_ms(300)
                            self.drogue_pin.value(0)
                            self.drogue2_pin.value(0)
                            sleep_ms(100)

                # ── DROGUE ────────────────────────────────────────────────────
                elif self.state == self.STATE_DROGUE:
                    main_cond = (np.all(self.alt_buf < self.main_alt) and
                                 self.t_log - self.t_events[2] > self.drogue_to_main_lockout)
                    ballistic = (np.all(self.vel_buf < self.max_re_entry_speed) and
                                 self.t_log - self.t_events[2] > self.ballistic_lockout_time)
                    if main_cond or ballistic:
                        if main_cond:
                            self.failure('MAIN_ALT_BRANCH at ' + str(self.t_log))
                        if ballistic:
                            self.failure('MAIN_BALLISTIC_BRANCH: ' +
                                         str(float(self.vel_buf[-1])))
                        self.state = self.STATE_MAIN
                        self.t_events[3] = self.t_log
                        self._flicker_event = True
                        for _ in range(3):
                            self.main_pin.value(1)
                            self.main2_pin.value(1)
                            sleep_ms(300)
                            self.main_pin.value(0)
                            self.main2_pin.value(0)
                            sleep_ms(100)

                # ── MAIN ──────────────────────────────────────────────────────
                elif self.state == self.STATE_MAIN:
                    if (np.all(self.alt_buf < self.touchdown_alt) and
                            abs(float(np.mean(self.vel_buf))) < self.touchdown_vel_limit and
                            self.t_log - self.t_events[3] > self.main_to_touchdown_lockout):
                        self.state = self.STATE_LANDED
                        self.t_events[4] = self.t_log
                        self.logging_done = True
                        self._flicker_event = True
                        self.failure('LANDED at ' + str(self.t_events[4]))

                # Always de-assert pyros
                self.drogue_pin.value(0);  self.drogue2_pin.value(0)
                self.main_pin.value(0);    self.main2_pin.value(0)
                self.last_t_log = self.t_log

            except Exception as e:
                self.failure('State machine: ' + str(e))

            await asyncio.sleep(0.05)

    # ── data logging ─────────────────────────────────────────────────────────
    async def log_data(self):
        while True:

            # ── B2B frame (UART → TDU) ────────────────────────────────────────
            # Lean: no ADS voltages — TDU doesn't need them.
            # Includes GP26/GP27 primary pyro voltages for TDU awareness.
            try:
                ustruct.pack_into(
                    self.B2B_FMT, self.data_b2b, 0,
                    b'F',
                    int(self.t_log),
                    float(self.state),
                    float(self.temp),
                    self.ax,   self.ay,   self.az,
                    self.gx,   self.gy,   self.gz,
                    self.kx_ax, self.kx_ay, self.kx_az,
                    float(self.alt),
                    float(self.vel_buf[-1]),
                    float(self.volt_drogue_primary),   # GP26
                    float(self.volt_main_primary),     # GP27
                    float(self.volt_main2),            # AIN2 PYRO_MAIN2
                    float(self.volt_drogue2),          # AIN3 PYRO_DROGUE2
                    float(self.dp),                    # differential pressure (Pa)
                )
            except Exception as e:
                self.failure('Pack B2B: ' + str(e))

            # ── LOG frame (flash) ─────────────────────────────────────────────
            # Fat: all sensor data + all pyro voltages. No GPS placeholders.
            try:
                ustruct.pack_into(
                    self.LOG_FMT, self.data_log, 0,
                    b'L',
                    int(self.t_log),
                    float(self.state),
                    float(self.temp),
                    self.ax,   self.ay,   self.az,
                    self.gx,   self.gy,   self.gz,
                    self.kx_ax, self.kx_ay, self.kx_az,
                    float(self.alt),
                    float(self.vel_buf[-1]),
                    float(self.volt_drogue_primary),   # GP26
                    float(self.volt_main_primary),     # GP27
                    float(self.volt_dp),         # AIN0 raw probe voltage
                    float(self.volt_main2),            # AIN2 PYRO_MAIN2
                    float(self.volt_drogue2),          # AIN3 PYRO_DROGUE2
                    float(self.dp),                    # differential pressure (Pa)
#                     float(self.airspeed),              # indicated airspeed (m/s)
                )
            except Exception as e:
                self.failure('Pack LOG: ' + str(e))

            # Write LOG frame to flash only during active flight
            if self.state not in (self.STATE_UNARMED,
                                  self.STATE_FLIGHT_READY,
                                  self.STATE_LANDED):
                try:
                    if self.free_bytes - self.size > 1000:
                        self.data_file.write(self.data_log)
                        self.size += self.LOG_LEN
                except Exception as e:
                    self.failure('Write data: ' + str(e))

            # Rolling pre-launch buffer (LOG frames, 20-frame window)
            if self.state in (self.STATE_UNARMED, self.STATE_FLIGHT_READY):
                try:
                    if len(self.data_buffer) < 20 * self.LOG_LEN:
                        self.data_buffer += self.data_log
                    else:
                        self.data_buffer = self.data_buffer[self.LOG_LEN:] + self.data_log
                except Exception as e:
                    self.failure('Buffer: ' + str(e))

            await asyncio.sleep(0)

    # ── UART telemetry to TDU ─────────────────────────────────────────────────
    async def comms_to_tdu(self):
        while True:
            try:
                self.b2b.write(self.data_b2b)
            except Exception as e:
                self.failure('UART TX: ' + str(e))
            await asyncio.sleep_ms(200)

    # ── post-touchdown ────────────────────────────────────────────────────────
    async def after_party(self):
        self.buzzer.duty_u16(30000)
        while True:
            self.b2b.write(self.data_b2b)
            for i in range(1, 6):
                self.buzzer.freq(i * 500)
                sleep_ms(1000)

    # =========================================================================
    # Main entry point
    # =========================================================================
    async def run(self):
        print('[FCU] Starting async loop')
        self.data_file = open('/data_fcu_' + str(self.index) + '.bin', 'wb')

        self.async_loop = asyncio.get_event_loop()
        self.async_loop.create_task(self.blink(500))
        self.async_loop.create_task(self.beep_task())
        self.async_loop.create_task(self.led_flicker())
        self.async_loop.create_task(self.get_data())
        self.async_loop.create_task(self.state_machine())
        self.async_loop.create_task(self.comms_to_tdu())
        self.async_loop.create_task(self.log_data())
        self.async_loop.run_forever()

        self.data_file.write(self.data_buffer)
        self.data_file.close()
        print('[FCU] Flight data saved. Entering after-party loop.')

        self.async_loop = asyncio.new_event_loop()
        self.async_loop.create_task(self.after_party())
        self.async_loop.create_task(self.blink(1000))
        self.async_loop.run_forever()


# =============================================================================
# Entry point
# =============================================================================
if __name__ == '__main__':
    fcu = FCU()
    sleep_ms(1000)
    fcu.init_hardware()
    try:
        asyncio.run(fcu.run())
    except KeyboardInterrupt:
        print('[FCU] Interrupted')
    finally:
        print('[FCU] Done') 
