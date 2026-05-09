# =============================================================================
# FCU_main.py  –  Flight Computer Unit (FCU)
# RPi Pico 2
#
# Responsibilities:
#   - Read BNO055 (IMU), BMP280 (barometer), KX134 (high-G accel)
#   - Run state machine, navigation Kalman filter, pyro logic
#   - Log flight data to onboard flash
#   - Send packed telemetry frames to TDU over UART (B2B connector)
#   - Receive ACK / commands from TDU over the same UART
#
# Pin assignments (from FCU schematic Ember_v0.2):
#   I2C-0  : SDA=GP20  SCL=GP21  (BNO055 0x28, BMP280 0x76)
#   SPI-0  : SCK=GP22  MOSI=GP23 MISO=GP24 (not used for KX134 on this board –
#            KX134 is on I2C-1, INT=GP25 per schematic)
#   KX134  : I2C-1 SDA=GP26 SCL=GP27  INT2=GP25
#   Buzzer : GP18  (via MOSFET, passive PWM)
#   LED    : GP25  (onboard LED – shared with KX INT2; drive LOW to light)
#   Pyros  : DROGUE=GP6  DROGUE2=GP7  MAIN=GP8  MAIN2=GP9
#   ARM    : GP28 (input, ARM switch)
#   B2B    : UART-0  TX=GP0  RX=GP1  (to TDU)
#   ADC_REF: GP35 (ADC voltage reference – read-only)
# =============================================================================

from machine import I2C, Pin, UART, PWM, freq
from math   import sqrt, pi, atan2, cos, sin
from time   import ticks_ms, sleep_ms
from ulab   import numpy as np
from bno055_base import BNO055_BASE   # bno055_base.py must be on FCU flash
import ustruct
import uasyncio as asyncio
import uos
import gc
import micropython

# ── optional high-G accel ────────────────────────────────────────────────────
try:
    from kx132 import KX132, ACC_RANGE_16   # kx132.py must be on FCU flash
    KX_AVAILABLE = True
except ImportError:
    KX_AVAILABLE = False

# ── optional barometer ───────────────────────────────────────────────────────
try:
    from bmp280 import BMP280           # bmp280.py must be on FCU flash
    BMP_AVAILABLE = True
except ImportError:
    BMP_AVAILABLE = False

micropython.alloc_emergency_exception_buf(100)
gc.collect()
freq(150_000_000)


# =============================================================================
# Clash Royale / Supercell fanfare  (plays on FCU after all sensors are ready)
# Ported from EFS_alpha board_init()
# =============================================================================
def play_clash_royale(buzzer: PWM):
    buzzer.duty_u16(0)

    C5       = 523
    D5       = 587
    F5       = 698
    Fsharp5  = 740
    G5       = 784
    Gsharp5  = 831
    A5       = 880
    Asharp5  = 932
    Csharp5  = 554
    C6       = 1047
    F6       = 1397

    melody  = [Csharp5, Fsharp5, Gsharp5, C6, F6]
    tempo   = [14,       14,       14,      14,  6]

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
# BNO055 subclass with NDOF orientation mode
# =============================================================================
class BNO055(BNO055_BASE):
    pass


# =============================================================================
# FCU  –  main flight computer class
# =============================================================================
class FCU:

    # ── physical constants ────────────────────────────────────────────────────
    EARTH_R = 6_371_000      # m
    G       = 9.80665        # m s⁻²

    # ── state indices ─────────────────────────────────────────────────────────
    STATE_PAD      = 0
    STATE_BOOST    = 1
    STATE_COAST    = 2
    STATE_DROGUE   = 3
    STATE_MAIN     = 4
    STATE_LANDED   = 5

    # ── telemetry frame format (sent to TDU) ─────────────────────────────────
    # 'F' | t_ms(i32) | state(i8) | temp(f) | ax(f) | ay(f) | az(f)
    # | gx(f) | gy(f) | gz(f) | kx_ax(f) | kx_ay(f) | kx_az(f)
    # | alt(f) | vel(f) | lat(f) | lon(f) | heading(f)
    # Total: 1 + 4 + 1 + 17*4 = 74 bytes
    TELEM_FMT = '!ci17f'
    TELEM_LEN = ustruct.calcsize(TELEM_FMT)

    def __init__(self):
        self.state      = self.STATE_PAD
        self.t_log      = ticks_ms()
        self.last_t_log = self.t_log
        self.async_loop = None

        # Sensor objects
        self.bno = None
        self.bmp = None
        self.kx  = None

        # UART to TDU
        self.b2b = None

        # Pyro pins
        self.drogue_pin  = None
        self.drogue2_pin = None
        self.main_pin    = None
        self.main2_pin   = None
        self.arm_pin     = None

        # Buzzer / LED
        self.buzzer = None
        self.led    = None

        # Sensor readings
        self.bno_accel   = [0.0, 0.0, 0.0]
        self.bno_scale   = 1.0
        self.gyro        = [0.0, 0.0, 0.0]
        self.kx_accel    = [0.0, 0.0, 0.0]
        self.kx_scale    = 1.0
        self.temp        = 0.0
        self.alt         = 0.0

        # GPS data forwarded from TDU via UART
        self.lat         = 0.0
        self.lon         = 0.0
        self.gps_heading = 0.0

        # Calibration
        self.calib_altitude = 0.0
        self.calib_temp     = 0.0
        self.calib_count    = 0

        # State machine buffers
        self.buf_len  = 10
        self.alt_buf  = np.zeros((self.buf_len,))
        self.vel_buf  = np.zeros((self.buf_len,))
        self.acc_buf  = np.zeros((self.buf_len,))
        self.t_events = [0.0] * 6        # liftoff, burnout, drogue, main, land

        # Data logging
        self.data_fast   = bytearray(self.TELEM_LEN)
        self.data_buffer = bytearray(0)
        self.data_file   = None
        self.index       = 0
        self.size        = 0
        storage          = uos.statvfs('/')
        self.free_bytes  = storage[0] * storage[3]
        self.logging_done = False

        # Nav filter state
        self.X     = np.full((5, 1), 0.0)
        self.Z     = np.full((5, 1), 0.0)
        self.track = 0.0
        self.dt    = 1.0 / 40.0

        # Settings
        self.liftoff_accel          = 2.0   # × g
        self.min_liftoff_alt        = 10    # m
        self.force_burnout_time     = 3_000
        self.lockout_drogue_time    = 10_000
        self.force_drogue_time      = 17_000
        self.main_alt               = 400   # m
        self.drogue_to_main_lockout = 10_000
        self.ballistic_lockout_time = 3_000
        self.max_re_entry_speed     = -40.0
        self.touchdown_alt          = 50    # m
        self.touchdown_vel_limit    = 0.1   # m/s
        self.main_to_touchdown_lockout = 10_000
        self.runtime                = 36_000_000  # ms

        # Sensor health flags
        self.bno_working = True
        self.bmp_working = True
        self.kx_working  = True

        # Handshake flag – set True once TDU acknowledges FCU init
        self.tdu_ready   = False

    # ── altitude from BMP pressure ────────────────────────────────────────────
    def altitude(self):
        return 4947.19 * (8.9611 - pow(self.bmp.pressure, 0.190255))

    # ── air density ───────────────────────────────────────────────────────────
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

    # ── beeper ────────────────────────────────────────────────────────────────
    def beep(self, n=1, t=75, f=2000):
        self.buzzer.freq(f)
        for _ in range(n):
            self.buzzer.duty_u16(30000)
            sleep_ms(t)
            self.buzzer.duty_u16(0)
            if n > 1:
                sleep_ms(t)

    # =========================================================================
    # Hardware initialisation
    # =========================================================================
    def init_hardware(self):
        print('[FCU] Hardware init start')

        # ── GPIO ─────────────────────────────────────────────────────────────
        self.led         = Pin(25, Pin.OUT)
        self.buzzer      = PWM(Pin(18))
        self.buzzer.duty_u16(0)

        self.drogue_pin  = Pin(6,  Pin.OUT); self.drogue_pin.value(0)
        self.drogue2_pin = Pin(7,  Pin.OUT); self.drogue2_pin.value(0)
        self.main_pin    = Pin(8,  Pin.OUT); self.main_pin.value(0)
        self.main2_pin   = Pin(9,  Pin.OUT); self.main2_pin.value(0)
        self.arm_pin     = Pin(28, Pin.IN,  Pin.PULL_DOWN)

        # ── UART to TDU (B2B connector) ───────────────────────────────────────
        self.b2b = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
        print('[FCU] B2B UART ready')

        # ── I2C-0: BNO055 + BMP280 ────────────────────────────────────────────
        i2c0 = I2C(0, scl=Pin(21), sda=Pin(20), freq=400_000)

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
            # Apply known accel offsets (from field calibration)
            bno_offsets = bytearray(
                b'\xef\xff\t\x00\xef\xff\xae\x03\xa2\xff\x98\x04'
                b'\xff\xff\x02\x00\xff\xff\xe8\x03\x02\x03'
            )
            self.bno.set_offsets(bno_offsets)
            # Switch to AMG mode (16 G / 2000 dps) – mode 7
            # bno055_base uses mode() helper; write mode register directly
            self.bno.mode(7)

            # Scale factor: normalise so 1 g = 9.80665 m/s²
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

        # ── KX134 on I2C-1 ───────────────────────────────────────────────────
        if KX_AVAILABLE:
            try:
                i2c1 = I2C(1, scl=Pin(27), sda=Pin(26), freq=400_000)
                self.kx = KX132(i2c1, address=0x1F)
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

        # ── Initial barometer calibration ─────────────────────────────────────
        self._calib_bmp()

        # ── Index file for data logging ───────────────────────────────────────
        try:
            with open('/index.txt', 'r') as f:
                self.index = int(f.read())
        except Exception:
            self.index = 0
        with open('/index.txt', 'w') as f:
            f.write(str(self.index + 1))

        print('[FCU] Hardware init complete')

        # ── Clash Royale fanfare ──────────────────────────────────────────────
        play_clash_royale(self.buzzer)

        # ── Signal TDU that FCU is up ("FCU_INIT_OK\n") ───────────────────────
        self.b2b.write(b'FCU_INIT_OK\n')
        print('[FCU] Sent FCU_INIT_OK to TDU')

    # ── barometer calibration ─────────────────────────────────────────────────
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
            self.calib_count   += 1
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

    # ── sensor read loop ──────────────────────────────────────────────────────
    async def get_data(self):
        last_beep = 0
        while True:
            self.t_log = ticks_ms() - (self.t_events[0] if self.t_events[0] else 0)

            # BNO055
            try:
                ba = np.array(self.bno.accel()) * self.bno_scale
                self.bno_accel = ba
                self.gyro = self.bno.gyro()
                self.bno_working = True
            except Exception as e:
                self.bno_working = False
                self.failure('BNO read: ' + str(e))

            # KX134
            if self.kx is not None:
                try:
                    ka = np.array(self.kx.acceleration) * self.kx_scale
                    self.kx_accel = ka
                    self.kx_working = True
                except Exception as e:
                    self.kx_working = False
                    self.failure('KX read: ' + str(e))

            # BMP280
            if self.bmp is not None:
                try:
                    self.temp = self.bmp.temperature
                    self.alt  = self.altitude() - self.calib_altitude
                    self.bmp_working = True
                except Exception as e:
                    self.bmp_working = False
                    self.failure('BMP read: ' + str(e))

            # Pad heartbeat beep every 2 s
            if self.state == self.STATE_PAD:
                bno_mag = sqrt(float(self.bno_accel[0])**2 +
                               float(self.bno_accel[1])**2 +
                               float(self.bno_accel[2])**2)
                if (abs(bno_mag - self.G) / self.G < 0.15
                        and ticks_ms() - last_beep > 2000):
                    self.beep(1, 50, 3000)
                    last_beep = ticks_ms()

            await asyncio.sleep(0)

    # ── state machine ─────────────────────────────────────────────────────────
    async def state_machine(self):
        while True:
            try:
                t_now = ticks_ms()

                # Update rolling buffers
                self.alt_buf = np.roll(self.alt_buf, -1)
                self.vel_buf = np.roll(self.vel_buf, -1)
                self.acc_buf = np.roll(self.acc_buf, -1)

                self.alt_buf[-1] = self.alt

                dt_ms = self.t_log - self.last_t_log
                if dt_ms > 0:
                    self.vel_buf[-1] = 1000.0 * (float(self.alt_buf[-1]) - float(self.alt_buf[-2])) / dt_ms

                # Acceleration – prefer averaged BNO+KX when both available
                if self.bno_working and self.kx_working:
                    self.acc_buf[-1] = (float(self.bno_accel[1]) + float(self.kx_accel[0])) / 2
                elif self.bno_working:
                    self.acc_buf[-1] = float(self.bno_accel[1])
                elif self.kx_working:
                    self.acc_buf[-1] = float(self.kx_accel[0])

                # Runtime exceeded → force landed
                if self.t_log > self.runtime or self.logging_done:
                    self.state = self.STATE_LANDED
                    self.async_loop.stop()

                acc_g = float(self.acc_buf[-1]) / self.G

                # ── state transitions ─────────────────────────────────────
                if self.state == self.STATE_PAD:
                    if (np.all(self.alt_buf > self.min_liftoff_alt)
                            and np.all(self.acc_buf > self.liftoff_accel * self.G)):
                        self.state = self.STATE_BOOST
                        self.t_events[0] = self.t_log
                        self.beep(3, 50, 2500)

                elif self.state == self.STATE_BOOST:
                    if ((np.all(self.acc_buf < 0) and float(self.acc_buf[-1]) > float(self.acc_buf[-2]))
                            or self.t_log - self.t_events[0] > self.force_burnout_time):
                        self.state = self.STATE_COAST
                        self.t_events[1] = self.t_log

                elif self.state == self.STATE_COAST:
                    if ((self.t_log - self.t_events[0] > self.lockout_drogue_time)
                            and (np.all(self.vel_buf < 0)
                                 or self.t_log - self.t_events[0] > self.force_drogue_time)):
                        self.state = self.STATE_DROGUE
                        self.t_events[2] = self.t_log
                        # Fire drogue
                        if self.arm_pin.value():
                            for _ in range(3):
                                self.drogue_pin.value(1)
                                self.drogue2_pin.value(1)
                                sleep_ms(300)
                                self.drogue_pin.value(0)
                                self.drogue2_pin.value(0)
                                sleep_ms(100)

                elif self.state == self.STATE_DROGUE:
                    main_cond = (np.all(self.alt_buf < self.main_alt)
                                 and self.t_log - self.t_events[2] > self.drogue_to_main_lockout)
                    ballistic = (np.all(self.vel_buf < self.max_re_entry_speed)
                                 and self.t_log - self.t_events[2] > self.ballistic_lockout_time)
                    if main_cond or ballistic:
                        self.state = self.STATE_MAIN
                        self.t_events[3] = self.t_log
                        if self.arm_pin.value():
                            for _ in range(3):
                                self.main_pin.value(1)
                                self.main2_pin.value(1)
                                sleep_ms(300)
                                self.main_pin.value(0)
                                self.main2_pin.value(0)
                                sleep_ms(100)

                elif self.state == self.STATE_MAIN:
                    if (np.all(self.alt_buf < self.touchdown_alt)
                            and abs(float(np.mean(self.vel_buf))) < self.touchdown_vel_limit
                            and self.t_log - self.t_events[3] > self.main_to_touchdown_lockout):
                        self.state = self.STATE_LANDED
                        self.t_events[4] = self.t_log
                        self.logging_done = True

                # Always de-assert pyros in main loop
                self.drogue_pin.value(0)
                self.drogue2_pin.value(0)
                self.main_pin.value(0)
                self.main2_pin.value(0)

                self.last_t_log = self.t_log

            except Exception as e:
                self.failure('State machine: ' + str(e))

            await asyncio.sleep(0)

    # ── data logging ─────────────────────────────────────────────────────────
    async def log_data(self):
        while True:
            try:
                # Pack telemetry frame (same format sent over UART to TDU)
                ustruct.pack_into(
                    self.TELEM_FMT, self.data_fast, 0,
                    b'F',
                    int(self.t_log),
                    int(self.state),
                    float(self.temp),
                    float(self.bno_accel[0]),
                    float(self.bno_accel[1]),
                    float(self.bno_accel[2]),
                    float(self.gyro[0]),
                    float(self.gyro[1]),
                    float(self.gyro[2]),
                    float(self.kx_accel[0]),
                    float(self.kx_accel[1]),
                    float(self.kx_accel[2]),
                    float(self.alt),
                    float(self.vel_buf[-1]),
                    float(self.lat),
                    float(self.lon),
                    float(self.track),
                )
            except Exception as e:
                self.failure('Pack data: ' + str(e))

            # Write during flight
            if self.state not in (self.STATE_PAD, self.STATE_LANDED):
                try:
                    if self.free_bytes - self.size > 1000:
                        self.data_file.write(self.data_fast)
                        self.size += self.TELEM_LEN
                except Exception as e:
                    self.failure('Write data: ' + str(e))

            # Pre-flight ring buffer (last ~2.5 s before liftoff)
            if self.state == self.STATE_PAD:
                try:
                    if len(self.data_buffer) < 20 * self.TELEM_LEN:
                        self.data_buffer += self.data_fast
                    else:
                        self.data_buffer = self.data_buffer[self.TELEM_LEN:] + self.data_fast
                except Exception as e:
                    self.failure('Buffer: ' + str(e))

            await asyncio.sleep(0)

    # ── UART telemetry to TDU ─────────────────────────────────────────────────
    async def comms_to_tdu(self):
        """
        Continuously sends packed telemetry frames to TDU.
        Also checks for incoming GPS data forwarded back by TDU.

        TDU → FCU frame:  b'G' + lat(f) + lon(f) + heading(f)  = 13 bytes
        FCU → TDU frame:  self.data_fast (TELEM_LEN bytes)
        """
        gps_buf = b''
        while True:
            # ── send to TDU ───────────────────────────────────────────────────
            try:
                self.b2b.write(self.data_fast)
            except Exception as e:
                self.failure('UART TX: ' + str(e))

            # ── receive GPS echoed back from TDU ──────────────────────────────
            try:
                rx = self.b2b.read(32)
                if rx:
                    gps_buf += rx
                    while len(gps_buf) >= 13:
                        if gps_buf[0:1] == b'G':
                            _, self.lat, self.lon, self.gps_heading = \
                                ustruct.unpack('!cfff', gps_buf[:13])
                            self.Z[0][0] = self.lat * pi / 180
                            self.Z[1][0] = self.lon * pi / 180
                            gps_buf = gps_buf[13:]
                        else:
                            # Resync
                            idx = gps_buf.find(b'G')
                            if idx < 0:
                                gps_buf = b''
                            else:
                                gps_buf = gps_buf[idx:]
            except Exception as e:
                self.failure('UART RX: ' + str(e))

            await asyncio.sleep_ms(100)

    # ── nav Kalman filter (altitude + position) ───────────────────────────────
    async def nav(self):
        counter = 0
        X_A = np.full((3, 1), 0.0)
        nx = ny = 1
        P   = np.full((5, 5), 0.0)
        Q   = np.full((5, 5), 0.0)
        R   = np.eye(5)
        F   = np.eye(5)
        vx  = 0.0
        gx = gy = gz = 0.0

        Q[0][0] = 0.1;  Q[1][1] = 0.1;  Q[2][2] = 0.1
        Q[3][3] = 0.01; Q[4][4] = 0.1

        while True:
            if counter > 0:
                try:
                    dt = (self.t_log - self.last_t_log) / 1000.0
                    if dt <= 0:
                        dt = self.dt

                    ax = float(self.bno_accel[1])
                    ay = float(self.bno_accel[0])
                    az = -float(self.bno_accel[2])
                    a  = sqrt(ax*ax + ay*ay + az*az)

                    gx = float(self.gyro[1]) * pi / 180
                    gy = float(self.gyro[0]) * pi / 180
                    gz = -float(self.gyro[2]) * pi / 180

                    e = abs(a - self.G) / self.G
                    if e < 0.05 and not self.t_events[0]:
                        X_A[0][0] = atan2(ay, az)
                        X_A[1][0] = atan2(-ax, sqrt(ay*ay + az*az))
                    else:
                        X_A[0][0] += nx * gx * dt
                        X_A[1][0] += ny * gy * dt

                    if self.t_events[0] or e > 0.05:
                        X_A[2][0] += gz * dt

                    if abs(X_A[0][0] + nx * gx * dt) >= pi / 2:
                        nx *= -1
                    if abs(X_A[1][0] + ny * gy * dt) >= pi / 2:
                        ny *= -1

                    # Vertical state propagation
                    F[2][3] = dt
                    self.Z[2][0] = self.alt
                    self.Z[3][0] = float(self.vel_buf[-1])
                    self.Z[4][0] = (float(self.vel_buf[-1]) - float(self.vel_buf[-2])) / dt if dt > 0 else 0

                    Xp = np.dot(F, self.X)
                    Pp = np.dot(np.dot(F, P), F.transpose()) + Q

                    try:
                        K = np.dot(Pp, np.linalg.inv(Pp + R))
                    except Exception:
                        K = np.zeros((5, 5))

                    self.X   = Xp + np.dot(K, self.Z - Xp)
                    P        = np.dot(np.eye(5) - K, Pp)
                    self.track = atan2(float(self.X[4][0]), float(self.X[3][0])) \
                        if float(self.X[3][0]) != 0 else 0.0

                except Exception as e:
                    self.failure('Nav: ' + str(e))

            counter += 1
            await asyncio.sleep(0)

    # ── post-touchdown loop ───────────────────────────────────────────────────
    async def after_party(self):
        self.buzzer.duty_u16(30000)
        while True:
            self.b2b.write(self.data_fast)
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
        self.async_loop.create_task(self.get_data())
        self.async_loop.create_task(self.state_machine())
        self.async_loop.create_task(self.nav())
        self.async_loop.create_task(self.comms_to_tdu())
        self.async_loop.create_task(self.log_data())
        self.async_loop.run_forever()

        # Post-run: write pre-flight buffer and close file
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
    fcu.init_hardware()          # init sensors, play Clash Royale, signal TDU
    try:
        asyncio.run(fcu.run())
    except KeyboardInterrupt:
        print('[FCU] Interrupted')
    finally:
        print('[FCU] Done')