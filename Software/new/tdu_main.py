# =============================================================================
# TDU_main.py  –  Telemetry & Data Unit (TDU)
# RPi Pico 2
#
# Responsibilities:
#   - Receive packed telemetry frames from FCU over UART (B2B connector)
#   - Parse GPS (NMEA) from onboard GPS module
#   - Echo GPS fix back to FCU over B2B UART
#   - Forward combined telemetry + GPS over LoRa SX1262 to ground station
#   - Optionally control camera trigger output (GP15 / GP20)
#   - Play Angry Birds jingle AFTER FCU sends FCU_INIT_OK
#   - Drive drogue camera-ignition pyro output (G_DROGUE via SISA96DN-T1)
#
# Pin assignments (from TDU schematic Ember_RF_v0):
#   B2B UART  : UART-0  TX=GP0  RX=GP1       (from/to FCU)
#   GPS I2C   : SDA=GP2  SCL=GP3             (for LC86G config only, not NMEA)
#   GPS UART  : UART-1  TX=GP4  RX=GP5       (NMEA stream)
#   GPS RESET : GP17
#   GPS WAKE  : GP16
#   LoRa SPI  : LORA_SCK=GP10  LORA_MOSI=GP11  LORA_MISO=GP8
#               LORA_CS=GP9   LORA_RST=GP13   LORA_DIO1=GP14  LORA_BUSY=GP12
#   Buzzer    : GP18  (via Q5 NPN transistor, active HIGH)
#   Camera    : GP15  (CAM trigger via Q2 SISA96DN-T1)
#   G_DROGUE  : SISA96DN-T1 gate on GP20   (camera/secondary drogue, "CAM")
#   LED       : GP25  (onboard)
# =============================================================================

from machine import UART, Pin, PWM, I2C
from time    import ticks_ms, sleep_ms
import ustruct
import uasyncio as asyncio
import gc
import micropython

from sx1262 import SX1262     # sx1262.py + sx126x.py + _sx126x.py must be on TDU

micropython.alloc_emergency_exception_buf(100)
gc.collect()


# =============================================================================
# Angry Birds jingle  (plays on TDU after FCU_INIT_OK is received)
# Taken directly from angrybird.py
# =============================================================================
NOTES = {
    'REST': 0,
    'C5':  523, 'CS5': 554, 'D5':  587, 'DS5': 622, 'E5':  659,
    'F5':  698, 'FS5': 740, 'G5':  784, 'GS5': 831, 'A5':  880,
    'AS5': 932, 'B5':  988, 'C6': 1047, 'CS6':1109, 'D6': 1175,
    'DS6':1245, 'E6': 1319, 'F6': 1397, 'FS6':1480, 'G6': 1568,
}

ANGRY_BIRDS_MELODY = [
    ('E5',  1), ('FS5', 1), ('G5', 2),  ('E5',  2),
    ('B5',  1), ('REST',1),
    ('E5',  1), ('FS5', 1), ('G5', 2),  ('B5',  2),
    ('B5',  1), ('REST',3),
    ('B5',  1), ('C6',  1), ('B5', 1),  ('A5',  1),
    ('G5',  2), ('G5',  1), ('FS5',1),  ('E5',  1),
]


def play_angry_birds(buzzer: PWM, tempo: int = 140):
    beat_ms = int(60000 / tempo / 4)
    for note, dur in ANGRY_BIRDS_MELODY:
        freq = NOTES.get(note, 0)
        ms   = beat_ms * dur
        if freq == 0:
            buzzer.duty_u16(0)
        else:
            buzzer.freq(freq)
            buzzer.duty_u16(32768)
        sleep_ms(ms)
        buzzer.duty_u16(0)
        sleep_ms(20)
    buzzer.duty_u16(0)


# =============================================================================
# GPS helpers  (from gps_test2.py)
# =============================================================================
def _validate_checksum(sentence: str) -> bool:
    if '*' not in sentence:
        return False
    try:
        data, chk = sentence[1:].split('*', 1)
    except ValueError:
        return False
    calc = 0
    for c in data:
        calc ^= ord(c)
    return '{:02X}'.format(calc) == chk.strip().upper()


def _nmea_to_decimal(raw: str, direction: str):
    if not raw:
        return None
    try:
        dot = raw.index('.')
        degrees = int(raw[:dot - 2])
        minutes = float(raw[dot - 2:])
    except (ValueError, IndexError):
        return None
    dec = degrees + minutes / 60.0
    if direction in ('S', 'W'):
        dec = -dec
    return dec


def _parse_rmc(parts):
    if len(parts) < 10 or parts[2] != 'A':
        return None
    lat = _nmea_to_decimal(parts[3], parts[4])
    lon = _nmea_to_decimal(parts[5], parts[6])
    if lat is None or lon is None:
        return None
    return {
        'lat':      lat,
        'lon':      lon,
        'speed_kn': parts[7],
        'heading':  float(parts[8]) if parts[8] else 0.0,
        'time':     parts[1],
        'date':     parts[9],
    }


def _parse_gga(parts):
    if len(parts) < 15:
        return None
    try:
        if not parts[6] or int(parts[6]) == 0:
            return None
    except ValueError:
        return None
    return {
        'fix_quality': int(parts[6]),
        'num_sats':    parts[7],
        'hdop':        float(parts[8]) if parts[8] else 99.0,
        'altitude_m':  float(parts[9]) if parts[9] else 0.0,
    }


# =============================================================================
# LoRa over-air telemetry frame
#
# Ground station receives:
#   b'T' | t_ms(i32) | state(i8) | temp(f) | ax(f) | ay(f) | az(f)
#   | gx(f) | gy(f) | gz(f) | alt(f) | vel(f) | lat(f) | lon(f) | heading(f)
#   = 1 + 4 + 1 + 12*4 = 54 bytes
# =============================================================================
LORA_FMT = '!ci12f'
LORA_LEN = ustruct.calcsize(LORA_FMT)

# FCU telemetry frame format (must match FCU_main.py TELEM_FMT)
FCU_FMT = '!ci17f'
FCU_LEN = ustruct.calcsize(FCU_FMT)

# GPS echo to FCU:  b'G' + lat(f) + lon(f) + heading(f) = 13 bytes
GPS_ECHO_FMT = '!cfff'


# =============================================================================
# TDU  –  main class
# =============================================================================
class TDU:

    def __init__(self):
        self.led     = None
        self.buzzer  = None
        self.sx      = None
        self.b2b     = None     # UART to FCU
        self.gps_uart= None

        # GPS fix data
        self.gps_lat     = 0.0
        self.gps_lon     = 0.0
        self.gps_heading = 0.0
        self.gps_alt     = 0.0
        self.gps_sats    = 0
        self.gps_hdop    = 99.0
        self.gps_valid   = False
        self.gps_buf     = b''

        # FCU telemetry
        self.fcu_t_ms  = 0
        self.fcu_state = 0
        self.fcu_temp  = 0.0
        self.fcu_ax    = self.fcu_ay = self.fcu_az = 0.0
        self.fcu_gx    = self.fcu_gy = self.fcu_gz = 0.0
        self.fcu_kx_ax = self.fcu_kx_ay = self.fcu_kx_az = 0.0
        self.fcu_alt   = 0.0
        self.fcu_vel   = 0.0
        self.fcu_lat   = 0.0
        self.fcu_lon   = 0.0
        self.fcu_heading = 0.0

        self.fcu_buf       = b''
        self.fcu_connected = False

        # Camera / secondary drogue
        self.cam_pin     = None
        self.g_drogue_pin= None

        self.lora_frame  = bytearray(LORA_LEN)
        self.gps_echo    = bytearray(ustruct.calcsize(GPS_ECHO_FMT))

        self.async_loop  = None

    # ── failure logger ────────────────────────────────────────────────────────
    def failure(self, e):
        try:
            with open('tdu_fail.txt', 'a') as f:
                f.write(str(ticks_ms()) + ',' + str(e) + '\n')
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

    # ── blink helper ─────────────────────────────────────────────────────────
    def blink(self, n=1, ms=50):
        for _ in range(n):
            self.led.on()
            sleep_ms(ms)
            self.led.off()
            sleep_ms(ms)

    # =========================================================================
    # Hardware initialisation
    # =========================================================================
    def init_hardware(self):
        print('[TDU] Hardware init start')

        # ── GPIO ──────────────────────────────────────────────────────────────
        self.led    = Pin(25, Pin.OUT)
        self.buzzer = PWM(Pin(18))
        self.buzzer.duty_u16(0)

        self.cam_pin      = Pin(15, Pin.OUT); self.cam_pin.value(0)
        self.g_drogue_pin = Pin(20, Pin.OUT); self.g_drogue_pin.value(0)

        # ── B2B UART to FCU ───────────────────────────────────────────────────
        self.b2b = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
        print('[TDU] B2B UART ready (waiting for FCU_INIT_OK)')

        # ── GPS ───────────────────────────────────────────────────────────────
        gps_reset = Pin(17, Pin.OUT)
        gps_reset.value(0)
        sleep_ms(100)
        gps_reset.value(1)
        sleep_ms(500)
        self.gps_uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
        print('[TDU] GPS UART ready')

        # ── LoRa SX1262 ───────────────────────────────────────────────────────
        try:
            self.sx = SX1262(spi_bus=1, clk=10, mosi=11, miso=8,
                             cs=9, irq=14, rst=13, gpio=12)
            self.sx.begin(
                freq=920, bw=500.0, sf=12, cr=8, syncWord=0x12,
                power=22, currentLimit=60.0, preambleLength=8,
                implicit=False, implicitLen=0xFF,
                crcOn=True, txIq=False, rxIq=False,
                tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True,
            )
            print('[TDU] LoRa SX1262 OK')
            self.blink(3, 50)
        except Exception as e:
            print('[TDU] LoRa FAIL:', e)
            self.failure('LoRa init: ' + str(e))

        # ── Wait for FCU_INIT_OK  ─────────────────────────────────────────────
        print('[TDU] Waiting for FCU_INIT_OK ...')
        self.blink(5, 30)
        deadline = ticks_ms() + 30_000     # wait up to 30 s
        rx_buf = b''
        while ticks_ms() < deadline:
            data = self.b2b.read(64)
            if data:
                rx_buf += data
                if b'FCU_INIT_OK' in rx_buf:
                    print('[TDU] FCU_INIT_OK received!')
                    break
            sleep_ms(20)
        else:
            print('[TDU] WARNING: FCU_INIT_OK not received – continuing anyway')

        # ── Angry Birds jingle (after FCU confirmed) ──────────────────────────
        print('[TDU] Playing Angry Birds jingle')
        play_angry_birds(self.buzzer)

        print('[TDU] Hardware init complete')

    # =========================================================================
    # Async tasks
    # =========================================================================

    async def async_blink(self, t):
        while True:
            self.led.value(0)
            await asyncio.sleep_ms(t)
            self.led.value(1)
            await asyncio.sleep_ms(t)

    # ── GPS reader ────────────────────────────────────────────────────────────
    async def gps_task(self):
        """
        Reads NMEA sentences from GPS UART, parses RMC and GGA.
        Echoes fix back to FCU and stores for LoRa transmission.
        """
        while True:
            data = self.gps_uart.read(128)
            if data:
                self.gps_buf += data
                while b'\n' in self.gps_buf:
                    line, self.gps_buf = self.gps_buf.split(b'\n', 1)
                    try:
                        sentence = line.decode('ascii').strip()
                    except UnicodeError:
                        continue
                    if sentence.startswith('$'):
                        self._handle_nmea(sentence)
            await asyncio.sleep(0)

    def _handle_nmea(self, sentence: str):
        if not _validate_checksum(sentence):
            return
        clean  = sentence.split('*')[0]
        parts  = clean.split(',')
        msg_id = parts[0]

        if msg_id in ('$GNRMC', '$GPRMC'):
            result = _parse_rmc(parts)
            if result:
                self.gps_lat     = result['lat']
                self.gps_lon     = result['lon']
                self.gps_heading = result['heading']
                self.gps_valid   = True
                print('[TDU][GPS] RMC  Lat:{:.6f} Lon:{:.6f} Hdg:{:.1f}'.format(
                    self.gps_lat, self.gps_lon, self.gps_heading))
                # Echo fix to FCU
                self._send_gps_to_fcu()

        elif msg_id in ('$GNGGA', '$GPGGA'):
            result = _parse_gga(parts)
            if result:
                self.gps_alt  = result['altitude_m']
                self.gps_sats = int(result['num_sats']) if result['num_sats'] else 0
                self.gps_hdop = result['hdop']
                print('[TDU][GPS] GGA  Alt:{:.1f}m Sats:{} HDOP:{:.1f}'.format(
                    self.gps_alt, self.gps_sats, self.gps_hdop))

    def _send_gps_to_fcu(self):
        try:
            ustruct.pack_into(GPS_ECHO_FMT, self.gps_echo, 0,
                              b'G',
                              float(self.gps_lat),
                              float(self.gps_lon),
                              float(self.gps_heading))
            self.b2b.write(self.gps_echo)
        except Exception as e:
            self.failure('GPS echo: ' + str(e))

    # ── FCU telemetry reader ──────────────────────────────────────────────────
    async def fcu_rx_task(self):
        """
        Reads packed telemetry frames from FCU over B2B UART.
        Stores decoded values for LoRa forwarding.
        """
        while True:
            data = self.b2b.read(FCU_LEN * 2)
            if data:
                self.fcu_buf += data
                # Consume complete frames starting with b'F'
                while len(self.fcu_buf) >= FCU_LEN:
                    idx = self.fcu_buf.find(b'F')
                    if idx < 0:
                        self.fcu_buf = b''
                        break
                    if idx > 0:
                        self.fcu_buf = self.fcu_buf[idx:]
                    if len(self.fcu_buf) < FCU_LEN:
                        break
                    frame = self.fcu_buf[:FCU_LEN]
                    self.fcu_buf = self.fcu_buf[FCU_LEN:]
                    try:
                        (_, t, state, temp,
                         ax, ay, az, gx, gy, gz,
                         kx_ax, kx_ay, kx_az,
                         alt, vel, lat, lon, heading) = \
                            ustruct.unpack(FCU_FMT, frame)
                        self.fcu_t_ms    = t
                        self.fcu_state   = state
                        self.fcu_temp    = temp
                        self.fcu_ax      = ax;  self.fcu_ay = ay; self.fcu_az = az
                        self.fcu_gx      = gx;  self.fcu_gy = gy; self.fcu_gz = gz
                        self.fcu_kx_ax   = kx_ax
                        self.fcu_alt     = alt
                        self.fcu_vel     = vel
                        self.fcu_lat     = lat
                        self.fcu_lon     = lon
                        self.fcu_heading = heading
                        self.fcu_connected = True
                    except Exception as e:
                        self.failure('FCU frame unpack: ' + str(e))
            await asyncio.sleep(0)

    # ── LoRa transmit task ────────────────────────────────────────────────────
    async def lora_tx_task(self):
        """
        Sends combined FCU + GPS telemetry over LoRa at ~2 Hz.
        Uses GPS lat/lon if a fresh fix is available, otherwise FCU's
        dead-reckoned position.
        """
        while True:
            try:
                lat = self.gps_lat if self.gps_valid else self.fcu_lat
                lon = self.gps_lon if self.gps_valid else self.fcu_lon
                hdg = self.gps_heading if self.gps_valid else self.fcu_heading

                ustruct.pack_into(
                    LORA_FMT, self.lora_frame, 0,
                    b'T',
                    int(self.fcu_t_ms),
                    int(self.fcu_state),
                    float(self.fcu_temp),
                    float(self.fcu_ax),
                    float(self.fcu_ay),
                    float(self.fcu_az),
                    float(self.fcu_gx),
                    float(self.fcu_gy),
                    float(self.fcu_gz),
                    float(self.fcu_alt),
                    float(self.fcu_vel),
                    float(lat),
                    float(lon),
                    float(hdg),
                )
                self.sx.send(self.lora_frame)
                self.led.toggle()

                # Human-readable GPS debug line every 5 transmits (≈10 s)
                if self.gps_valid:
                    print('[TDU][LoRa] TX  state={} alt={:.1f}m lat={:.5f} lon={:.5f}'.format(
                        self.fcu_state, self.fcu_alt, lat, lon))

            except Exception as e:
                self.failure('LoRa TX: ' + str(e))

            await asyncio.sleep_ms(500)

    # =========================================================================
    # Main entry point
    # =========================================================================
    async def run(self):
        print('[TDU] Starting async loop')
        self.async_loop = asyncio.get_event_loop()
        self.async_loop.create_task(self.async_blink(500))
        self.async_loop.create_task(self.gps_task())
        self.async_loop.create_task(self.fcu_rx_task())
        self.async_loop.create_task(self.lora_tx_task())
        self.async_loop.run_forever()


# =============================================================================
# Entry point
# =============================================================================
if __name__ == '__main__':
    tdu = TDU()
    tdu.init_hardware()
    try:
        asyncio.run(tdu.run())
    except KeyboardInterrupt:
        print('[TDU] Interrupted')
    finally:
        print('[TDU] Done')