from machine import UART, Pin
from sx1262 import SX1262
import time

# ─── LED ─────────────────────────────────────────────────────────────────────
led = Pin(25, Pin.OUT)

def blink(n=1, ms=50):
    for _ in range(n):
        led.on()
        time.sleep_ms(ms)
        led.off()
        time.sleep_ms(ms)

# ─── LoRa init ───────────────────────────────────────────────────────────────
sx = SX1262(spi_bus=1, clk=10, mosi=11, miso=8, cs=9, irq=14, rst=13, gpio=12)
sx.begin(freq=920, bw=500.0, sf=12, cr=8, syncWord=0x12,
         power=22
         , currentLimit=60.0, preambleLength=8,
         implicit=False, implicitLen=0xFF,
         crcOn=True, txIq=False, rxIq=False,
         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)

# ─── GPS init ────────────────────────────────────────────────────────────────
reset = Pin(17, Pin.OUT)
reset.value(0)
time.sleep_ms(100)
reset.value(1)
time.sleep_ms(500)

gps_uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

buf = b''

# ─── Helpers ─────────────────────────────────────────────────────────────────
def validate_checksum(sentence):
    if '*' not in sentence:
        return False
    try:
        data, checksum = sentence[1:].split('*', 1)
    except ValueError:
        return False
    calc = 0
    for c in data:
        calc ^= ord(c)
    return "{:02X}".format(calc) == checksum.strip().upper()


def nmea_to_decimal(raw, direction):
    if not raw:
        return None
    try:
        dot = raw.index('.')
        degrees = int(raw[:dot - 2])
        minutes = float(raw[dot - 2:])
    except (ValueError, IndexError):
        return None
    decimal = degrees + minutes / 60.0
    if direction in ('S', 'W'):
        decimal = -decimal
    return decimal


# ─── Parsers ─────────────────────────────────────────────────────────────────
def parse_rmc(parts):
    if len(parts) < 10 or parts[2] != 'A':
        return None
    lat = nmea_to_decimal(parts[3], parts[4])
    lon = nmea_to_decimal(parts[5], parts[6])
    if lat is None or lon is None:
        return None
    return {
        'lat':      lat,
        'lon':      lon,
        'speed_kn': parts[7],
        'heading':  parts[8],
        'time':     parts[1],
        'date':     parts[9],
    }


def parse_gga(parts):
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
        'hdop':        parts[8],
        'altitude_m':  parts[9],
        'alt_unit':    parts[10],
    }


def parse_antenna(parts):
    if len(parts) < 4:
        return None
    return {
        'present': parts[1] == '1',
        'shorted': parts[2] == '1',
        'active':  parts[3] == '1',
    }


# ─── Sentence handler ────────────────────────────────────────────────────────
def handle_sentence(sentence):
    if not validate_checksum(sentence):
        # 3 rapid blinks = bad checksum
        blink(3, 30)
        print("[BAD CHECKSUM]", sentence)
        return

    clean  = sentence.split('*')[0]
    parts  = clean.split(',')
    msg_id = parts[0]

    if msg_id in ('$GNRMC', '$GPRMC'):
        result = parse_rmc(parts)
        if result:
            msg = "[RMC] {} {}Z | Lat: {:.6f} Lon: {:.6f} | Speed: {} kn Heading: {}".format(
                result['date'], result['time'],
                result['lat'],  result['lon'],
                result['speed_kn'], result['heading']
            )
            print(msg)
            sx.send(msg.encode())
            # 1 short blink = TX ok
            blink(1, 50)
        else:
            print("[RMC] No fix")

    elif msg_id in ('$GNGGA', '$GPGGA'):
        result = parse_gga(parts)
        if result:
            msg = "[GGA] Sats: {} Alt: {}{} HDOP: {} Fix: {}".format(
                result['num_sats'],
                result['altitude_m'], result['alt_unit'],
                result['hdop'],
                result['fix_quality']
            )
            print(msg)
            sx.send(msg.encode())
            # 2 short blinks = TX ok
            blink(2, 50)

    elif msg_id == '$PQTMANTENNASTATUS':
        result = parse_antenna(parts)
        if result:
            if result['shorted']:
                # long blink = hardware fault
                blink(1, 500)
                print("[ANT] *** ANTENNA SHORT CIRCUIT ***")
            else:
                print("[ANT] Present: {}  Active: {}  Status: {}".format(
                    result['present'],
                    result['active'],
                    "OK" if result['active'] else "INACTIVE"
                ))


# ─── Startup sequence ────────────────────────────────────────────────────────
# 5 fast blinks = boot ok
blink(5, 30)
print("GPS ready, waiting for data...")

# ─── Main loop ───────────────────────────────────────────────────────────────
while True:
    data = gps_uart.read(64)
    if data:
        buf += data
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            try:
                sentence = line.decode('ascii').strip()
            except UnicodeError:
                continue
            if sentence.startswith('$'):
                handle_sentence(sentence)