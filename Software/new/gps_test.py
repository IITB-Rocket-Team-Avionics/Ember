from machine import UART, Pin
import time

reset = Pin(17, Pin.OUT)
reset.value(0)
time.sleep_ms(100)
reset.value(1)   # release reset
time.sleep_ms(500)  # give it time to boot

# L89HA default baud is 9600
# Adjust TX/RX pins to wherever you've wired it
gps_uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1)

def parse_gprmc(sentence):
    """Parse GPRMC sentence — gives fix status, lat, lon, speed, date"""
    parts = sentence.split(',')
    if len(parts) < 10:
        return None

    status = parts[2]           # 'A' = active/valid, 'V' = void
    if status != 'A':
        return None

    # Raw NMEA lat/lon format: DDMM.MMMM
    raw_lat = parts[3]
    lat_dir = parts[4]
    raw_lon = parts[5]
    lon_dir = parts[6]

    def nmea_to_decimal(raw, direction):
        if not raw:
            return None
        dot = raw.index('.')
        degrees = int(raw[:dot - 2])
        minutes = float(raw[dot - 2:])
        decimal = degrees + minutes / 60.0
        if direction in ('S', 'W'):
            decimal = -decimal
        return decimal

    lat = nmea_to_decimal(raw_lat, lat_dir)
    lon = nmea_to_decimal(raw_lon, lon_dir)
    speed_knots = parts[7]
    utc_time = parts[1]  # HHMMSS.ss
    utc_date = parts[9]  # DDMMYY

    return {
        'lat': lat,
        'lon': lon,
        'speed_kn': speed_knots,
        'time': utc_time,
        'date': utc_date
    }

def parse_gpgga(sentence):
    """Parse GPGGA — gives fix quality, altitude, num satellites"""
    parts = sentence.split(',')
    if len(parts) < 15:
        return None

    fix_quality = int(parts[6]) if parts[6] else 0
    if fix_quality == 0:
        return None

    num_sats = parts[7]
    altitude = parts[9]
    alt_unit = parts[10]

    return {
        'fix_quality': fix_quality,
        'num_sats': num_sats,
        'altitude_m': altitude,
        'alt_unit': alt_unit
    }

def validate_checksum(sentence):
    """NMEA checksum: XOR of all chars between $ and *"""
    if '*' not in sentence:
        return False
    data, checksum = sentence[1:].split('*', 1)
    calc = 0
    for c in data:
        calc ^= ord(c)
    return hex(calc)[2:].upper().zfill(2) == checksum.strip().upper()

buf = b''

print("Waiting for GPS data...")

while True:
    if gps_uart.any():
        buf += gps_uart.read(gps_uart.any())

        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            try:
                sentence = line.decode('ascii').strip()
            except UnicodeError:
                continue
            
            # Inside your while loop, after decoding the sentence:
            print(f"RAW: {sentence}")

            if not sentence.startswith('$'):
                continue

#             if not validate_checksum(sentence):
#                 print(f"Bad checksum: {sentence}")
#                 continue

            # Strip the checksum tail before parsing
            clean = sentence.split('*')[0]

            if clean.startswith('$GPRMC') or clean.startswith('$GNRMC'):
                result = parse_gprmc(clean)
                if result:
                    print(f"[RMC] {result['date']} {result['time']}Z | "
                          f"Lat: {result['lat']:.6f}  Lon: {result['lon']:.6f} | "
                          f"Speed: {result['speed_kn']} kn")
                else:
                    print("[RMC] No fix yet")

            elif clean.startswith('$GPGGA') or clean.startswith('$GNGGA'):
                result = parse_gpgga(clean)
                if result:
                    print(f"[GGA] Sats: {result['num_sats']}  "
                          f"Alt: {result['altitude_m']}{result['alt_unit']}  "
                          f"Fix: {result['fix_quality']}")

    # replace time.sleep_ms(50) with just
    pass