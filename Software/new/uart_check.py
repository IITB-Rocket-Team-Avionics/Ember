from machine import UART, Pin
import time

reset = Pin(17, Pin.OUT)
reset.value(0)
time.sleep_ms(100)
reset.value(1)   # release reset
time.sleep_ms(500)  # give it time to boot

for baud in [9600, 4800, 115200, 38400]:
    uart = UART(1, baudrate=baud, tx=Pin(4), rx=Pin(5))
    print(f"Trying {baud}...")
    for _ in range(20):
        data = uart.read(32)
        if data and data != b'\xff' * len(data):
            print(f"GOT DATA at {baud}:", data)
            break
    uart.deinit()