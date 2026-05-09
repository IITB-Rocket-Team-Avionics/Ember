from machine import UART, Pin
import time

uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1)
print("Trying 4800...")
time.sleep(2)  # give it a moment to buffer up

data = uart.read(256)
print(data)