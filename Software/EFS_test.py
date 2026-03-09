
# ------------------------------------------- as_GPS HAS BEEN MODIFIED --------------------------------------------------
# --------------------------------- DOCUMENTING THIS IS GOING TO BE EXTREMELY PAINFUL -----------------------------------

from machine import I2C,Pin,UART,SPI,freq,PWM, ADC
from math import sqrt,pi,asin,atan,tan,cos,sin,atan2,acos
from time import ticks_ms,sleep,sleep_us,sleep_ms
from ulab import numpy as np
from bno055 import *
from bmp280 import *
# KX134/KX132 not used on this board
# import kx132
import as_GPS
import uasyncio as asyncio
import ustruct
import uos
import gc
import micropython
# ADCs removed on new board
# import ads1x15
import sys
import uio
from sx1262 import SX1262
# Encoder not present on this board
# import encoder
micropython.alloc_emergency_exception_buf(100)
gc.collect()

freq(150_000_000)

class async_test:
    
    # Altitude from static air pressure
    def altitude(self):
        
        return 4947.19 * (8.9611 - pow(self.bmp.pressure,0.190255))

    # Density
    def rho(self,y):
        return pow(8.9611 - (y + self.calib_altitude)/4947.19,5.2479)/(78410.439 + 287.06*self.calib_temp - 1.86589*(y + self.calib_altitude))
    
    # LED thingy
    async def blink(self, t):
        while True:
            self.led.value(0)
            await asyncio.sleep_ms(t)
            self.led.value(1)
            await asyncio.sleep_ms(t)
    
    # Mario in this bitch. Credits - ChatGPT.
    def board_init(self):
        
#         pwm = self.buzzer
#         
#         G6 = 1568
#         C7 = 2093
#         E7 = 2637
#         G7 = 3136
# 
#         melody = [
#             E7, E7, 0, E7,
#             0, C7, E7, 0,
#             G7, 0, 0, 0,
#             G6, 0, 0, 0,
# 
#         ]
# 
#         tempo = [
#             11, 11, 11, 11,
#             11, 11, 11, 11,
#             11, 11, 11, 11,
#             11, 11, 11, 11,
#         ]
# 
#         def play_tone(frequency, duration):
#             if frequency == 0:
#                 sleep(duration / 1000)
#             else:
#                 pwm.freq(frequency)
#                 pwm.duty_u16(32768)
#                 sleep(duration / 1000)
#                 pwm.duty_u16(0)
# 
#         for i in range(len(melody)):
#             play_tone(melody[i], tempo[i] * 10)
#             sleep(0.05)

        pwm = self.buzzer
        pwm.duty_u16(0)

        # ---------- NOTE FREQUENCIES ----------
        C5  = 523
        D5  = 587
        F5  = 698
        Fsharp5 = 740
        G5  = 784
        Gsharp5 = 831
        A5  = 880
        Asharp5 = 932
        C6  = 1047
        D6  = 1175
        F6  = 1397

        # ---------- SUPERCELL FANFARE ----------
        # Opening chord-hit illusion
        melody = [
            Csharp5 := 554, Fsharp5, Gsharp5, C6, F6
        ]
        tempo = [
            14, 14, 14, 14, 6
        ]

        # Melodic run
        melody1 = [
            Asharp5, A5,
            F5, G5,
            D5,
            C5
        ]
        tempo1 = [
            4, 10,
            12, 12,
            8,
            6
        ]

        # ---------- PLAYER ----------
        def play(melody, tempo):
            for i in range(len(melody)):
                pwm.freq(melody[i])
                pwm.duty_u16(32000)
                sleep_ms(1000 // tempo[i])
                pwm.duty_u16(0)
                sleep_ms(20)

        # ---------- PLAY SEQUENCE ----------
        play(melody, tempo)
        sleep_ms(1000)        # 1 second pause
        play(melody1, tempo1)

            
    # Warning for general failures like myself
    def failure(self,e):
        
        # Transmit error through telemetry if on pad
        if self.state == 0 and getattr(self, 'sx', None) is not None:
            try:
                self.sx.send(e)
            except Exception:
                pass
        
        # Write error to failure log if enough space is available
        if self.free_bytes - self.size > 1000:
            failure_log = open('fail_log.txt','a')
            failure_log.write('data_' + str(self.index) + ' : ' + str(self.t_log) + ',' + str(e) + '\n')
            failure_log.close()

    # Beeper
    def beep(self,*args):
             
        n = 1                                             # Number of beeps
        f = 2000                                          # Frequency of beeps
        t = 75                                            # Sleep time

        if len(args) == 1:
            n = args[0]
        elif len(args) == 2:
            n = args[0]
            t = args[1]
        elif len(args) == 3:
            n = args[0]
            t = args[1]
            f = args[2]
        
        self.buzzer.freq(f)

        for i in range(n):
            self.buzzer.duty_u16(30000)
            sleep_ms(t)
            self.buzzer.duty_u16(0)
            if n > 1:
                sleep_ms(t)

    
    # Startup 
    def init(self):
        
        # Constants
        
        self.r = 6371000                                  # Radius of the pale blue dot (m)
        self.g = 9.80665           # put 9.80665                       # Accelertaion due to gravity (m/s^2)
#       State of Avionics = Always Bussin                 # We bus while VBUS be bussin what the fuck has happened to my sense of humor send help please
        
        
#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        # COBS encoder removed (no encoder on this board)
        # self.cobs_encoder = encoder.COBS_Encoder()
        
        # Peripherals
        
        # Single LED on new board is on pin 25
        self.led = Pin(25,Pin.OUT)
        self.buzzer = PWM(Pin(2))
        
        self.bmp = None                                   # Barometer
        self.bno = None                                   # IMU
#         self.kx = None                                    # High-G Accelerometer
        self.gps = None                                   # Grumble, Pause and Swear ( WHY THE FUCK DOES THIS SHIT NOT WORK INSIDE THE ROCKET ?! )

#         self.adc1 = None                                  # Analog to Digital 1 Converter (For measuring voltages on different channels of the flight computer)
#         self.adc2 = None                                  # Analog to Digital 2 Converter (For measuring voltages on different channels of the flight computer)
#         self.adc3 = None                                  # Analog to Digital 3 Converter (For measuring voltages on different channels of the flight computer)
#         self.adc4 = None                                  # Analog to Digital 4 Converter (For measuring voltages on different channels of the flight computer)

        self.drogue_pin = Pin(7,Pin.OUT)                 # Drive this high to blow up drogue pyro
        self.main_pin = Pin(17,Pin.OUT)                   # Drive this high to blow up main pyro / cut reefing line
#         self.ign1_pin = Pin(19,Pin.OUT)                   # Drive this high to blow up IGN1
#         self.ign2_pin = Pin(21,Pin.OUT)                   # Drive this high to blow up IGN2
        
        self.drogue_pin.value(0)                          # Start low
        self.main_pin.value(0)                            # Start low
#         self.ign1_pin.value(0)                            # Start low
#         self.ign2_pin.value(0)                            # Start low
        
#         self.ctrl_pin = Pin(3,Pin.OUT)                    # Pin to enable control power
#         self.cam_pin = Pin(2,Pin.OUT)                     # Pin to power camera
        
#         self.ctrl_pin.value(1)
#         self.cam_pin.value(1)
        
#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        
        # Telemetry - SX1262 on this board
        
        self.sx = None                                    # Radio
        self.tel_delay = 100                              # Delay between telemetry cycles in ms (Choose fast/slow in code)
        self.tel_delay_fast = 100                         # Delay between telemetry cycles in ms (Fast mode - In flight)
        self.tel_delay_slow = 100                         # Delay between telemetry cycles in ms (Slow mode - On Pad / After Touchdown)


#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


        # Data logging & Storage
        
        self.flash = None                                 # PTSD
        
        self.size = 0                                     # Size of data file in bytes
        storage = uos.statvfs("/")                        # Stats of main directory I think?
        self.free_bytes = storage[0]*storage[3]           # Free bytes (Available for data file)
        
        self.data_buffer = bytearray(0)                   # Buffer for storing upto ~2.5 seconds before launch
        
        self.packing_str = '!s2i37f'
        self.packing_str_tel = '<siii3h2f3h'  # Telemetry disabled
        
        self.data_fast = bytearray(157)                    # Bytearray for sending data via telemetry or writing to flash
        self.data_tel = bytearray(49)                    # Telemetry disabled
        
        self.bno_accel = [0,0,0]                          # Array for BNO acceleraton readings
#         self.kx_accel = [0,0,0]                           # Array for KX acceleration readings
        self.gyro = [0,0,0]                               # Array for rate gyro readings 
        self.mag = [0,0,0]                                # Array for magnetometer readings (Disabled because only works in low-G mode)
        self.orientation = [0,0,0]                        # Array for BNO's orientation estimate (Disabled because only works in low-G mode)
        self.temp = 0                                     # Temperature
        self.alt = 0                                      # Altitude from ground
        self.dp_scale = 100000/4.46346                    # Differential pressure scale
        self.bno_scale = 0                                # Multiply BNO acceleration reaings by this to correct scaling errors
#         self.kx_scale = 0                                 # Multiply KX acceleration reaings by this to correct scaling errors
        self.density = 1.15                               # Ambient air density
        
        self.volt_main = 0                                # Voltage on MAIN channel
        self.volt_drogue = 0                              # Voltage on DROGUE channel
        self.volt_ign1 = 0                                # Voltage of IGN1 channel
        self.volt_ign2 = 0
        
        self.reg_1 = 0                                    # Fin 1 Regualtor Voltage
        self.reg_2 = 0                                    # Fin 2 Regualtor Voltage
        self.reg_3 = 0                                    # Fin 3 Regualtor Voltage
        self.reg_4 = 0                                    # Fin 4 Regualtor Voltage
        
        self.fin_1 = 0                                    # Fin 1 Voltage
        self.fin_2 = 0                                    # Fin 2 Voltage
        self.fin_3 = 0                                    # Fin 3 Voltage
        self.fin_4 = 0                                    # Fin 4 Voltage
        
        self.volt_3v3 = 0                                 # Voltage on 3V3 channel
        self.volt_bat = 0                                 # Voltage on VCC channel
        self.volt_dp = 0                                  # Differential Pressure Sensor Voltage
        self.volt_motor = 0                               # Pressure Transducer Voltage

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        
        # Calibration
        
        self.send_calib = False                           # Send calibration data to ground station? (Telemetry disabled)
        self.calib_time = 0                               # Time when calibration was performed
        self.calib_altitude = 0                           # Altitude from mean sea level where calibration was performed
        self.calib_temp = 0                               # Temperature (in degrees Celsius) inside the rocket when calibration was performed
#         self.dp_offset = 0                                # Ram barometer offset when calibration was performed
        self.calib_data = bytes(32)                       # Bytearray for sending data via telemetry or writing to flash
        
        
#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        
        # GPS
        
        self.speed = 0                                    # Horizontal speed I think? Hasn't worked in a rocket yet.
        self.latitude = (0,0,0.0,'N')                     
        self.longitude = (0,0,0.0,'E')
        self.course = 0                                   # Angle of flight path from True North
        self.hdop = 0                                     # Horizontal Dilution of Precision. Refer - https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
        self.vdop = 0                                     # Vertical Dilution of Precision
        self.gps_altitude = 0                             # GPS altitude
        self._fix_time = 0                                # Time of fix since startup in ms
        self.time_since_fix = 0                           # Time since when fix was acquired and it was realized in code in ms (Usually small)
        self.gps_counter = 0                              
        self.last_gps_counter = 0                         # New fix if gps_counter > last_gps_counter
        self.sentence_type = b'NUL'                       # Type of NMEA message - GGA/GLL/RMC/VTG (Legit) , VNC/ROV (Made up by me and added to as_GPS library) , GSA/GSV are useless for us                                                             
        

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        
        # Settings
        
        self.runtime = 36000 * 1000                       # Stop logging and assume touchdown has occured after this time in ms from startup
        
        self.calib_gap = 10 * 1000                        # Gap between calibration in ms
        self.calib_count = 0                              # Number of calibrations performed so far
        self.calib_max = 1                                # Maximum number of automatic calibrations (Manual Telemetry Command will override this limit)
        
        self.liftoff_accel = 0.7  #2                            # Minimum sustained Gs for liftoff detection
        self.min_liftoff_alt = 300                # put 10        # Minimum altitude to be cleared for liftoff detection in m
        
        self.force_burnout_time = 1 * 1000    # put 5*1000            # Force burnout after this time from liftoff in ms
        
        self.force_drogue_time = 17 * 1000                # Force drogue after this time from liftoff in ms
        self.lockout_drogue_time = 3 * 1000     # put 6*1000          # Cannot fire drogue before this time after liftoff in ms
        
        self.main_alt = 400                               # Deploy main chute / de-reef chute below this altitude in m during descent
        self.drogue_to_main_lockout = 500        # put 5000        # Cannot fire main / de-reef chute before this time after drogue ejection in ms (Incase ejection fucks up pressure readings)
        
        self.ballistic_lockout_time = 1000        # delay to force main in case drogue fails and rocket is in ballisitc re-entry
        self.max_re_entry_speed = -2000.0             # max tolerable speed until vehicle enters ballistic 
        
        self.touchdown_alt = 50                           # Maximum altitude for touchdown detection in m
        self.touchdown_vel_limit = 0.1                    # Maximum speed for touchdown detection in m/s
        self.main_to_touchdown_lockout =  2000     # put 20000      # Cannot detect touchdown before this time after main ejection / de-reefing in ms (Incase ejection fucks up pressure readings)                                        
        
#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        # Miscellaneous
        
        self.async_loop = None                            
        self.t_events = [0.,0.,0.,0.,0.]                  # Array of events (Liftoff, Burnout, Drogue, Main, Touchdown)                
        self.logging_done = False                         # Killswitch? Kinda
        self.state = 0                                    # (0 - On Pad, 1 - Boost, 2 - Coasting, 3 - Descent under Drogue, 4 - Descent under Main, 5 - Landed)
        self.t_log = ticks_ms()                           # Timestamp of loop
        sleep_ms(10)                                      # 10 ms wait to avoid divide by zero on first loop
        self.last_t_log = self.buf_len = 10
        # Length of buffer
        self.alt_buf = np.zeros((self.buf_len))           # Altitude buffer for state machine
        self.acc_buf = np.zeros((self.buf_len))           # Acceleration buffer for state machine
        self.vel_buf = np.zeros((self.buf_len))           # Velocity buffer for state machine
        
        self.bmp_working = True                           # Is BMP returning data?
        self.bno_working = True                           # Is BNO returning data?
#         self.kx_working = True                            # Is KX returning data?
        
        self.board_init()                                 # Mario


    
    # Runs for each GPS fix
    def callback_gps(self,gps, *_):
        try:
            self.gps_counter += 1

            self.latitude = gps.latitude(coord_format=as_GPS.DMS)
            self.longitude = gps.longitude(coord_format=as_GPS.DMS)
            self.speed = 0.5144*gps.speed()                           # Converting knots to m/s
            self.course = gps.course
            self.sentence_type = gps.sentence_type
            self.hdop = gps.hdop
            self.vdop = gps.vdop
            self.gps_altitude = gps.altitude - self.calib_altitude
            self._fix_time = gps._fix_time
            self.time_since_fix = gps.time_since_fix()
#             print(f'{self.latitude} {self.longitude}') 
                                    
        except Exception as e:
            
            e = 'GPS Error : ' + str(e)
            print(e)
            
    def cb(sx, events):
        if events & SX1262.TX_DONE:
            print('TX done.')
    
    # Calibrate pad altitude and temperature
    def calib_bmp(self, n = 10):
        try:
            self.calib_temp = self.bmp.temperature
            avg_alt = 0
            for i in range(n):
                avg_alt += self.altitude()
                sleep_ms(10)
            self.calib_altitude = avg_alt/n
        except Exception as e:
            
            self.calib_altitude = 0
            
            e = 'Failed to calibrate BMP280 : ' + str(e)
            print(e)
            self.failure(e)
        
    # Calculate ram barometer offset
#     def calib_dp(self,n = 50):
#         try:
#             # Ram/airspeed sensor removed on this board; skip ADC-based calibration
#             self.dp_offset = 0
#             return
# 
#         except Exception as e:
#             e = 'Failed to calibrate airspeed sensor : ' + str(e)
#             print(e)
#             self.failure(e)

                
    
    # Initialize sensors
    def init_board(self):
        print("starting init")

        # UART bus of GPS (Pico pins GP4=TX, GP5=RX on this board)
        self.uart = UART(1, 9600, rx = Pin(5), tx = Pin(4))
        
        # UART bus of XBEE - removed on this board
        # self.xbee = UART(1, 57600, rx = Pin(25), tx = Pin(24))
        # XBee removed; keep self.xbee as None
        
        # Initialize SX1262
        try:
            self.sx = SX1262(spi_bus=1, clk=14, mosi=15, miso=8, cs=13, irq=11, rst=12, gpio=9)
            self.sx.begin(freq=928, bw=500.0, sf=12, cr=8, syncWord=0x12,
                         power=22, currentLimit=60.0, preambleLength=8,
                         implicit=False, implicitLen=0xFF,
                         crcOn=True, txIq=False, rxIq=False,
                         tcxoVoltage=1.7, useRegulatorLDO=False, blocking=True)
            self.sx.setBlockingCallback(False, self.cb)
            print('sx1262 done')
            
        except Exception as e:
            e = 'Failed to initialize SX1262 : ' + str(e)
            print(e)
            self.failure(e)
            
        
        # Initialize BMP
        try:
            self.bmp = BMP280(I2C(0,
                            scl = Pin(21),
                            sda = Pin(20)))
            print('bmp done')
        except Exception as e:
            e = 'Failed to initialize BMP280 : ' + str(e)
            print(e)
            self.failure(e)
        
        # INITIALISE GPS
        try:
            sreader = asyncio.StreamReader(self.uart)  # Create a StreamReader
            # fix_cb defines the function called when a fix is acquired
            # cb_mask defines the sentence types that can trigger a callback (RMC,VTG,GLL,GGA)[VNC,ROV also trigger it]
            self.gps = as_GPS.AS_GPS(sreader,fix_cb = self.callback_gps,cb_mask= as_GPS.RMC | as_GPS.VTG | as_GPS.GLL | as_GPS.GGA)
            
            print('gps done')
        except Exception as e:
            e = 'Failed to initialize GPS : ' + str(e)
            print(e)
            self.failure(e)
         
        # Initialize BNO 
        try:
            self.bno = BNO055(I2C(0,
                scl = Pin(21),
                sda = Pin(20)))
            
            # Register addresses for accelerometer offsets
            ACCEL_OFFSET_X_LSB_ADDR = const(0x55)
            ACCEL_OFFSET_X_MSB_ADDR = const(0x56)
            ACCEL_OFFSET_Y_LSB_ADDR = const(0x57)
            ACCEL_OFFSET_Y_MSB_ADDR = const(0x58)
            ACCEL_OFFSET_Z_LSB_ADDR = const(0x59)
            ACCEL_OFFSET_Z_MSB_ADDR = const(0x5A)
            ACCEL_RADIUS_LSB_ADDR = const(0x67)
            ACCEL_RADIUS_MSB_ADDR = const(0x68)
            
            # Set BNO to config mode
            self.bno.mode(0)

            # Bytearray of offsets           
            bno_offsets = bytearray(b'\xef\xff\t\x00\xef\xff\xae\x03\xa2\xff\x98\x04\xff\xff\x02\x00\xff\xff\xe8\x03\x02\x03')

            # Write accelerometer offsets
            self.bno._write(ACCEL_OFFSET_X_LSB_ADDR, bno_offsets[0])
            self.bno._write(ACCEL_OFFSET_X_MSB_ADDR, bno_offsets[1])
            self.bno._write(ACCEL_OFFSET_Y_LSB_ADDR, bno_offsets[2])
            self.bno._write(ACCEL_OFFSET_Y_MSB_ADDR, bno_offsets[3])
            self.bno._write(ACCEL_OFFSET_Z_LSB_ADDR, bno_offsets[4])
            self.bno._write(ACCEL_OFFSET_Z_MSB_ADDR, bno_offsets[5])
            self.bno._write(ACCEL_RADIUS_LSB_ADDR, bno_offsets[18])
            self.bno._write(ACCEL_RADIUS_MSB_ADDR, bno_offsets[19])
                    
            # Switch to 16G and 2000 dps mode
            self.bno.mode(7)
            self.bno.config(ACC,(16,62))
            self.bno.config(GYRO,(2000,32))
            self.bno.config(MAG,(30,))
                
            # Calculate accelerometer scale
            print('Place IMU at rest')

            self.bno_scale = 0
            
            for i in range(10):
                bno_accel = (0.0,0.0,0.0)
                # Keep taking readings until you get a valid one (Can return all zeros on startup sometimes)
                while bno_accel == (0.0,0.0,0.0):
                    bno_accel = self.bno.accel()
                    sleep_ms(1)
                
                self.bno_scale += self.g/(sqrt(pow(bno_accel[0],2) + pow(bno_accel[1],2) + pow(bno_accel[2],2)))/10
            
            print('bno done')
        except Exception as e:
            e = 'Failed to initialize BNO055 : ' + str(e)
            print(e)
            self.failure(e)

        # Initialize KX-134
#         try:
#             self.kx = kx132.KX132(i2c = I2C(0,scl=Pin(13), sda=Pin(12)), address = 31)
#             self.kx.acc_range = kx132.ACC_RANGE_16
#             
#             self.kx_scale = 0
#             
#             for i in range(10):
#                 kx_accel = (0.0,0.0,0.0)
#                 while kx_accel == (0.0,0.0,0.0):
#                     kx_accel = self.kx.acceleration
#                     sleep_ms(1)
#                 self.kx_scale += self.g/(sqrt(pow(kx_accel[0],2) + pow(kx_accel[1],2) + pow(kx_accel[2],2)))/10
#             
#             print('kx done')
#         except Exception as e:
#             e = 'Failed to initialize KX134 : ' + str(e)
#             print(e)
#             self.failure(e)

        # INITIALISE ADC - Removed on this board
        # ADCs and associated ADS1115 chips are not present on the new board.
        # The adc* attributes remain defined earlier as None/zeros so code that
        # references voltages will continue to see default values.
        
        # Initialise Pyro monitoring ADC line
    
    # Calibration
    async def calibrate(self):

        try:
            self.calib_count += 1
            
            self.calib_time = ticks_ms()
            
            print('Calibration Number ' + str(self.calib_count))
            
            # Calibrate both barometers
            self.calib_bmp()
            print("bmp calibrated")
#             self.calib_dp()

            lat = self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600
            lon = self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600
            
            # Pack data to bytes
            self.calib_data = ustruct.pack('!4ffff',
                                                  self.calib_altitude,
#                                                   self.dp_offset,
                                                  self.calib_temp,
                                                  lat,
                                                  (ord(self.latitude[3])),
                                                  lon,
                                                  (ord(self.longitude[3]))
                                                  )
            
            
            # comms() subroutine will transmit data - telemetry disabled
            self.send_calib = True
            
            self.led.value(0)
            self.beep()
            self.led.value(1)
            self.beep()
            
            print("beeped")

            # Play a short tune (~2s) to indicate calibration complete
            try:
                notes = [1568, 2093, 2637]
                durations = [200, 200, 300]
                for i, n in enumerate(notes):
                    self.buzzer.freq(n)
                    self.buzzer.duty_u16(30000)
                    sleep_ms(durations[i])
                    self.buzzer.duty_u16(0)
                    sleep_ms(50)
                print("calibration done")
            except Exception:
                pass
            
        except Exception as e:
            e = 'Failed calibration procedure : ' + str(e)
            print(e)
            self.failure(e)

        await asyncio.sleep(0)

    # Data acquisition
    async def get_data(self):
                
        while True:

            # Timestamp
            self.t_log = ticks_ms() - self.calib_time
            
#             print(self.vel_buf[-1])
#             
            print(self.state,self.acc_buf[-1], self.vel_buf[-1],self.alt_buf[-1])

            # Try BNO
            try:
                # BNO readings are tuples which themselves cannot be edited. So, first store it in a np array then scale them.
                self.bno_accel = np.array(self.bno.accel())*self.bno_scale
                self.gyro = self.bno.gyro()
                self.bno_working = True
            except Exception as e:
                e = 'Failed to read bno : ' + str(e)
                print(e)
                self.failure(e)
                self.bno_working = False
                
            # Try KX
#             try:
#                 # KX readings are tuples which themselves cannot be edited. So, first store it in a np array then scale them.
#                 self.kx_accel = np.array(self.kx.acceleration)*self.kx_scale
#                 self.kx_working = True
#             except Exception as e:
#                 e = 'Failed to read kx : ' + str(e)
#                 print(e)
#                 self.failure(e)
#                 self.kx_working = False
            
            # Try BMP
            try:
                self.temp = self.bmp.temperature
                self.alt = self.altitude() - self.calib_altitude
                self.density = self.rho(self.alt)
                self.bmp_working = True
            except Exception as e:
                e = 'Failed to read bmp : ' + str(e)
                print(e)
                self.failure(e)
                self.bmp_working = False

            # Read ADC - removed (no ADS1115 on new board)
            # Voltage and regulator variables remain as their initialized defaults
            # so logging/state machine will continue to work without ADC reads.
                
            # Check if calibration is scheduled  
            if self.state == 0:
                if self.t_log > self.calib_gap and self.calib_count < self.calib_max:
                    self.async_loop.create_task(self.calibrate()) # Run calibration once
            
#             print("got data")
            await asyncio.sleep(0)
        
        
    # State Machine
    async def state_machine(self):
        
        last_beep_time = 0
        
        while True:
                        
            try:
                t = ticks_ms()
                                
                # Shift values one place to the left
                self.alt_buf = np.roll(self.alt_buf,-1)
                self.vel_buf = np.roll(self.vel_buf,-1)
                self.acc_buf = np.roll(self.acc_buf,-1)
                # Update last value  
                self.alt_buf[-1] = self.alt
                self.vel_buf[-1] = 1000 * (self.alt_buf[-1] - self.alt_buf[-2])/(self.t_log - self.last_t_log)
                
                # If both accelerometer, work average their readings. If only one works, use that one. If none work, pray.
                if self.bno_working:
                    self.acc_buf[-1] = self.bno_accel[1] # + self.kx_accel[0]
#                 elif self.bno_working:
#                     self.acc_buf[-1] = -self.bno_accel[1]
#                 elif self.kx_working:
#                     self.acc_buf[-1] = self.kx_accel[0]

                # If on pad & not accelerating & upright (LOL RUSSIA), then beep regularly to indicate code is running and nothing is stuck
                if self.state == 0 and abs(self.acc_buf[-1]-self.g)/self.g < 0.15 and ticks_ms() - last_beep_time > 2000:
                    self.beep(1,50,3000)
                    last_beep_time = ticks_ms()
                
                # Send fast telemetry in flight - telemetry disabled
                # if (self.state != 0 and self.state != 5):
                #     self.tel_delay = self.tel_delay_fast
                # else:
                #     self.tel_delay = self.tel_delay_slow
                
                # Run state machine until runtime ends or shutdown
                if (self.t_log > self.runtime or self.logging_done):  
                    self.data_array = bytearray()
                    self.state = 5
                    self.async_loop.stop()
            
                # Liftoff detected if - (State is 0 & (Altitude > Minimum liftoff altitude & Sustained acceleration) or Altitude high enough)
                if self.state==0 and ((np.all(self.alt_buf > self.min_liftoff_alt) and np.all(self.acc_buf > self.liftoff_accel*self.g))): #Liftoff, acc commented out for vacuum test
                    self.state = 1
                    self.t_events[0] = self.t_log
                            
                # Burnout detected if - (Liftoff detected & Sustained deceleration & deceleration(drag) is decreasing) or (Enough time has passed since liftoff)
                elif self.state==1 and ((np.all(self.acc_buf < 0) and self.acc_buf[-1] > self.acc_buf[-2]) or self.t_log - self.t_events[0] > self.force_burnout_time):
                    self.state = 2
                    self.t_events[1] = self.t_log
                
                # Drogue deployed if - (Burnout detected & Lockout time crossed) & (Vertical velocity is negative or Enough time has passed since liftoff)
                elif (self.state==2 and self.t_log - self.t_events[0] > self.lockout_drogue_time) and (np.all(self.vel_buf < 0) or self.t_log - self.t_events[0] > self.force_drogue_time):
                    self.state = 3
                    self.t_events[2] = self.t_log
                    
                    # Spam drogue pin
                    for i in range(3):
                        self.drogue_pin.value(1)
                        sleep_ms(300)
                        self.drogue_pin.value(0)
                        sleep_ms(100)
                
                # Main deployed if - (Drogue deployed & Altitude < Main deployment altitude & Lockout time crossed) or (decent rate > max decent rate threshold and ballisitc entry lockout exceeded)
                elif self.state==3 and ((np.all(self.alt_buf < self.main_alt) and self.t_log - self.t_events[2] > self.drogue_to_main_lockout) or (np.all(self.vel_buf < self.max_re_entry_speed) and self.t_log - self.t_events[2] > self.ballistic_lockout_time)):
                    if np.all(self.alt_buf < self.main_alt) and self.t_log - self.t_events[2] > self.drogue_to_main_lockout:
                        self.failure("MAIN_ALT_BRANCH")
                    elif np.all(self.vel_buf < self.max_re_entry_speed) and self.t_log - self.t_events[2] > self.ballistic_lockout_time:
                        self.failure("MAIN_BALLISTIC_BRANCH: "+ str(self.vel_buf[0])+" "+str(self.vel_buf[-1])+" "+str(self.vel_buf[-2])+" "+str(self.vel_buf[-3])+" "+str(self.vel_buf[-4]))
                        self.failure("BUF_DTYPE: "+str(type(self.vel_buf[-1]))+" Threshold type: "+str(type(self.max_re_entry_speed))+" THRESHOLD: "+str(self.max_re_entry_speed))
                    self.state = 4
                    self.t_events[3] = self.t_log
                    
                    # Spam main pin
                    for i in range(3):
                        self.main_pin.value(1)
                        sleep_ms(300)
                        self.main_pin.value(0)
                        sleep_ms(100)

                        
                # Touchdown detected if - (Altitude < Maximum Touchdown Altitude & Vertical velocity is small enough & Lockout time is crossed)
                elif self.state == 4 and np.all(self.alt_buf < self.touchdown_alt) and abs(np.mean(self.vel_buf)) < self.touchdown_vel_limit and self.t_log - self.t_events[3] > self.main_to_touchdown_lockout:
                    self.state = 5
                    self.t_events[4] = self.t_log
                    self.logging_done = True
                    # self.tel_delay = self.tel_delay_slow  # Telemetry disabled                    
                    
                # After touchdown we chill
                else:
                    pass
                
                # Keep it low if you don't wanna blow
                self.drogue_pin.value(0)
                self.main_pin.value(0)

                self.last_t_log = self.t_log
#                 print(self.state)
                
#                 print("state machine started")
                
            except Exception as e:
                e = 'Failure in State Machine : ' + str(e)
                print(e)
                self.failure(e)
            
            await asyncio.sleep(0) 

    # Log data
    async def log_data(self):
        
        pad_loc_count = 0
        
        while True:
            
            
            t = ticks_ms()
            
            try:
                # Pack fast readings into bytes
                ustruct.pack_into(self.packing_str,       # Pack according to this format. Refer - https://docs.python.org/3/library/struct.html & https://learn.microsoft.com/en-us/cpp/cpp/data-type-ranges?view=msvc-170
                                  self.data_fast,         # Into this buffer
                                  0,                      # With offset 0 (Data starts from here)
                                  
                                  b'F',                   # F indicates fast readings
                                  (int)(self.t_log),
                                  (int)(self.state),
                                  (float)(self.temp),
                                  (float)(self.bno_accel[0]),
                                  (float)(self.bno_accel[1]),
                                  (float)(self.bno_accel[2]),
#                                   (float)(self.kx_accel[0]),
#                                   (float)(self.kx_accel[1]),
#                                   (float)(self.kx_accel[2]),
                                  (float)(self.gyro[0]),
                                  (float)(self.gyro[1]),
                                  (float)(self.gyro[2]),
                                  (float)(self.volt_dp),
                                  (float)(self.volt_motor),
                                  (float)(self.volt_bat),
                                  (float)(self.volt_3v3),
                                  (float)(self.volt_drogue),
                                  (float)(self.volt_main),
                                  (float)(self.volt_ign1),
                                  (float)(self.volt_ign2),      
                                  (float)(self.reg_1),
                                  (float)(self.reg_2),
                                  (float)(self.reg_3),
                                  (float)(self.reg_4),
                                  (float)(self.fin_1),
                                  (float)(self.fin_2),
                                  (float)(self.fin_3),
                                  (float)(self.fin_4),
                                  (float)(self.Z[0][0]),
                                  (float)(self.Z[1][0]),
                                  (float)(self.Z[2][0]),
                                  (float)(self.Z[3][0]),
                                  (float)(self.Z[4][0]),
                                  (float)(self.X[0][0]),
                                  (float)(self.X[1][0]),
                                  (float)(self.X[2][0]),
                                  (float)(self.X[3][0]),
                                  (float)(self.X[4][0]),
                                  (float)(self.track)
                                  )
                                
            except Exception as e:
                e = 'Failed to pack fast data : ' + str(e)
                print(e)
                self.failure(e)
            
            # Only log data if - Not on pad or Not touched down or More than 1kB of space is available
            if (self.state != 0 and self.state != 5 and self.free_bytes - self.size > 1000):
                try:
                    self.data_file.write(self.data_fast)
                    self.size += len(self.data_fast)
                except Exception as e:
                    e = 'Failed to write fast data : ' + str(e)
                    print(e)
                    self.failure(e)
                        
            # If on pad
            if self.state == 0:
                
                # Store data from before liftoff in a buffer (Will be written to file after touchdown, not after liftoff becuase writing it takes time and readings from that time will be lost)
                try:
                    if len(self.data_buffer) < 20*len(self.data_fast):
                        self.data_buffer += self.data_fast
                    else:
                        self.data_buffer = self.data_buffer[len(self.data_fast):] + self.data_fast
                except Exception as e:
                    e = 'Failed to store fast data in pre-flight buffer : ' + str(e)
                    print(e)
                    
#             print("data logging")
            
            await asyncio.sleep_ms(0)        
     
             
    # Communication between rocket and ground station   
    async def comms(self):
        
        stop_count = 0
        
        a = 0
        
        while True:
                                        
            t = ticks_ms()
                        
            try:
                
                ustruct.pack_into(self.packing_str_tel,       # Pack according to this format. Refer - https://docs.python.org/3/library/struct.html & https://learn.microsoft.com/en-us/cpp/cpp/data-type-ranges?view=msvc-170
                                  self.data_tel,              # Into this buffer
                                  0,                          # With offset 0 (Data starts from here)
                                  
                                  b'T',                       # T indicates telemetry readings
                                  (int)(self.t_log),
                                  (int)(self.state),
                                  (int)(self.temp),
                                  (int)(self.bno_accel[0]*10),
                                  (int)(self.bno_accel[1]*10),
                                  (int)(self.bno_accel[2]*10),
    #                                   (int)(self.kx_accel[0]*10),
    #                                   (int)(self.kx_accel[1]*10),
    #                                   (int)(self.kx_accel[2]*10),
    #                                   (int)(self.gyro[0]*10),
    #                                   (int)(self.gyro[1]*10),
    #                                   (int)(self.gyro[2]*10),
    #                                   (int)(self.volt_bat*10),
    #                                   (int)(self.volt_3v3*10),
    #                                   (int)(self.volt_drogue*10),
    #                                   (int)(self.volt_main*10),
    #                                   (int)(self.volt_ign1*10),
    #                                   (int)(self.volt_ign2*10),      
    #                                   (int)(self.reg_1*10),
    #                                   (int)(self.reg_2*10),
    #                                   (int)(self.reg_3*10),
    #                                   (int)(self.reg_4*10),
                                  (float)(self.Z[0][0]),
                                  (float)(self.Z[1][0]),
                                  (int)(self.Z[2][0]),
                                  (int)(self.Z[3][0]),
                                  (int)(self.Z[4][0])
                                  )
            
                # COBS encoder removed; send raw telemetry buffer instead
#                     encoded_tel = self.data_tel
                            
                a += 1
                
    #                 print(a,self.latitude)
                
                if self.send_calib:
                    self.sx.send(self.calib_data)
                    self.send_calib = False
                elif self.state != 5:
                    self.sx.send(self.data_tel)
                    pass
                    
            except Exception as e:
                e = 'Failed to send telemetry : ' + str(e)
                print(e)
                self.failure(e)

            await asyncio.sleep_ms(self.tel_delay)

    # Nav Filter
    async def nav(self):
        
        counter = 0
        
        self.X_A = np.full((3,1),0.)        # Orientation vector
        C1 = np.full((3,3),0.)              # Rotation matrix

        self.X = np.full((5,1),0.)          # State vector
        W = np.full((5,1),0.)               # Idk what you call this
        F = np.eye(5)                       # State transition matrix
        P = np.full((5,5),0.)               # Uncertainity matrix
        Q = np.full((5,5),0.)               # Idk what you call this either
        R = np.eye(5)                       # Something something noise matrix I think
        self.Z = np.full((5,1),0.)               # Measurement Vector
        U = np.full((5,1),0.)               # Control vector
        
        self.dt = 1/40
        nx = ny = 1
        init = False
                     
        Q[0][0] = 0.1
        Q[1][1] = 0.1
        Q[2][2] = 0.1
        Q[3][3] = 0.01
        Q[4][4] = 0.1
        
        vx = 0 
        self.track = 0
#         airspeed = 0
                    
        self.gx = 0
        self.gy = 0
        self.gz = 0
                    
        while True:
                                                                                
            # Try-except in case it decides to crash
            if counter > 0:
                              
                self.dt = (self.t_log - self.last_loop_t)/1000
                
                try:
                    ax = self.bno_accel[1]
                    ay = self.bno_accel[0]
                    az = -self.bno_accel[2]
#                     print(f'{ax} {ay} {az}') 
                                                  
                    a = sqrt(ax*ax + ay*ay + az*az)
                    
                    self.gx = self.gyro[1]*pi/180
                    self.gy = self.gyro[0]*pi/180
                    self.gz = -self.gyro[2]*pi/180
#                     print(f'{self.gx} {self.gy} {self.gz}')
                                        
                except Exception as e:
                    e = 'Error in pulling raw sensor readings in KF : ' + str(e) 
                    print(e)
                    self.failure(e)
                
#                 try:
#                     # Calculate airspeed
#                     if self.volt_dp > self.dp_offset:
#                         dp = (pow(10,5)*(self.volt_dp - self.dp_offset))/4.5
#                         p0 = pow((-(self.alt/4947.19)+8.9611),1/0.190255)
#                         airspeed = sqrt(7*(p0/self.density)*(pow(1+(dp/p0),0.2857)-1))
#                         
#                 except Exception as e:
#                     
#                     airspeed = 0
#                     
#                     e = 'Error in calculating airspeed : ' + str(e)
#                     print(e)
#                     self.failure(e)

                e = abs(a-self.g)/self.g

                cutoff = 0.05
                
                try:
                    # If not accelerating and on pad, calculate pitch and yaw from accelerometer
                    if e < cutoff and not self.t_events[0]:
                        self.X_A[0][0] = atan2(ay,az)                                  # Yaw
                        self.X_A[1][0] = atan2(-ax,sqrt(pow(ay,2) + pow(az,2)))        # Pitch
                    # Else integrate rate gyros
                    else:
                        self.X_A[0][0] += nx*self.gx*self.dt
                        self.X_A[1][0] += ny*self.gy*self.dt
                    
                    # If liftoff or acceleration, start integrating rat gyro for roll. Else 0.
                    if self.t_events[0] or e > cutoff:
                        self.X_A[2][0] += self.gz*self.dt
                    else: 
                        self.X_A[2][0] = 0
                                                
                    if abs(self.X_A[0][0] + nx*self.gx*self.dt) >= pi/2:
                        nx *= -1
                    if abs(self.X_A[1][0] + ny*self.gy*self.dt) >= pi/2:
                        ny *= -1
                    
                    # Update rotation matrix
                    C1[0][0] = cos(self.X_A[1][0])
                    C1[0][1] = sin(self.X_A[1][0])*sin(self.X_A[0][0])
                    C1[0][2] = sin(self.X_A[1][0])*cos(self.X_A[0][0])
                    
                    C1[1][0] = 0
                    C1[1][1] = cos(self.X_A[0][0])
                    C1[1][2] = -sin(self.X_A[0][0])
             
                    C1[2][0] = -sin(self.X_A[1][0]) 
                    C1[2][1] = cos(self.X_A[1][0])*sin(self.X_A[0][0])
                    C1[2][2] = cos(self.X_A[1][0])*cos(self.X_A[0][0])
                    
                except Exception as e:
                    
                    e = 'Error in calculating attitude : ' + str(e)
                    print(e)
                    self.failure(e)
        # -------------------------------------------------------------------------------------------------------------------
                
                last_alt = self.Z[2][0]
#                 print(last_alt)
                
                # Caculate gravity vector
                try:
                    gravity = np.dot(np.linalg.inv(C1),np.array([[0],[0],[-self.g]]))
                
                    # CALCULATE ACCELERATION WRT GROUND
                    lin_acc_x =  ax + gravity[0][0]
                    lin_acc_y =  ay + gravity[1][0]
                    lin_acc_z = az + gravity[2][0]
                                                                                
                except Exception as e:
                    e = 'Error in calculating gravity vector : ' + str(e)
                    print(e)
                    self.failure(e)
                
                try:
                    # Accelerations in rotated frame of reference (self.Z-axis up)
                    if np.linalg.det(C1) > pow(10,-6):
                        a_r = np.dot(C1,np.array([[lin_acc_x],[lin_acc_y],[lin_acc_z]]))
                    else:
                        a_r = np.array([[0.],[0.],[0.]])
                        

                    dv = sqrt(pow(a_r[0][0],2) + pow(a_r[1][0],2))*self.dt
                    alpha = atan2(a_r[0][0],a_r[1][0])

                    self.track = atan2(vx*sin(self.track)+dv*sin(alpha),vx*cos(self.track)+dv*cos(alpha))
                    
                except Exception as e:
                    e = 'Error in rotating frame of reference : ' + str(e)
                    print(e)
                    self.failure(e)        
                
                try:
                    # Jump-start GPS in KF when fix is acquired, no waiting to converge
                    if self.latitude[0] != 0 and self.longitude[0] != 0 and init == False:
                        self.X[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                        self.X[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                        init = True
                        
                except Exception as e:
                    e = 'Error in GPS jump-start in KF : ' + str(e)
                    print(e)
                    self.failure(e)
                
                try:
                    # Update state transition matrix
                    F[2][3] = self.dt
                    
                    # Update control matrix
                    U[0][0] = cos(self.track)*vx*self.dt/self.r
                    if abs(self.Z[0][0]*180/pi - 90) > 10:
                        U[1][0] = sin(self.track)*vx*self.dt/(self.r*cos(self.Z[0][0]))
                    else:
                        U[1][0] = 0
                    U[2][0] = a_r[2][0]*self.dt*self.dt/2
                    U[3][0] = a_r[2][0]*self.dt
                    U[4][0] = lin_acc_z*self.dt
                    
                except Exception as e:
                    e = 'Error in updating Control/State Transition matrix : ' + str(e)
                    print(e)
                    self.failure(e)
                
                # Predict next state
                try:                
                    Xp = np.dot(F,self.X) + U
                except Exception as e:
                    e = 'Error in calculating Predicted State matrix : ' + str(e)
                    print(e)
                    self.failure(e)

                try:
                    Pp = np.dot(np.dot(F,P),F.T) + Q
                except Exception as e:
                    e = 'Error in updating KF uncertainity : ' + str(e)
                    print(e)
                    self.failure(e)
                    
                try:
                    # Altitude and velocty readings
                    if self.bmp_working:
                        self.Z[2][0] = self.alt
                        self.Z[3][0] = (self.Z[2][0] - last_alt)/self.dt
                    elif self.bno_working:
                        self.Z[2][0] += (self.Z[3][0] + a_r[2][0]*self.dt/2)*self.dt
                        self.Z[3][0] += a_r[2][0]*self.dt
                        
#                     if airspeed >= self.X[3][0]:
#                         self.Z[4][0] = airspeed
#                     else:
#                         self.Z[4][0] += lin_acc_z*self.dt
                        
                except Exception as e:
                    e = 'Error in updating Raw State in KF : ' + str(e)
                    print(e)
                    self.failure(e)

                try:
                    # If GPS fix
                    if self.gps_counter > self.last_gps_counter and ('GGA' in self.sentence_type or 'GLL' in self.sentence_type or 'RMC' in self.sentence_type or 'ROV' in self.sentence_type):
                        # Update coordinates from GPS
                        # Make sure latitude does not go near 90 so dead reckoning doesn't blow up
                        if abs((self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600) - 90) > 10:
                            self.Z[0][0] = (self.latitude[0] + self.latitude[1]/60 + self.latitude[2]/3600)*pi/180
                        else:
                            self.Z[0][0] += vx*self.dt*cos(self.track)/self.r
                            
                        self.Z[1][0] = (self.longitude[0] + self.longitude[1]/60 + self.longitude[2]/3600)*pi/180
                    else:
                        if abs(self.Z[0][0]*180/pi - 90) > 10:
                            self.Z[1][0] += vx*self.dt*sin(self.track)/(self.r*cos(self.Z[0][0]))
                        self.Z[0][0] += vx*self.dt*cos(self.track)/self.r
                        
                except Exception as e:
                    e = 'Error in GPS update in KF : ' + str(e)
                    print(e)
                    self.failure(e)

                # Update Kalman Gain
                try:
                    K = np.dot(Pp,np.linalg.inv(Pp + R))
                except Exception as e:
                    e = 'Error in updating Kalman Gain : ' + str(e)
                    print(e)
                    self.failure(e)

                # Update state
                try:
                    self.X = Xp + np.dot(K,self.Z - Xp)
                except Exception as e:
                    e = 'Error in updating State : ' + str(e)
                    print(e)
                    self.failure(e)
                
                # Update state uncertainity
                try:    
                    P = np.dot(np.eye(len(K)) - K,Pp)
                except Exception as e:
                    e = 'Error in updating State Uncertainity : ' + str(e)
                    print(e)
                    self.failure(e)

                # Calculate horizontal velocity
                try:
                    if abs(self.X[4][0]) > abs(self.X[3][0]):
                        vx = sqrt(self.X[4][0]**2 - self.X[3][0]**2)
                    else:
                        vx = sqrt(vx**2 + dv**2 + 2*vx*dv*cos(self.track - alpha))
                
                except Exception as e:
                    e = 'Error in updating horizontal velocity : ' + str(e)
                    print(e)
                    self.failure(e)
                  
            counter += 1              
            
            self.last_loop_t = self.t_log   
            self.last_gps_counter = self.gps_counter
            
#             print("started nav loop")
#             print(self.Z[2][0])
                  
            await asyncio.sleep(0)
    
    async def gui(self):
        
        self.roll_setpoint = 0
        self.setpoint_switch = False
        setpoint_interval = 10 * 1000
        last_roll_setpoint = 0

        while True:
            
            if self.state == 0:
                if int((self.t_log - self.t_events[1])/setpoint_interval)%2 == 1:
                    self.roll_setpoint = 90 * pi/180
                    if last_roll_setpoint == 0:
                        self.setpoint_switch = True
                    else:
                        self.setpoint_switch = False
                else:
                    self.roll_setpoint = 0
                    if last_roll_setpoint == 90 * pi/180:
                        self.setpoint_switch = True
                    else:
                        self.setpoint_switch = False

                last_roll_setpoint = self.roll_setpoint

            await asyncio.sleep_ms(0)
            
#     async def ctrl(self):
#         
#         kp = 0
#         kd = 0
#         ki = 0
#         
#         I = 0
#         
#         roll_deflection = 0
#         deflection_limit = 10 * pi/180
#         
#         while True:
#             
#             if self.state == 2:
#                 kp = 0.6656*pow(self.X[4][0],-0.843)
#                 ki = 0.2541*pow(self.X[4][0],-0.652)
#                 kd = 0.4261*pow(self.X[4][0],-1.03)
#             else:
#                 kp = 0
#                 ki = 0
#                 kd = 0
#             
#             kp = 0.6656*pow(50,-0.843)
#             ki = 0.2541*pow(50,-0.652)
#             kd = 0.4261*pow(50,-1.03)
#             
#             e = self.roll_setpoint - self.X_A[2][0]
#             
#             if abs(roll_deflection) < deflection_limit:
#                 I += e*self.dt
#             
#             roll_deflection = kp*e + ki*I + kd*(-self.gz)
# 
#             if self.setpoint_switch:
#                 I = 0
#                             
#             if abs(roll_deflection) > deflection_limit:
#                 roll_deflection *= abs(deflection_limit/roll_deflection)                
# 
#             await asyncio.sleep_ms(0)
       
    # Mission Accomplished
    async def after_party(self):
        
#         self.cam_pin.value(0)
        self.buzzer.duty_u16(30000)
    
        while True:
            # XBee removed on this board; skip transmitting data here
            # self.xbee.write(self.data_fast)
            self.sx.send(self.data_fast)
            i = 1
            while i < 6:
                self.buzzer.freq(i*500)
                i += 1
                sleep_ms(1000)
            

    async def fast_core(self):
        
        print('fast core init')
        
        # Open file with unique name using index.txt
        index_file = open('/index.txt','r')
        self.index = int(index_file.read())
        index_file.close()
        
        index_file = open('/index.txt','w')
        index_file.write(str(self.index + 1))
        index_file.close()

        self.data_file = open('/data_' + str(self.index) + '.bin','wb')
        
        print("starting event loop")
        
        self.async_loop = asyncio.get_event_loop()
        self.async_loop.create_task(self.calibrate())        
        self.async_loop.create_task(self.get_data())         
        self.async_loop.create_task(self.state_machine())    
        self.async_loop.create_task(self.nav())
#         self.async_zloop.create_task(''''''wwwwself.gui())
#         self.async_loop.create_task(self.ctrl()) 
        self.async_loop.create_task(self.comms())  # Telemetry disabled
        self.async_loop.create_task(self.log_data())        
        self.async_loop.run_forever()                        
        
        # Write pre-liftoff data to  file
        self.data_file.write(self.data_buffer)
        
        # Write calibration data
        try:
            self.data_file.write(b'C' + self.calib_data)
            self.size += len(self.calib_data)
            print('Calibration data written')
        except Exception as e:
            e = 'Failed to write Calibration Data : ' + str(e)
            print('Failed to write Calibration Data')
            self.failure(e)
            pass
        
        self.data_file.close()
        
        print("logging done, now sending only GPS")
        
        # Post touchdown loop
        self.async_loop = asyncio.new_event_loop()
        self.async_loop.create_task(self.after_party())    # TRANSMIT TO GROUND STATION
        self.async_loop.create_task(self.blink(1000))      # BLINKY BLINKY
        self.async_loop.run_forever()


# Start instance if class
if __name__ == "__main__":
    test_instance = async_test()                    
    test_instance.init()                        
    test_instance.init_board()                     
    try:
        asyncio.run(test_instance.fast_core())      
    except KeyboardInterrupt as e:
        print("User has ended loop, closing asyncio")
        test_instance.async_loop.stop()
        test_instance.async_loop.close()
    finally:
        print("done")
        pass



#flyte.