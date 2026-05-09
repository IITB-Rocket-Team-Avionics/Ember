"""
Micropython (Raspberry Pi Pico)
Plays music written on onlinesequencer.net through a passive piezo buzzer.
Fixed version - smooth playback, no choppiness.
"""

from machine import Pin, PWM, Timer
from math import ceil

tones = {
    'C0':16,'C#0':17,'D0':18,'D#0':19,'E0':21,'F0':22,'F#0':23,'G0':24,
    'G#0':26,'A0':28,'A#0':29,'B0':31,'C1':33,'C#1':35,'D1':37,'D#1':39,
    'E1':41,'F1':44,'F#1':46,'G1':49,'G#1':52,'A1':55,'A#1':58,'B1':62,
    'C2':65,'C#2':69,'D2':73,'D#2':78,'E2':82,'F2':87,'F#2':92,'G2':98,
    'G#2':104,'A2':110,'A#2':117,'B2':123,'C3':131,'C#3':139,'D3':147,
    'D#3':156,'E3':165,'F3':175,'F#3':185,'G3':196,'G#3':208,'A3':220,
    'A#3':233,'B3':247,'C4':262,'C#4':277,'D4':294,'D#4':311,'E4':330,
    'F4':349,'F#4':370,'G4':392,'G#4':415,'A4':440,'A#4':466,'B4':494,
    'C5':523,'C#5':554,'D5':587,'D#5':622,'E5':659,'F5':698,'F#5':740,
    'G5':784,'G#5':831,'A5':880,'A#5':932,'B5':988,'C6':1047,'C#6':1109,
    'D6':1175,'D#6':1245,'E6':1319,'F6':1397,'F#6':1480,'G6':1568,
    'G#6':1661,'A6':1760,'A#6':1865,'B6':1976,'C7':2093,'C#7':2217,
    'D7':2349,'D#7':2489,'E7':2637,'F7':2794,'F#7':2960,'G7':3136,
    'G#7':3322,'A7':3520,'A#7':3729,'B7':3951,'C8':4186,'C#8':4435,
    'D8':4699,'D#8':4978,'E8':5274,'F8':5588,'F#8':5920,'G8':6272,
    'G#8':6645,'A8':7040,'A#8':7459,'B8':7902,'C9':8372,'C#9':8870,
    'D9':9397,'D#9':9956,'E9':10548,'F9':11175,'F#9':11840,'G9':12544,
    'G#9':13290,'A9':14080,'A#9':14917,'B9':15804
}

class music:
    def __init__(self, songString='0 D4 8 0', looping=True, tempo=120, pin=None, pins=[Pin(18)]):
        """
        songString : onlinesequencer.net format
        looping    : loop forever if True
        tempo      : BPM
        pin        : single Pin (overrides pins)
        pins       : list of Pins for polyphony
        """
        self.song = songString
        self.looping = looping
        self.stopped = False
        self.beat = 0
        self.arpnote = 0
        self.playingNotes = []
        self.playingDurations = []
        self._hw_timer = Timer()

        # ms per beat based on BPM
        self.tick_ms = int(60000 / tempo)

        if pin is not None:
            pins = [pin]
        self.pins = pins
        self.pwms = []
        for p in pins:
            pwm = PWM(p)
            pwm.freq(440)
            pwm.duty_u16(0)  # start silent
            self.pwms.append(pwm)

        # Parse song into a list indexed by beat
        self.notes = []
        self.end = 0
        splitSong = self.song.split(";")
        for note in splitSong:
            snote = note.split(" ")
            testEnd = round(float(snote[0])) + ceil(float(snote[2]))
            if testEnd > self.end:
                self.end = testEnd

        while self.end > len(self.notes):
            self.notes.append(None)

        for note in splitSong:
            snote = note.split(" ")
            b = round(float(snote[0]))
            if self.notes[b] is None:
                self.notes[b] = []
            self.notes[b].append([snote[1], ceil(float(snote[2]))])

        self.end = ceil(self.end / 8) * 8

        # Start timer
        self._hw_timer.init(period=self.tick_ms, mode=Timer.PERIODIC, callback=self._cb)

    def _cb(self, t):
        # --- Remove expired notes ---
        i = 0
        while i < len(self.playingDurations):
            self.playingDurations[i] -= 1
            if self.playingDurations[i] <= 0:
                self.playingNotes.pop(i)
                self.playingDurations.pop(i)
            else:
                i += 1

        # --- Add notes starting on this beat ---
        if self.beat < len(self.notes) and self.notes[self.beat] is not None:
            for note in self.notes[self.beat]:
                self.playingNotes.append(note[0])
                self.playingDurations.append(note[1])

        # --- Drive PWM: only update freq if note changed, never silence mid-song ---
        for i, pwm in enumerate(self.pwms):
            if i < len(self.playingNotes):
                freq = tones[self.playingNotes[i]]
                pwm.freq(freq)
                pwm.duty_u16(32768)  # 50% duty - turn on
            else:
                pwm.duty_u16(0)  # no note, silence

        # --- Arpeggio for extra notes beyond pin count ---
        extra = len(self.playingNotes) - len(self.pwms)
        if extra > 0:
            if self.arpnote >= extra + 1:
                self.arpnote = 0
            idx = len(self.pwms) - 1 + self.arpnote
            if idx < len(self.playingNotes):
                self.pwms[-1].freq(tones[self.playingNotes[idx]])
                self.pwms[-1].duty_u16(32768)
            self.arpnote += 1

        # --- Advance beat ---
        self.beat += 1
        if self.beat >= self.end:
            if self.looping:
                self.beat = 0
            else:
                self.stop()

    def stop(self):
        self._hw_timer.deinit()
        for pwm in self.pwms:
            pwm.duty_u16(0)
            pwm.deinit()
        self.stopped = True

    def restart(self):
        self.stop()
        self.beat = 0
        self.arpnote = 0
        self.playingNotes = []
        self.playingDurations = []
        self.pwms = []
        for p in self.pins:
            pwm = PWM(p)
            pwm.freq(440)
            pwm.duty_u16(0)
            self.pwms.append(pwm)
        self.stopped = False
        self._hw_timer.init(period=self.tick_ms, mode=Timer.PERIODIC, callback=self._cb)

    def resume(self):
        self.restart()