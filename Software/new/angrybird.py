import machine
import time

# Frequencies for the required notes in Hz
NOTES = {
    'REST': 0,
    'C5': 523, 'CS5': 554, 'D5': 587, 'DS5': 622, 'E5': 659,
    'F5': 698, 'FS5': 740, 'G5': 784, 'GS5': 831, 'A5': 880,
    'AS5': 932, 'B5': 988, 'C6': 1047, 'CS6': 1109, 'D6': 1175,
    'DS6': 1245, 'E6': 1319, 'F6': 1397, 'FS6': 1480, 'G6': 1568
}

# Angry Birds main theme snippet
# Tuple format: (Note string, duration in 16th notes)
ANGRY_BIRDS_MELODY = [
    ('E5', 1), ('FS5', 1), ('G5', 2), ('E5', 2),
    ('B5', 1), ('REST', 1),
    ('E5', 1), ('FS5', 1), ('G5', 2), ('B5', 2),
    ('B5', 1), ('REST', 3),
    ('B5', 1), ('C6', 1), ('B5', 1), ('A5', 1),
    ('G5', 2), ('G5', 1), ('FS5', 1), ('E5', 1),
]

class BuzzerController:
    """
    Manages PWM signals for a passive buzzer.
    """
    def __init__(self, pin_number: int):
        """
        Initializes the PWM pin for the buzzer.
        
        Args:
            pin_number (int): The GPIO pin connected to the buzzer signal.
        """
        self.pwm = machine.PWM(machine.Pin(pin_number))
        self.pwm.duty_u16(0)

    def play_tone(self, frequency: int, duration_ms: int):
        """
        Plays a single frequency tone for a specified duration.
        
        Args:
            frequency (int): Frequency in Hz. A frequency of 0 indicates a rest.
            duration_ms (int): Duration to play the note in milliseconds.
        """
        if frequency == 0:
            self.pwm.duty_u16(0)
        else:
            self.pwm.freq(frequency)
            # 50% duty cycle for maximum resonance
            self.pwm.duty_u16(32768)
            
        time.sleep_ms(duration_ms)
        
        # Brief pause between notes for clear articulation
        self.pwm.duty_u16(0)
        time.sleep_ms(20)

    def play_melody(self, melody: list, tempo: int = 120):
        """
        Calculates timing and plays a sequence of notes.
        
        Args:
            melody (list): List of tuples containing (Note String, Duration multiplier).
            tempo (int): Beats per minute.
        """
        # Calculate the duration of a 16th note in milliseconds
        beat_duration_ms = int((60000 / tempo) / 4)
        
        for note, duration_multiplier in melody:
            frequency = NOTES.get(note, 0)
            duration_ms = beat_duration_ms * duration_multiplier
            self.play_tone(frequency, duration_ms)

    def cleanup(self):
        """
        Disables the PWM output to prevent hardware hanging.
        """
        self.pwm.deinit()

def main():
    """
    Entry point to initialize the buzzer on Pin 18 and execute the melody.
    """
    buzzer_pin = 18
    tempo = 140 
    
    player = BuzzerController(pin_number=buzzer_pin)
    
    try:
        player.play_melody(ANGRY_BIRDS_MELODY, tempo)
    finally:
        player.cleanup()

if __name__ == '__main__':
    main()