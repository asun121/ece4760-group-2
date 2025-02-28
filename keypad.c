/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * Keypad Demo
 *
 * KEYPAD CONNECTIONS
 *  - GPIO 9   -->  330 ohms  --> Pin 1 (button row 1)
 *  - GPIO 10  -->  330 ohms  --> Pin 2 (button row 2)
 *  - GPIO 11  -->  330 ohms  --> Pin 3 (button row 3)
 *  - GPIO 12  -->  330 ohms  --> Pin 4 (button row 4)
 *  - GPIO 13  -->     Pin 5 (button col 1)
 *  - GPIO 14  -->     Pin 6 (button col 2)
 *  - GPIO 15  -->     Pin 7 (button col 3)
 *
 * VGA CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * SERIAL CONNECTIONS
 *  - GPIO 0        -->     UART RX (white)
 *  - GPIO 1        -->     UART TX (green)
 *  - RP2040 GND    -->     UART GND
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/spi.h"

// VGA graphics library
#include "vga16_graphics.h"
#include "pt_cornell_rp2040_v1_3.h"

// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS 4
#define NUMKEYS 12

#define LED 25

unsigned int keycodes[12] = {0x28, 0x11, 0x21, 0x41, 0x12,
                             0x22, 0x42, 0x14, 0x24, 0x44,
                             0x18, 0x48};
unsigned int scancodes[4] = {0x01, 0x02, 0x04, 0x08};
unsigned int button = 0x70;

char keytext[40];
int prev_key = 0;

// variables/constants add from beep_beep

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0))
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)((((signed long long)(a)) << 15) / (b))

// Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0 // 2^32 (a constant)
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
// volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// variable accumulator instead of a fixed one
// accumulator value changes based on current frequency
volatile unsigned int phase_incr_main_0;
// track the frequency (i.e. swoop/chirp)
volatile unsigned int current_frequency;
// variable to store 2^32 / Fs instead of calculating it every time
volatile unsigned int two32_fs = two32 / Fs;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size];

// Values output to DAC
int DAC_output_0;
int DAC_output_1;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1); // maximum amplitude
fix15 attack_inc;                   // rate at which sound ramps up
fix15 decay_inc;                    // rate at which sound ramps down
fix15 current_amplitude_0 = 0;      // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0;      // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME 250
#define DECAY_TIME 250
#define SUSTAIN_TIME 10000
#define BEEP_DURATION 6500
#define BEEP_REPEAT_INTERVAL 50000

// State machine variables
volatile unsigned int STATE_0 = 0;
volatile unsigned int count_0 = 0;

// button state: 0 = not pressed, 1 = maybe pressed, 2 = pressed, 3 = maybe not pressed
volatile unsigned int BUTTON_STATE = 0;

// Mode state: 0 = normal play sound, 1 = record sound, 2 = playback sound
volatile unsigned int MODE = 0;

// Track the pressed button
volatile unsigned int TRACKED_BUTTON = 0;

// SPI data
uint16_t DAC_data_1; // output value
uint16_t DAC_data_0; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define LDAC 8
#define LED 25
#define SPI_PORT spi0

// GPIO for timing the ISR
#define ISR_GPIO 2

bool pressed = false;
bool action = false;
static void play_sound(int sound);

// Playback array and variables.

// Max amount of sounds we can store and play back
#define max_sounds 50

int playback_index = 0;
int sound_index = 0;
int sounds[max_sounds];

// This timer ISR is called on core 0
static void alarm_irq(void)
{
    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1);

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Normal playback mode
    if (MODE == 0)
    {
        // If a button is pressed
        if (action)
        {
            // Play the sound associated with the button
            if (TRACKED_BUTTON == 1)
            {
                play_sound(1);
            }
            else if (TRACKED_BUTTON == 2)
            {
                play_sound(2);
            }

            else if (TRACKED_BUTTON == 3)
            {
                play_sound(3);
            }

            // If we want to start recording, enter record mode
            else if (TRACKED_BUTTON == 4)
            {
                MODE = 1;
                action = false;
            }
        }
    }

    // Record mode
    else if (MODE == 1)
    {
        // If a button is pressed
        if (action)
        {
            // When we want to stop recording, enter playback mode
            if (TRACKED_BUTTON == 5)
            {
                MODE = 2;
                action = false;
            }

            // Else we record the pressed button
            else
            {
                // If we exceed the max amount of sounds we can store, we automatically start playing back
                if (sound_index >= max_sounds)
                {
                    MODE = 2;
                }
                else
                {
                    sounds[sound_index] = TRACKED_BUTTON;
                    sound_index++;
                    action = false;
                }
            }
        }
    }

    // Playback mode
    else
    {
        play_sound(sounds[playback_index]);
    }

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);
}

static void play_sound(int sound)
{
    // Calculate the frequency based on the sound we want to play
    if (sound == 1)
    {
        current_frequency = (-1.0 / 40625) * count_0 * count_0 + 0.16 * count_0 + 1740;
    }
    else if (sound == 2)
    {
        current_frequency = (.0001183 * count_0 * count_0) + 2000;
    }
    else if (sound == 3)
    {
        current_frequency = 0;
    }

    phase_incr_main_0 = current_frequency * two32_fs;
    // DDS phase and sine table lookup
    phase_accum_main_0 += phase_incr_main_0;
    if (current_amplitude_0 > int2fix15(1))
    {
        current_amplitude_0 = int2fix15(1);
    }
    DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
                                       sin_table[phase_accum_main_0 >> 24])) +
                   2048;

    // Ramp up amplitude
    if (count_0 < ATTACK_TIME)
    {
        current_amplitude_0 = (current_amplitude_0 + attack_inc);
    }
    // Ramp down amplitude
    else if (count_0 > BEEP_DURATION - DECAY_TIME)
    {
        current_amplitude_0 = (current_amplitude_0 - decay_inc);
    }

    // Mask with DAC control bits
    DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));

    // SPI write (no spinlock b/c of SPI buffer)
    spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);

    // Increment the counter
    count_0 += 1;

    // State transition
    if (count_0 >= BEEP_DURATION)
    {
        STATE_0 = 1;
        count_0 = 0;
        action = false;

        // If we are in playback mode and the previous sound finished, we increment to the next sound
        if (MODE == 2)
        {
            playback_index++;

            // We have played all the sounds and go back to normal mode
            if (playback_index >= sound_index)
            {
                MODE = 0;
                sound_index = 0;
                playback_index = 0;
            }
        }
    }
}

// This thread runs on core 0
static PT_THREAD(protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt);

    // Some variables
    static int i;
    static uint32_t keypad;

    while (1)
    {
        gpio_put(LED, !gpio_get(LED));

        // Scan the keypad!
        for (i = 0; i < KEYROWS; i++)
        {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN));
            // Small delay required
            sleep_us(1);
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F);
            // Break if button(s) are pressed
            if (keypad & button)
            {
                pressed = true;
                break;
            }
            else
            {
                pressed = false;
            }
        }
        // If we found a button . . .
        if (keypad & button)
        {
            // Look for a valid keycode.
            for (i = 0; i < NUMKEYS; i++)
            {
                TRACKED_BUTTON = i;
                if (keypad == keycodes[i])
                    break;
            }
            // If we don't find one, report invalid keycode
            if (i == NUMKEYS)
                (i = -1);
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else
            (i = -1);

        // debounce here
        // We only take action when transitioning between maybe pressed to pressed state - only chirp on state transition
        switch (BUTTON_STATE)
        {
        case 0:
            if (pressed)
            {
                BUTTON_STATE = 1;
            }
            break;
        case 1:
            if (pressed)
            {
                BUTTON_STATE = 2;
                action = true;
            }
            else
            {
                BUTTON_STATE = 0;
            }
            break;
        case 2:
            if (pressed)
            {
                BUTTON_STATE = 2;
            }
            else
            {
                BUTTON_STATE = 3;
            }
            break;
        case 3:
            if (pressed)
            {
                BUTTON_STATE = 2;
            }
            else
            {
                BUTTON_STATE = 0;
            }
            break;

        default:
            BUTTON_STATE = 0;
            break;
        }
        PT_YIELD_usec(30000);
    }
    // Indicate thread end
    PT_END(pt);
}

int main()
{

    // Initialize stdio
    stdio_init_all();

    // Initialize the VGA screen
    initVGA();

    // Draw some filled rectangles
    fillRect(64, 0, 176, 50, BLUE);   // blue box
    fillRect(250, 0, 176, 50, RED);   // red box
    fillRect(435, 0, 176, 50, GREEN); // green box

    // Write some text
    setTextColor(WHITE);
    setCursor(65, 0);
    setTextSize(1);
    writeString("Raspberry Pi Pico");
    setCursor(65, 10);
    writeString("Keypad demo");
    setCursor(65, 20);
    writeString("Hunter Adams");
    setCursor(65, 30);
    writeString("vha3@cornell.edu");
    setCursor(250, 0);
    setTextSize(2);
    writeString("Key Pressed:");

    // Map LED to GPIO port, make it low
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN));
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN));
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN));
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4));
    gpio_pull_down((BASE_KEYPAD_PIN + 5));
    gpio_pull_down((BASE_KEYPAD_PIN + 6));

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC);
    gpio_set_dir(LDAC, GPIO_OUT);
    gpio_put(LDAC, 0);

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    // Map LED to GPIO port, make it low
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME));
    decay_inc = divfix(max_amplitude, int2fix15(DECAY_TIME));

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++)
    {
        sin_table[ii] = float2fix15(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
    }

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    // Add core 0 threads
    pt_add_thread(protothread_core_0);

    // Start scheduling core 0 threads
    pt_schedule_start;
}
