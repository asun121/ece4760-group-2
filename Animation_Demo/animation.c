
/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga16_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/adc.h"

// Include protothreads
#include "pt_cornell_rp2040_v1_3.h"

//=====  DMA Config  ========================================

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size];

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size];

// Pointer to the address of the DAC data table
unsigned short *dac_pointer = &DAC_data[0];

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

// SPI configurations
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;

int data_chan = 0;
int ctrl_chan = 0;

//====================================================
// === the fixed point macros ========================================
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0)) // 2^15
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)(div_s64s64((((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b > int2fix15(480))
#define hitTop(b) (b < int2fix15(0))
#define hitLeft(a) (a < int2fix15(0))
#define hitRight(a) (a > int2fix15(640))

// uS per frame
#define FRAME_RATE 33000

// First Peg location
#define START_X_COORD 320
#define START_Y_COORD 100

#define NUM_ROWS 16
#define NUM_COLS (640 / 38)

#define NUM_BALLS 2000
// Fixed x-position for nozzle
#define BALL_SPAWN_X 320
// Fixed y-position at top
#define BALL_SPAWN_Y 50

#define LED_PIN 25

//=== ===
// Define constants
const int BALL_RADIUS = 1;
const int PEG_RADIUS = 6;

const fix15 collision_dist = int2fix15(BALL_RADIUS + PEG_RADIUS);
const fix15 GRAVITY = float2fix15(0.75);
// const fix15 BOUNCINESS = float2fix15(0.5);
fix15 BOUNCINESS = float2fix15(0.6);

const int HORIZONTAL_SPACING = 38;
const int VERTICAL_SPACING = 19;

// the color of the boid
char color = WHITE;
char peg_color = BLUE;
char balls_color = WHITE;
char hist_color_1 = GREEN;
char hist_color_2 = GREEN;
char text_color = GREEN;
int text_size = 1;

// 0 for balls, 1 for bounciness
int MODE = 0;

// Boid on core 0
fix15 boid0_x;
fix15 boid0_y;
fix15 boid0_vx;
fix15 boid0_vy;

// Peg on core 0
fix15 peg0_x;
fix15 peg0_y;

struct ball
{
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
  bool counted;
};

struct peg
{
  fix15 x;
  fix15 y;
};

static uint32_t last_update_time = 0;
static uint32_t elapsed_time_sec = 0;

struct peg pegs[NUM_ROWS][NUM_COLS];

#define SCREEN_WIDTH 640
#define SCREEN_MIDLINE_X 320

// Arrays to store multiple ball states
fix15 balls_x[NUM_BALLS];
fix15 balls_y[NUM_BALLS];
fix15 balls_vx[NUM_BALLS];
fix15 balls_vy[NUM_BALLS];
bool counted[NUM_BALLS];

#define PEGS_BOTTOM START_Y_COORD + 15 * VERTICAL_SPACING
#define HIST_OFFSET SCREEN_MIDLINE_X - (15 * HORIZONTAL_SPACING) / 2
#define HIST_MAX 50
#define FONT_SIZE 10

fix15 hist_highest = 0;

fix15 hist_data[15] = {int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0), int2fix15(0)};
// fix15 hist_data[15]={int2fix15(10),int2fix15(0),int2fix15(10),int2fix15(0),int2fix15(10),int2fix15(0),int2fix15(10),int2fix15(0),int2fix15(10),int2fix15(0),int2fix15(10),int2fix15(0),int2fix15(10),int2fix15(0),int2fix15(10)};

// ADC conversion, num_Balls is the total balls
const float conversion_factor = (float)(NUM_BALLS) / (1 << 12);
int spawned_balls = 0;
fix15 total_balls = 0;

const float bounce_conversion = 0.0002;

int prev_spawned = 0;

// Create a boid
void spawnBoid(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy)
{
  // Start in center of screen
  *x = int2fix15(BALL_SPAWN_X);
  *y = int2fix15(BALL_SPAWN_Y);
  // Choose left or right
  fix15 temp_vx = (rand() & 0xFFFF) - int2fix15(1);
  *vx = temp_vx;

  *vy = int2fix15(0);
}

void spawnPeg(fix15 *x, fix15 *y)
{
  // Start in center of screen
  *x = int2fix15(320);
  *y = int2fix15(240);
}

// updated spawnpegs func
void spawnPegs()
{
  for (int row = 0; row < NUM_ROWS; row++)
  {
    // Number of pegs in row follows Pascal's Triangle pattern
    // Number of pegs in row increases by 1 each row starting from 1. i.e. 1-2-3-4
    int pegs_in_row = row + 1;
    // Midline of the screen
    int center_x = SCREEN_MIDLINE_X;
    // Symmetric offset
    int row_offset = center_x - ((pegs_in_row - 1) * HORIZONTAL_SPACING) / 2;

    for (int col = 0; col < pegs_in_row; col++)
    {
      // Distribute pegs evenly and maintain spacing
      // direct assignment here is better than having a temp peg struct
      pegs[row][col].x = int2fix15(row_offset + col * HORIZONTAL_SPACING);
      pegs[row][col].y = int2fix15(START_Y_COORD + row * VERTICAL_SPACING);
    }
  }
}

void draw_pegs()
{
  for (int row = 0; row < NUM_ROWS; row++)
  {
    for (int col = 0; col <= row; col++)
    {
      fillCircle(fix2int15(pegs[row][col].x), fix2int15(pegs[row][col].y), PEG_RADIUS, peg_color);
    }
  }
}

// Spawn multiple balls at the top of the screen
void spawnBalls(fix15 x[], fix15 y[], fix15 vx[], fix15 vy[])
{
  for (int i = 0; i < NUM_BALLS; i++)
  {
    // All balls start from the exact same coordinate
    x[i] = int2fix15(BALL_SPAWN_X);
    y[i] = int2fix15(BALL_SPAWN_Y);
    vx[i] = (rand() & 0xFFFF) - int2fix15(1); // Random x-velocity
    vy[i] = int2fix15(0);                     // Zero initial y-velocity
    counted[i] = false;
  }
}

void multiBallsAndPegs(fix15 x[], fix15 y[], fix15 vx[], fix15 vy[])
{
  bool active;
  for (int i = 0; i < NUM_BALLS; i++)
  {
    active = i < spawned_balls;
    // erase ball
    if (active)
    {
      // fillCircle(fix2int15(x[i]), fix2int15(y[i]), BALL_RADIUS, BLACK);
      fillRect(fix2int15(x[i]), fix2int15(y[i]), 2,2, BLACK);
      // Collision detection with pegs
      for (int row = 0; row < NUM_ROWS; row++)
      {
        for (int col = 0; col <= row; col++)
        {
          fix15 dx = x[i] - pegs[row][col].x;
          fix15 dy = y[i] - pegs[row][col].y;
          if (absfix15(dx) < collision_dist && absfix15(dy) < collision_dist)
          {
            //   fillCircle(pegs[row][col].x, pegs[row][col].y, PEG_RADIUS, color);

            // Compute exact Euclidean distance
            fix15 distance = float2fix15(
                sqrt(fix2float15(multfix15(dx, dx) + multfix15(dy, dy))));

            // Avoid division by zero
            if (distance != 0)
            {
              // Compute normal vectors
              fix15 normal_x = divfix(dx, distance);
              fix15 normal_y = divfix(dy, distance);

              // Compute intermediate term for collision response
              fix15 intermediate_term = multfix15(
                  int2fix15(-2),
                  multfix15(normal_x, vx[i]) + multfix15(normal_y, vy[i]));

              // Only reflect if moving toward peg
              if (intermediate_term > 0)
              {
                // Teleport ball outside collision distance
                x[i] = pegs[row][col].x + multfix15(normal_x, distance + int2fix15(1));
                y[i] = pegs[row][col].y + multfix15(normal_y, distance + int2fix15(1));

                // Update velocity
                vx[i] = vx[i] + multfix15(normal_x, intermediate_term);
                vy[i] = vy[i] + multfix15(normal_y, intermediate_term);

                // Apply bounciness factor
                vx[i] = multfix15(vx[i], BOUNCINESS);
                vy[i] = multfix15(vy[i], BOUNCINESS);

                // Play sound effect
                dma_start_channel_mask(1u << ctrl_chan);
              }
            }
          }
        }
      }

      if (!counted[i] && y[i] > int2fix15(PEGS_BOTTOM))
      {
        fix15 hi = ((x[i] - int2fix15(HIST_OFFSET)) / int2fix15(HORIZONTAL_SPACING));
        // int hi = (x[i]) / HORIZONTAL_SPACING;
        //  printf("%d\n",hi);
        if (hi > -1 && hi < 15)
        {
          hist_data[hi]++;
          if (hist_data[hi] > hist_highest)
            hist_highest = hist_data[hi];
        }

        counted[i] = true;
      }

      // Handle wall collisions
      if (hitBottom(y[i]))
      {
        counted[i] = false;
        total_balls++;
        // Respawn at top
        x[i] = int2fix15(BALL_SPAWN_X);
        y[i] = int2fix15(BALL_SPAWN_Y);
        // vx[i] = float2fix15((float)(rand() % 200 - 100) / 100.0);
        vx[i] = (rand() & 0xFFFF) - int2fix15(1);

        vy[i] = int2fix15(0);
      }
      else
      {
        if (hitTop(y[i]))
        {
          vy[i] = -vy[i];
          y[i] = y[i] + int2fix15(5);
        }
        if (hitLeft(x[i]))
        {
          vx[i] = -vx[i];
          x[i] = x[i] + int2fix15(5);
        }
        if (hitRight(x[i]))
        {
          vx[i] = -vx[i];
          x[i] = x[i] - int2fix15(5);
        }
      }
      // apply gravity
      vy[i] = vy[i] + GRAVITY;
      // Update position
      x[i] = x[i] + vx[i];
      y[i] = y[i] + vy[i];
      //fillCircle(fix2int15(x[i]), fix2int15(y[i]), BALL_RADIUS, balls_color);
      fillRect(fix2int15(x[i]), fix2int15(y[i]), 2,2, balls_color);

    }
  }
}

void showHist()
{
  for (int i = 0; i < 15; i++)
  {
    fix15 h = HIST_MAX * fix2float15(hist_data[i]) / fix2float15(hist_highest);

    fillRect(HIST_OFFSET + i * HORIZONTAL_SPACING + 2, 480 - h, HORIZONTAL_SPACING - 2, h, hist_color_1);
    fillRect(HIST_OFFSET + i * HORIZONTAL_SPACING + 2, 480 - HIST_MAX, HORIZONTAL_SPACING - 2, HIST_MAX - h, BLACK);
  }
}

void eraseAllBalls()
{
  for (int i = 0; i < NUM_BALLS; i++)
  {
    // fillCircle(balls_x[i],balls_y[i],BALL_RADIUS,BLACK);
    fillCircle(fix2int15(balls_x[i]), fix2int15(balls_y[i]), BALL_RADIUS, BLACK);
  }
}

void changeSpawnedNum()
{
  if (spawned_balls != prev_spawned)
  {
    if (spawned_balls < prev_spawned)
    {
      // fillRect(0,0,SCREEN_WIDTH, 480,BLACK);
      eraseAllBalls();
      prev_spawned = spawned_balls;
    }
    else if (spawned_balls > prev_spawned)
    {
      for (int i = prev_spawned; i < spawned_balls; i++)
      {
        counted[i] = false;
        // Respawn at top
        balls_x[i] = int2fix15(BALL_SPAWN_X);
        balls_y[i] = int2fix15(BALL_SPAWN_Y);
        // vx[i] = float2fix15((float)(rand() % 200 - 100) / 100.0);
        balls_vx[i] = (rand() & 0xFFFF) - int2fix15(1);

        balls_vy[i] = int2fix15(0);
      }
    }
    prev_spawned = spawned_balls;
  }
}

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;

  // Spawn multiple balls
  spawnBalls(balls_x, balls_y, balls_vx, balls_vy);
  // Spawn and draw our pascal triangle layout pegs
  spawnPegs();
  draw_pegs();
  // Set text attributes
  setTextColor2(text_color, BLACK);
  setTextSize(text_size);

  char temp[50];

  static uint32_t frames = 0;

  bool last_button_state = false; // Track previous button state

  while (1)
  {
    frames++;

    
    // Measure time at start of thread
    begin_time = time_us_32();

    changeSpawnedNum();

    // // handle multiple balls interaction with pegs
    multiBallsAndPegs(balls_x, balls_y, balls_vx, balls_vy);
    // showHist();

    // TEXT START

    // Covers old text with a black rectangle
    // fillRect(0, 0, 200, 40, BLACK);

    setCursor(5, 5);
    sprintf(temp, "Total Active Balls: %d     ", spawned_balls);
    writeString(temp);

    setCursor(5, 15);
    sprintf(temp, "Total Fallen Balls: %d     ", total_balls);
    writeString(temp);

    // Update once per second
    if (time_us_32() - last_update_time >= 1000000)
    {
      elapsed_time_sec++;
      last_update_time = time_us_32();
    }

    setCursor(5, 25);
    sprintf(temp, "Time (since boot):  %d", frames);
    writeString(temp);

    setCursor(5, 35);
    sprintf(temp, "Bounciness :  %d", BOUNCINESS);
    writeString(temp);

    // TEXT END

    // Check ADC value
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    // if(MODE == 0)
    // {
    spawned_balls = spawned_balls + (((int)(adc_read() * conversion_factor) - (int)spawned_balls) >> 6);
    // }
    // else if(MODE == 1)
    // {
    //   BOUNCINESS = float2fix15(adc_read() * bounce_conversion);
    // }

    // bool button_state = gpio_get(14);  // Read button state

    // Detect button press (rising edge)
    // if (button_state && !last_button_state) {
    //     MODE = !MODE;  // Toggle mode
    // }

    // last_button_state = button_state;  // Update last button state

    // delay in accordance with frame rate
    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    if (spare_time < 0)
    {
      gpio_put(LED_PIN, 1);
    }
    else
    {
      gpio_put(LED_PIN, 0);
    }
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

// Animation on core 1
static PT_THREAD(protothread_anim1(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);
  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;

  while (1)
  {
    // Measure time at start of thread
    begin_time = time_us_32();
    // erase boid
    showHist();
    draw_pegs();

    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

void core1_main()
{
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start;
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
  // initialize stio
  stdio_init_all();

  // Set up the button as an input with an internal pull-down resistor
  gpio_init(14);
  gpio_set_dir(14, GPIO_IN);
  gpio_pull_down(14);

  // initialize VGA
  initVGA();

  //=============DMA

  // Initialize SPI channel (channel, baud rate set to 20MHz)
  spi_init(SPI_PORT, 20000000);

  // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Initialize the LED pin
  gpio_init(LED_PIN);
  // Configure the LED pin as an output
  gpio_set_dir(LED_PIN, GPIO_OUT);

  // Build sine table and DAC data table
  int i;
  for (i = 0; i < (sine_table_size); i++)
  {
    raw_sin[i] = (int)(2047 * sin((float)i * 6.283 / (float)sine_table_size) + 2047); // 12 bit
    DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff);
  }

  // Select DMA channels
  data_chan = dma_claim_unused_channel(true);
  ;
  ctrl_chan = dma_claim_unused_channel(true);
  ;

  // Setup the control channel
  dma_channel_config c = dma_channel_get_default_config(ctrl_chan); // default configs
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);           // 32-bit txfers
  channel_config_set_read_increment(&c, false);                     // no read incrementing
  channel_config_set_write_increment(&c, false);                    // no write incrementing
  channel_config_set_chain_to(&c, data_chan);                       // chain to data channel

  dma_channel_configure(
      ctrl_chan,                        // Channel to be configured
      &c,                               // The configuration we just created
      &dma_hw->ch[data_chan].read_addr, // Write address (data channel read address)
      &dac_pointer,                     // Read address (POINTER TO AN ADDRESS)
      1,                                // Number of transfers
      false                             // Don't start immediately
  );

  // Setup the data channel
  dma_channel_config c2 = dma_channel_get_default_config(data_chan); // Default configs
  channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);           // 16-bit txfers
  channel_config_set_read_increment(&c2, true);                      // yes read incrementing
  channel_config_set_write_increment(&c2, false);                    // no write incrementing
  // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
  // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
  dma_timer_set_fraction(0, 0x0017, 0xffff);
  // 0x3b means timer0 (see SDK manual)
  channel_config_set_dreq(&c2, 0x3b); // DREQ paced by timer 0
  // chain to the controller DMA channel
  // channel_config_set_chain_to(&c2, ctrl_chan);                        // Chain to control channel

  dma_channel_configure(
      data_chan,                 // Channel to be configured
      &c2,                       // The configuration we just created
      &spi_get_hw(SPI_PORT)->dr, // write address (SPI data register)
      DAC_data,                  // The initial read address
      sine_table_size,           // Number of transfers
      false                      // Don't start immediately.
  );

  // dma_start_channel_mask(1u << ctrl_chan) ;

  //================

  //============= ADC ============================
  adc_init();

  // Make sure GPIO is high-impedance, no pullups etc
  adc_gpio_init(26);

  // Select ADC input 0 (GPIO26)
  adc_select_input(0);

  //===================================================================

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_anim1);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start;
}