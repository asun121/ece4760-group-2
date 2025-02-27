
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
// Include protothreads
#include "pt_cornell_rp2040_v1_3.h"

//=====  DMA Config  ========================================

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size] ;

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size] ;

// Pointer to the address of the DAC data table
unsigned short * dac_pointer = &DAC_data[0] ;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000

//SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size ;

int data_chan = 0;
int ctrl_chan = 0;

//====================================================
// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b>int2fix15(480))
#define hitTop(b) (b<int2fix15(0))
#define hitLeft(a) (a<int2fix15(0))
#define hitRight(a) (a>int2fix15(640))

// uS per frame
#define FRAME_RATE 33000

//First Peg location
#define START_X_COORD 320
#define START_Y_COORD 100

//=== ===
// Define constants
const int BALL_RADIUS = 4;
const int PEG_RADIUS = 6;

const int PEG_DEPTH = 16;

const fix15 collision_dist = int2fix15(BALL_RADIUS + PEG_RADIUS);
const fix15 GRAVITY = float2fix15(0.75);  
const fix15 BOUNCINESS = float2fix15(0.5); 

const int HORIZONTAL_SPACING = 38;
const int VERTICAL_SPACING = 19;

// the color of the boid
char color = WHITE ;

// Boid on core 0
fix15 boid0_x ;
fix15 boid0_y ;
fix15 boid0_vx ;
fix15 boid0_vy ;

// Peg on core 0
fix15 peg0_x ;
fix15 peg0_y ;

struct ball
{
  fix15 x;
  fix15 y;
  fix15 vx;
  fix15 vy;
};

struct peg
{
  fix15 x;
  fix15 y;
};

void spawnBall(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(50) ;
  // Choose left or right
  fix15 temp_vx = float2fix15((float)(rand() % 200 - 100) / (float)100);
  if(temp_vx == 0)
  {
    temp_vx = 100002;
  }
  //printf("temp_vx = %d\n", fix2float15(temp_vx));
  *vx = temp_vx;
  //*vx = int2fix15(0);
  // Moving down
  *vy = int2fix15(0) ;
}

// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(130) ;
  // Choose left or right
  fix15 temp_vx = float2fix15((float)(rand() % 200 - 100) / (float)100);
  if(temp_vx == 0)
  {
    temp_vx = 100002;
  }
  //printf("temp_vx = %d\n", fix2float15(temp_vx));
  *vx = temp_vx;
  //*vx = int2fix15(0);
  // Moving down
  *vy = int2fix15(0) ;
}

void spawnPeg(fix15* x, fix15* y)
{
  // Start in center of screen
  *x = int2fix15(320) ;
  *y = int2fix15(240) ;
}

#define NUM_ROWS PEG_DEPTH
#define NUM_COLS (640 / HORIZONTAL_SPACING)

struct peg pegs[NUM_ROWS][NUM_COLS];

#define SCREEN_WIDTH 640 
#define SCREEN_MIDLINE_X 320

// updated spawnpegs func 
void spawnPegs() {
    for (int row = 0; row < NUM_ROWS; row++) {
        // Number of pegs in row follows Pascal's Triangle pattern
        // Number of pegs in row increases by 1 each row starting from 1. i.e. 1-2-3-4 
        int pegs_in_row = row + 1; 
        // Midline of the screen
        int center_x = SCREEN_MIDLINE_X; 
        // Symmetric offset
        int row_offset = center_x - ((pegs_in_row - 1) * HORIZONTAL_SPACING) / 2; 

        for (int col = 0; col < pegs_in_row; col++) {
            // Distribute pegs evenly and maintain spacing
            // direct assignment here is better than having a temp peg struct
            pegs[row][col].x = int2fix15(row_offset + col * HORIZONTAL_SPACING);
            pegs[row][col].y = int2fix15(START_Y_COORD + row * VERTICAL_SPACING);
        }
    }
}

#define NUM_BALLS 10  
// Fixed x-position for nozzle
#define BALL_SPAWN_X 320  
// Fixed y-position at top
#define BALL_SPAWN_Y 50   

// Arrays to store multiple ball states
fix15 balls_x[NUM_BALLS];
fix15 balls_y[NUM_BALLS];
fix15 balls_vx[NUM_BALLS];
fix15 balls_vy[NUM_BALLS];

// Spawn multiple balls at the top of the screen
void spawnBalls(fix15 x[], fix15 y[], fix15 vx[], fix15 vy[]) {
    for (int i = 0; i < NUM_BALLS; i++) {
        // All balls start from the exact same coordinate
        x[i] = int2fix15(BALL_SPAWN_X);  
        y[i] = int2fix15(BALL_SPAWN_Y);  
        vx[i] = float2fix15((float)(rand() % 200 - 100) / (float)100);  // Small random x-velocity
        vy[i] = int2fix15(0);  // Zero initial y-velocity
    }
}


// Added ball-multipeg collsions in the same frame 
void multiBallsAndPegs2(fix15 x[], fix15 y[], fix15 vx[], fix15 vy[]) {
    for (int i = 0; i < NUM_BALLS; i++) {
        vy[i] = vy[i] + GRAVITY; 
        x[i] = x[i] + vx[i];
        y[i] = y[i] + vy[i];

        // Track closest peg collision per frame
        fix15 min_distance = int2fix15(99999);  
        int closest_row = -1, closest_col = -1;

        // Collision detection with pegs
        for (int row = 0; row < NUM_ROWS; row++) {
            for (int col = 0; col <= row; col++) {
                fix15 dx = x[i] - pegs[row][col].x;
                fix15 dy = y[i] - pegs[row][col].y;
                fix15 collision_dist = int2fix15(BALL_RADIUS + PEG_RADIUS);
                
                // Compute exact Euclidean distance
                fix15 distance = float2fix15(
                    sqrt(fix2float15(multfix15(dx, dx) + multfix15(dy, dy)))
                );
                
                // Avoid division by zero and track closest peg
                if (distance != 0 && distance < collision_dist && distance < min_distance) {
                    min_distance = distance;
                    closest_row = row;
                    closest_col = col;
                }
            }
        }
        
        // Process only the closest peg collision
        if (closest_row != -1 && closest_col != -1) {
            fix15 dx = x[i] - pegs[closest_row][closest_col].x;
            fix15 dy = y[i] - pegs[closest_row][closest_col].y;
            fix15 distance = min_distance;
            fix15 normal_x = divfix(dx, distance);
            fix15 normal_y = divfix(dy, distance);
            fix15 intermediate_term = multfix15(
                int2fix15(-2),
                multfix15(normal_x, vx[i]) + multfix15(normal_y, vy[i])
            );
            
            if (intermediate_term > 0) {
                x[i] = pegs[closest_row][closest_col].x + multfix15(normal_x, distance + int2fix15(1));
                y[i] = pegs[closest_row][closest_col].y + multfix15(normal_y, distance + int2fix15(1));
                vx[i] = vx[i] + multfix15(normal_x, intermediate_term);
                vy[i] = vy[i] + multfix15(normal_y, intermediate_term);
                vx[i] = multfix15(vx[i], BOUNCINESS);
                vy[i] = multfix15(vy[i], BOUNCINESS);
                dma_start_channel_mask(1u << ctrl_chan);
            }
        }
        
        // Handle wall collisions
        if (hitBottom(y[i])) {
            x[i] = int2fix15(BALL_SPAWN_X);
            y[i] = int2fix15(BALL_SPAWN_Y);
            vx[i] = float2fix15((float)(rand() % 200 - 100) / 100.0);
            vy[i] = int2fix15(0);
        } else {
            if (hitTop(y[i])) {
                vy[i] = -vy[i];
                y[i] = y[i] + int2fix15(5);
            }
            if (hitLeft(x[i])) {
                vx[i] = -vx[i];
                x[i] = x[i] + int2fix15(5);
            }
            if (hitRight(x[i])) {
                vx[i] = -vx[i];
                x[i] = x[i] - int2fix15(5);
            }
        }
    }
}


void multiBallsAndPegs(fix15 x[], fix15 y[], fix15 vx[], fix15 vy[]) {
    for (int i = 0; i < NUM_BALLS; i++) {
        // apply gravity
        vy[i] = vy[i] + GRAVITY;  
        // Update position
        x[i] = x[i] + vx[i];
        y[i] = y[i] + vy[i];

        // Collision detection with pegs
        for (int row = 0; row < NUM_ROWS; row++) {
            for (int col = 0; col <= row; col++) {
                fix15 dx = x[i] - pegs[row][col].x;
                fix15 dy = y[i] - pegs[row][col].y;
                
                // Compute exact Euclidean distance
                fix15 distance = float2fix15(
                    sqrt(fix2float15(multfix15(dx, dx) + multfix15(dy, dy)))
                );
                
                // Avoid division by zero
                if (distance != 0 && distance < collision_dist) {
                    // Compute normal vectors
                    fix15 normal_x = divfix(dx, distance);
                    fix15 normal_y = divfix(dy, distance);
                    
                    // Compute intermediate term for collision response
                    fix15 intermediate_term = multfix15(
                        int2fix15(-2),
                        multfix15(normal_x, vx[i]) + multfix15(normal_y, vy[i])
                    );
                    
                    // Only reflect if moving toward peg
                    if (intermediate_term > 0) {
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
        
        // Handle wall collisions
        if (hitBottom(y[i])) {
            // Respawn at top
            x[i] = int2fix15(BALL_SPAWN_X);
            y[i] = int2fix15(BALL_SPAWN_Y);
            vx[i] = float2fix15((float)(rand() % 200 - 100) / 100.0);
            vy[i] = int2fix15(0);
        } else {
            if (hitTop(y[i])) {
                vy[i] = -vy[i];
                y[i] = y[i] + int2fix15(5);
            }
            if (hitLeft(x[i])) {
                vx[i] = -vx[i];
                x[i] = x[i] + int2fix15(5);
            }
            if (hitRight(x[i])) {
                vx[i] = -vx[i];
                x[i] = x[i] - int2fix15(5);
            }
        }
    }
}


void ballsAndPegs(fix15* ball_x, fix15* ball_y, fix15* peg_x, fix15* peg_y, fix15* ball_vx, fix15* ball_vy) 
{
  
    // Calculate distances between ball and peg
    fix15 dx = *ball_x - *peg_x;
    fix15 dy = *ball_y - *peg_y;
    
    // Check collision (using fixed-point math)
    if (absfix15(dx) < collision_dist && absfix15(dy) < collision_dist) {
      // Distance between ball and peg
      fix15 distance = float2fix15(
          sqrt(fix2float15(multfix15(dx, dx) + multfix15(dy, dy))));
        // Avoid division by zero
        if (distance != 0) {
            // Compute normal vectors from peg to ball 
            fix15 normal_x = divfix(dx, distance);
            fix15 normal_y = divfix(dy, distance);
            
            // Compute intermediate term for collision physics
            fix15 intermediate_term = multfix15(
                int2fix15(-2),
                multfix15(normal_x, *ball_vx) + multfix15(normal_y, *ball_vy)
            );
            
            // Check if vectors are in opposite directions
            if (intermediate_term > 0) {
                // Teleport ball outside collision distance
                *ball_x = *peg_x + multfix15(normal_x, distance + int2fix15(1));
                *ball_y = *peg_y + multfix15(normal_y, distance + int2fix15(1));
                // Update velocity
                *ball_vx = *ball_vx + multfix15(normal_x, intermediate_term);
                *ball_vy = *ball_vy + multfix15(normal_y, intermediate_term);

                dma_start_channel_mask(1u << ctrl_chan) ;

                // Apply bounciness factor
                *ball_vx = multfix15(*ball_vx, BOUNCINESS);
                *ball_vy = multfix15(*ball_vy, BOUNCINESS);
            }
        }
    }
    
    // Handle wall collisions
    if (hitBottom(*ball_y)) {
        // Respawn at top
        spawnBoid(ball_x, ball_y, ball_vx, ball_vy);
    } 
    else {
        // Handle other wall collisions
        if (hitTop(*ball_y)) {
            *ball_vy = -(*ball_vy);
            *ball_y = *ball_y + int2fix15(5);
        }
        if (hitLeft(*ball_x)) {
            *ball_vx = -(*ball_vx);
            *ball_x = *ball_x + int2fix15(5);
        }
        if (hitRight(*ball_x)) {
            *ball_vx = -(*ball_vx);
            *ball_x = *ball_x - int2fix15(5);
        }
    }
    
    // Apply gravity
    *ball_vy = *ball_vy + GRAVITY;
    
    // Update position
    *ball_x = *ball_x + *ball_vx;
    *ball_y = *ball_y + *ball_vy;
}

// ==================================================
// === users serial input thread
// ==================================================
// static PT_THREAD (protothread_serial(struct pt *pt))
// {
//     PT_BEGIN(pt);
//     // stores user input
//     static int user_input ;
//     // wait for 0.1 sec
//     PT_YIELD_usec(1000000) ;
//     // announce the threader version
//     sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
//     // non-blocking write
//     serial_write ;
//       while(1) {
//         // print prompt
//         sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
//         // non-blocking write
//         serial_write ;
//         // spawn a thread to do the non-blocking serial read
//         serial_read ;
//         // convert input string to number
//         sscanf(pt_serial_in_buffer,"%d", &user_input) ;
//         // update boid color
//         if ((user_input > 0) && (user_input < 16)) {
//           color = (char)user_input ;
//         }
//       } // END WHILE(1)
//   PT_END(pt);
// } // timer thread


// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;

    // Spawn a boid
    spawnBoid(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy);
    spawnPeg(&peg0_x, &peg0_y);

    // Uncomment for Multiball case with triangular layout
    // // Spawn multiple balls 
    // spawnBalls(balls_x, balls_y, balls_vx, balls_vy);
    // // Spawn our pascal triangle layout pegs 
    // spawnPegs();

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      // erase boid
      // drawRect(fix2int15(boid0_x), fix2int15(boid0_y), 2, 2, BLACK);
      fillCircle(fix2int15(boid0_x), fix2int15(boid0_y), BALL_RADIUS, BLACK);
      fillCircle(fix2int15(peg0_x), fix2int15(peg0_y), PEG_RADIUS, color);


      // update boid's position and velocity
      ballsAndPegs(&boid0_x, &boid0_y, &peg0_x, &peg0_y, &boid0_vx, &boid0_vy);

      // // handle multiple balls interaction with pegs
      // multiBallsAndPegs(balls_x, balls_y, balls_vx, balls_vy);

      // draw the boid at its new position
      fillCircle(fix2int15(boid0_x), fix2int15(boid0_y), BALL_RADIUS, color);


      // // draw the boundaries
      // drawArena() ;
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;


//=============DMA

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Build sine table and DAC data table
    int i ;
    for (i=0; i<(sine_table_size); i++){
        raw_sin[i] = (int)(2047 * sin((float)i*6.283/(float)sine_table_size) + 2047); //12 bit
        DAC_data[i] = DAC_config_chan_A | (raw_sin[i] & 0x0fff) ;
    }

    // Select DMA channels
    data_chan = dma_claim_unused_channel(true);;
    ctrl_chan = dma_claim_unused_channel(true);;

    // Setup the control channel
    dma_channel_config c = dma_channel_get_default_config(ctrl_chan);   // default configs
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);             // 32-bit txfers
    channel_config_set_read_increment(&c, false);                       // no read incrementing
    channel_config_set_write_increment(&c, false);                      // no write incrementing
    channel_config_set_chain_to(&c, data_chan);                         // chain to data channel

    dma_channel_configure(
        ctrl_chan,                          // Channel to be configured
        &c,                                 // The configuration we just created
        &dma_hw->ch[data_chan].read_addr,   // Write address (data channel read address)
        &dac_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers
        false                               // Don't start immediately
    );

    // Setup the data channel
    dma_channel_config c2 = dma_channel_get_default_config(data_chan);  // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);            // 16-bit txfers
    channel_config_set_read_increment(&c2, true);                       // yes read incrementing
    channel_config_set_write_increment(&c2, false);                     // no write incrementing
    // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
    // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
    dma_timer_set_fraction(0, 0x0017, 0xffff) ;
    // 0x3b means timer0 (see SDK manual)
    channel_config_set_dreq(&c2, 0x3b);                                 // DREQ paced by timer 0
    // chain to the controller DMA channel
    channel_config_set_chain_to(&c2, ctrl_chan);                        // Chain to control channel


    dma_channel_configure(
        data_chan,                  // Channel to be configured
        &c2,                        // The configuration we just created
        &spi_get_hw(SPI_PORT)->dr,  // write address (SPI data register)
        DAC_data,                   // The initial read address
        sine_table_size,            // Number of transfers
        false                       // Don't start immediately.
    );

   //dma_start_channel_mask(1u << ctrl_chan) ;




//================



  // add threads
  //pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
