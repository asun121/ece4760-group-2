/**
 * V. Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries
#include "vga16_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_3.h"

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// character array
char screentext[40];

// draw speed
int threshold = 10;

// Some macros for max/min/abs
#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a < b) ? b : a)
#define abs(a) ((a > 0) ? a : -a)

// semaphore
static struct pt_sem vga_semaphore;

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV 25.0
uint slice_num;
fix15 accel_angle, gyro_angle_delta, complementary_angle = 0;
fix15 filtered_ay = 0, filtered_az = 0;

fix15 error = 0;
fix15 target_angle = int2fix15(5);

fix15 error_integral = 0;
fix15 dt = float2fix15(0.001);
// change this value later to adjust the maximum integral term
fix15 max_integral = int2fix15(3000); // 5000?
// Rate of change of error
fix15 error_derivative = 0;
fix15 error_previous = 0;

// GPIO we're using for PWM
#define PWM_OUT 4

// Variable to hold PWM slice number
uint slice_num;

// PWM duty cycle
volatile int control=0;
volatile int old_control=0;
int motor_disp = 0;


fix15 kp = float2fix15(80.0);
fix15 ki = float2fix15(20.0);
fix15 kd = float2fix15(50000.0);
fix15 p_term = 0;
fix15 i_term = 0;
fix15 d_term = 0;

// Interrupt service routine
void on_pwm_wrap()
{
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // Accelerometer angle (degrees - 15.16 fixed point)
    // Only ONE of the two lines below will be used, depending whether or not a small angle approximation is appropriate

    // SMALL ANGLE APPROXIMATION
    // accel_angle = multfix15(divfix(acceleration[0], acceleration[1]), oneeightyoverpi);
    // NO SMALL ANGLE APPROXIMATION

    filtered_ay = filtered_ay + ((acceleration[1] - filtered_ay) >> 6);
    filtered_az = filtered_az + ((acceleration[2] - filtered_az) >> 6);
    // ccel_angle = multfix15(float2fix15(atan2(-fix2float15(acceleration[2]), fix2float15(acceleration[1]))), oneeightyoverpi) + int2fix15(180);

    accel_angle = multfix15(float2fix15(atan2(-fix2float15(filtered_az), fix2float15(filtered_ay))), oneeightyoverpi) + int2fix15(180); // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    if (fix2int15(accel_angle) > 270)
    {
        accel_angle = accel_angle - int2fix15(360);
    }
    gyro_angle_delta = multfix15(gyro[0], zeropt001);

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle = multfix15(complementary_angle + gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    error = target_angle - complementary_angle;

    // I-term: Calculate integral term (with anti-windup)
    // if ((control > 0 && fix2int15(error) > 0) || (control < 0 && fix2int15(error) < 0))
    // {
    error_integral = error_integral + multfix15(error, dt);
    // }

    // D-term: Calculate derivative term
    error_derivative = divfix(error_previous - complementary_angle, dt);
    error_previous = complementary_angle; // Store current error for next cycle

    // Calculate complete PID control output
    p_term = multfix15(kp, error);
    i_term = multfix15(ki, error_integral);
    d_term = multfix15(kd, error_derivative);


    // limit the integral term to prevent excessive buildup
    if (i_term > max_integral)
        i_term = max_integral;
    if (i_term < -max_integral)
        i_term = -max_integral;

    // Combine all terms (p, i, d)
    fix15 pid_output = p_term + i_term + d_term;
    // fix15 pid_output = p_term + d_term;

    // control = fix2int15(multfix15(kp, error));
    control = min(max(fix2int15(pid_output), 0), 3000); // Clamp to 0-3000
    // Update duty cycle
    if (control != old_control)
    {
        old_control = control;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    }
    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}


// Thread that draws to VGA display
static PT_THREAD(protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt);

    // We will start drawing at column 81
    static int xcoord = 81;

    // Rescale the measurements for display
    static float OldRange = 500.; // (+/- 250)
    static float NewRange = 150.; // (looks nice on VGA)
    static float OldMin = -250.;
    static float OldMax = 250.;


    // Control rate of drawing
    static int throttle;

    // Draw the static aspects of the display
    setTextSize(1);
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN);
    drawHLine(75, 355, 5, CYAN);
    drawHLine(75, 280, 5, CYAN);
    drawVLine(80, 280, 150, CYAN);
    sprintf(screentext, "2500");
    setCursor(50, 350);
    writeString(screentext);
    sprintf(screentext, "5000");
    setCursor(50, 280);
    writeString(screentext);
    sprintf(screentext, "0");
    setCursor(50, 425);
    writeString(screentext);

    // Draw top plot
    drawHLine(75, 230, 5, CYAN);
    drawHLine(75, 155, 5, CYAN);
    drawHLine(75, 80, 5, CYAN);
    drawVLine(80, 80, 150, CYAN);
    sprintf(screentext, "0");
    setCursor(50, 150);
    writeString(screentext);
    sprintf(screentext, "+250");
    setCursor(45, 75);
    writeString(screentext);
    sprintf(screentext, "-250");
    setCursor(45, 225);
    writeString(screentext);

    while (true)
    {
        // printf("%f\n",fix2float15(complementary_angle));
        //  Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold)
        {
            // Zero drawspeed controller
            throttle = 0;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK);

            // printf("%d, %d\n",control, motor_disp);

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            motor_disp = motor_disp + ((control - motor_disp)>>6) ;


            drawPixel(xcoord, 430 - (int)(NewRange * ((float)(motor_disp) / 5000.0)), WHITE);
            drawPixel(xcoord, 430 - (int)(NewRange * (fix2float15(p_term) / 5000.0)), RED);
            drawPixel(xcoord, 430 - (int)(NewRange * (fix2float15(i_term) / 5000.0)), GREEN);
            drawPixel(xcoord, 430 - (int)(NewRange * (fix2float15(d_term) / 5000.0)), BLUE);
            // printf("%f,%f,%f, %f, %f\n", fix2float15(p_term), fix2float15(i_term), fix2float15(d_term), fix2float15(complementary_angle), fix2float15(error));
            // drawPixel(xcoord, 430 - (int)(NewRange * ((float)((fix2float15(acceleration[0]) * 120.0) - OldMin) / OldRange)), WHITE);
            // drawPixel(xcoord, 430 - (int)(NewRange * ((float)((fix2float15(complementary_angle) * 120.0) - OldMin) / OldRange)), WHITE);

            // drawPixel(xcoord, 430 - (int)(NewRange * ((float)((fix2float15(acceleration[1]) * 120.0) - OldMin) / OldRange)), RED);
            //  drawPixel(xcoord, 430 - (int)(NewRange * ((float)((fix2float15(acceleration[2]) * 120.0) - OldMin) / OldRange)), GREEN);

            // // Draw top plot
            // drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(gyro[0]))-OldMin) / OldRange)), WHITE);

            drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(complementary_angle))-OldMin) / OldRange)), WHITE);
            // drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(gyro[1]))-OldMin) / OldRange)), RED);
            //  drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(gyro[2]))-OldMin) / OldRange)), GREEN);

            // Update horizontal cursor
            if (xcoord < 609)
            {
                xcoord += 1;
            }
            else
            {
                xcoord = 81;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD(protothread_serial(struct pt *pt))
{

    // PT_END(pt);
    PT_BEGIN(pt);
    static int test_in;
    static int angle;
    static float p, i, d;
    ;
    while (1)
    {
        // chose a value to change
        // sprintf(pt_serial_out_buffer, "What do you want to tune? (a, p, i or d): ");
        // serial_write;
        // // spawn a thread to do the non-blocking serial read
        // serial_read;
        // if (pt_serial_in_buffer[0] == 'a')
        // {
        //     // spawn a thread to do the non-blocking serial read
        //     serial_read;
        //     // convert input string to number
        //     sscanf(pt_serial_in_buffer, "%d", &angle);
        //     if (angle > 180)
        //         continue;
        //     else if (angle < 0)
        //         continue;
        //     else
        //         target_angle = int2fix15(angle);
        // }
        // else if (pt_serial_in_buffer[0] == 'p')
        // {
        //     serial_read;
        //     // convert input string to number
        //     sscanf(pt_serial_in_buffer, "%f", &p);
        //     kp = float2fix15(p);
        // }
        // else if (pt_serial_in_buffer[0] == 'i')
        // {
        //     serial_read;
        //     // convert input string to number
        //     sscanf(pt_serial_in_buffer, "%f", &i);
        //     ki = float2fix15(i);
        // }
        // else if (pt_serial_in_buffer[0] == 'd')
        // {
        //     serial_read;
        //     // convert input string to number
        //     sscanf(pt_serial_in_buffer, "%f", &d);
        //     kd = float2fix15(d);
        // }
        // else
        // {
        //     printf("Invalid input. Please enter a valid option.\n");
        //     continue;
        // }
        sprintf(pt_serial_out_buffer, "Choose a target angle (0-180) and P I D: ");
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer, "%d %f %f %f", &angle, &p, &i, &d);
        if (angle > 180)
            continue;
        else if (angle < 0)
            continue;
        else
            target_angle = int2fix15(angle);

        kp = float2fix15(p);
        ki = float2fix15(i);
        kd = float2fix15(d);

        // sprintf(pt_serial_out_buffer, "Chose a P, I, D: ");
        // serial_write;
        // // spawn a thread to do the non-blocking serial read
        // serial_read;
        // // convert input string to number
        // sscanf(pt_serial_in_buffer, "%f %f %f", &p, &i, &d);

        // kp = float2fix15(p);
        // ki = float2fix15(i);
        // kd = float2fix15(d);
    }
    PT_END(pt);
}

// Entry point for core 1
void core1_entry()
{
    pt_add_thread(protothread_vga);
    pt_schedule_start;
}

int main()
{

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // Pullup resistors on breakout board, don't need to turn on internals
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL);
    pwm_set_clkdiv(slice_num, CLKDIV);

    pwm_set_output_polarity(slice_num, 1, 0);

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial);
    pt_schedule_start;
}
