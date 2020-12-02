/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "TM1637.h"
#include "rotary_encoder.h"
#include "Font_7Seg.h"
#include "Timer.h"
#include "ModbusMaster.h"
#include <analogin_api.h>
#include <cstdint>
#include <limits>
#include <cmath>
#include <Ticker.h>
#include <gpio_api.h>

#define FLASH_LOCATION MBED_ROM_START+MBED_ROM_SIZE-0x10000 
#define SECTOR_SIZE 2048
#define BUTTON_DEBOUNCE_MILLISECONDS 3
#define ROTARY_CLICK_ADJUSTMENT 20
#define SPINDLE_MAX_RPM 3000
#define MODBUS_BAUD_RATE 115200
#define SERVO_MOTOR_DRIVER_ID 1
#define MOTOR_SPINDLE_SPEED_REGISTER 169

enum SpindleDirection
{
    STOPPED_DIRECTION = 0,
    FORWARD_DIRECTION,
    REVERSE_DIRECTION
};

DigitalOut led1(LED1);

// Right rotary switch
//DigitalIn rotary_button_right(PC_0); //SW
DigitalIn rotary_line_right_low_1st_cw(PC_2); //CLK
DigitalIn rotary_line_right_low_2nd_cw(PC_1); //DT

// Left rotary switch
//DigitalIn rotary_button_left(PC_9); //SW
DigitalIn rotary_line_left_low_1st_cw(PB_8); //CLK
DigitalIn rotary_line_left_low_2nd_cw(PB_9); //DT

//DigitalIn user_button(USER_BUTTON);
DigitalIn stop_button(PC_4); // From E switch
DigitalIn start_button(PF_1); // From LED momentary switch
DigitalIn forward_button(PC_8); // From lathe down on bar, doesn't exist on grinder
DigitalIn reverse_button(PC_6); // From lathe up on bar, doesn't exist on grinder

// PB_10 ARM TX pin to 3.3v side of 3.3 to 5v level shifter 5v from shifter to TX pin TTL
// PB_11 ARM RX pin to 3.3v side of 3.3 to 5v level shifter 5v from shifter to RX pin TT
/* 
   RS232 soldering instructions
   PB_10 goes to 3.3LV -> 5v pin goes to RX pin on TTL chip
   PB_11 goes to 3.3LV -> 5v pin goes to TX pin on TTL chip
   RX pin on RS232 chip goes to TX pin on PS/2 connector
   TX pin on RS232 chip goes to RX pin on PS/2 connector
 */
// PB_14 to 3.3v to 5v to CLK pin on LED display
// PB_13 to 3.3v to 5v to DIO on LED display

typedef void (*ButtonDownCallback)();
void rotary_switch_init(RotarySwitchState& currentState);
RotarySwitchState rotary_switch_state(const RotarySwitchState switchState, DigitalIn low_first_cw, DigitalIn low_second_cw);
void rotary_switch_task(RotarySwitchState& currentState, int16_t& spindleRpm, int16_t increment_amount, DigitalIn low_first_cw, DigitalIn low_second_cw);
void reverse_button_task(ModbusMaster& servoBus);
void spindle_display_init(TM1637& display, int16_t spindleRpm);
void spindle_display_task(TM1637& display, int16_t spindleRpm, int16_t previousSpindleRpm);
void modbus_init(const uint8_t servoMotorDriverSlaveId, UnbufferedSerial& servoSerial, ModbusMaster& servoBus);
void speed_adjust_task(ModbusMaster& modbus, int16_t spindleRpm, int16_t previousSpindleRpm);
void flash_init(FlashIAP& flashDevice, int16_t& spindleRpm);
void flash_write(FlashIAP& flashDevice, int16_t spindleRpm);
void send_servo_speed(ModbusMaster& servoBus, int16_t spindleRpm);
int16_t read_saved_spindle_speed(FlashIAP& flash);
int16_t read_servo_speed(ModbusMaster& servoBus);
void init_lathe_buttons();
void init_grinder_buttons();
void stop_button_task(ModbusMaster& servoBus);
void lathe_start_button_task(FlashIAP& flash, ModbusMaster& servoBus, int16_t spindleRpm);
void grinder_start_button_task(FlashIAP& flash, ModbusMaster& servoBus, int16_t spindleRpm);
void restore_spindle_speed(ModbusMaster& servoBus);
void forward_button_task(ModbusMaster& servoBus, int16_t spindleRpm);
void reverse_button_task(ModbusMaster& servoBus, int16_t spindleRpm);
void movement_task(ModbusMaster& servoBus, int16_t spindleRpm);
void debounce_delay(int delayMilliseconds);

SpindleDirection spindle_direction(int16_t spindleRpm);
int16_t spindle_forward_direction_polarity();
int16_t spindle_reverse_direction_polarity();
int16_t grinder_direction_polarity();
void grinder_head_speed_encoder_switch_task();

static RotarySwitchState left_rotary_switch_state = ROTARY_STATE_INIT;
static RotarySwitchState right_rotary_switch_state = ROTARY_STATE_INIT;

ButtonState g_startButtonState = ButtonState_UP;
ButtonState g_stopButtonState = ButtonState_UP;
ButtonState g_forwardButtonState = ButtonState_UP;
ButtonState g_reverseButtonState = ButtonState_UP;

int16_t g_lastSentSpindleRpm = 0;
int16_t g_rotarySpindleRpm = 0;
bool g_stopped = true;

int lathe_main();
int grinder_main();

#define WAIT_TIME_MS 50 
// max 5 revolutions per second to go from bottom to top in about 60 seconds
#define MAX_PULSES_PER_SECOND 3000*4
#define PULSES_PER_REVOLUTION 2000
#define INCREMENT_RATE_IN_MICROSECONDS 1000
#define PULSES_PER_HALF_TEN_THOUSANDTH 2
#define PULSES_PER_ONE_THOUSANDTH 40
#define VOLTAGE_DIFF 1000


uint32_t steps_per_second_speed_curve(uint32_t desired_speed, int time_increment);
void increase_pwm();
void decrease_pwm();

static volatile uint32_t s_desired_steps_per_second = 0;
static volatile uint32_t s_current_steps_per_second = 0;
static volatile uint32_t s_starting_speed = 0;
static unsigned int s_last_speed_percent = 0;
static volatile int s_speed_increment = -20;
static volatile bool s_speed_ticker_running = false;
static int32_t s_last_voltage = 0;
static Ticker s_speed_ticker;  
static const unsigned int one_percent = 418;

// Bottom rotary switch
static DigitalIn s_rotary_line_bottom_low_1st_cw(PA_0); //CLK
static DigitalIn s_rotary_line_bottom_low_2nd_cw(PA_1); //DT

// Top rotary switch
static DigitalIn s_rotary_line_top_low_1st_cw(PA_10); //CLK
static DigitalIn s_rotary_line_top_low_2nd_cw(PC_0); //DT

static ButtonState s_up_button_state = ButtonState_UP;
static DigitalIn s_up_button(PA_5); 
static ButtonState s_down_button_state = ButtonState_UP;
static DigitalIn s_down_button(PA_6); 

static DigitalOut s_head_enable_pin(PA_4);
static DigitalOut s_head_direction_pin(PA_9);
static DigitalOut s_head_motor_rpm_pin(PWM_OUT);
static PwmOut s_pwm(PWM_OUT); // PB_4

static RotarySwitchState s_top_rotary_encoder_state = ROTARY_STATE_INIT;
static RotarySwitchState s_bottom_rotary_encoder_state = ROTARY_STATE_INIT;

void move_grinder_head_task(analogin_t& speed_control);
void move_motor_head(analogin_t& speed_control);
void pulse_motor(bool direction_up, unsigned int number_of_pulses, DigitalOut&& pin);
void set_pwm_high();
void init_pwm();
void stop_motor_head();
void set_motor_head_up();
void set_motor_head_down();
void enable_motor_head();
void disable_motor_head();
void motor_head_button_up();
void grinder_head_up_down_rotary_switches_task();

#define NUM_SAMPLES 40
static int32_t s_voltages[NUM_SAMPLES];
static int32_t s_write_index = 0;
static int32_t s_voltage_average = 0;
static bool s_pwm_suspended = true;

void add_voltage(uint32_t sample)
{
    if (s_write_index >= NUM_SAMPLES)
    {
        s_write_index = 0;
    }
    s_voltages[s_write_index++] = sample;

    int32_t average = 0;
    for(int32_t i=0; i<NUM_SAMPLES; ++i)
    {
        average += s_voltages[i];
    }
    s_voltage_average = average/NUM_SAMPLES;
}

int main()
{
    //lathe_main();
    grinder_main();
}

int lathe_main()
{
    printf("Lathe controller running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
    
    FlashIAP flash;
    flash_init(flash, g_rotarySpindleRpm);
    printf("Stored spindle speed: %u\n", g_rotarySpindleRpm);

    RotarySwitchState currentState;
    rotary_switch_init(currentState);

    UnbufferedSerial servoSerial(PB_10, PB_11, MODBUS_BAUD_RATE);
    servoSerial.format(8, SerialBase::None, 2); 
    ModbusMaster servoBus;
    modbus_init(SERVO_MOTOR_DRIVER_ID, servoSerial, servoBus);

    init_lathe_buttons();
    restore_spindle_speed(servoBus);

    TM1637 ledDisplay(PB_13 /* DIO*/, PB_14 /* CLK */); // Bit banging
    spindle_display_init(ledDisplay, g_rotarySpindleRpm);

    while (true)
    {
        stop_button_task(servoBus);
        lathe_start_button_task(flash, servoBus, g_rotarySpindleRpm);
        rotary_switch_task(currentState, rotary_line_right_low_1st_cw, rotary_line_right_low_2nd_cw, 
                           []()
                           {
                                g_rotarySpindleRpm += 5;
                                if (g_rotarySpindleRpm > SPINDLE_MAX_RPM)
                                {
                                    g_rotarySpindleRpm = SPINDLE_MAX_RPM;
                                }
                           },
                           []()
                           {
                                if (g_rotarySpindleRpm < 5)
                                {
                                    g_rotarySpindleRpm = 0;
                                }
                                else
                                {
                                    g_rotarySpindleRpm -= 5;
                                }
                           });
        rotary_switch_task(currentState, rotary_line_left_low_1st_cw, rotary_line_left_low_2nd_cw,
                           []()
                           {
                                g_rotarySpindleRpm += 100;
                                if (g_rotarySpindleRpm > SPINDLE_MAX_RPM)
                                {
                                    g_rotarySpindleRpm = SPINDLE_MAX_RPM;
                                }
                           },
                           []()
                           {
                                if (g_rotarySpindleRpm < 100)
                                {
                                    g_rotarySpindleRpm = 0;
                                }
                                else
                                {
                                    g_rotarySpindleRpm -= 100;
                                }
                           });
        spindle_display_task(ledDisplay, g_rotarySpindleRpm, g_lastSentSpindleRpm);
        movement_task(servoBus, g_rotarySpindleRpm);
    }
}

int grinder_main()
{
    printf("Surface grinder controller running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
    
    FlashIAP flash;
    flash_init(flash, g_rotarySpindleRpm);
    printf("Stored spindle speed: %u\n", g_rotarySpindleRpm);

    UnbufferedSerial servoSerial(PB_10, PB_11, MODBUS_BAUD_RATE);
    servoSerial.format(8, SerialBase::None, 2); 
    ModbusMaster servoBus;
    modbus_init(SERVO_MOTOR_DRIVER_ID, servoSerial, servoBus);

    init_grinder_buttons();
    restore_spindle_speed(servoBus);

    TM1637 ledDisplay(PB_13 , PB_14); // DIO // CLK // Bit banging
    spindle_display_init(ledDisplay, g_rotarySpindleRpm);
    
    analogin_t speed_control;
    analogin_init(&speed_control, PA_7);

    disable_motor_head();
    debounce_delay(100);
    init_pwm();
    enable_motor_head();
    debounce_delay(100);

    while (true)
    {
        move_grinder_head_task(speed_control);
        grinder_head_up_down_rotary_switches_task();
        
        stop_button_task(servoBus);
        grinder_start_button_task(flash, servoBus, g_rotarySpindleRpm);
        grinder_head_speed_encoder_switch_task();
        spindle_display_task(ledDisplay, g_rotarySpindleRpm, g_lastSentSpindleRpm);
        speed_adjust_task(servoBus, g_rotarySpindleRpm, g_lastSentSpindleRpm);
        g_lastSentSpindleRpm = g_rotarySpindleRpm;
    }
}

void grinder_head_speed_encoder_switch_task()
{
    rotary_switch_task(right_rotary_switch_state, rotary_line_right_low_1st_cw, rotary_line_right_low_2nd_cw, 
                        []()
                        {
                            g_rotarySpindleRpm += 5;
                            if (g_rotarySpindleRpm > SPINDLE_MAX_RPM)
                            {
                                g_rotarySpindleRpm = SPINDLE_MAX_RPM;
                            }
                        },
                        []()
                        {
                            if (g_rotarySpindleRpm < 5)
                            {
                                g_rotarySpindleRpm = 0;
                            }
                            else
                            {
                                g_rotarySpindleRpm -= 5;
                            }
                        });
                        
    rotary_switch_task(left_rotary_switch_state, rotary_line_left_low_1st_cw, rotary_line_left_low_2nd_cw,
                        []()
                        {
                            g_rotarySpindleRpm += 100;
                            if (g_rotarySpindleRpm > SPINDLE_MAX_RPM)
                            {
                                g_rotarySpindleRpm = SPINDLE_MAX_RPM;
                            }
                        },
                        []()
                        {
                            if (g_rotarySpindleRpm < 100)
                            {
                                g_rotarySpindleRpm = 0;
                            }
                            else
                            {
                                g_rotarySpindleRpm -= 100;
                            }
                        });
}

void restore_spindle_speed(ModbusMaster& servoBus)
{
    int16_t servoPrebootSpeed = read_servo_speed(servoBus);
    printf("read servo speed %d\n", servoPrebootSpeed);
    if (g_rotarySpindleRpm == 0)
    {
        g_rotarySpindleRpm = servoPrebootSpeed;
    }
}

void init_lathe_buttons()
{
    stop_button.mode(PinMode::PullUp);
    start_button.mode(PinMode::PullUp);
    forward_button.mode(PinMode::PullUp);
    reverse_button.mode(PinMode::PullUp);
}

void init_grinder_buttons()
{
    stop_button.mode(PinMode::PullUp);
    start_button.mode(PinMode::PullUp);
}

void flash_init(FlashIAP& flash, int16_t& spindleRpm)
{
    flash.init();
    spindleRpm = read_saved_spindle_speed(flash);
    if (spindleRpm > SPINDLE_MAX_RPM || spindleRpm < 0)
    {
        spindleRpm = 0;
    }
}

void flash_write(FlashIAP& flash, int16_t spindleRpm)
{
    flash.erase(FLASH_LOCATION, SECTOR_SIZE);
    flash.program(&spindleRpm, FLASH_LOCATION, sizeof(spindleRpm));
}

int16_t read_saved_spindle_speed(FlashIAP& flash)
{
    int16_t temp = 0;
    flash.read(&temp, FLASH_LOCATION, sizeof(temp));
    return temp;
}

SpindleDirection spindle_direction(int16_t spindleRpm)
{
    if (spindleRpm == 0)
    {
        return STOPPED_DIRECTION;
    } 
    else if (spindleRpm > 0)
    {
        return FORWARD_DIRECTION;
    }
    return REVERSE_DIRECTION;
}

void forward_button_task(ModbusMaster& servoBus, int16_t spindleRpm)
{
    if (forward_button.read() == 0)
    {
        if (g_forwardButtonState == ButtonState_UP && !g_stopped)
        {
            g_forwardButtonState = ButtonState_DOWN;
            send_servo_speed(servoBus, spindleRpm);
            printf("fwd sent rpm %d\n", spindleRpm);
        }
        debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
    }
    else
    {
        g_forwardButtonState = ButtonState_UP;
    }
}

void reverse_button_task(ModbusMaster& servoBus, int16_t spindleRpm)
{
    if (reverse_button.read() == 0)
    {
        if (g_reverseButtonState == ButtonState_UP && !g_stopped)
        {
            g_reverseButtonState = ButtonState_DOWN;
            send_servo_speed(servoBus, spindleRpm);
            printf("rev sent rpm %d\n", spindleRpm);
        }
        debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
    }
    else
    {
        g_reverseButtonState = ButtonState_UP;
    }
}

bool allowed_to_rotate(DigitalIn pin, bool stopped)
{
    return (pin == 0 && !stopped);
}

void movement_task(ModbusMaster& servoBus, int16_t spindleRpm)
{
    if (allowed_to_rotate(forward_button, g_stopped))
    {
        auto new_rpm = spindleRpm * spindle_forward_direction_polarity();
        if (new_rpm != g_lastSentSpindleRpm)
        {
            send_servo_speed(servoBus, new_rpm);
            if (g_forwardButtonState == ButtonState_UP)
            {
                g_forwardButtonState = ButtonState_DOWN;
                debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
            }
            //printf("sent fwd=%d rev=%d send rpm %d last rpm %d new rpm %d\n", forward_button.read(), reverse_button.read(), spindleRpm, g_lastSentSpindleRpm, new_rpm);
        }
    }
    else if (allowed_to_rotate(reverse_button, g_stopped))
    {
        auto new_rpm = spindleRpm * spindle_reverse_direction_polarity();
        if (new_rpm != g_lastSentSpindleRpm)
        {
            send_servo_speed(servoBus, new_rpm);
            if (g_reverseButtonState == ButtonState_UP)
            {
                g_reverseButtonState = ButtonState_DOWN;
                debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
            }
            //printf("sent fwd=%d rev=%d send rpm %d last rpm %d new rpm %d\n", forward_button.read(), reverse_button.read(), spindleRpm, g_lastSentSpindleRpm, new_rpm);
        }
    }
    else if (forward_button == 1 && reverse_button == 1)
    {
        g_forwardButtonState = ButtonState_UP;
        g_reverseButtonState = ButtonState_UP;
        if (g_lastSentSpindleRpm != 0)
        {
            send_servo_speed(servoBus, 0);
            //printf("fwd=%d rev=%d send rpm %d last rpm %d\n", forward_button.read(), reverse_button.read(), spindleRpm, g_lastSentSpindleRpm);
            debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
        }
    }
}

int16_t spindle_forward_direction_polarity()
{
    return 1;
}

int16_t spindle_reverse_direction_polarity()
{
    return -1;
}

int16_t grinder_direction_polarity()
{
    return -1;
}

void modbus_init(const uint8_t servoMotorDriverSlaveId, UnbufferedSerial& servoSerial, ModbusMaster& servoBus)
{
    servoBus.begin(servoMotorDriverSlaveId, servoSerial);
}

void speed_adjust_task(ModbusMaster& servoBus, int16_t spindleRpm, int16_t previousSpindleRpm)
{
    if (spindleRpm != previousSpindleRpm && stop_button.read() == 1 && !g_stopped)
    {
        if (spindleRpm > 0 && grinder_direction_polarity() < 0)
        {
            auto rpm = -1 * spindleRpm;
            send_servo_speed(servoBus, rpm);
            printf("Sent servo rpm %d\n", rpm);
        }
        else if (spindleRpm < 0 && grinder_direction_polarity() > 0)
        {
            auto rpm = -1 * spindleRpm;
            send_servo_speed(servoBus, rpm);
            printf("Sent servo rpm %d\n", rpm);
        }
        else
        {
            send_servo_speed(servoBus, spindleRpm);
            printf("Sent servo rpm %d\n", spindleRpm);
        }
    }
}

void stop_button_task(ModbusMaster& servoBus)
{
    if (stop_button.read() == 0)
    {
        if (g_stopButtonState == ButtonState_UP)
        {
            g_stopButtonState = ButtonState_DOWN;

            auto status = servoBus.writeSingleRegister(MOTOR_SPINDLE_SPEED_REGISTER, 0);
            g_stopped = true;
            printf("stopped sent\n");
        }
        
        debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
    }
    else
    {
        g_stopButtonState = ButtonState_UP;
    }
}

void grinder_start_button_task(FlashIAP& flash, ModbusMaster& servoBus, int16_t spindleRpm)
{
    if (start_button == 0)
    {
        if (stop_button)
        {
            if (g_startButtonState == ButtonState_UP)
            {
                g_startButtonState = ButtonState_DOWN;
                if (spindleRpm > 0 && grinder_direction_polarity() < 0)
                {
                    send_servo_speed(servoBus, -1 * spindleRpm);
                }
                else if (spindleRpm < 0 && grinder_direction_polarity() > 0)
                {
                    send_servo_speed(servoBus, -1 * spindleRpm);
                }
                else
                {
                    send_servo_speed(servoBus, spindleRpm);
                }
                g_stopped = false;
            }
        }
        debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
    }
    else
    {
        g_startButtonState = ButtonState_UP;
    }
}


void lathe_start_button_task(FlashIAP& flash, ModbusMaster& servoBus, int16_t spindleRpm)
{
    if (start_button == 0)
    {
        if (stop_button)
        {
            g_stopped = false;
        }
    }
}


void send_servo_speed(ModbusMaster& servoBus, int16_t spindleRpm)
{
    auto status = servoBus.writeSingleRegister(MOTOR_SPINDLE_SPEED_REGISTER, spindleRpm);
    if (status != 0)
    {
        printf("Modbus transaction error: %s\n", ModbusMaster::ModbusErrorString(status));
    }
    else
    {
        auto numValuesReceived = servoBus.available();
        if (numValuesReceived > 0)
        {
            for(auto i=0; i<numValuesReceived; ++i)
            {
                auto value = servoBus.receive();
                printf("Received %#04X\n", value);
            }
        }
        g_lastSentSpindleRpm = spindleRpm;
    }
}

int16_t read_servo_speed(ModbusMaster& servoBus)
{
    int16_t speed = 0;
    auto status = servoBus.readHoldingRegisters(MOTOR_SPINDLE_SPEED_REGISTER, 1);
    if (status != 0)
    {
        printf("Modbus transaction error: %s\n", ModbusMaster::ModbusErrorString(status));
    }
    else
    {
        auto numValuesReceived = servoBus.available();
        speed = servoBus.receive();
    }
    return speed;
}

void spindle_display_init(TM1637& display, int16_t spindleRpm)
{
    display.cls(); 
    display.setBrightness(TM1637_BRT7); 
    display.writeData(spindleRpm); 
}

void spindle_display_task(TM1637& display, int16_t spindleRpm, int16_t previousSpindleRpm)
{
    if (spindleRpm != previousSpindleRpm)
    {
        display.writeData(spindleRpm);
    } 
}


void grinder_head_up_down_rotary_switches_task()
{
    if (1 == s_down_button && 1 == s_up_button)
    {
        //printf("rotary_buttons both up\n");
        rotary_switch_task(s_bottom_rotary_encoder_state, 
                           s_rotary_line_bottom_low_1st_cw, 
                           s_rotary_line_bottom_low_2nd_cw, 
                           []()
                           { 
                               printf("rotary_switch_task cw event right\n");
                               pulse_motor(false, PULSES_PER_HALF_TEN_THOUSANDTH, DigitalOut(PWM_OUT)); 
                            }, 
                           []()
                           { 
                               printf("rotary_switch_task cw event left\n");
                               pulse_motor(true, PULSES_PER_HALF_TEN_THOUSANDTH, DigitalOut(PWM_OUT)); 
                            });

        rotary_switch_task(s_top_rotary_encoder_state, 
                           s_rotary_line_top_low_1st_cw, 
                           s_rotary_line_top_low_2nd_cw, 
                           []()
                           { 
                               printf("rotary_switch_task ccw event right\n");
                               pulse_motor(false, PULSES_PER_ONE_THOUSANDTH, DigitalOut(PWM_OUT)); 
                            }, 
                           []()
                           { 
                               printf("rotary_switch_task ccw event left\n");
                               pulse_motor(true, PULSES_PER_ONE_THOUSANDTH, DigitalOut(PWM_OUT)); 
                            });
    }
}

void init_pwm()
{
    s_pwm.suspend();
    set_pwm_high();
}

void set_pwm_high()
{
    gpio_t pin;
    gpio_init_out(&pin, PWM_OUT);
    gpio_write(&pin, 1);
}

void pulse_motor(bool direction_up, unsigned int number_of_pulses, DigitalOut&& pin)
{
    enable_motor_head();

    printf("pulse motor\n");
    // Set direction to move 
    if (direction_up)
    {
        set_motor_head_up();
    }
    else 
    {
        set_motor_head_down();
    }

    for(uint32_t i=0; i<number_of_pulses; ++i)
    {
        pin.write(0);
        wait_us(500000/PULSES_PER_REVOLUTION);
        pin.write(1);
        wait_us(500000/PULSES_PER_REVOLUTION);
    }
    disable_motor_head();
}

void move_grinder_head_task(analogin_t& speed_control)
{
    add_voltage(analogin_read_u16(&speed_control));
    // first check up and down buttons
    // If one is down set the direction pin, enable servo control and run below algorithm
    if (0 == s_up_button)
    {
        //printf("move_grinder_head_task s_up_button down\n");
        if (!s_speed_ticker_running)
        {
            // Set direction pin to up
            set_motor_head_up();
            move_motor_head(speed_control);
            s_up_button_state = ButtonState_DOWN;
            debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
        }
    }
    else if (0 == s_down_button)
    {
        //printf("move_grinder_head_task s_down_button down\n");
        if (!s_speed_ticker_running)
        {            
            // Set direction pin to down
            set_motor_head_down();
            move_motor_head(speed_control);
            s_down_button_state = ButtonState_DOWN;
            debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
        }
    }
    else if (ButtonState_DOWN == s_up_button_state ||  ButtonState_DOWN == s_down_button_state)
    {
        printf("move_grinder_head_task stop motor\n");
        if (s_speed_ticker_running)
        {
            s_speed_ticker.detach();
        }
        motor_head_button_up();
        s_up_button_state = ButtonState_UP;
        s_down_button_state = ButtonState_UP;
        debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
    }
}

void set_motor_head_up()
{
    s_head_direction_pin = 0;
}

void set_motor_head_down()
{
    s_head_direction_pin = 1;
}

void enable_motor_head()
{
    s_head_enable_pin = 0;
}

void disable_motor_head()
{
    s_head_enable_pin = 1;
}

void move_motor_head(analogin_t& speed_control)
{    
    auto diff = abs(s_voltage_average - s_last_voltage);

    if (diff > VOLTAGE_DIFF || (s_voltage_average < one_percent && 0 != s_desired_steps_per_second))
    {
        auto speed_percent = s_voltage_average / one_percent;
        if (speed_percent > 100)
        {
            speed_percent = 100;
        }
        s_last_voltage = s_voltage_average;

        s_desired_steps_per_second = (MAX_PULSES_PER_SECOND * speed_percent)/100;
        unsigned int timer_delay = 0;
        Callback<void()> timer_event = nullptr;
        if (s_desired_steps_per_second < s_current_steps_per_second)
        {
            timer_event = decrease_pwm;
            s_speed_increment = 20;
            timer_delay = 10000;//(s_current_steps_per_second - s_desired_steps_per_second) * INCREMENT_RATE_IN_MICROSECONDS / MAX_PULSES_PER_SECOND;
        }
        else 
        {
            timer_event = increase_pwm;
            s_speed_increment = -20;
            timer_delay = 10000;//(s_desired_steps_per_second - s_current_steps_per_second) * INCREMENT_RATE_IN_MICROSECONDS / MAX_PULSES_PER_SECOND;
        } 

        printf("voltage value=%u last-speed-percent=%u speed-percent=%u time-delay=%u current-steps-sec=%u desired-steps-sec/sec=%u\n", 
            s_voltage_average, 
            s_last_speed_percent, 
            speed_percent, 
            timer_delay,
            s_current_steps_per_second,
            s_desired_steps_per_second); 
        
        s_speed_ticker_running = true;
        s_last_speed_percent = speed_percent;
        s_speed_ticker.attach(timer_event, std::chrono::microseconds(timer_delay));
    }    
}

void motor_head_button_up()
{
    if (s_speed_ticker_running)
    {
        s_speed_ticker.detach();
    }
    s_desired_steps_per_second = 0;
    s_speed_increment = 20;
    auto timer_delay = 10000;//s_current_steps_per_second * INCREMENT_RATE_IN_MICROSECONDS / MAX_PULSES_PER_SECOND;
    s_last_speed_percent = 0;
    s_speed_ticker_running = true;
    s_speed_ticker.attach(decrease_pwm, std::chrono::microseconds(timer_delay));
}

void stop_motor_head()
{
    printf("stop_motor_head\n");
    core_util_critical_section_enter();
    if (s_speed_ticker_running)
    {
        s_speed_ticker.detach();
    }
    core_util_critical_section_exit();
    s_speed_ticker_running = false;
    s_last_speed_percent = 0;
    s_last_voltage = 0;
    s_starting_speed = 0;
    s_current_steps_per_second = 0;
    s_desired_steps_per_second = 0;
    s_pwm_suspended = true;
    s_pwm.suspend();
    set_pwm_high();
    //disable_motor_head();
}

uint32_t steps_per_second_speed_curve(uint32_t desired_speed, int time_increment, uint32_t starting_speed)
{
    if (desired_speed > starting_speed)
    {
        return ((desired_speed - starting_speed) / (1.0f + exp(-0.2*time_increment))) + starting_speed;
    }
    else 
    {
        return ((starting_speed - desired_speed) / (1.0f + exp(-0.2*time_increment)));
    }
}

void increase_pwm()
{
    if (s_desired_steps_per_second != s_starting_speed)
    {
        s_current_steps_per_second = steps_per_second_speed_curve(s_desired_steps_per_second, s_speed_increment++, s_starting_speed);
        printf("increase_pwm desired=%u  current=%u\n", s_desired_steps_per_second, s_current_steps_per_second);

        if (s_desired_steps_per_second == 0)
        {
            stop_motor_head();
            return;
        }

        s_pwm.period_us(1000000.0/s_current_steps_per_second);
        if (s_pwm_suspended)
        {
            enable_motor_head();
            s_pwm.resume();
            s_pwm.write(0.50);
            s_pwm.period_us(1000000.0/s_current_steps_per_second);
            s_pwm_suspended = false;
        }

        if (s_speed_increment > 20)
        {
            s_pwm.period_us(1000000.0/s_desired_steps_per_second);
            s_speed_ticker.detach();
            s_speed_ticker_running = false;
            s_starting_speed = s_current_steps_per_second = s_desired_steps_per_second;
        }
        else 
        {
            s_pwm.period_us(1000000.0/s_current_steps_per_second);
        }
    }
    else 
    {
        s_speed_ticker.detach();
        s_speed_ticker_running = false;
    }
}

void decrease_pwm()
{
    s_current_steps_per_second = s_desired_steps_per_second + steps_per_second_speed_curve(s_desired_steps_per_second, s_speed_increment--, s_starting_speed);
    printf("decrease_pwm desired=%u  current=%u starting speed=%u\n", s_desired_steps_per_second, s_current_steps_per_second, s_starting_speed);
        
    if (s_speed_increment < -20)
    {
        if (s_desired_steps_per_second > 0)
        {
            s_pwm.period_us(1000000.0/s_desired_steps_per_second);
            if (s_pwm_suspended)
            {
                enable_motor_head();
                s_pwm_suspended = false;
                s_pwm.resume();
                s_pwm.write(0.50);
                s_pwm.period_us(1000000.0/s_desired_steps_per_second);
            }
            s_speed_ticker.detach();
            s_speed_ticker_running = false;
            s_starting_speed = s_current_steps_per_second = s_desired_steps_per_second;
        }
        else 
        {
            stop_motor_head();
        }
    }
    else 
    {
        if (s_current_steps_per_second > 0)
        {
            s_pwm.period_us(1000000.0/s_current_steps_per_second);
            if (s_pwm_suspended)
            {
                enable_motor_head();
                s_pwm_suspended = false;
                s_pwm.resume();
                s_pwm.write(0.50);
                s_pwm.period_us(1000000.0/s_current_steps_per_second);
            }
        }
        else 
        {
            stop_motor_head();
        }
    }
}