/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "TM1637.h"
#include "Font_7Seg.h"
#include "Timer.h"
#include "ModbusMaster.h"

#define FLASH_LOCATION MBED_ROM_START+MBED_ROM_SIZE-0x10000 
#define SECTOR_SIZE 2048
#define BUTTON_DEBOUNCE_MILLISECONDS 3
#define ROTARY_CLICK_ADJUSTMENT 20
#define SPINDLE_MAX_RPM 3000
#define MODBUS_BAUD_RATE 115200
#define SERVO_MOTOR_DRIVER_ID 1
#define MOTOR_SPINDLE_SPEED_REGISTER 169

enum RotarySwitchState
{
    ROTARY_STATE_INIT = 0,
    ROTARY_STATE_WAITING_CW,
    ROTARY_STATE_CW,
    ROTARY_STATE_CW_ENDING,
    ROTARY_STATE_WAITING_CCW,
    ROTARY_STATE_CCW,
    ROTARY_STATE_CCW_ENDING
};

enum ButtonState
{
    ButtonState_DOWN = 0,
    ButtonState_UP
};

enum SpindleDirection
{
    STOPPED_DIRECTION = 0,
    FORWARD_DIRECTION,
    REVERSE_DIRECTION
};

DigitalOut led1(LED1);

// Right rotary switch
DigitalIn rotary_button_right(PC_0); //SW
DigitalIn rotary_line_right_low_1st_cw(PC_2); //CLK
DigitalIn rotary_line_right_low_2nd_cw(PC_1); //DT

// Left rotary switch
DigitalIn rotary_button_left(PC_9); //SW
DigitalIn rotary_line_left_low_1st_cw(PB_8); //CLK
DigitalIn rotary_line_left_low_2nd_cw(PB_9); //DT

DigitalIn user_button(USER_BUTTON);
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
void rotary_right_button_down();
void rotary_left_button_down();
void rotary_button_task(FlashIAP& flash, ModbusMaster& servoBus, DigitalIn pin, ButtonState& state, ButtonDownCallback button_down);
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
void sendServoSpeed(ModbusMaster& servoBus, int16_t spindleRpm);
int16_t readSavedSpindleSpeed(FlashIAP& flash);
int16_t readServoSpeed(ModbusMaster& servoBus);
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

ButtonState g_rotaryButtonRightState = ButtonState_UP;
ButtonState g_rotaryButtonLeftState = ButtonState_UP;
ButtonState g_userButtonState = ButtonState_UP;
ButtonState g_startButtonState = ButtonState_UP;
ButtonState g_stopButtonState = ButtonState_UP;
ButtonState g_forwardButtonState = ButtonState_UP;
ButtonState g_reverseButtonState = ButtonState_UP;

int16_t g_lastSentSpindleRpm = 0;
int16_t g_rotarySpindleRpm = 0;
bool g_stopped = true;

int lathe_main();
int grinder_main();

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

    while (1)
    {
        stop_button_task(servoBus);
        lathe_start_button_task(flash, servoBus, g_rotarySpindleRpm);
        rotary_switch_task(currentState, g_rotarySpindleRpm, 5, rotary_line_right_low_1st_cw, rotary_line_right_low_2nd_cw);
        rotary_switch_task(currentState, g_rotarySpindleRpm, 100, rotary_line_left_low_1st_cw, rotary_line_left_low_2nd_cw);
        spindle_display_task(ledDisplay, g_rotarySpindleRpm, g_lastSentSpindleRpm);
        rotary_button_task(flash, servoBus, rotary_button_right, g_rotaryButtonRightState, &rotary_right_button_down);
        rotary_button_task(flash, servoBus, rotary_button_left, g_rotaryButtonLeftState, &rotary_left_button_down);
        movement_task(servoBus, g_rotarySpindleRpm);
    }
}

int grinder_main()
{
    printf("Surface grinder controller running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
    
    FlashIAP flash;
    flash_init(flash, g_rotarySpindleRpm);
    printf("Stored spindle speed: %u\n", g_rotarySpindleRpm);

    RotarySwitchState currentState;
    rotary_switch_init(currentState);

    UnbufferedSerial servoSerial(PB_10, PB_11, MODBUS_BAUD_RATE);
    servoSerial.format(8, SerialBase::None, 2); 
    ModbusMaster servoBus;
    modbus_init(SERVO_MOTOR_DRIVER_ID, servoSerial, servoBus);

    init_grinder_buttons();
    restore_spindle_speed(servoBus);

    TM1637 ledDisplay(PB_13 /* DIO*/, PB_14 /* CLK */); // Bit banging
    spindle_display_init(ledDisplay, g_rotarySpindleRpm);

    while (1)
    {
        stop_button_task(servoBus);
        grinder_start_button_task(flash, servoBus, g_rotarySpindleRpm);
        rotary_switch_task(currentState, g_rotarySpindleRpm, 5, rotary_line_right_low_1st_cw, rotary_line_right_low_2nd_cw);
        rotary_switch_task(currentState, g_rotarySpindleRpm, 100, rotary_line_left_low_1st_cw, rotary_line_left_low_2nd_cw);
        spindle_display_task(ledDisplay, g_rotarySpindleRpm, g_lastSentSpindleRpm);
        rotary_button_task(flash, servoBus, rotary_button_right, g_rotaryButtonRightState, &rotary_right_button_down);
        rotary_button_task(flash, servoBus, rotary_button_left, g_rotaryButtonLeftState, &rotary_left_button_down);
        speed_adjust_task(servoBus, g_rotarySpindleRpm, g_lastSentSpindleRpm);
        g_lastSentSpindleRpm = g_rotarySpindleRpm;
    }
}

void restore_spindle_speed(ModbusMaster& servoBus)
{
    int16_t servoPrebootSpeed = readServoSpeed(servoBus);
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
    spindleRpm = readSavedSpindleSpeed(flash);
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

int16_t readSavedSpindleSpeed(FlashIAP& flash)
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
            sendServoSpeed(servoBus, spindleRpm);
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
            sendServoSpeed(servoBus, spindleRpm);
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
            sendServoSpeed(servoBus, new_rpm);
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
            sendServoSpeed(servoBus, new_rpm);
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
            sendServoSpeed(servoBus, 0);
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

void rotary_button_task(FlashIAP& flash, ModbusMaster& servoBus, DigitalIn pin, ButtonState& state, ButtonDownCallback callback)
{
    if (pin == 0)
    {
        if (state == ButtonState_UP)
        {
            state = ButtonState_DOWN;
            if (callback)
            {
                callback();
            }
        }
    }
    else
    {
        state = ButtonState_UP;
    }
}

void rotary_right_button_down(){}

void rotary_left_button_down(){}

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
            sendServoSpeed(servoBus, rpm);
            printf("Sent servo rpm %d\n", rpm);
        }
        else if (spindleRpm < 0 && grinder_direction_polarity() > 0)
        {
            auto rpm = -1 * spindleRpm;
            sendServoSpeed(servoBus, rpm);
            printf("Sent servo rpm %d\n", rpm);
        }
        else
        {
            sendServoSpeed(servoBus, spindleRpm);
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
                    sendServoSpeed(servoBus, -1 * spindleRpm);
                }
                else if (spindleRpm < 0 && grinder_direction_polarity() > 0)
                {
                    sendServoSpeed(servoBus, -1 * spindleRpm);
                }
                else
                {
                    sendServoSpeed(servoBus, spindleRpm);
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


void sendServoSpeed(ModbusMaster& servoBus, int16_t spindleRpm)
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

int16_t readServoSpeed(ModbusMaster& servoBus)
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

void rotary_switch_init(RotarySwitchState& currentState)
{
    currentState = ROTARY_STATE_INIT;
}

void rotary_switch_task(RotarySwitchState& currentState, int16_t& spindleRpm, int16_t increment_amount, DigitalIn low_first_cw, DigitalIn low_second_cw)
{
    do
    {
        debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
        
        RotarySwitchState newState = rotary_switch_state(currentState, low_first_cw, low_second_cw);

        if (currentState == ROTARY_STATE_CW_ENDING && newState == ROTARY_STATE_INIT)
        {
            spindleRpm += increment_amount;
            if (spindleRpm > SPINDLE_MAX_RPM)
            {
                spindleRpm = SPINDLE_MAX_RPM;
            }
        }
        else if (currentState == ROTARY_STATE_CCW_ENDING && newState == ROTARY_STATE_INIT)
        {
            if (spindleRpm < increment_amount)
            {
                spindleRpm = 0;
            }
            else
            {
                spindleRpm -= increment_amount;
            }
        }

        currentState = newState;
    } while(currentState != ROTARY_STATE_INIT);
    //printf("rotary rpm %d\n", spindleRpm);
}

RotarySwitchState rotary_switch_state(const RotarySwitchState switchState, DigitalIn rotary_line_low_1st_cw, DigitalIn rotary_line_low_2nd_cw)
{
    switch (switchState)
    {
        case ROTARY_STATE_INIT:
            if (rotary_line_low_1st_cw.read() == 0)
            {
                if (rotary_line_low_2nd_cw.read() == 1)
                {
                    return ROTARY_STATE_WAITING_CW;
                }
            }
            else if (rotary_line_low_2nd_cw.read() == 0)
            {
                if (rotary_line_low_1st_cw.read() == 1)
                {
                    return ROTARY_STATE_WAITING_CCW;
                }
            }
            break;

        case ROTARY_STATE_WAITING_CW:
            if (rotary_line_low_1st_cw.read() == 0)
            {
                if (rotary_line_low_2nd_cw.read() == 0)
                {
                    return ROTARY_STATE_CW;
                }
                else 
                {
                    return ROTARY_STATE_WAITING_CW;
                }
            }
            break;

        case ROTARY_STATE_CW:
            if (rotary_line_low_1st_cw.read() == 1)
            {
                if (rotary_line_low_2nd_cw.read() == 0)
                {
                    return ROTARY_STATE_CW_ENDING;
                }
                else 
                {
                    return ROTARY_STATE_CW;
                }
            }
            else if (rotary_line_low_1st_cw.read() == 0 && rotary_line_low_2nd_cw.read() == 0)
            {
                return ROTARY_STATE_CW;
            }
            break;

        case ROTARY_STATE_CW_ENDING:
            if (rotary_line_low_1st_cw.read() == 1)
            {
                if (rotary_line_low_2nd_cw.read() == 1)
                {
                    return ROTARY_STATE_INIT;
                }
                else 
                {
                    return ROTARY_STATE_CW_ENDING;
                }
            }
            break;

        case ROTARY_STATE_WAITING_CCW:
            if (rotary_line_low_2nd_cw.read() == 0)
            {
                if (rotary_line_low_1st_cw.read() == 0)
                {
                    return ROTARY_STATE_CCW;
                }
                else 
                {
                    return ROTARY_STATE_WAITING_CCW;
                }
            }
            break;

        case ROTARY_STATE_CCW:
            if (rotary_line_low_2nd_cw.read() == 1)
            {
                if (rotary_line_low_1st_cw.read() == 0)
                {
                    return ROTARY_STATE_CCW_ENDING;
                }
                else 
                {
                    return ROTARY_STATE_CCW;
                }
            }
            else if (rotary_line_low_1st_cw.read() == 0 && rotary_line_low_2nd_cw.read() == 0)
            {
                return ROTARY_STATE_CCW;
            }
            break;

        case ROTARY_STATE_CCW_ENDING:
            if (rotary_line_low_2nd_cw.read() == 1)
            {
                if (rotary_line_low_1st_cw.read() == 1)
                {
                    return ROTARY_STATE_INIT;
                }
                else 
                {
                    return ROTARY_STATE_CCW_ENDING;
                }
            }
            break;

        default:
            break;
    }

    return ROTARY_STATE_INIT;
}

void debounce_delay(int delayMilliseconds)
{
    Timer timer;
    timer.start();
    while (timer.elapsed_time().count() < delayMilliseconds);
}
