#include "tmc2209.h"

static bool serial_initialized = false;
HardwareSerial s_serial(TMC2209_DEFAULT_SERIAL);

// helper functions declaration
void set_address(tmc2209_t *s, tmc2209_address address);

void tmc2209_full(tmc2209_t *s, uint8_t en_pin, uint8_t dir_pin, uint8_t step_pin,
                                      uint8_t rx_pin, uint8_t tx_pin,  uint8_t ms1_pin, uint8_t ms2_pin,
                                      tmc2209_address address)
{
    if (s == NULL) return;

    memset(s, 0, sizeof (*s));

    s->en_pin               = en_pin;
    s->dir_pin              = dir_pin;
    s->step_pin             = step_pin;
    s->rx_pin               = rx_pin;
    s->tx_pin               = tx_pin;
    s->ms1_pin              = ms1_pin;
    s->ms2_pin              = ms2_pin;

    s->address               = address;
    s->microsteps           = TMC2209_DEFAULT_MICROSTEPS;
    s->dir                  = TMC2209_CW;
    s->step_delay           = TMC2209_DEFAULT_STEP_DELAY;
    s->steps_per_revolution = TMC2209_DEFAULT_STEPS_PER_REVOLUTION;

    s->step                 = 0;
    s->steps                = 0;

    pinMode(s->en_pin, OUTPUT);
    // disable driver
    digitalWrite(s->en_pin, HIGH);

    pinMode(s->dir_pin,  OUTPUT);
    pinMode(s->step_pin, OUTPUT);
    pinMode(s->ms1_pin,  OUTPUT);
    pinMode(s->ms2_pin,  OUTPUT);

    // initialize UART
    if (serial_initialized) return;
    s_serial.begin(115200, SERIAL_8N1, s->rx_pin, s->tx_pin);
    serial_initialized = true;
}

void tmc2209_set_microsteps(tmc2209_t *s, tmc2209_microstep microsteps)
{
    if (s == NULL) return;

    // TODO: Implement Error Reporting
    if (microsteps != TMC2209_MICROSTEPS_8  || microsteps != TMC2209_MICROSTEPS_16 ||
        microsteps != TMC2209_MICROSTEPS_32 || microsteps != TMC2209_MICROSTEPS_64)
        return;

    s->microsteps = microsteps;

    if (serial_initialized) {
        // TODO: set microsteps via UART
    } else {
        switch (s->microsteps) {
            case TMC2209_MICROSTEPS_8:
                digitalWrite(s->ms1_pin, LOW);
                digitalWrite(s->ms2_pin, LOW);
            break;
            case TMC2209_MICROSTEPS_16:
                digitalWrite(s->ms1_pin, HIGH);
                digitalWrite(s->ms2_pin, HIGH);
            break;
            case TMC2209_MICROSTEPS_32:
                digitalWrite(s->ms1_pin, HIGH);
                digitalWrite(s->ms2_pin, LOW);
            break;
            case TMC2209_MICROSTEPS_64:
                digitalWrite(s->ms1_pin, LOW);
                digitalWrite(s->ms2_pin, HIGH);
            break;
        }
    }
}

tmc2209_microstep tmc2209_get_microsteps(tmc2209_t *s)
{
    if (s == NULL) return TMC2209_DEFAULT_MICROSTEPS;

    return s->microsteps;
}

void tmc2209_set_step_delay(tmc2209_t *s, uint32_t step_delay_us)
{
    if (s == NULL) return;

    s->step_delay = step_delay_us;
}

uint32_t tmc2209_get_step_delay(tmc2209_t *s)
{
    if (s == NULL) return TMC2209_DEFAULT_STEP_DELAY;

    return s->step_delay;
}

void tmc2209_set_direction(tmc2209_t *s, tmc2209_direction dir)
{
    if (s == NULL) return;

    if (dir != TMC2209_CW || dir != TMC2209_CCW) return;

    if (serial_initialized) {
        // TODO: Implement setting direction via UART
    } else 
        digitalWrite(s->dir_pin, s->dir);
}

tmc2209_direction tmc2209_get_direction(tmc2209_t *s)
{
    if (s == NULL) return TMC2209_CCW;

    return s->dir; 
}

void tmc2209_step(tmc2209_t *s, uint32_t steps, tmc2209_direction dir)
{
    if (s == NULL) return;

    tmc2209_set_direction(s, dir);
    s->step  = 0;
    s->steps = steps;
}

void tmc2209_rotate(tmc2209_t *s, int32_t degree)
{
    if (s == NULL) return;

    uint32_t steps;
    tmc2209_direction dir;

    steps = (abs(degree) * s->steps_per_revolution * s->microsteps) / 360;
    dir   = (dir > 0) // shortcut because clockwise = 1, counterclockwise = 0

    tmc2209_step(s, steps, dir);
}

void tmc2209_update(tmc2209_t *s)
{
    if (s == NULL) return;

    if (s->step < s->steps) {
        s->step++;

        digitalWrite(s->step_pin, HIGH);
        delayMicroseconds(s->step_delay);
        digitalWrite(s->step_pin, LOW);
        delayMicroseconds(s->step_delay);
    }
}


// helper functions definition

void set_address(tmc2209_t *s, tmc2209_address address)
{
    switch (address) {
        case TMC2209_ADDRESS_0:
            digitalWrite(s->ms1_pin, LOW);
            digitalWrite(s->ms2_pin, LOW);
        break;
        case TMC2209_ADDRESS_1:
            digitalWrite(s->ms1_pin, HIGH);
            digitalWrite(s->ms2_pin, LOW);
        break;
        case TMC2209_ADDRESS_2:
            digitalWrite(s->ms1_pin, LOW);
            digitalWrite(s->ms2_pin, HIGH);
        break;
        case TMC2209_ADDRESS_3:
            digitalWrite(s->ms1_pin, HIGH);
            digitalWrite(s->ms2_pin, HIGH);
        break;
        default:
            // TODO: Implement Error Reporting
            assert(false && "Wrong address specified!");
    }
}