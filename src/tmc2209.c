#include "tmc2209.h"

typedef enum {
    TMC2209_MASK_WRITE = 0x80,
    TMC2209_MASK_READ  = 0x00,
    TMC2209_MASK_SYNC  = 0x05,
} tmc2209_mask;

#define REPLY_DELAY 2 // ms

static bool serial_initialized = false;
HardwareSerial s_serial(TMC2209_DEFAULT_SERIAL);

// helper functions declaration
void set_address(tmc2209_t *s, tmc2209_address address);
void register_write(tmc2209_t *s, uint8_t address, uint32_t val);
uint8_t calc_crc(uint8_t datagram[], uint8_t len);

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

void register_write(tmc2209_t *s, uint8_t address, uint32_t val)
{
    while (s_serial.available() > 0) s_serial.read();

    address |= TMC2209_MASK_WRITE;

    uint8_t len = 8;
    uint8_t datagram[len] = {
        TMC2209_MASK_SYNC,  // sync byte
        s->address,         // slave address
        address,            // register address with write flag
        (val >> 24) & 0xFF, // data bit 1 (MSB)
        (val >> 16) & 0xFF, // data bit 2
        (val >>  8) & 0xFF, // data bit 3
        (val >>  0) & 0xFF, // data bit 4 (LSB)
        0x00,               // placeholder for crc
    };

    datagram[len - 1] = calc_crc(datagram, len - 1); // leave out placeholder for crc calculation

    for (uint8_t i = 0; i < len; ++i) {
        s_serial.write(datagram[i]);
    }

    delay(REPLY_DELAY);
}

uint8_t calc_crc(uint8_t datagram[], uint8_t len)
{
    uint8_t crc = 0;
	for (uint8_t i = 0; i < len; ++i) {
		uint8_t current = datagram[i];
		for (uint8_t j = 0; j < 8; ++j) {
			if ((crc >> 7) ^ (current & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xFF;
			current = current >> 1;
		}
	}
	return crc;
}