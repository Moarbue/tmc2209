#include "tmc2209.h"

typedef enum {
    TMC2209_MASK_WRITE = 0x80,
    TMC2209_MASK_READ  = 0x00,
    TMC2209_MASK_SYNC  = 0x05,
} tmc2209_mask;

#define REPLY_DELAY 2  // ms
#define READ_TIMEOUT 5 // ms
#define READ_MAX_RETRIES 2

#define TOFF_DEFAULT 3

static bool serial_initialized = false;
HardwareSerial s_serial(TMC2209_DEFAULT_SERIAL);

// helper functions declaration
void set_address(tmc2209_t *s, tmc2209_address address);
void register_write(tmc2209_t *s, uint8_t address, uint32_t val);
uint32_t register_read(tmc2209_t *s, uint8_t address);
uint8_t calc_crc(uint8_t datagram[], uint8_t len);

bool tmc2209_full(tmc2209_t *s, uint8_t en_pin, uint8_t dir_pin, uint8_t step_pin,
                                      uint8_t rx_pin, uint8_t tx_pin,  uint8_t ms1_pin, uint8_t ms2_pin,
                                      tmc2209_address address)
{
    if (s == NULL) return false;

    memset(s, 0, sizeof (*s));

    s->_en_pin               = en_pin;
    s->_dir_pin              = dir_pin;
    s->_step_pin             = step_pin;
    s->_rx_pin               = rx_pin;
    s->_tx_pin               = tx_pin;
    s->_ms1_pin              = ms1_pin;
    s->_ms2_pin              = ms2_pin;

    s->_address               = address;
    s->_microsteps           = TMC2209_DEFAULT_MICROSTEPS;
    s->_dir                  = TMC2209_CW;
    s->_step_delay           = TMC2209_DEFAULT_STEP_DELAY;
    s->_steps_per_revolution = TMC2209_DEFAULT_STEPS_PER_REVOLUTION;

    s->_step                 = 0;
    s->_steps                = 0;

    pinMode(s->_en_pin, OUTPUT);
    // disable driver
    digitalWrite(s->_en_pin, HIGH);

    pinMode(s->_dir_pin,  OUTPUT);
    pinMode(s->_step_pin, OUTPUT);
    pinMode(s->_ms1_pin,  OUTPUT);
    pinMode(s->_ms2_pin,  OUTPUT);
    // enable driver
    digitalWrite(s->_en_pin, LOW);

    set_address(s, address);

    // initialize UART
    if (!serial_initialized) {
        s_serial.begin(115200, SERIAL_8N1, s->_rx_pin, s->_tx_pin);
        serial_initialized = true;
    }

    TMC2209_REGISTER_SET(s->_gconf, GCONF_PDN_DISABLE, 1);
    TMC2209_REGISTER_SET(s->_gconf, GCONF_MSTEP_REG_SELECT, 1);
    register_write(s, GCONF_ADRESS, s->_gconf);
    return (s->_communicating = tmc2209_check_connection(s));
}

void tmc2209_set_microsteps(tmc2209_t *s, tmc2209_microstep microsteps)
{
    if (s == NULL) return;

    // TODO: Implement Error Reporting
    if (microsteps != TMC2209_MICROSTEPS_8  || microsteps != TMC2209_MICROSTEPS_16 ||
        microsteps != TMC2209_MICROSTEPS_32 || microsteps != TMC2209_MICROSTEPS_64)
        return;

    s->_microsteps = microsteps;

    if (s->_communicating) {
        TMC2209_REGISTER_VAL(s->_chopconf, CHOPCONF_MRES, 4, microsteps);
        register_write(s, CHOPCONF_ADDRESS, s->_chopconf);
    } else {
        switch (s->_microsteps) {
            case TMC2209_MICROSTEPS_8:
                digitalWrite(s->_ms1_pin, LOW);
                digitalWrite(s->_ms2_pin, LOW);
            break;
            case TMC2209_MICROSTEPS_16:
                digitalWrite(s->_ms1_pin, HIGH);
                digitalWrite(s->_ms2_pin, HIGH);
            break;
            case TMC2209_MICROSTEPS_32:
                digitalWrite(s->_ms1_pin, HIGH);
                digitalWrite(s->_ms2_pin, LOW);
            break;
            case TMC2209_MICROSTEPS_64:
                digitalWrite(s->_ms1_pin, LOW);
                digitalWrite(s->_ms2_pin, HIGH);
            break;
        }
    }
}

tmc2209_microstep tmc2209_get_microsteps(tmc2209_t *s)
{
    if (s == NULL) return TMC2209_DEFAULT_MICROSTEPS;

    return s->_microsteps;
}

void tmc2209_set_step_delay(tmc2209_t *s, uint32_t step_delay_us)
{
    if (s == NULL) return;

    s->_step_delay = step_delay_us;
}

uint32_t tmc2209_get_step_delay(tmc2209_t *s)
{
    if (s == NULL) return TMC2209_DEFAULT_STEP_DELAY;

    return s->_step_delay;
}

void tmc2209_set_direction(tmc2209_t *s, tmc2209_direction dir)
{
    if (s == NULL) return;

    if (dir != TMC2209_CW || dir != TMC2209_CCW) return;

    if (s->_communicating) {
        // TODO: Implement setting direction via UART
    } else 
        digitalWrite(s->_dir_pin, s->_dir);
}

tmc2209_direction tmc2209_get_direction(tmc2209_t *s)
{
    if (s == NULL) return TMC2209_CCW;

    return s->_dir; 
}

void tmc2209_step(tmc2209_t *s, uint32_t steps, tmc2209_direction dir)
{
    if (s == NULL) return;

    if (s->_step < s->_steps) return;

    tmc2209_set_direction(s, dir);
    s->_step  = 0;
    s->_steps = steps;
}

void tmc2209_rotate(tmc2209_t *s, int32_t degree)
{
    if (s == NULL) return;

    uint32_t steps;
    tmc2209_direction dir;

    steps = (abs(degree) * s->_steps_per_revolution * s->_microsteps) / 360;
    dir   = (degree > 0) ? TMC2209_CW : TMC2209_CCW;

    tmc2209_step(s, steps, dir);
}

void tmc2209_update(tmc2209_t *s)
{
    if (s == NULL) return;

    if (s->_step < s->_steps) {
        s->_step++;

        digitalWrite(s->_step_pin, HIGH);
        delayMicroseconds(s->_step_delay);
        digitalWrite(s->_step_pin, LOW);
        delayMicroseconds(s->_step_delay);
    }
}

void tmc2209_disable(tmc2209_t *s)
{
    if (s == NULL) return;

    if (s->_communicating) {
        TMC2209_REGISTER_CLR(s->_chopconf, CHOPCONF_TOFF, 4);
        register_write(s, CHOPCONF_ADDRESS, s->_chopconf);
    }

    digitalWrite(s->_en_pin, HIGH);
}

void tmc2209_enable(tmc2209_t *s)
{
    if (s == NULL) return;

    if (s->_communicating) {
        TMC2209_REGISTER_VAL(s->_chopconf, CHOPCONF_TOFF, 4, TOFF_DEFAULT);
        register_write(s, CHOPCONF_ADDRESS, s->_chopconf);
    }

    digitalWrite(s->_en_pin, LOW);
}

bool tmc2209_check_connection(tmc2209_t * s)
{
    s->_gconf = register_read(s, GCONF_ADRESS) & GCONF_BIT_MASK;
    return (s->_gconf >> GCONF_PDN_DISABLE) & 0x1;
}

void tmc2209_toff(tmc2209_t *s, uint8_t val)
{
    if (s == NULL) return;

    if (s->_communicating) {
        TMC2209_REGISTER_VAL(s->_chopconf, CHOPCONF_TOFF, 4, val & 0xF);
        register_write(s, CHOPCONF_ADDRESS, s->_chopconf);
    }
}

void tmc2209_stallguard_thrs(tmc2209_t *s, uint8_t threshold)
{
    if (s == NULL) return;

    if (s->_communicating) {
        TMC2209_REGISTER_VAL(s->_sgthrs, 0, 8, threshold);
        register_write(s, SGTHRS_ADDRESS, s->_sgthrs);
    }
}

uint16_t tmc2209_stallguard_result(tmc2209_t *s)
{
   if (s == NULL) return 0;

   if (s->_communicating) {
        s->_sg_result = register_read(s, SG_RESULT_ADDRESS) & SG_RESULT_BIT_MASK;
        return s->_sg_result;
   }
   return 0;
}

bool tmc2209_is_stalling(tmc2209_t *s)
{
    if (s == NULL) return false;

    if (s->_communicating) {
        s->_sg_result = register_read(s, SG_RESULT_ADDRESS) & SG_RESULT_BIT_MASK;
        return s->_sg_result <= (2 * (uint16_t)s->_sgthrs);
    }
    return false;
}

// helper functions definition

void set_address(tmc2209_t *s, tmc2209_address address)
{
    switch (address) {
        case TMC2209_ADDRESS_0:
            digitalWrite(s->_ms1_pin, LOW);
            digitalWrite(s->_ms2_pin, LOW);
        break;
        case TMC2209_ADDRESS_1:
            digitalWrite(s->_ms1_pin, HIGH);
            digitalWrite(s->_ms2_pin, LOW);
        break;
        case TMC2209_ADDRESS_2:
            digitalWrite(s->_ms1_pin, LOW);
            digitalWrite(s->_ms2_pin, HIGH);
        break;
        case TMC2209_ADDRESS_3:
            digitalWrite(s->_ms1_pin, HIGH);
            digitalWrite(s->_ms2_pin, HIGH);
        break;
        default:
            // TODO: Implement Error Reporting
            assert(false && "Wrong address specified!");
    }
}

void register_write(tmc2209_t *s, uint8_t address, uint32_t val)
{
    while (s_serial.available() > 0) s_serial.read(); // flush buffer

    address |= TMC2209_MASK_WRITE;

    uint8_t len = 8;
    uint8_t datagram[len] = {
        TMC2209_MASK_SYNC,     // sync byte
        (uint8_t) s->_address, // slave address
        address,               // register address with write flag
        (uint8_t) (val >> 24), // data byte 1
        (uint8_t) (val >> 16), // data byte 2
        (uint8_t) (val >>  8), // data byte 3
        (uint8_t) (val >>  0), // data byte 4
        0x00,                  // placeholder for crc
    };

    datagram[len - 1] = calc_crc(datagram, len - 1); // leave out placeholder for crc calculation

    for (uint8_t i = 0; i < len; ++i) {
        s_serial.write(datagram[i]);
    }

    delay(REPLY_DELAY);
}

uint32_t register_read(tmc2209_t *s, uint8_t address)
{
    while (s_serial.available() > 0) s_serial.read(); // flush buffer

    uint64_t reply;

    address |= TMC2209_MASK_READ;

    uint8_t len = 4;
    uint8_t datagram[len] = {
        TMC2209_MASK_SYNC,          // sync byte
        (uint8_t) s->_address,      // slave address
        address,                    // register address with read flag
        0x00,                       // placeholder for crc
    };

    datagram[len - 1] = calc_crc(datagram, len - 1); // leave out placeholder for crc calculation

    for (uint8_t i = 0; i < READ_MAX_RETRIES; ++i) {
        uint8_t j, crc;
        unsigned long mills, last_mills;
        uint32_t sync_target;
        uint16_t timeout;
        int8_t byte;

    // send read request
        for (j = 0; j < len; ++j) {
            s_serial.write(datagram[j]);
        }

        delay(REPLY_DELAY);

    // check for reply
        // sync byte + master adress (from datasheet) + register address with read flag
        sync_target = (TMC2209_MASK_SYNC << 16) | (0xFF << 8) | (address << 0);
        reply       = 0;
        timeout     = READ_TIMEOUT;

        mills = last_mills = millis();

        while ((uint32_t)reply != sync_target && timeout != 0) {
            mills = millis();

            // at least one millisecond passed
            if (mills != last_mills) {
                timeout -= (mills - last_mills);
                last_mills = mills;
            }

            byte = s_serial.read();
            // no data available
            if (byte == -1) continue;

            reply <<= 8;           // make space for incomming byte
            reply  |= byte & 0xFF; // shift new byte in
            reply  &= 0xFFFFFF;    // due to frame being a uint62_t, only look at first 3 bytes
        }

        // check if timeout occured
        if (timeout == 0) assert(false && "Read timeout occured!");


        // read the remaining 5 bytes of the datagram
        j = 0;
        timeout = READ_TIMEOUT;
        mills = last_mills = millis();

        while (j < 5 && timeout != 0) {
            mills = millis();

            // at least one millisecond passed
            if (mills != last_mills) {
                timeout -= (mills - last_mills);
                last_mills = mills;
            }

            byte = s_serial.read();
            // no data available
            if (byte == -1) continue;

            reply <<= 8;           // make space for incomming byte
            reply  |= byte & 0xFF; // shift new byte in
            j++;
        }

        // check if timeout occured
        if (timeout == 0) assert(false && "Read timeout occured!");


        while (s_serial.available() > 0) s_serial.read(); // flush buffer

        delay(REPLY_DELAY);

    // perform crc
        uint8_t reply_datagram[] = {
            (uint8_t) (reply >> 56), // sync byte
            (uint8_t) (reply >> 48), // master adress
            (uint8_t) (reply >> 40), // register address with read flag
            (uint8_t) (reply >> 32), // data byte 1
            (uint8_t) (reply >> 24), // data byte 2
            (uint8_t) (reply >> 16), // data byte 3
            (uint8_t) (reply >>  8), // data byte 4
            (uint8_t) (reply >>  0), // crc
        };

        crc = calc_crc(reply_datagram, 7);
        if (crc != reply_datagram[7] || crc == 0) {
            reply = 0;
        } else {
            break;
        }
    }

    // return data bytes
    return (uint32_t) ((reply >> 8) & 0xFFFFFFFF);
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