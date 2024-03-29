#include "tmc2209.h"
#include "driver/rmt.h"

#define MAX_STEPPER_COUNT RMT_CHANNEL_MAX
#define INITIALIZE_RMT(rmt, ch, step_pin) do {                  \
    (rmt).rmt_mode = RMT_MODE_TX;                               \
    (rmt).channel = (rmt_channel_t)(ch);                        \
    (rmt).gpio_num = (gpio_num_t) (step_pin);                   \
    (rmt).mem_block_num = 1;                                    \
    (rmt).tx_config.loop_en = 0;                                \
    (rmt).tx_config.carrier_en = 0;                             \
    (rmt).tx_config.idle_output_en = 1;                         \
    (rmt).tx_config.idle_level = RMT_IDLE_LEVEL_LOW;            \
    (rmt).tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;     \
    (rmt).clk_div = 80;                                         \
    rmt_config(&(rmt));                                         \
    rmt_driver_install((rmt_channel_t)(ch), 0, 0);              \
    } while(0)

// microsteps go from 0 (256) to 8 (0) in power of two steps
// 1 << (8 - 0) = 256, 1 << (8 - 1) = 128, ...
#define MICROSTEPS_TO_NUMBER(msteps) (1 << (8 - msteps))

typedef enum {
    TMC2209_MASK_WRITE = 0x80,
    TMC2209_MASK_READ  = 0x00,
    TMC2209_MASK_SYNC  = 0x05,
} tmc2209_mask;

#define REPLY_DELAY 2  // ms
#define READ_TIMEOUT 5 // ms
#define READ_MAX_RETRIES 2

#define TOFF_DEFAULT 3
#define TBLANK_DEFAULT 0b01 // 24 clock cycles

// TODO: actually test these values
#define STEP_DELAY_MIN 150
#define STEP_DELAY_MAX 2000000

static bool serial_initialized = false;
HardwareSerial s_serial(TMC2209_DEFAULT_SERIAL);

static uint8_t stepper_count = 0;
static tmc2209_t **steppers = NULL;
static rmt_config_t *rmts = NULL;

// helper functions declaration
void set_address(tmc2209_t *s, tmc2209_address address);
void register_write(tmc2209_t *s, uint8_t address, uint32_t val);
uint32_t register_read(tmc2209_t *s, uint8_t address);
uint8_t calc_crc(uint8_t datagram[], uint8_t len);
void update_stepper(rmt_channel_t channel, void *arg);

bool tmc2209_full(tmc2209_t *s, uint8_t en_pin, uint8_t dir_pin, uint8_t step_pin,
                                      uint8_t rx_pin, uint8_t tx_pin,  uint8_t ms1_pin, uint8_t ms2_pin,
                                      tmc2209_address address, float rsense)
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
    s->_rsense               = rsense;

    s->_step                 = 0;
    s->_steps                = 0;

    pinMode(s->_en_pin, OUTPUT);
    // disable driver
    digitalWrite(s->_en_pin, HIGH);

    pinMode(s->_dir_pin,  OUTPUT);
    pinMode(s->_step_pin, OUTPUT);
    pinMode(s->_ms1_pin,  OUTPUT);
    pinMode(s->_ms2_pin,  OUTPUT);
    set_address(s, address);
    // enable driver
    digitalWrite(s->_en_pin, LOW);

    // check if all rmt channels have been used
    assert((stepper_count < MAX_STEPPER_COUNT) && "Cannot create more stepper motor instances! No RMT-Channels left");
    // allocate memory for stepper and rmt and initialize them
    steppers = (tmc2209_t **) realloc(steppers, (stepper_count + 1) * sizeof (tmc2209_t *));
    steppers[stepper_count] = s;
    rmts = (rmt_config_t *) realloc(rmts, (stepper_count + 1) * sizeof (rmt_config_t));
    INITIALIZE_RMT(rmts[stepper_count], stepper_count, s->_step_pin);

    // register tx_end callback once
    if (stepper_count == 0) rmt_register_tx_end_callback(update_stepper, NULL);

    stepper_count++;

    // initialize UART
    if (!serial_initialized) {
        s_serial.begin(115200, SERIAL_8N1, s->_rx_pin, s->_tx_pin);
        serial_initialized = true;
    }

    // set default values
    TMC2209_REGISTER_SET(s->_gconf, GCONF_I_SCALE_ANALOG,  1);
    TMC2209_REGISTER_CLR(s->_gconf, GCONF_INTERNAL_RSENSE, 1);
    TMC2209_REGISTER_CLR(s->_gconf, GCONF_EN_SPREAD_CYCLE, 1);
    TMC2209_REGISTER_SET(s->_gconf, GCONF_MULTISTEP_FILT,  1);
    TMC2209_REGISTER_SET(s->_ihold_irun, IHOLD_IRUN_IHOLDDELAY, 1);
    TMC2209_REGISTER_VAL(s->_tpowerdown, 0, TPOWERDOWN_SIZE, 20);
    TMC2209_REGISTER_VAL(s->_chopconf, 0, CHOPCONF_SIZE, 0x10000053);
    TMC2209_REGISTER_VAL(s->_pwmconf, 0, PWMCONF_SIZE, 0xC10D0024);

    // needed for UART mode to function
    TMC2209_REGISTER_SET(s->_gconf, GCONF_PDN_DISABLE, 1);
    TMC2209_REGISTER_SET(s->_gconf, GCONF_MSTEP_REG_SELECT, 1);
    register_write(s, GCONF_ADRESS, s->_gconf);

    delay(REPLY_DELAY);
    uint8_t retries = 0;
    while(!(s->_communicating = tmc2209_check_connection(s)) && retries++ < 5) {}
    if (retries > 5) return false;
    return true;
}

void tmc2209_set_microsteps(tmc2209_t *s, tmc2209_microstep microsteps)
{
    if (s == NULL) return;

    if (microsteps != TMC2209_MICROSTEPS_NONE &&
        microsteps != TMC2209_MICROSTEPS_2    && microsteps != TMC2209_MICROSTEPS_4  &&
        microsteps != TMC2209_MICROSTEPS_8    && microsteps != TMC2209_MICROSTEPS_16 &&
        microsteps != TMC2209_MICROSTEPS_32   && microsteps != TMC2209_MICROSTEPS_64 &&
        microsteps != TMC2209_MICROSTEPS_128  && microsteps != TMC2209_MICROSTEPS_256)
        return;

    s->_microsteps = microsteps;

    if (s->_communicating) {
        TMC2209_REGISTER_VAL(s->_chopconf, CHOPCONF_MRES, 4, s->_microsteps);
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
            default:
            // NOTE: None, 2, 4, 128 and 256 microsteps are not supported in legacy mode
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

    uint32_t step_delay = step_delay_us < STEP_DELAY_MIN ? STEP_DELAY_MIN : 
                         (step_delay_us > STEP_DELAY_MAX ? STEP_DELAY_MAX : step_delay_us);
    s->_step_delay = step_delay;
}

uint32_t tmc2209_get_step_delay(tmc2209_t *s)
{
    if (s == NULL) return TMC2209_DEFAULT_STEP_DELAY;

    return s->_step_delay;
}

void tmc2209_set_rpm(tmc2209_t *s, uint16_t rpm)
{
    if (s == NULL) return;

    // step_delay * 2 * spr * microsteps = 1 rotation
    uint32_t step_delay = 60000000L / (rpm * s->_steps_per_revolution * MICROSTEPS_TO_NUMBER(s->_microsteps));

    tmc2209_set_step_delay(s, step_delay);
}

void tmc2209_set_direction(tmc2209_t *s, bool dir)
{
    if (s == NULL) return;

    if (dir != TMC2209_CW && dir != TMC2209_CCW) return;

    s->_dir = dir;

    if (s->_communicating) {
        TMC2209_REGISTER_VAL(s->_gconf, GCONF_SHAFT, 1, s->_dir);
        register_write(s, GCONF_ADRESS, s->_gconf);
    } else 
        digitalWrite(s->_dir_pin, s->_dir);
}

bool tmc2209_get_direction(tmc2209_t *s)
{
    if (s == NULL) return TMC2209_CCW;

    return s->_dir; 
}

void tmc2209_step(tmc2209_t *s, uint32_t steps, bool dir)
{
    if (s == NULL) return;

    // otherwise step/dir interface doesn't work
    tmc2209_vactual(s, 0);

    if (s->_step < s->_steps) return;

    tmc2209_set_direction(s, dir);
    s->_step  = 0;
    s->_steps = steps;

    rmt_item32_t pulse;
    pulse.duration0 = s->_step_delay;
    pulse.level0    = 1;
    pulse.duration1 = s->_step_delay;
    pulse.level1    = 0;
    
    for (uint8_t i = 0; i < MAX_STEPPER_COUNT; i++) {
        if (steppers[i] == s) {
            rmt_tx_start((rmt_channel_t)i, true);
            rmt_write_items((rmt_channel_t)i, &pulse, 1, false);
            break;
        }
    }
}

void tmc2209_step_reset(tmc2209_t *s)
{
    if (s == NULL) return;

    s->_step = s->_steps;
}

bool tmc2209_step_is_idle(tmc2209_t *s)
{
    if (s == NULL) return true;

    return (s->_step >= s->_steps);
}

void tmc2209_rotate(tmc2209_t *s, int32_t degree)
{
    if (s == NULL) return;

    uint32_t steps;
    bool dir;

    steps = (abs(degree) * s->_steps_per_revolution * MICROSTEPS_TO_NUMBER(s->_microsteps)) / 360;
    dir   = (degree > 0); // Clockwise = true, Counterclockwise = false

    tmc2209_step(s, steps, dir);
}

void tmc2209_disable(tmc2209_t *s)
{
    if (s == NULL) return;
    if (!s->_communicating) return;

    TMC2209_REGISTER_CLR(s->_chopconf, CHOPCONF_TOFF, 4);
    register_write(s, CHOPCONF_ADDRESS, s->_chopconf);

    digitalWrite(s->_en_pin, HIGH);
}

void tmc2209_enable(tmc2209_t *s)
{
    if (s == NULL) return;
    if (!s->_communicating) return;

    TMC2209_REGISTER_VAL(s->_chopconf, CHOPCONF_TOFF, 4, TOFF_DEFAULT);
    register_write(s, CHOPCONF_ADDRESS, s->_chopconf);

    digitalWrite(s->_en_pin, LOW);
}

bool tmc2209_check_connection(tmc2209_t * s)
{
    if (s == NULL) return false;

    s->_gconf = register_read(s, GCONF_ADRESS) & GCONF_BIT_MASK;
    return (s->_gconf >> GCONF_PDN_DISABLE) & 0x1;
}

void tmc2209_toff(tmc2209_t *s, uint8_t val)
{
    if (s == NULL) return;
    if (!s->_communicating) return;

    TMC2209_REGISTER_VAL(s->_chopconf, CHOPCONF_TOFF, 4, val & 0xF);
    register_write(s, CHOPCONF_ADDRESS, s->_chopconf);
}

void tmc2209_blank_time(tmc2209_t *s, uint8_t clock_cycles)
{
    if (s == NULL) return;
    if (!s->_communicating) return;

    uint8_t val = TBLANK_DEFAULT;

    switch (clock_cycles) {
        case 16:
            val = 0b00;
        break;
        case 24:
            val = 0b01;
        break;
        case 32:
            val = 0b10;
        break;
        case 40:
            val = 0b11;
        break;
    }

    TMC2209_REGISTER_VAL(s->_chopconf, CHOPCONF_TBL, 2, val);
    register_write(s, CHOPCONF_ADDRESS, s->_chopconf);
}

void tmc2209_rms_current(tmc2209_t *s, uint16_t mA, double hold_percentage)
{
    if (s == NULL) return;
    if (!s->_communicating) return;

    uint8_t CS; // current scaling

    // formula from tmc2209 datasheet (Page 50 & 67)
    CS = (uint8_t)(32.0 * 1.41421 * (mA / 1000.0) * (s->_rsense + 0.02) / 0.325 - 1);

    // CS < 16 reduces effective microstep resolution
    if (CS < 16) {
        // turn on high sensitivity rsense
        TMC2209_REGISTER_SET(s->_chopconf, CHOPCONF_VSENSE, 1);
        register_write(s, CHOPCONF_ADDRESS, s->_chopconf);
        CS = 32.0 * 1.41421 * (mA / 1000.0) * (s->_rsense + 0.02) / 0.18 - 1;
    } else {
        // turn of high sensitivity rsense
        TMC2209_REGISTER_CLR(s->_chopconf, CHOPCONF_VSENSE, 1);
        register_write(s, CHOPCONF_ADDRESS, s->_chopconf);
    }

    // limit CS to 31
    CS = CS > 31 ? 31 : CS;

    TMC2209_REGISTER_VAL(s->_ihold_irun, IHOLD_IRUN_IRUN,  5, CS);
    TMC2209_REGISTER_VAL(s->_ihold_irun, IHOLD_IRUN_IHOLD, 5, (uint8_t)(CS * hold_percentage));
    register_write(s, IHOLD_IRUN_ADDRESS, s->_ihold_irun);
}

void tmc2209_tcoolthrs(tmc2209_t *s, uint32_t val)
{
    if (s == NULL) return;
    if (!s->_communicating) return;

    TMC2209_REGISTER_VAL(s->_tcoolthrs, 0, TCOOLTHRS_SIZE, val & TCOOLTHRS_BIT_MASK);
    register_write(s, TCOOLTHRS_ADDRESS, s->_tcoolthrs);
}

void tmc2209_semin(tmc2209_t *s, uint8_t val)
{
    if (s == NULL) return;
    if (!s->_communicating) return;
    
    // limit val to 15
    val = val > 15 ? 15 : val;

    TMC2209_REGISTER_VAL(s->_coolconf, COOLCONF_SEMIN, 4, val);
    register_write(s, COOLCONF_ADDRESS, s->_coolconf);
}

void tmc2209_semax(tmc2209_t *s, uint8_t val)
{
    if (s == NULL) return;
    if (!s->_communicating) return;
    
    // limit val to 15
    val = val > 15 ? 15 : val;

    TMC2209_REGISTER_VAL(s->_coolconf, COOLCONF_SEMAX, 4, val);
    register_write(s, COOLCONF_ADDRESS, s->_coolconf);
}

void tmc2209_sedn(tmc2209_t *s, uint8_t val)
{
    if (s == NULL) return;
    if (!s->_communicating) return;

    // limit val to 3
    val = val > 3 ? 3 : val;

    TMC2209_REGISTER_VAL(s->_coolconf, COOLCONF_SEDN, 2, val);
    register_write(s, COOLCONF_ADDRESS, s->_coolconf);
}

void tmc2209_vactual(tmc2209_t *s, int32_t speed)
{
    if (s == NULL) return;
    if (!s->_communicating) return;

    // limit speed to +- 2^23
    speed = speed >  (1 << 23) - 1 ?  (1 << 23) - 1 : speed;
    speed = speed < -(1 << 23) - 1 ? -(1 << 23) - 1 : speed;

    TMC2209_REGISTER_VAL(s->_vactual, 0, VACTUAL_SIZE, speed);
    register_write(s, VACTUAL_ADDRESS, s->_vactual);
}

uint16_t tmc2209_mscnt(tmc2209_t *s)
{
    if (s == NULL) return 0;
    if (!s->_communicating) return 0;

    s->_mscnt = register_read(s, MSCNT_ADDRESS) & MSCNT_BITMASK;
    return s->_mscnt;
}

void tmc2209_stallguard_thrs(tmc2209_t *s, uint8_t threshold)
{
    if (s == NULL) return;
    if (!s->_communicating) return;

    TMC2209_REGISTER_VAL(s->_sgthrs, 0, SGTHRS_SIZE, threshold);
    register_write(s, SGTHRS_ADDRESS, s->_sgthrs);
}

uint16_t tmc2209_stallguard_result(tmc2209_t *s)
{
    if (s == NULL) return 0;
    if (!s->_communicating) return 0;

    s->_sg_result = register_read(s, SG_RESULT_ADDRESS) & SG_RESULT_BIT_MASK;
    return s->_sg_result;
}

bool tmc2209_is_stalling(tmc2209_t *s)
{
    if (s == NULL) return false;
    if (!s->_communicating) return false;

    s->_sg_result = register_read(s, SG_RESULT_ADDRESS) & SG_RESULT_BIT_MASK;
    return s->_sg_result <= (2 * (uint16_t)s->_sgthrs);
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
        int16_t byte;

        while (s_serial.available() > 0) s_serial.read(); // flush buffer

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
            if (byte < 0) continue;

            reply <<= 8;           // make space for incomming byte
            reply  |= byte & 0xFF; // shift new byte in
            reply  &= 0xFFFFFF;    // due to frame being a uint62_t, only look at first 3 bytes
        }

        // check if timeout occured
        if (timeout == 0) continue;


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
            if (byte < 0) continue;

            reply <<= 8;           // make space for incomming byte
            reply  |= byte & 0xFF; // shift new byte in
            j++;
        }

        // check if timeout occured
        if (timeout == 0) continue;


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

void update_stepper(rmt_channel_t channel, void *arg)
{
    tmc2209_t *s = steppers[(uint8_t)channel];

    if (s->_step < s->_steps) {
        s->_step++;
        rmt_item32_t pulse;
        pulse.duration0 = s->_step_delay;
        pulse.level0    = 1;
        pulse.duration1 = s->_step_delay;
        pulse.level1    = 0;
        rmt_write_items(channel, &pulse, 1, false);
    }
    else rmt_tx_stop(channel);
}