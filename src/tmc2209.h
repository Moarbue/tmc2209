#include "Arduino.h"

#ifndef TMC2209_H_
#define TMC2209_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "registers.h"

typedef enum {
    TMC2209_ADDRESS_0  = 0x0b00,
    TMC2209_ADDRESS_1  = 0x0b01,
    TMC2209_ADDRESS_2  = 0x0b10,
    TMC2209_ADDRESS_3  = 0x0b11,
    TMC2209_NO_ADDRESS = 0xFFFF,
} tmc2209_address;

typedef enum {
    TMC2209_MICROSTEPS_8  = 8,
    TMC2209_MICROSTEPS_16 = 16,
    TMC2209_MICROSTEPS_32 = 32,
    TMC2209_MICROSTEPS_64 = 64,
} tmc2209_microstep;

typedef enum {
    TMC2209_CCW = 0,
    TMC2209_CW,
} tmc2209_direction;

#ifndef TMC2209_DEFAULT_STEPS_PER_REVOLUTION
#define TMC2209_DEFAULT_STEPS_PER_REVOLUTION 200 // most stepper motor (1.9° per step)
#endif

#ifndef TMC2209_DEFAULT_MICROSTEPS
#define TMC2209_DEFAULT_MICROSTEPS TMC2209_MICROSTEPS_8
#endif

#ifndef TMC2209_DEFAULT_STEP_DELAY
#define TMC2209_DEFAULT_STEP_DELAY 1000 // µs
#endif

#ifndef TMC2209_DEFAULT_SERIAL
#define TMC2209_DEFAULT_SERIAL 1
#endif

typedef struct {
    uint8_t en_pin;     // enable
    uint8_t dir_pin;    // direction
    uint8_t step_pin;   // step-pulsing
    uint8_t rx_pin;     // UART
    uint8_t tx_pin;     // UART
    uint8_t ms1_pin;    // microstep/address LSB
    uint8_t ms2_pin;    // microstep/address MSB

    tmc2209_address   address;              // address for UART (see enum)
    tmc2209_microstep microsteps;           // divisor for single step (see enum)
    tmc2209_direction dir;                  // spinning direction of the motor
    uint32_t          step_delay;           // delay in µs between one low/high pulse
    uint16_t          steps_per_revolution; // this number * microsteps = steps for one full turn

    uint32_t step;   // current steps (for STEP/DIR interface)
    uint32_t steps;  // final steps (for STEP/DIR interface)

    bool communicating;
    // registers
    GCONF_t        gconf;
    GSTAT_t        gstat;
    IFCNT_t        ifcnt;
    SLAVECONF_t    slaveconf;
    OTP_PROG_t     otp_prog;
    OTP_READ_t     otp_read;
    IOIN_t         ioin;
    FACTORY_CONF_t factory_conf;
    IHOLD_IRUN_t   ihold_irun;
    TPOWERDOWN_t   tpowerdown;
    TSTEP_t        tstep;
    TPWMTHRS_t     tpwmthrs;
    VACTUAL_t      vactual;
    TCOOLTHRS_t    tcoolthrs;
    SGTHRS_t       sgthrs;
    SG_RESULT_t    sg_result;
    COOLCONF_t     coolconf;
    MSCNT_t        mscnt;
    MSCURACT_t     mscuract;
    CHOPCONF_t     chopconf;
    DRV_STATUS_t   drv_status;
    PWMCONF_t      pwmconf;
    PWM_SCALE_t    pwm_scale;
    PWM_AUTO_t     pwm_auto;
} tmc2209_t;

// initialize the driver, both interfaces
bool tmc2209_full(tmc2209_t *s, uint8_t en_pin, uint8_t dir_pin, uint8_t step_pin,
                                      uint8_t rx_pin, uint8_t tx_pin,  uint8_t ms1_pin, uint8_t ms2_pin,
                                      tmc2209_address address);

// set the microsteps of the driver
void tmc2209_set_microsteps(tmc2209_t *s, tmc2209_microstep microsteps);
// get the microsteps of the driver
tmc2209_microstep tmc2209_get_microsteps(tmc2209_t *s);

// set the step delay between one high/low pulse in µs
void tmc2209_set_step_delay(tmc2209_t *s, uint32_t step_delay_us);
// get the step delay between one high/low pulse in µs
uint32_t tmc2209_get_step_delay(tmc2209_t *s);

// set the spin-direction of the motor
void tmc2209_set_direction(tmc2209_t *s, tmc2209_direction dir);
// get the spin-direction of the motor
tmc2209_direction tmc2209_get_direction(tmc2209_t *s);


// functions for moving the motor via STEP/DIR interface

// move the motor by steps and direction
void tmc2209_step(tmc2209_t *s, uint32_t steps, tmc2209_direction dir);

// move the motor by degrees, sign specifies the direction
void tmc2209_rotate(tmc2209_t *s, int32_t degree);

// all functions are non blocking, therefore call update function every loop iteration
void tmc2209_update(tmc2209_t *s);

// disable the driver
void tmc2209_disable(tmc2209_t *s);

// enable the driver
void tmc2209_enable(tmc2209_t *s);

// Check UART connection with the driver
bool check_connection(tmc2209_t * s);

// toff time 0..15
void tmc2209_toff(tmc2209_t *s, uint8_t val);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // TMC2209_H_