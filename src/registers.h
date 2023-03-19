#include "Arduino.h"

#ifndef TMC2209_REGISTERS_H_
#define TMC2209_REGISTERS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TMC2209_REGISTER_CLR(register, pos, bits) ((register) &= ~(((1 << (bits)) - 1) << pos))
#define TMC2209_REGISTER_SET(register, pos, bits) ((register) |=  (((1 << (bits)) - 1) << pos))
#define TMC2209_REGISTER_VAL(register, pos, bits, val) do { \
        TMC2209_REGISTER_CLR(register, pos, bits);          \
        (register) |= ((val) << (pos));                     \
    } while (0)

// GCONF settings
typedef uint16_t GCONF_t;
typedef enum {
    GCONF_ADRESS           = 0x00,
    GCONF_I_SCALE_ANALOG   = 0,
    GCONF_INTERNAL_RSENSE  = 1,
    GCONF_EN_SPREAD_CYCLE  = 2,
    GCONF_SHAFT            = 3,
    GCONF_INDEX_OTPW       = 4,
    GCONF_INDEX_STEP       = 5,
    GCONF_PDN_DISABLE      = 6,
    GCONF_MSTEP_REG_SELECT = 7,
    GCONF_MULTISTEP_FILT   = 8,
    GCONF_TEST_MODE        = 9,
    GCONF_BIT_MASK         = 0x3FF, // 0000 0011 1111 1111
} tmc2209_register_gconf;

// GSTAT settings
typedef uint8_t GSTAT_t;
typedef enum {
    GSTAT_ADRESS   = 0x01,
    GSTAT_RESET    = 0,
    GSTAT_DRV_ERR  = 1,
    GSTAT_UV_CP    = 2,
    GSTAT_BIT_MASK = 0x7, // 0000 0111
} tmc2209_register_gstat;

// IFCNT settings
typedef uint8_t IFCNT_t;
#define IFCNT_ADRESS 0x02

// SLAVECONF settings
typedef uint8_t SLAVECONF_t;
// number * time for transmitting 8 bits = SENDDELAY
typedef enum {
    SLAVECONF_ADDRESS      = 0x03,
    SLAVECONF_BIT_MASK     = 0xF, // 0000 1111
} tmc2209_register_slaveconf;

// OTP_PROG settings
typedef uint16_t OTP_PROG_t;
typedef enum {
    OTP_PROG_ADDRESS  = 0x04,
    OTP_PROG_OTPBIT   = 0,
    OTP_PROG_OTPBYTE  = 4,
    OTP_PROG_OTPMAGIC = 8,
    OTP_PROG_BIT_MASK = 0xFF33, // 1111 1111 0011 0011
} tmc2209_register_otp_prog;

// OTP_READ settings
typedef uint32_t OTP_READ_t;
typedef enum {
    OTP_READ_ADDRESS  = 0x05,
    OTP_READ_OTP0     =  0,
    OTP_READ_OTP1     =  8,
    OTP_READ_OTP2     = 16,
    OTP_READ_BIT_MASK = 0xFFFFFF, // 0000 0000 1111 1111 1111 1111 1111 1111
} tmc2209_register_otp_read;

// IOIN settings
typedef uint32_t IOIN_t;
typedef enum {
    IOIN_ADDRESS  = 0x06,
    IOIN_ENN       =  0,
    IOIN_MS1       =  2,
    IOIN_MS2       =  3,
    IOIN_DIAG      =  4,
    IOIN_PDN_UART  =  6,
    IOIN_STEP      =  7,
    IOIN_SPREAD_EN =  8,
    IOIN_DIR       =  9,
    IOIN_VERSION   = 24,
    IOIN_BIT_MASK  = 0xFF0003FF, // 1111 1111 0000 0000 0000 0011 1111 1111
} tmc2209_register_ioin;

// FACTORY_CONF settings
typedef uint16_t FACTORY_CONF_t;
typedef enum {
    FACTORY_CONF_ADDRESS  = 0x07,
    FACTORY_CONF_FCLKTRIM = 0,
    FACTORY_CONF_OTTRIM   = 8,
    FACTORY_CONF_BIT_MASK = 0x31F, // 0000 0011 0001 1111
} tmc2209_register_factory_conf;

// IHOLD_IRUN settings
typedef uint32_t IHOLD_IRUN_t;
typedef enum {
    IHOLD_IRUN_ADDRESS    = 0x10,
    IHOLD_IRUN_IHOLD      =  0,
    IHOLD_IRUN_IRUN       =  8,
    IHOLD_IRUN_IHOLDDELAY = 16,
    IHOLD_IRUN_BIT_MASK   = 0xF1F1F, // 0000 0000 0000 1111 0001 1111 0001 1111
} tmc2209_register_ihold_irun;

// TPOWERDOWN settings
typedef uint8_t TPOWERDOWN_t;
#define TPOWERDOWN_ADDRESS 0x11

// TSTEP settings
typedef uint32_t TSTEP_t;
typedef enum {
    TSTEP_ADRESS   = 0x12,
    TSTEP_BIT_MASK = 0xFFFFF, // 0000 0000 0000 1111 1111 1111 1111 1111
} tmc2209_register_tstep;

// TPWMTHRS settings
typedef uint32_t TPWMTHRS_t;
typedef enum {
    TPWMTHRS_ADRESS   = 0x13,
    TPWMTHRS_BIT_MASK = 0xFFFFF, // 0000 0000 0000 1111 1111 1111 1111 1111
} tmc2209_register_tpwmthrs;

// VACTUAL settings
typedef int32_t VACTUAL_t;
typedef enum {
    VACTUAL_ADDRESS  = 0x22,
    VACTUAL_DISABLE  = 0x00,
    VACTUAL_BIT_MASK = 0xFFFFFF, // 0000 0000 1111 1111 1111 1111 1111 1111
} tmc2209_register_vactual;

// TCOOLTHRS settings
typedef uint32_t TCOOLTHRS_t;
typedef enum {
    TCOOLTHRS_ADDRESS  = 0x14,
    TCOOLTHRS_BIT_MASK = 0xFFFFF, // 0000 0000 0000 1111 1111 1111 1111 1111
} tmc2209_register_tcoolthrs;

// SGTHRS settings
typedef uint8_t SGTHRS_t;
#define SGTHRS_ADDRESS 0x40

// SG_RESULT settings
typedef uint16_t SG_RESULT_t;
typedef enum {
    SG_RESULT_ADDRESS  = 0x41,
    SG_RESULT_BIT_MASK = 0x3FF, // 0000 0011 1111 1111
} tmc2209_register_sg_result;

// COOLCONF settings
typedef uint16_t COOLCONF_t;
typedef enum {
    COOLCONF_ADDRESS  = 0x42,
    COOLCONF_SEIMIN   = 15,
    COOLCONF_SEDN     = 13,
    COOLCONF_SEMAX    =  8,
    COOLCONF_SEUP     =  5,
    COOLCONF_SEMIN    =  0,
    COOLCONF_BIT_MASK = 0xEF6F, // 1110 1111 0110 1111
} tmc2209_register_coolconf;

// MSCNT settings
typedef uint16_t MSCNT_t;
typedef enum {
    MSCNT_ADDRESS  = 0x6A,
    MSCNT_BITMASK  = 0x3FF, // 0000 0011 1111 1111
} tmc2209_register_mscnt;

// MSCURACT settings
typedef uint32_t MSCURACT_t;
typedef enum {
    MSCURACT_ADDRESS  = 0x6B,
    MSCURACT_CUR_A    =  0,
    MSCURACT_CUR_B    = 16,
    MSCURACT_BIT_MASK = 0x1FF01FF, // 0000 0001 1111 1111 0000 0001 1111 1111
} tmc2209_register_mscuract;

// CHOPCONF settings
typedef uint32_t CHOPCONF_t;
typedef enum {
    CHOPCONF_ADDRESS  = 0x6C,
    CHOPCONF_DISS_2VS = 31,
    CHOPCONF_DISS_2G  = 30,
    CHOPCONF_DEDGE    = 29,
    CHOPCONF_INTPOL   = 28,
    CHOPCONF_MRES     = 24,
    CHOPCONF_VSENSE   = 17,
    CHOPCONF_TBL      = 15,
    CHOPCONF_HEND     =  7,
    CHOPCONF_HSTRT    =  4,
    CHOPCONF_TOFF     =  0,
    CHOPCONF_BIT_MASK = 0xFF0387FF, // 1111 1111 0000 0011 1000 0111 1111 1111
} tmc2209_register_chopconf;

// DRV_STATUS settings
typedef uint32_t DRV_STATUS_t;
typedef enum {
    DRV_STATUS_ADDRESS   = 0x6F,
    DRV_STATUS_STST      = 31,
    DRV_STATUS_STEALTH   = 30,
    DRV_STATUS_CS_ACTUAL = 16,
    DRV_STATUS_T157      = 11,
    DRV_STATUS_T150      = 10,
    DRV_STATUS_T143      =  9,
    DRV_STATUS_T120      =  8,
    DRV_STATUS_OLB       =  7,
    DRV_STATUS_OLA       =  6,
    DRV_STATUS_S2VSB     =  5,
    DRV_STATUS_S2VSA     =  4,
    DRV_STATUS_S2GB      =  3,
    DRV_STATUS_S2GA      =  2,
    DRV_STATUS_OT        =  1,
    DRV_STATUS_OTWP      =  0,
    DRV_STATUS_BIT_MASK  = 0x300787FF,   // 0011 0000 0000 0111 1000 0111 1111 1111
} tmc2209_register_drv_status;

// PWMCONF settings
typedef uint32_t PWMCONF_t;
typedef enum {
    PWMCONF_ADDRESS       = 0x70,
    PWMCONF_PWM_LIM       = 28,
    PWMCONF_PWM_REG       = 24,
    PWMCONF_FREEWHEEL     = 20,
    PWMCONF_PWM_AUTOGRAD  = 19,
    PWMCONF_PWM_AUTOSCALE = 18,
    PWMCONF_PWM_FREQ      = 16,
    PWMCONF_PWM_GRAD      =  8,
    PWMCONF_PWM_OFS       =  0,
    PWMCONF_BIT_MASK      = 0xFF3FFFFF,   // 1111 1111 0011 1111 1111 1111 1111 1111
} tmc2209_register_pwmconf;

// PWM_SCALE settings
typedef uint32_t PWM_SCALE_t;
typedef enum {
    PWM_SCALE_ADDRESS  = 0x71,
    PWM_SCALE_SUM      =  0,
    PWM_SCALE_AUTO     = 16,
    PWM_SCALE_BIT_MASK = 0x1FF00FF,     // 0001 1111 1111 0000 0000 1111 1111
} tmc2209_register_pwm_scale;

// PWM_AUTO settings
typedef uint32_t PWM_AUTO_t;
typedef enum {
    PWM_AUTO_ADDRESS       = 0x72,
    PWM_AUTO_PWM_OFS_AUTO  =  0,
    PWM_AUTO_PWM_GRAD_AUTO = 16,
    PWM_AUTO_BIT_MASK      = 0xFF00FF,     // 0000 0000 1111 1111 0000 0000 1111 1111
} tmc2209_register_pwm_auto;


#ifdef __cplusplus
} // extern "C"
#endif

#endif // TMC2209_REGISTERS_H_