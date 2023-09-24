#ifndef _ADAFRUIT_MMA8451_H_
#define _ADAFRUIT_MMA8451_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "main.h"

#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON (1.6F)      /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN (275.0F)     /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX                                             \
  (60.0F) /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN                                             \
  (30.0F) /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA                                           \
  (1013.25F) /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS                                                    \
  (0.017453293F) /**< Degrees/s to rad/s multiplier                            \
                  */
#define SENSORS_RADS_TO_DPS                                                    \
  (57.29577793F) /**< Rad/s to degrees/s  multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA                                            \
  (100) /**< Gauss to micro-Tesla multiplier */


#define MMA8451_REG_OUT_X_MSB 0x01 //!< Read-only device output register
#define MMA8451_REG_OUT_X_LSB 0x02 //!< Read-only device output register
#define MMA8451_REG_OUT_Y_MSB 0x03 //!< Read-only device output register
#define MMA8451_REG_OUT_Y_LSB 0x04 //!< Read-only device output register
#define MMA8451_REG_OUT_Z_MSB 0x05 //!< Read-only device output register
#define MMA8451_REG_OUT_Z_LSB 0x06 //!< Read-only device output register
#define MMA8451_REG_SYSMOD 0x0B    //!< SYSMOD system mode register
#define MMA8451_REG_WHOAMI 0x0D    //!< WHO_AM_I device ID register
#define MMA8451_REG_XYZ_DATA_CFG                                               \
  0x0E //!< XYZ_DATA_CFG register, sets dynamic range and high-pass filter for
       //!< output data
#define MMA8451_REG_PL_STATUS                                                  \
  0x10 //!< PL_STATUS portrait/landscape status register
#define MMA8451_REG_PL_CFG 0x11 //!< Portrait/landscape configuration register
#define MMA8451_REG_CTRL_REG1 0x2A //!< CTRL_REG1 system control 1 register
#define MMA8451_REG_CTRL_REG2 0x2B //!< CTRL_REG2 system control 2 register
#define MMA8451_REG_CTRL_REG4 0x2D //!< CTRL_REG4 system control 4 register
#define MMA8451_REG_CTRL_REG5 0x2E //!< CTRL_REG5 system control 5 register

///@{
//* Different portrait and landscape settings */
#define MMA8451_PL_PUF 0
#define MMA8451_PL_PUB 1
#define MMA8451_PL_PDF 2
#define MMA8451_PL_PDB 3
#define MMA8451_PL_LRF 4
#define MMA8451_PL_LRB 5
#define MMA8451_PL_LLF 6
#define MMA8451_PL_LLB 7
///@}

// Differents interrupts
#define INT_EN_FF_MT 0x04
#define INT_EN_DRDY  0x01
#define INT_EN_PULSE 0x08
#define INT_EN_TRANS 0x20


/*!
 * @brief Different range settings
 */
typedef enum {
  MMA8451_RANGE_8_G = 0b10, // +/- 8g
  MMA8451_RANGE_4_G = 0b01, // +/- 4g
  MMA8451_RANGE_2_G = 0b00  // +/- 2g (default value)
} mma8451_range_t;

/*! Used with register 0x2A (MMA8451_REG_CTRL_REG1) to set bandwidth */
typedef enum {
  MMA8451_DATARATE_800_HZ = 0b000,  //  800Hz
  MMA8451_DATARATE_400_HZ = 0b001,  //  400Hz
  MMA8451_DATARATE_200_HZ = 0b010,  //  200Hz
  MMA8451_DATARATE_100_HZ = 0b011,  //  100Hz
  MMA8451_DATARATE_50_HZ = 0b100,   //   50Hz
  MMA8451_DATARATE_12_5_HZ = 0b101, // 12.5Hz
  MMA8451_DATARATE_6_25HZ = 0b110,  // 6.25Hz
  MMA8451_DATARATE_1_56_HZ = 0b111, // 1.56Hz

  MMA8451_DATARATE_MASK = 0b111
} mma8451_dataRate_t;

#define MMA8451_DEFAULT_ADDRESS (0x1D<<1) /*0x1D*/ //!< Default MMA8451 I2C address, if A is GND, its 0x1C

typedef struct{
    int16_t x, y, z;         // Valor en cada eje
    float x_g, y_g, z_g;     // Aceleración en cada eje
    char orientation[25];    // Orientación
} mma8451_t;

// Escritura en un registro del acelerometro
void write_register(uint8_t , uint8_t , I2C_HandleTypeDef, UART_HandleTypeDef);

// Lectura de un registro del acelerometro
uint8_t read_register(uint8_t , I2C_HandleTypeDef, UART_HandleTypeDef);

// Inicializa el acelerometro con una configuración por defecto
void init_MMA8451(I2C_HandleTypeDef, UART_HandleTypeDef);


// Actualiza los datos sobre la orientación
void get_orientation(mma8451_t *data, I2C_HandleTypeDef, UART_HandleTypeDef);

// Actualiza los datos sobre la aceleración
void read(mma8451_t *data, I2C_HandleTypeDef, UART_HandleTypeDef);

// Cambia el rango de aceleración
void set_range(mma8451_range_t range, I2C_HandleTypeDef, UART_HandleTypeDef);

// Devuelve el range actual
mma8451_range_t get_range(I2C_HandleTypeDef, UART_HandleTypeDef);

// Printea la información del acelerometro
void print_info(mma8451_t data, UART_HandleTypeDef);


#endif
