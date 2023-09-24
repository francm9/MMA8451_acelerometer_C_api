#include "MMA_8451.h"

void write_register(uint8_t reg, uint8_t value, I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2){
	uint8_t buffer[2];
	buffer[0] = reg;
	buffer[1] = value;

	while(HAL_I2C_IsDeviceReady(&hi2c1, MMA8451_DEFAULT_ADDRESS, 1, HAL_MAX_DELAY) != HAL_OK);

    while (HAL_I2C_Master_Transmit(&hi2c1, MMA8451_DEFAULT_ADDRESS, buffer, 2, HAL_MAX_DELAY) != HAL_OK){
        // Error handler
        char msg[24];
        sprintf(msg, "Error writing register\n\r");
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

uint8_t read_register(uint8_t reg, I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2){
    uint8_t data[1];

    while(HAL_I2C_IsDeviceReady(&hi2c1, MMA8451_DEFAULT_ADDRESS, 1, HAL_MAX_DELAY) != HAL_OK);

    //Indicamos de qué registro queremos leer
    while (HAL_I2C_Mem_Read(&hi2c1, MMA8451_DEFAULT_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY) != HAL_OK){
        // Error handler
        char msg[24];
        sprintf(msg, "Error reading register\n\r");
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }

    return data[0];
}

void init_MMA8451(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2){
    //Inicialización del acelerómetro
    write_register(MMA8451_REG_CTRL_REG2, 0x40, hi2c1, huart2);             //Reset

    // Espera a que se resetee
    while ((read_register(MMA8451_REG_CTRL_REG2, hi2c1, huart2) & 0x40) >> 6);

    // Se establece de rango 4G
    write_register(MMA8451_REG_XYZ_DATA_CFG, MMA8451_RANGE_4_G, hi2c1, huart2);

    // Se establece la frecuencia de muestreo a 200Hz
    write_register(MMA8451_REG_CTRL_REG1, MMA8451_DATARATE_200_HZ, hi2c1, huart2);

    // -- interrupt en INT1
    write_register(MMA8451_REG_CTRL_REG4, INT_EN_DRDY, hi2c1, huart2);             // Activamos la interrupción
    write_register(MMA8451_REG_CTRL_REG5, 0x01, hi2c1, huart2);             		// Se establece la interrupción en INT1

    // Activar detección de orientación
    write_register(MMA8451_REG_PL_CFG, 0x40, hi2c1, huart2);

    // Modo activo con ruido reducido
    write_register(MMA8451_REG_CTRL_REG1, 0x01 | 0x04, hi2c1, huart2);

}

// Actualiza la orientación del acelerómetro
void get_orientation(mma8451_t *data, I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2){
    uint8_t raw_data = read_register(MMA8451_REG_PL_STATUS, hi2c1, huart2);
    switch (raw_data & 0x07) {
    case MMA8451_PL_PUF:
        strcpy(data -> orientation, "Portrait Up Front");
        break;
    case MMA8451_PL_PUB:
        strcpy(data -> orientation, "Portrait Up Back");
        break;
    case MMA8451_PL_PDF:
        strcpy(data -> orientation, "Portrait Down Front");
        break;
    case MMA8451_PL_PDB:
        strcpy(data -> orientation, "Portrait Down Back");
        break;
    case MMA8451_PL_LRF:
        strcpy(data -> orientation, "Landscape Right Front");
        break;
    case MMA8451_PL_LRB:
        strcpy(data -> orientation, "Landscape Right Back");
        break;
    case MMA8451_PL_LLF:
        strcpy(data -> orientation, "Landscape Left Front");
        break;
    case MMA8451_PL_LLB:
        strcpy(data -> orientation, "Landscape Left Back");
        break;
    }
}

// Devuelve el valor del range que se está utilizando
mma8451_range_t get_range(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2){
    return (mma8451_range_t) (read_register(MMA8451_REG_XYZ_DATA_CFG, hi2c1, huart2) & 0x03);
}

// Actualiza los valores de los ejes del acelerómetro
void read(mma8451_t *data, I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2){
    // Actualizamos el valor de x
    uint8_t x_msb = read_register(MMA8451_REG_OUT_X_MSB, hi2c1, huart2);
    uint8_t x_lsb = read_register(MMA8451_REG_OUT_X_LSB, hi2c1, huart2);
    data -> x = x_msb;
    data -> x <<= 8;
    data -> x |= x_lsb;
    data -> x >>= 2;

    // Actualizamos el valor de y
    uint8_t y_msb = read_register(MMA8451_REG_OUT_Y_MSB, hi2c1, huart2);
    uint8_t y_lsb = read_register(MMA8451_REG_OUT_Y_LSB, hi2c1, huart2);
    data -> y = y_msb;
    data -> y <<= 8;
    data -> y |= y_lsb;
    data -> y >>= 2;

    // Actualizamos el valor de z
    uint8_t z_msb = read_register(MMA8451_REG_OUT_Z_MSB, hi2c1, huart2);
    uint8_t z_lsb = read_register(MMA8451_REG_OUT_Z_LSB, hi2c1, huart2);
    data -> z = z_msb;
    data -> z <<= 8;
    data -> z |= z_lsb;
    data -> z >>= 2;

    // Actualizamos aceleraciones
    uint8_t range = get_range(hi2c1, huart2);
    uint16_t divider = 1;
    if (range == MMA8451_RANGE_8_G)
        divider = 1024;
    if (range == MMA8451_RANGE_4_G)
        divider = 2048;
    if (range == MMA8451_RANGE_2_G)
        divider = 4096;

    // Aceleración en m/s^2
    data -> x_g = ((float) (data -> x) / divider) * SENSORS_GRAVITY_STANDARD;
    data -> y_g = ((float) (data -> y) / divider) * SENSORS_GRAVITY_STANDARD;
    data -> z_g = ((float) (data -> z) / divider) * SENSORS_GRAVITY_STANDARD;

}

// Actualiza el range del acelerómetro
void set_range(mma8451_range_t range, I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2){
    uint8_t reg = read_register(MMA8451_REG_CTRL_REG1, hi2c1, huart2);
    write_register(MMA8451_REG_CTRL_REG1, 0x00, hi2c1, huart2);             // Se desactiva el acelerómetro
    write_register(MMA8451_REG_XYZ_DATA_CFG, range & 0x03, hi2c1, huart2);  // Se establece el nuevo range
    write_register(MMA8451_REG_CTRL_REG1, reg | 0x01, hi2c1, huart2);       // Se vuelve a activar el acelerómetro
}

// Printea la información del acelerómetro
void print_info(mma8451_t data, UART_HandleTypeDef huart2){
    char message[200];
    sprintf(message ,"POSICION\n\r"
           "x = %d \n\ry= %d \n\rz= %d \n\n\r"
           "ACELERACION\n\r"
           "x = %f \n\ry= %f \n\rz= %f \n\n\r"
           "ORIENTACION -> %s \n\r", data.x, data.y, data.z,
                                     data.x_g, data.y_g, data.z_g,
                                     data.orientation);
    HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message), HAL_MAX_DELAY);
}
