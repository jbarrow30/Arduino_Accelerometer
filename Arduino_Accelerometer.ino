#include <Wire.h>

// MMA8451 I2C address
#define MMA8451_I2C 0x1D
#define STATUS_REG 0x00
#define OUT_Z_MSB 0x05
#define OUT_Z_LSB 0x06
#define CTRL_REG1 0x2A

// Acceleration scale factor for 14-bit data and 2g range (1g = 9.7953 m/s^2)
#define ACCEL_SCALE_FACTOR 4096;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // standby mode
  writeRegister(CTRL_REG1, 0x00);

  // Configure MMA8451: 1.56 Hz data rate, low noise, and 14-bit data
  writeRegister(CTRL_REG1, 0xFC); // 11 111 1 0 0
  // Activate the MMA8451
  writeRegister(CTRL_REG1, 0xFD); // 11 111 1 0 1
}

void loop() {
  // Read status register
  uint8_t status = readRegister(STATUS_REG);

  // Check if new acceleration data is available
  if (status & 0x04) {    // status reg AND 00000100
    int16_t z_accel_raw = readZAcceleration();  //if data ready, go get it
        float z_accel_actual = (z_accel_raw * 9.7953) / ACCEL_SCALE_FACTOR; // convert value

    // Print the accel val
    Serial.print("Z-axis acceleration: ");
    Serial.print(z_accel_actual);
    Serial.println(" m/s^2");
  }
}

// Function to read a register value from MMA8451
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MMA8451_I2C);  //use I2C address
  Wire.write(reg);
  Wire.endTransmission(false);  //no no, don't end yet
  Wire.requestFrom(MMA8451_I2C, 1); //grab 1 byte of data
  return Wire.read();
}

// Function to write a value to a register in MMA8451
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MMA8451_I2C);  //use I2C address
  Wire.write(reg);    //define address to write to
  Wire.write(value);  //define what to write to that address
  Wire.endTransmission();
}

// Function to read Z-axis acceleration
int16_t readZAcceleration() {
  uint8_t z_msb = readRegister(OUT_Z_MSB);
  uint8_t z_lsb = readRegister(OUT_Z_LSB);
  return (int16_t)((z_msb << 8) | (z_lsb)) >> 2;    //shift msb 8 and OR with lsb to create 16bit data
                                                    // also shift 16bit data left 2 so that 14 bit data
                                                    //is available at new lsb
}
