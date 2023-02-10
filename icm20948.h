
#define SERIAL_DEBUG true
#include <stdint.h>
#include <inttypes.h>
#include <I2C.h>
#include "mbed.h"
#include "mbed.h"
#include <math.h>
#include <stdint.h>
#include <inttypes.h>
#define DEG_TO_RAD (1/57.2957795)
#define RAD_TO_DEG 57.2957795
using namespace std::chrono;
Timer t;
I2C i2c(p28, p27);
//static BufferedSerial pc(USBTX, USBRX);

/*float clock_s() { return us_ticker_read() / 1000000.0f; }
uint64_t clock_ms() { return us_ticker_read() / 1000; }
uint64_t clock_us() { return us_ticker_read(); }
*/
// See also ICM-20948 Datasheet, Register Map and Descriptions, Revision 1.3,
// https://www.invensense.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
// and AK09916 Datasheet and Register Map
// https://www.akm.com/akm/en/file/datasheet/AK09916C.pdf

//Magnetometer Registers
#define AK09916_ADDRESS  0x0C 
#define WHO_AM_I_AK09916 0x01 // (AKA WIA2) should return 0x09
#define AK09916_ST1      0x10  // data ready status bit 0
#define AK09916_XOUT_L   0x11  // data
#define AK09916_XOUT_H   0x12
#define AK09916_YOUT_L   0x13
#define AK09916_YOUT_H   0x14
#define AK09916_ZOUT_L   0x15
#define AK09916_ZOUT_H   0x16
#define AK09916_ST2      0x18  // Data overflow bit 3 and data read error status bit 2
#define AK09916_CNTL     0x30  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK09916_CNTL2    0x31  // Normal (0), Reset (1)

// ICM-20948

// USER BANK 0 REGISTER MAP
#define WHO_AM_I_ICM20948  0x00 // Should return 0xEA
#define USER_CTRL          0x03  // Bit 7 enable DMP, bit 3 reset DMP
#define LP_CONFIG          0x05 // Not found in MPU-9250
#define PWR_MGMT_1         0x06 // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x07
#define INT_PIN_CFG        0x0F
#define INT_ENABLE         0x10
#define INT_ENABLE_1       0x11 // Not found in MPU-9250
#define INT_ENABLE_2       0x12 // Not found in MPU-9250
#define INT_ENABLE_3       0x13 // Not found in MPU-9250
#define I2C_MST_STATUS     0x17
#define INT_STATUS         0x19
#define INT_STATUS_1       0x1A // Not found in MPU-9250
#define INT_STATUS_2       0x1B // Not found in MPU-9250
#define INT_STATUS_3       0x1C // Not found in MPU-9250
#define DELAY_TIMEH        0x28 // Not found in MPU-9250
#define DELAY_TIMEL        0x29 // Not found in MPU-9250
#define ACCEL_XOUT_H       0x2D
#define ACCEL_XOUT_L       0x2E
#define ACCEL_YOUT_H       0x2F
#define ACCEL_YOUT_L       0x30
#define ACCEL_ZOUT_H       0x31
#define ACCEL_ZOUT_L       0x32
#define GYRO_XOUT_H        0x33
#define GYRO_XOUT_L        0x34
#define GYRO_YOUT_H        0x35
#define GYRO_YOUT_L        0x36
#define GYRO_ZOUT_H        0x37
#define GYRO_ZOUT_L        0x38
#define TEMP_OUT_H         0x39
#define TEMP_OUT_L         0x3A
#define EXT_SENS_DATA_00   0x3B
#define EXT_SENS_DATA_01   0x3C
#define EXT_SENS_DATA_02   0x3D
#define EXT_SENS_DATA_03   0x3E
#define EXT_SENS_DATA_04   0x3F
#define EXT_SENS_DATA_05   0x40
#define EXT_SENS_DATA_06   0x41
#define EXT_SENS_DATA_07   0x42
#define EXT_SENS_DATA_08   0x43
#define EXT_SENS_DATA_09   0x44
#define EXT_SENS_DATA_10   0x45
#define EXT_SENS_DATA_11   0x46
#define EXT_SENS_DATA_12   0x47
#define EXT_SENS_DATA_13   0x48
#define EXT_SENS_DATA_14   0x49
#define EXT_SENS_DATA_15   0x4A
#define EXT_SENS_DATA_16   0x4B
#define EXT_SENS_DATA_17   0x4C
#define EXT_SENS_DATA_18   0x4D
#define EXT_SENS_DATA_19   0x4E
#define EXT_SENS_DATA_20   0x4F
#define EXT_SENS_DATA_21   0x50
#define EXT_SENS_DATA_22   0x51
#define EXT_SENS_DATA_23   0x52
#define FIFO_EN_1          0x66
#define FIFO_EN_2          0x67 // Not found in MPU-9250
#define FIFO_RST           0x68 // Not found in MPU-9250
#define FIFO_MODE          0x69 // Not found in MPU-9250
#define FIFO_COUNTH        0x70
#define FIFO_COUNTL        0x71
#define FIFO_R_W           0x72
#define DATA_RDY_STATUS    0x74 // Not found in MPU-9250
#define FIFO_CFG           0x76 // Not found in MPU-9250
#define REG_BANK_SEL       0x7F // Not found in MPU-9250

// USER BANK 1 REGISTER MAP
#define SELF_TEST_X_GYRO            0x02
#define SELF_TEST_Y_GYRO            0x03
#define SELF_TEST_Z_GYRO            0x04
#define SELF_TEST_X_ACCEL           0x0E
#define SELF_TEST_Y_ACCEL           0x0F
#define SELF_TEST_Z_ACCEL           0x10
#define XA_OFFSET_H                 0x14
#define XA_OFFSET_L                 0x15
#define YA_OFFSET_H                 0x17
#define YA_OFFSET_L                 0x18
#define ZA_OFFSET_H                 0x1A
#define ZA_OFFSET_L                 0x1B
#define TIMEBASE_CORRECTION_PLL     0x28

// USER BANK 2 REGISTER MAP
#define GYRO_SMPLRT_DIV         0x00 // Not found in MPU-9250
#define GYRO_CONFIG_1           0x01 // Not found in MPU-9250
#define GYRO_CONFIG_2           0x02 // Not found in MPU-9250
#define XG_OFFSET_H             0x03  // User-defined trim values for gyroscope
#define XG_OFFSET_L             0x04
#define YG_OFFSET_H             0x05
#define YG_OFFSET_L             0x06
#define ZG_OFFSET_H             0x07
#define ZG_OFFSET_L             0x08
#define ODR_ALIGN_EN            0x09 // Not found in MPU-9250
#define ACCEL_SMPLRT_DIV_1      0x10 // Not found in MPU-9250
#define ACCEL_SMPLRT_DIV_2      0x11 // Not found in MPU-9250
#define ACCEL_INTEL_CTRL        0x12 // Not found in MPU-9250
#define ACCEL_WOM_THR           0x13 // Not found in MPU-9250 (could be WOM_THR)
#define ACCEL_CONFIG            0x14
#define ACCEL_CONFIG_2          0x15 // Not found in MPU-9250 (could be ACCEL_CONFIG2)
#define FSYNC_CONFIG            0x52 // Not found in MPU-9250
#define TEMP_CONFIG             0x53 // Not found in MPU-9250
#define MOD_CTRL_USR            0x54 // Not found in MPU-9250

// USER BANK 3 REGISTER MAP
#define I2C_MST_ODR_CONFIG      0x00 // Not found in MPU-9250
#define I2C_MST_CTRL            0x01
#define I2C_MST_DELAY_CTRL      0x02
#define I2C_SLV0_ADDR           0x03
#define I2C_SLV0_REG            0x04
#define I2C_SLV0_CTRL           0x05
#define I2C_SLV0_DO             0x06
#define I2C_SLV1_ADDR           0x07
#define I2C_SLV1_REG            0x08
#define I2C_SLV1_CTRL           0x09
#define I2C_SLV1_DO             0x0A
#define I2C_SLV2_ADDR           0x0B
#define I2C_SLV2_REG            0x0C
#define I2C_SLV2_CTRL           0x0D
#define I2C_SLV2_DO             0x0E
#define I2C_SLV3_ADDR           0x0F
#define I2C_SLV3_REG            0x10
#define I2C_SLV3_CTRL           0x11
#define I2C_SLV3_DO             0x12
#define I2C_SLV4_ADDR           0x13
#define I2C_SLV4_REG            0x14
#define I2C_SLV4_CTRL           0x15
#define I2C_SLV4_DO             0x16
#define I2C_SLV4_DI             0x17

// Using the ICM-20948 breakout board, ADO is set to 1
// Seven-bit device address is 1000100 for ADO = 0 and 1000101 for ADO = 1
#define ADO 0 
#if ADO
#define ICM20948_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define ICM20948_ADDRESS 0x68<<1  // Device address when ADO = 0
#define AK09916_ADDRESS  0x0C   // Address of magnetometer
#endif // AD0

#define READ_FLAGS 0x80

    enum Ascale
    {
      AFS_2G = 0,
      AFS_4G,
      AFS_8G,
      AFS_16G
    };

    enum Gscale {
      GFS_250DPS = 0,
      GFS_500DPS,
      GFS_1000DPS,
      GFS_2000DPS
    };

    enum Mscale {
      MFS_14BITS = 0, // 0.6 mG per LSB
      MFS_16BITS      // 0.15 mG per LSB
    };

    enum M_MODE {
      M_8HZ = 0x02,  // 8 Hz update
      M_100HZ = 0x06 // 100 Hz continuous magnetometer
    };

    // TODO: Add setter methods for this hard coded stuff
    // Specify sensor full scale
    uint8_t Gscale = GFS_250DPS;
    uint8_t Ascale = AFS_2G;

    // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    uint8_t Mmode = M_100HZ;

    uint8_t writeByteWire(uint8_t, uint8_t, uint8_t);
 
    uint8_t readByteWire(uint8_t address, uint8_t subAddress);
    


    float pitch, yaw, roll;
    float temperature;   // Stores the real internal chip temperature in Celsius
    int16_t tempCount;   // Temperature raw count output
    uint32_t delt_t = 0; // Used to control display output rate

    uint32_t counts = 0, sumCount = 0; // used to control display output rate
    float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
    uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
    uint32_t Now = 0;        // used to calculate integration interval

    int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
    int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
    // Scale resolutions per LSB for the sensors
    float aRes, gRes, mRes;
    // Variables to hold latest sensor data values
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    // Factory mag calibration and mag bias
    float factoryMagCalibration[3] = {0, 0, 0}, factoryMagBias[3] = {0, 0, 0};
    // Bias corrections for gyro, accelerometer, and magnetometer
    float gyroBias[3]  = {0, 0, 0},
          accelBias[3] = {0, 0, 0},
          magBias[3]   = {0, 0, 0},
          magScale[3]  = {0, 0, 0};
  //  float selfTest[6];
    // Stores the 16-bit signed accelerometer sensor output
    int16_t accelCount[3];

    // Public method declarations
    void getMres();
    void getGres();
    void getAres();
    void readAccelData(int16_t *);
    void readGyroData(int16_t *);
    void readMagData(int16_t *);
    int16_t readTempData();
    void updateTime();
    void initAK09916();
    void initICM20948();
    void calibrateICM20948(float * gyroBias, float * accelBias);
    void ICM20948SelfTest(float * destination);
    void magCalICM20948(float * dest1, float * dest2);
    uint8_t writeByte(uint8_t, uint8_t, uint8_t);
    uint8_t readByte(uint8_t, uint8_t);
    uint8_t readBytes(uint8_t, uint8_t, uint8_t, uint8_t *);
    uint8_t readBytesWire(uint8_t, uint8_t, uint8_t, uint8_t *);
    bool begin();


bool begin(void)
{
    i2c.frequency(400000);  // use fast (400 kHz) I2C 
    t.start();
    return true;
}

void getMres()
{
    mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
}

void getGres()
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GFS_250DPS:
      gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      gRes = 2000.0f / 32768.0f;
      break;
  }
}

void getAres()
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case AFS_2G:
      aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      aRes = 16.0f / 32768.0f;
      break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  // Read the six raw data registers into data array
 // readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
    for(int z=0;z<6;z++)
    {
        rawData[z]=readByte(ICM20948_ADDRESS, ACCEL_XOUT_H+z);
    }
  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = (int16_t)(rawData[0] << 8) | rawData[1];
  destination[1] = (int16_t)(rawData[2] << 8) | rawData[3];
  destination[2] = (int16_t)(rawData[4] << 8) | rawData[5];
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  // Read the six raw data registers sequentially into data array
  readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = (int16_t)(rawData[0] << 8) | rawData[1];
  destination[1] = (int16_t)(rawData[2] << 8) | rawData[3];
  destination[2] = (int16_t)(rawData[4] << 8) | rawData[5];
}

void readMagData(int16_t * destination)
{
  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end
  // of data acquisition
  uint8_t rawData[8];
  // thread_sleep_for for magnetometer data ready bit to be set
  if (readByte(AK09916_ADDRESS, AK09916_ST1) & 0x01)
  {
      
    // Read the six raw data and ST2 registers sequentially into data array
    readBytes(AK09916_ADDRESS, AK09916_XOUT_L, 8, &rawData[0]);
    uint8_t c = rawData[7]; // End data read by reading ST2 register
    // Check if magnetic sensor overflow set, if not then report data
      // Remove once finished
    
    if (!(c & 0x08))
    {
      // Turn the MSB and LSB into a signed 16-bit value
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
      // Data stored as little Endian
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2]; // x/y/z gyro register data stored here
  // Read the two raw data registers sequentially into data array
  readBytes(ICM20948_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);
  // Turn the MSB and LSB into a 16-bit value
  return ((int16_t)rawData[0] << 8) | rawData[1];
}

// Calculate the time the last update took for use in the quaternion filters
// TODO: This doesn't really belong in this class.
void updateTime()
{
  Now = t.elapsed_time().count();;

  // Set integration time by time elapsed since last filter update
  deltat = ((Now - lastUpdate) / 1000000.0f);
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
}

void initAK09916()
{
    
    // Write code to initialise magnetometer
    // Bypass I2C master interface and turn on magnetometer
 // writeByte(ICM20948_ADDRESS, INT_PIN_CFG, 0x02); //Already set in initICM20948

  // Configure the magnetometer for continuous read and highest resolution.
  // Enable continuous mode data acquisition Mmode (bits [3:0]),
  // 0010 for 8 Hz and 0110 for 100 Hz sample rates.

  // Set magnetometer data resolution and sample ODR
  writeByte(AK09916_ADDRESS, AK09916_CNTL2, 0x08);
  thread_sleep_for(10);
}

void initICM20948()
{
    // Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  thread_sleep_for(200);
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
  // Configure Gyro and Thermometer
  // Disable FSYNC and set gyro bandwidth to 51.2 Hz,
  // respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion
  // update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the ICM20948, it is possible to get gyro sample rates of 32 kHz (!),
  // 8 kHz, or 1 kHz
  // Set gyroscope full scale range to 250 dps
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x19);
  writeByte(ICM20948_ADDRESS, TEMP_CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + GYRO_SMPLRT_DIV)
  // Use a 220 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above.
  
  writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x04);

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3

  // Set accelerometer full-scale range configuration
  // Get current ACCEL_CONFIG register value
  uint8_t c = readByte(ICM20948_ADDRESS, ACCEL_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x06;  // Clear AFS bits [4:3]
  c = c | Ascale << 1; // Set full scale range for the accelerometer
  c = c | 0x01; // Set enable accel DLPF for the accelerometer
  c = c | 0x18; // and set DLFPFCFG to 50.4 hz
  // Write new ACCEL_CONFIG register value
  writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, c);

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
  // 1.13 kHz
  writeByte(ICM20948_ADDRESS, ACCEL_SMPLRT_DIV_2, 0x04);
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the GYRO_SMPLRT_DIV setting

  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
  // until interrupt cleared, clear on read of INT_STATUS, and enable
  // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
  // controlled by the Arduino as master.
  writeByte(ICM20948_ADDRESS, INT_PIN_CFG, 0x22);
  // Enable data ready (bit 0) interrupt
  writeByte(ICM20948_ADDRESS, INT_ENABLE_1, 0x01);
}


// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateICM20948(float * gyroBias, float * accelBias)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0}; 
  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, READ_FLAGS);
  thread_sleep_for(200);

  // get stable time source; Auto select clock source to be PLL gyroscope
  // reference if ready else use the internal oscillator, bits 2:0 = 001
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  thread_sleep_for(200);

  // Configure device for bias calculation
  // Disable all interrupts
  writeByte(ICM20948_ADDRESS, INT_ENABLE, 0x00);
  // Disable FIFO
  writeByte(ICM20948_ADDRESS, FIFO_EN_1, 0x00);
  writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x00);
  // Turn on internal clock source
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x00);
  // Disable I2C master
  //writeByte(ICM20948_ADDRESS, I2C_MST_CTRL, 0x00); Already disabled
  // Disable FIFO and I2C master modes
  writeByte(ICM20948_ADDRESS, USER_CTRL, 0x00);
  // Reset FIFO and DMP
  writeByte(ICM20948_ADDRESS, USER_CTRL, 0x08);
  writeByte(ICM20948_ADDRESS, FIFO_RST, 0x1F);
  thread_sleep_for(10);
  writeByte(ICM20948_ADDRESS, FIFO_RST, 0x00);
  thread_sleep_for(15);

  // Set FIFO mode to snapshot
  writeByte(ICM20948_ADDRESS, FIFO_MODE, 0x1F);
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
  // Configure ICM20948 gyro and accelerometer for bias calculation
  // Set low-pass filter to 188 Hz
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x01);
  // Set sample rate to 1 kHz
  writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x00);
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x00);
  // Set accelerometer full-scale to 2 g, maximum sensitivity
  writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, 0x00);

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(ICM20948_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
  // ICM20948)
  writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x1E);
  thread_sleep_for(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  // Disable gyro and accelerometer sensors for FIFO
  writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x00);
  // Read FIFO sample count
  readBytes(ICM20948_ADDRESS, FIFO_COUNTH, 2, &data[0]);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    // Read data for averaging
    readBytes(ICM20948_ADDRESS, FIFO_R_W, 12, &data[0]);
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit
    // biases.
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  if (accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t) accelsensitivity;
  }
  else
  {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup.
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
  // format.
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
  // Biases are additive, so change sign on calculated average gyro biases
  data[1] = (-gyro_bias[0]/4)       & 0xFF;
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);

  // Push gyro biases to hardware registers
  writeByte(ICM20948_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(ICM20948_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(ICM20948_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(ICM20948_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(ICM20948_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(ICM20948_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer
  // bias registers. These registers contain factory trim values which must be
  // added to the calculated accelerometer biases; on boot up these registers
  // will hold non-zero values. In addition, bit 0 of the lower byte must be
  // preserved since it is used for temperature compensation calculations.
  // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.
  
  // Switch to user bank 1
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);
  // A place to hold the factory accelerometer trim biases
  int32_t accel_bias_reg[3] = {0, 0, 0};
  // Read factory accelerometer trim values
  readBytes(ICM20948_ADDRESS, XA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(ICM20948_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(ICM20948_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  // Define mask for temperature compensation bit 0 of lower byte of
  // accelerometer bias registers
  uint32_t mask = 1uL;
  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3] = {0, 0, 0};

  for (ii = 0; ii < 3; ii++)
  {
    // If temperature compensation bit is set, record that fact in mask_bit
    if ((accel_bias_reg[ii] & mask))
    {
      mask_bit[ii] = 0x01;
    }
  }

  // Construct total accelerometer bias, including calculated average
  // accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
  // (16 g full scale)
  accel_bias_reg[0] -= (accel_bias[0]/8);
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0]) & 0xFF;
  // preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1]) & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2]) & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[5] = data[5] | mask_bit[2];

  // Apparently this is not working for the acceleration biases in the ICM-20948
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(ICM20948_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(ICM20948_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(ICM20948_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(ICM20948_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(ICM20948_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(ICM20948_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
  accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
  accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less
// deviation is a pass.
void ICM20948SelfTest(float * destination)
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;
  // Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  thread_sleep_for(200);
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
  // Set gyro sample rate to 1 kHz
  writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x00);
  // Set gyro sample rate to 1 kHz, DLPF to 119.5 Hz and FSR to 250 dps
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x11);
  // Set accelerometer rate to 1 kHz and bandwidth to 111.4 Hz
  // Set full scale range for the accelerometer to 2 g
  writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, 0x11);
  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
  // Get average current values of gyro and acclerometer
  for (int ii = 0; ii < 200; ii++)
  {

    // Read the six raw data registers into data array
    for(int z=0;z<6;z++)
    {
        rawData[z]=readByte(ICM20948_ADDRESS, ACCEL_XOUT_H+z);
    }
    // Turn the MSB and LSB into a signed 16-bit value
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six raw data registers sequentially into data array
    for(int z=0;z<6;z++)
    {
        rawData[z]=readByte(ICM20948_ADDRESS, GYRO_XOUT_H+z);
    }
    // Turn the MSB and LSB into a signed 16-bit value
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  // Get average of 200 values and store as average current readings
  for (int ii =0; ii < 3; ii++)
  {
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
  // Configure the accelerometer for self-test
  // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(ICM20948_ADDRESS, ACCEL_CONFIG_2, 0x1C);
  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_2,  0x38);
  thread_sleep_for(25);  // Delay a while to let the device stabilize
  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
  // Get average self-test values of gyro and acclerometer
  for (int ii = 0; ii < 200; ii++)
  {
    // Read the six raw data registers into data array
   // readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
   for(int z=0;z<6;z++)
    {
        rawData[z]=readByte(ICM20948_ADDRESS, ACCEL_XOUT_H+z);
    }
    // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six raw data registers sequentially into data array
    //readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
    for(int z=0;z<6;z++)
    {
        rawData[z]=readByte(ICM20948_ADDRESS, GYRO_XOUT_H+z);
    }
    // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  // Get average of 200 values and store as average self-test readings
  for (int ii =0; ii < 3; ii++)
  {
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }
  
  // Switch to user bank 2
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
  // Configure the gyro and accelerometer for normal operation
  writeByte(ICM20948_ADDRESS, ACCEL_CONFIG_2, 0x00);
  writeByte(ICM20948_ADDRESS, GYRO_CONFIG_2,  0x00);
  thread_sleep_for(25);  // Delay a while to let the device stabilize
  // Switch to user bank 1
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);
  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  // X-axis accel self-test results
  selfTest[0] = readByte(ICM20948_ADDRESS, SELF_TEST_X_ACCEL);
  // Y-axis accel self-test results
  selfTest[1] = readByte(ICM20948_ADDRESS, SELF_TEST_Y_ACCEL);
  // Z-axis accel self-test results
  selfTest[2] = readByte(ICM20948_ADDRESS, SELF_TEST_Z_ACCEL);
  // X-axis gyro self-test results
  selfTest[3] = readByte(ICM20948_ADDRESS, SELF_TEST_X_GYRO);
  // Y-axis gyro self-test results
  selfTest[4] = readByte(ICM20948_ADDRESS, SELF_TEST_Y_GYRO);
  // Z-axis gyro self-test results
  selfTest[5] = readByte(ICM20948_ADDRESS, SELF_TEST_Z_GYRO);
  // Switch to user bank 0
  writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
  // Retrieve factory self-test value from self-test code reads
  // FT[Xa] factory trim calculation
  factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));
  // FT[Ya] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));
  // FT[Za] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));
  // FT[Xg] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));
  // FT[Yg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));
  // FT[Zg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
  // of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++)
  {
    // Report percent differences
    destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i]- 100./*selfTest[i]*/;
    // Report percent differences
    destination[i+3] =100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]- 100./*selfTest[i+3]*/;
  }
}

// Function which accumulates magnetometer data after device initialization.
// It calculates the bias and scale in the x, y, and z axes.
void magCalICM20948(float * bias_dest, float * scale_dest)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3]  = {0, 0, 0},
          mag_scale[3] = {0, 0, 0};
  int32_t mag_max[3]  = {0x8000, 0x8000, 0x8000},
          mag_min[3]  = {0x7FFF, 0x7FFF, 0x7FFF},
          mag_temp[3] = {0, 0, 0};

  // Make sure resolution has been calculated
  getMres();
  thread_sleep_for(4000);

  // shoot for ~fifteen seconds of mag data
  // at 8 Hz ODR, new mag data is available every 125 ms
  if (Mmode == M_8HZ)
  {
    sample_count = 128;
  }
  // at 100 Hz ODR, new mag data is available every 10 ms
  if (Mmode == M_100HZ)
  {
    sample_count = 1500;
  }

  for (ii = 0; ii < sample_count; ii++)
  {
    readMagData((int16_t *) mag_temp);  // Read the mag data

    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj])
      {
        mag_max[jj] = mag_temp[jj];
      }
      if (mag_temp[jj] < mag_min[jj])
      {
        mag_min[jj] = mag_temp[jj];
      }
    }

    if (Mmode == M_8HZ)
    {
      thread_sleep_for(135); // At 8 Hz ODR, new mag data is available every 125 ms
    }
    if (Mmode == M_100HZ)
    {
      thread_sleep_for(12);  // At 100 Hz ODR, new mag data is available every 10 ms
    }
  }

  // pc.println("mag x min/max:"); pc.println(mag_max[0]); pc.println(mag_min[0]);
  // pc.println("mag y min/max:"); pc.println(mag_max[1]); pc.println(mag_min[1]);
  // pc.println("mag z min/max:"); pc.println(mag_max[2]); pc.println(mag_min[2]);

  // Get hard iron correction
  // Get 'average' x mag bias in counts
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
  // Get 'average' y mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
  // Get 'average' z mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

  // Save mag biases in G for main program
  bias_dest[0] = (float)mag_bias[0] * mRes;// * factoryMagCalibration[0];
  bias_dest[1] = (float)mag_bias[1] * mRes;// * factoryMagCalibration[1];
  bias_dest[2] = (float)mag_bias[2] * mRes;// * factoryMagCalibration[2];

  // Get soft iron correction estimate
  // Get average x axis max chord length in counts
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
  // Get average y axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
  // Get average z axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  scale_dest[0] = avg_rad / ((float)mag_scale[0]);
  scale_dest[1] = avg_rad / ((float)mag_scale[1]);
  scale_dest[2] = avg_rad / ((float)mag_scale[2]);
}

// Wire.h read and write protocols
uint8_t writeByte(uint8_t deviceAddress, uint8_t registerAddress,uint8_t data)
{
  //writeByteWire(deviceAddress,registerAddress, data);
    char tmp[2];
    tmp[0]=registerAddress;
    tmp[1]=data;
    i2c.write(deviceAddress,  tmp, 2, 0); // no stop
    return NULL;
}

uint8_t writeByteOne(uint8_t deviceAddress, uint8_t registerAddress)
{
  char tmp[2];
    tmp[0]=registerAddress;
    i2c.write(deviceAddress,  tmp, 1, 1);
  return NULL;
}

/*
uint8_t writeByteWire(uint8_t deviceAddress, uint8_t registerAddress,
                            uint8_t data)
{ // i2c.write(address, data_write, 1, 1); // no stop
    char tmp[2];
    tmp[0]=registerAddress;
    i2c.write(deviceAddress,  tmp, 1, 1); // no stop
    tmp[0]=data;
    i2c.write(deviceAddress,  tmp, 1, 0); // stop
  // TODO: Fix this to return something meaningful
  return NULL;
}
*/
// Read a byte from given register on device. Calls necessary SPI or I2C
// implementation. This was configured in the constructor.
uint8_t readByte(uint8_t deviceAddress, uint8_t registerAddress)
{
 char tmp[1];
    tmp[0]=registerAddress;
  i2c.write(deviceAddress,tmp, 1, 1); // no stop
  //tmp[0]=data;
  i2c.read(deviceAddress, tmp, 1, 0);//stop
  // Return data read from slave register
  return (uint8_t) tmp[0];
}
/*

uint8_t readByteWire(uint8_t deviceAddress, uint8_t registerAddress)
{
  uint8_t data; // `data` will store the register data
// i2c.write(address, data_write, 1, 1); // no stop
//    i2c.read(address, data, count, 0); 
  // Initialize the Tx buffer
  char tmp[2];
    tmp[0]=registerAddress;
  i2c.write(deviceAddress,tmp, 1, 0); // no stop
  //tmp[0]=data;
  i2c.read(deviceAddress, tmp, 1, 0);//stop
  // Return data read from slave register
  return tmp[0];
}

*/
// Read 1 or more bytes from given register and device using I2C
uint8_t readBytesWire(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest)
{
    char tmp[2];
    tmp[0]=registerAddress;
  i2c.write(deviceAddress, tmp, 1, 1); // no stop
  i2c.read(deviceAddress,(char *) dest, count, 0);//stop
  // Initialize the Tx buffer
/*  Wire.beginTransmission(deviceAddress);
  // Put slave register address in Tx buffer
  Wire.write(registerAddress);
  // Send the Tx buffer, but send a restart to keep connection alive
  Wire.endTransmission(false);

  uint8_t i = 0;
  // Read bytes from slave register address
  Wire.requestFrom(deviceAddress, count);
  while (Wire.available())
  {
    // Put read results in the Rx buffer
    dest[i++] = Wire.read();
  }
*/
  return count; // Return number of bytes written
}

uint8_t readBytes(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest)
{
 
    return readBytesWire(deviceAddress, registerAddress, count, dest);
  
}
