/* ICM20948 Basic Example Code
 by: Kris Winer
 modified by Eric Nativel MBEB_OS6 port 
 date: 29, MArch 2021
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016
 Demonstrate basic ICM20948 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Pimoroni icm20948 and stm32L432kc nucleo board
 */

#include "mbed.h"
#include "ahrs.h"
#include "icm20948.h"
#include "uLCD_4DGL.h"
#include <cstdio>
#include <stdint.h>

using namespace std::chrono;
Timer t1;
typedef unsigned char byte;
float selft[6];
static BufferedSerial pc(USBTX, USBRX);


char msg[255];

void setup()
{
     //Set up I2C
    
    pc.set_baud(9600);
    pc.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );
  // Reset ICM20948
  begin();
 
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, READ_FLAGS);
  thread_sleep_for(100);
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  thread_sleep_for(100);
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(ICM20948_ADDRESS, WHO_AM_I_ICM20948);
  sprintf(msg,"ICM20948 I AM 0x %x I should be 0x %x",c,0xEA);
  pc.write(msg, strlen(msg));
  if (c == 0xEA) // WHO_AM_I should always be 0x71
  {
    sprintf(msg,"ICM20948 is online...\n");
    pc.write(msg, strlen(msg));
   // writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);
    // Start by performing self test and reporting values
    ICM20948SelfTest(selft);
    sprintf(msg,"x-axis self test: acceleration trim within : %f of factory value\n",selft[0]);
    pc.write(msg, strlen(msg));
    sprintf(msg,"y-axis self test: acceleration trim within : %f of factory value\n",selft[1]);
    pc.write(msg, strlen(msg));
    sprintf(msg,"z-axis self test: acceleration trim within : %f  of factory value\n",selft[2]);
    pc.write(msg, strlen(msg));
    sprintf(msg,"x-axis self test: gyration trim within : %f of factory value\n",selft[3]);
    pc.write(msg, strlen(msg));
    sprintf(msg,"y-axis self test: gyration trim within : %f of factory value\n",selft[4]);
    pc.write(msg, strlen(msg));
    sprintf(msg,"z-axis self test: gyration trim within : %f of factory value\n",selft[5]);
    pc.write(msg, strlen(msg));
        // Calibrate gyro and accelerometers, load biases in bias registers
    calibrateICM20948(gyroBias, accelBias);

    initICM20948();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    sprintf(msg,"ICM20948 initialized for active data mode....\n");
    pc.write(msg, strlen(msg));
        // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    tempCount =readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
    temperature = ((float) tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
    sprintf(msg,"Temperature is %f degrees C\n",temperature);
    pc.write(msg, strlen(msg));
    byte d = readByte(AK09916_ADDRESS<<1, WHO_AM_I_AK09916);
    sprintf(msg,"AK8963 I AM 0x %x  I should be 0x %d\n",d,0x09);
    pc.write(msg, strlen(msg));

    if (d != 0x09)
    {
      // Communication failed, stop here
    sprintf(msg,"Communication with magnetometer failed, abort!\n");
    pc.write(msg, strlen(msg));
    exit(0);
    }

    // Get magnetometer calibration from AK8963 ROM
    initAK09916();
    // Initialize device for active mode read of magnetometer
    sprintf(msg,"AK09916 initialized for active data mode....\n");
    pc.write(msg, strlen(msg));
   
  

    // Get sensor resolutions, only need to do this once
    getAres();
    getGres();
    getMres();
    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    magCalICM20948(magBias, magScale);
    sprintf(msg,"AK09916 mag biases (mG)\n %f\n%f\n%f\n",magBias[0],magBias[1],magBias[2]);
    pc.write(msg, strlen(msg));
   sprintf(msg,"AK09916 mag scale (mG)\n %f\n%f\n%f\n",magScale[0],magScale[1],magScale[2]);
    pc.write(msg, strlen(msg));
    thread_sleep_for(2000); // Add delay to see results before pc spew of data
  } // if (c == 0x71)
  else
  {
    sprintf(msg,"Could not connect to ICM20948: 0x%x",c);
    pc.write(msg, strlen(msg));
    // Communication failed, stop here
    sprintf(msg," Communication failed, abort!\n");
    pc.write(msg, strlen(msg));
    exit(0);
  }
}
int main(void)
{int i=0;
    setup();
while(i<100)
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (readByte(ICM20948_ADDRESS, INT_STATUS_1) & 0x01)
{
    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    ax = (float)accelCount[0] * aRes; // - accelBias[0];
    ay = (float)accelCount[1] * aRes; // - accelBias[1];
    az = (float)accelCount[2] * aRes; // - accelBias[2];
    sprintf(msg,"X-acceleration: %f mg\n",1000*ax);
    pc.write(msg, strlen(msg));
    sprintf(msg,"Y-acceleration: %f mg\n",1000*ay);
    pc.write(msg, strlen(msg));
    sprintf(msg,"Z-acceleration: %f mg\n",1000*az);
    pc.write(msg, strlen(msg));
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    gx = (float)gyroCount[0] * gRes;
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;
sprintf(msg,"x -gyroscope: %f and bias %f deg/s\n",gx,gyroBias[0]);
        pc.write(msg, strlen(msg));
   readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    mx = (float)magCount[0] * mRes - magBias[0];
    my = (float)magCount[1] * mRes - magBias[1];
    mz = (float)magCount[2] * mRes - magBias[2];
   // if (readByte(ICM20948_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // ICM20948, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(ax, ay, az, gx * DEG_TO_RAD,
                         gy * DEG_TO_RAD, gz * DEG_TO_RAD, my,
                         mx, mz, deltat);

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      pitch *= RAD_TO_DEG;
      yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      //    1° 46' E 2021-03-27
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      yaw  -= 1.7666;
      roll *= RAD_TO_DEG;

     
        sprintf(msg,"Yaw %f, Pitch %f, Roll %f\n ",yaw,pitch,roll);
        pc.write(msg, strlen(msg));
    sumCount = 0;
    sum = 0;
}    
      i++;
     //thread_sleep_for(200);
    }
  return 0;
}