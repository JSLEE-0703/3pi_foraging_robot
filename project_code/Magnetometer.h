/***************************************
 ,        .       .           .     ,-.  
 |        |       |           |        ) 
 |    ,-: |-. ,-. |-. ,-. ,-. |-      /  
 |    | | | | `-. | | |-' |-' |      /   
 `--' `-` `-' `-' ' ' `-' `-' `-'   '--' 
****************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3

class Magnetometer_c {

  public:

    // Instance of the LIS3MDL class used to
    // interact with the magnetometer device.
    LIS3MDL mag;

    // A place to store the latest readings 
    // from the magnetometer
    float readings[ MAX_AXIS ];

    // Constructor, must exist.
    Magnetometer_c () {
      // Leave this empty.
      // If you put Wire.begin() into this function
      // it will crash your microcontroller.
    }

    // Call this function witin your setup() function
    // to initialise the I2C protocol and the
    // magnetometer sensor
    bool initialise() {
  Wire.begin();

  if ( !mag.init() ) {
    return false;
  } else {
    mag.enableDefault();   // ✅ 激活磁力计工作模式！
    return true;
  }
}
// End of initialise()

    // Function to update readings array with
    // latest values from the sensor over i2c
    void getReadings() {
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
    } // End of getReadings()

}; // End of Magnetometer_c class definition

#endif
