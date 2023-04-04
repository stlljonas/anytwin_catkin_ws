/** \file XsensMTDriverTest.cpp
    \brief used to conduct testings on XsensMTDriver
  */

#include <iostream>
#include <string>

#include "xsensmt_interface/XsensMTInterface.h"

using namespace std;

int main(int argc, char** argv) {

    xsensmt::XsensMTInterface xsens_driver;

    /** no need to specify baudrate & port 
     *  will automatically search for them.
     */
    ImuConfig param;
    param.serialKey_ = "R945-E2EF-RPP0-W86J-P408";
    // baudrate needed for timing correction
    param.serialBaudrate_ = 921600; 
    param.outputFrequency_ = 400;
    param.bufferSize_ = 3;
    param.outputOrientation_ = false;
    // enable timing correction to compensate for
    // serial port delay
    param.correctTiming_ = true;

    /** try connect to sensor
     *  only needed for the first time
     *  after this, the XsensMTInterface class will automatically
     *  try to reconnect when sensor is lost.
     */
    while(true){
        /* configure sensor interface */
        if (xsens_driver.init())
            cout << " driver initialization successful" << endl;
        else {
            cout << " driver initialization failed, retry in 0.5s" << endl;
            usleep(500000);
            continue;
        }

        if (xsens_driver.configure(param)){
            cout << " sensor configuration successful" << endl;
            break;
        }
        else {
            int err = xsens_driver.getStatus().errCode_;
            cout << " sensor configuration failed: " 
                 << xsens_driver.errToString(err).c_str()
                 << ", retry in 0.5s" << endl;
            usleep(500000);
            continue;
        }
    }

    /**
     *  demonstration: take 3000 measurements, display results
     *  you can modify this as needed
     */
    ImuMeasurement data;
    bool measurement_start = false;
    chrono::steady_clock::time_point lastPkgTime_;
    int interval_ = 0;
    int i = 0;
    int err;
    while(i<3000){

        /* try to obtain sensor data */
        if( xsens_driver.getMeasurement( data) ){       

             cout << "pkg counter: " << data.counter_
                  << "\t x accel: " << data.linearAcceleration_[0] 
                  << "\t x omega: " << data.angularVelocity_[0] << endl;
             i++;
        }

        /* analysis error code */
        err = xsens_driver.getStatus().errCode_;
        if( err != 0 ){
            cout << "error detected: " << xsens_driver.errToString(err) << endl;
        }

        /* sleep a short while */
        usleep(600);
    }
    
    xsens_driver.cleanup();
    return 0;
}
