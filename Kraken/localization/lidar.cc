// Make use of the CMake build system or compile manually, e.g. with:
// gcc -std=c99 example.c -lsweep

#include <assert.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "../CtrlStruct_gr3.h"
#include "../useful/mytime.h"

#include "lidar.h"
#include "rplidar.h"

using namespace rp::standalone::rplidar;

void init_LIDAR(CtrlStruct *cvs) {

  CtrlIn *inputs = cvs->inputs;
  
  const char * opt_com_path = "/dev/ttyUSB0";
  
  // create the driver instance
  RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
  
  if(!drv)
      drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
  if(IS_OK(drv->connect(opt_com_path, 115200)))
  {
      rplidar_response_device_info_t devinfo;
      if (IS_OK(drv->getDeviceInfo(devinfo))) 
      {
	  fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
	      , opt_com_path);
      }
      else
      {
	  delete drv;
	  drv = NULL;
      }
  }
  
  drv->startMotor();
  drv->startScan(0,1);
  
  inputs->drv = drv;
  
}
 
void get_LIDAR_data(CtrlStruct *cvs) {

	CtrlIn *inputs = cvs->inputs;
	RPlidarDriver *drv = inputs->drv;

        rplidar_response_measurement_node_t nodes[8192];
        size_t   count = _countof(nodes);

        if (IS_OK(drv->grabScanData(nodes, count))) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                /*printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                    nodes[pos].distance_q2/4.0f,
                    nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);*/
		
		inputs->lidar_signals[pos] = (int) (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
		if (inputs->lidar_signals[pos] > 0) {
		  inputs->lidar_distances[pos] = (double) (nodes[pos].distance_q2/4000.0f);
		  inputs->lidar_angles[pos]     = (double) (-(nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/11520.0f * M_PI) + M_PI + 1.5 * M_PI / 180.0;
		} else {
		  inputs->lidar_angles[pos]    = 0.0;
		  inputs->lidar_distances[pos] = 99.0;
		}
		//printf("Get LIDAR data: %d %f %f\n", inputs->lidar_signals[pos], inputs->lidar_angles[pos] * 180 / M_PI, inputs->lidar_distances[pos]);
            }
	    //printf("LIDAR count: %d\n", count);
	    inputs->lidar_count = count;
        }
}

void free_LIDAR(CtrlStruct *cvs) {

  CtrlIn *inputs = cvs->inputs;
  RPlidarDriver *drv = inputs->drv;
  
  drv->stop();
  drv->stopMotor();
  RPlidarDriver::DisposeDriver(drv);
  
}
