// Make use of the CMake build system or compile manually, e.g. with:
// gcc -std=c99 example.c -lsweep

#include <assert.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "../CtrlStruct_gr3.h"

#include <sweep/sweep.h>

// Utility functions for error handling: we simply shut down here; you should do a better job
void die(sweep_error_s error) {
	assert(error);
	fprintf(stderr, "LIDAR Error: %s. Probably not connected...\n", sweep_error_message(error));
	sweep_error_destruct(error);
	exit(EXIT_FAILURE);
}

void init_LIDAR(CtrlStruct *cvs) {

  CtrlIn *inputs = cvs->inputs;

  // Makes sure the installed library is compatible with the interface
  assert(sweep_is_abi_compatible());

  // Grab the port name from the input argument
  const char* port = "/dev/ttyUSB0";

  // All functions which can potentially fail write into an error object
  sweep_error_s error = NULL;

  // Create a Sweep device from the specified USB serial port; there is a second constructor for advanced usage
  sweep_device_s sweep = sweep_device_construct_simple(port, &error);
  if (error) die(error);

  // The Sweep's rotating speed in Hz
  sweep_device_set_motor_speed(sweep, 6, &error);
  int32_t speed = sweep_device_get_motor_speed(sweep, &error);
  if (error) die(error);

  fprintf(stdout, "Motor Speed Setting: %" PRId32 " Hz\n", speed);

  // The Sweep's sample rate in Hz
  sweep_device_set_sample_rate(sweep, LIDAR_SAMPLES, &error);
  int32_t rate = sweep_device_get_sample_rate(sweep, &error);
  if (error) die(error);

  fprintf(stdout, "Sample Rate Setting: %" PRId32 " Hz\n", rate);

  inputs->sweep = sweep;
  inputs->error = error;
}
 
void get_LIDAR_data(CtrlStruct *cvs) {

	CtrlIn *inputs = cvs->inputs;

	sweep_error_s error = inputs->error;
	sweep_device_s sweep = inputs->sweep;

	sweep_device_start_scanning(sweep, &error);
	if (error) die(error);

	// Let's do 10 full 360 degree scans
	for (int32_t num_scans = 0; num_scans < 1; ++num_scans) {
		// This blocks until a full 360 degree scan is available
		sweep_scan_s scan = sweep_device_get_scan(sweep, &error);
		if (error) die(error);

    // int* angle_table = malloc(sizeof(int));

    // For each sample in a full 360 degree scan print angle and distance.
    // In case you're doing expensive work here consider using a decoupled producer / consumer pattern.
		for (int32_t n = 0; n < LIDAR_SAMPLES; ++n) {
			inputs->lidar_angles[n] = (double) (sweep_scan_get_angle(scan, n) % 360000) * M_PI / 180000.0;
			inputs->lidar_distances[n] = (double) sweep_scan_get_distance(scan, n) / 1000.0;
			inputs->lidar_signals[n] = (int) sweep_scan_get_signal_strength(scan, n);
			//printf("LIDAR: Angle %f [rad], Distance %f [m], Signal Strength: %d\n", inputs->lidar_angles[n], inputs->lidar_distances[n], inputs->lidar_signals[n]);
		}

		// Cleanup scan response
		sweep_scan_destruct(scan);

    }
}

void free_LIDAR(CtrlStruct *cvs) {

  CtrlIn *inputs = cvs->inputs;

  // Stop capturing scans
  sweep_device_stop_scanning(inputs->sweep, &inputs->error);
  if (inputs->error) die(inputs->error);

  // Shut down and cleanup Sweep device
  sweep_device_destruct(inputs->sweep);
}
