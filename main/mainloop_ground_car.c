/*
 * mainloop.c
 *
 *  Created on: 08.10.2010
 *      Author: mackayl
 */


#include "conf.h"

#if PX_VEHICLE_TYPE == PX_GROUND_CAR

#include "mainloop_ground_car.h"
#include "common_mainloop_functions.h"
#include "mainloop_generic.h"

#include "inttypes.h"
#include "mcu_init.h"

// Include comm
#include "comm.h"
#include "mavlink.h"

// Include globals
#include "global_data.h"
#include "sensors.h"
#include "calibration.h"

#include "adc.h"
#include "led.h"
#include "ppm.h"
#include "pwm.h"
#include "sys_time.h"
#include "uart.h"
#include "dac.h"

#include "watchdog.h"
#include "control.h"

#include "compass.h"
#include "acceleration.h"
#include "gyros.h"
#include "motors.h"
#include "servos.h"
#include "radio_control.h"
#include "battery.h"
#include "shutter.h"
#include "infrared_distance.h"
#include "gps.h"
#include "gps_transformations.h"

#include "altitude_speed.h"
#include "global_pos.h"
#include "communication.h"

#include "simple_altitude_moving_average.h"
#include "attitude_compl_euler.h"
#include "altitude_kalman.h"
#include "lookup_sin_cos.h"
#include "least_square.h"

#include "control_quadrotor_attitude.h"
#include "control_fixed_wing_attitude.h"
#include "control_quadrotor_position.h"
#include "control_quadrotor_start_land.h"
#include "remote_control.h"
#include "position_kalman3.h"
#include "vision_buffer.h"

#include "debug.h"
#include "transformation.h"
#include "eeprom.h"
#include "params.h"
#include "attitude_observer.h"
#include "matrix.h"
#include "float_checks.h"

#include "mmc_spi.h"
#include "dos.h"
#include "fat.h"

#include "outdoor_position_kalman.h"
#include "vision_position_kalman.h"

// Executiontime debugging
float_vect3 time_debug;
//static uint32_t count = 0;

// Static variables
// these variables are used during the whole
// code runtime in the mainloop
static const uint32_t min_mainloop_time = 5000;  ///< The minimum wait interval between two mainloop software timer calls, = 1/max rate, initialized to 1 sec = 1000000 microseconds
//static uint32_t loop_max_time = 0;               ///< The maximum time in microseconds one mainloop took
static uint64_t last_mainloop_idle = 0;				///< Starvation Prevention

/**
* @brief Initialize the whole system
*
* All functions that need to be called before the first mainloop iteration
* should be placed here.
*/
void main_init_ground_car(void)
{
	main_init_generic();
}

/**
* @brief This is the main loop
*
* It will be executed at maximum MCU speed (60 Mhz)
*/
void main_loop_ground_car(void)
{
	// Execution time debugging
//	time_debug.x=0;
//	time_debug.y=0;
//	time_debug.z=0;

	last_mainloop_idle = sys_time_clock_get_time_usec();
	debug_message_buffer("Starting main loop");

	while (1)
	{
		// Time Measurement
		uint64_t loop_start_time = sys_time_clock_get_time_usec();

		///////////////////////////////////////////////////////////////////////////
		/// CRITICAL 200 Hz functions
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(5000, COUNTER2, loop_start_time))
		{
			// Kalman Attitude filter, used on all systems
			gyro_read();
			sensors_read_acc();

			sensor_1_mouse_read();
			sensor_2_mouse_read();

			// Read out magnetometer at its default 50 Hz rate
			static uint8_t mag_count = 0;
			if (mag_count == 3)
			{
				sensors_read_mag();
				attitude_observer_correct_magnet(global_data.magnet_corrected);
				mag_count = 0;
			}
			else
			{
				mag_count++;
			}

			// Correction step of observer filter
			attitude_observer_correct_accel(global_data.accel_raw);

			// Write in roll and pitch
			static float_vect3 att; //if not static we have spikes in roll and pitch on bravo !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			attitude_observer_get_angles(&att);
			global_data.attitude.x = att.x;
			global_data.attitude.y = att.y;
			if (global_data.param[PARAM_ATT_KAL_IYAW])
			{
				global_data.attitude.z += 0.005 * global_data.gyros_si.z;
			}
			else
			{
				global_data.attitude.z = att.z;
			}
			// Prediction step of observer
			attitude_observer_predict(global_data.gyros_si);

			//position_integrate(&global_data.attitude,&global_data.position,&global_data.velocity,&global_data.accel_si);

//			if (global_data.state.gps_mode == 1)
//			{
//				outdoor_position_kalman();
//			}
//			else
//			{
////				vision_position_kalman();
//				fuse_vision_altitude_200hz();
//			}

//			control_quadrotor_attitude();

			//debug counting number of executions
//			count++;
		}
		///////////////////////////////////////////////////////////////////////////



//		///////////////////////////////////////////////////////////////////////////
//		/// Camera Shutter - This takes 50 usecs!!!
//		///////////////////////////////////////////////////////////////////////////
//		// Set camera shutter with 2.5ms resolution
//		else if (us_run_every(2500, COUNTER1, loop_start_time))
//		{
//			camera_shutter_handling(loop_start_time);
//
//		// Measure time for debugging
//			time_debug.x = max(time_debug.x, sys_time_clock_get_time_usec()
//					- loop_start_time);
//
//		}

		///////////////////////////////////////////////////////////////////////////
		/// CRITICAL FAST 50 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(20000, COUNTER3, loop_start_time))
		{
			// Read Analog-to-Digital converter
			adc_read();
			// Read remote control
			remote_control();
			// Send the raw sensor/ADC values
			communication_send_raw_data(loop_start_time);
		}
		///////////////////////////////////////////////////////////////////////////




//		///////////////////////////////////////////////////////////////////////////
//		/// CRITICAL FAST 20 Hz functions
//		///////////////////////////////////////////////////////////////////////////
//		else if (us_run_every(50000, COUNTER4, loop_start_time))
//		{
//			//update global_data.state
//			global_data.state.position_fix = global_data.state.vision_ok; //we only have vision atm
//
//			update_system_statemachine(loop_start_time);
//			update_controller_setpoints();
//
//			//STARTING AND LANDING
//			quadrotor_start_land_handler(loop_start_time);
//		}
//		///////////////////////////////////////////////////////////////////////////




//		///////////////////////////////////////////////////////////////////////////
//		/// NON-CRITICAL SLOW 100 Hz functions
//		///////////////////////////////////////////////////////////////////////////
//		else if (us_run_every(10000, COUNTER6, loop_start_time))
//		{
//			// Send the raw sensor/ADC values
////			communication_send_raw_data(loop_start_time);
//		}
//		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// UNCRITICAL SLOW 5 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(200000, COUNTER8, loop_start_time))
		{
			// Send buffered data such as debug text messages
			communication_queued_send();
			// Empty one message out of the buffer
			debug_message_send_one();

			// Toggle status led
			//led_toggle(LED_YELLOW);
//			led_toggle(LED_RED); // just for green LED on alpha at the moment

			// Toggle active mode led
			if (global_data.state.mav_mode == MAV_MODE_MANUAL || global_data.state.mav_mode
					== MAV_MODE_GUIDED || global_data.state.mav_mode == MAV_MODE_AUTO)
			{
//				led_on(LED_GREEN);
			}
			else
			{
//				led_off(LED_GREEN);
			}

			handle_eeprom_write_request();
			handle_reset_request();

			communication_send_remote_control();

			// Pressure sensor driver works, but not tested regarding stability
			sensors_pressure_bmp085_read_out();
		}
		///////////////////////////////////////////////////////////////////////////



		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 1 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(1000000, COUNTER9, loop_start_time))
		{
			// Send system state, mode, battery voltage, etc.
//			send_system_state();

			//update state from recieved parameters
//			sync_state_parameters();

			//enable gps push thru:
			//global_data.param[PARAM_GPS_MODE]=20;

			if (global_data.param[PARAM_GPS_MODE] >= 10)
			{
				//Send GPS information
				float_vect3 gps;
				gps.x = gps_utm_north / 100.0f;//m
				gps.y = gps_utm_east / 100.0f;//m
				gps.z = gps_utm_zone;// gps_week;
				debug_vect("GPS", gps);

			}
			else if (global_data.param[PARAM_GPS_MODE] == 9
					|| global_data.param[PARAM_GPS_MODE] == 8)
			{
//				static float_vect3 gps_local, gps_local_home;
//				static bool gps_local_home_init = false;
//				static float gps_cos_home_lat;

				if (global_data.param[PARAM_GPS_MODE] == 8)
				{
					gps_set_local_origin();
//					gps_local_home_init = false;
				}
				if (gps_lat == 0)
				{
					debug_message_buffer("GPS Signal Lost");
				}
				else
				{
					float_vect3 gps_local, gps_local_velocity;
					gps_get_local_position(&gps_local);
					debug_vect("GPS local", gps_local);
					gps_get_local_velocity(&gps_local_velocity);
					debug_vect("GPS loc velocity", gps_local_velocity);
//					const float r_earth = 6378140;
//					if (!gps_local_home_init)
//					{
//						gps_local_home.x = gps_lat / 1e7f;
//						gps_local_home.y = gps_lon / 1e7f;
//						gps_local_home.z = gps_alt / 100e0f;
//						gps_cos_home_lat = cos(gps_local_home.x * 3.1415 / 180);
//						gps_local_home_init = true;
//						debug_message_buffer("GPS Local Origin saved");
//					}
//
//					gps_local.x = r_earth * tan((gps_lat / 1e7f
//							- gps_local_home.x) * 3.1415 / 180);
//					gps_local.y = r_earth * gps_cos_home_lat * tan((gps_lon
//						/ 1e7f - gps_local_home.y) * 3.1415 / 180);
//					gps_local.z = gps_alt / 100e0f - gps_local_home.z;
//					debug_vect("GPS local", gps_local);
				}
			}
			if (global_data.state.gps_mode)
			{
				gps_send_local_origin();
			}

			beep_on_low_voltage();

			if (global_data.state.mav_mode == MAV_MODE_RC_TRAINING)
			{
				send_system_state();
				static uint8_t uart_unconfigured = 1;
				if (uart_unconfigured)
				{
					// Mode for FMSPIC adapter
					uart1_init(19200, COMM_UART_MODE, UART_FIFO_8);
					uart_unconfigured = 0;
				}
				while (1)
				{
					loop_start_time = sys_time_clock_get_time_usec();

					///////////////////////////////////////////////////////////////////////////
					/// RC INTERFACE HACK AT 50 Hz
					///////////////////////////////////////////////////////////////////////////

					if (us_run_every(20000, COUNTER8, loop_start_time))
					{
						// Write start byte
						uart1_transmit(0xFF);
						// Write channels 1-8
						// The format works with FMS and CRRCSim model flight simulators
						for (int i = 1; i < 9; i++)
						{
							uart1_transmit(clamp((radio_control_get_channel(i)+1)*127, 0, 254));
						}
						led_toggle(LED_RED);
					}
				}
			}
		}
		///////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 20 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(10000, COUNTER7, loop_start_time))
		{
			//led_toggle(LED_YELLOW);
			communication_send_attitude_position(loop_start_time);
		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 200 Hz functions                                     //
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(5000, COUNTER5, loop_start_time))
		{
			if (global_data.state.status == MAV_STATE_STANDBY)
			{
				//Check if parameters should be written or read
				param_handler();
			}

		}
		///////////////////////////////////////////////////////////////////////////

		else {
			// All Tasks are fine and we have no starvation
			last_mainloop_idle = loop_start_time;
		}

		// Read out comm at max rate - takes only a few microseconds in worst case
		communication_receive();

		// MCU load measurement
		uint64_t loop_stop_time = sys_time_clock_get_time_usec();
		global_data.cpu_usage = measure_avg_cpu_load(loop_start_time, loop_stop_time, min_mainloop_time);
		global_data.cpu_peak = measure_peak_cpu_load(loop_start_time, loop_stop_time, min_mainloop_time);
		time_debug.y = max(time_debug.y, global_data.cpu_usage);
		time_debug.z = max(time_debug.z, global_data.cpu_peak);
		if (loop_start_time - last_mainloop_idle >= 5000)
		{
			debug_message_buffer(
					"CRITICAL WARNING! CPU LOAD TO HIGH. STARVATION!");
			last_mainloop_idle = loop_start_time;//reset to prevent multiple messages
		}
		if (global_data.cpu_usage > 800)
		{
			// CPU load higher than 80%
			debug_message_buffer("CRITICAL WARNING! CPU LOAD HIGHER THAN 80%");
		}
	} // End while(1)

}

/**
 * @brief Update the controller gains
 *
 * Calling this function reads the controller gains from the global_data.param[]
 * data structure. This structure can be written from the GCS and is read from
 * EEPROM on startup.
 */
void update_controller_parameters(void)
{
	// "Zero position drift" attitude output
	// This the same as trimming an aircraft (the trim levers on a remote control)
	// To compensate mechanical imprecision
	// Using this parameter allows to exchange the remote control
	// and leave the trim at zero

	global_data.attitude_setpoint_offset.x = global_data.param[PARAM_ATT_OFFSET_X];
	global_data.attitude_setpoint_offset.y = global_data.param[PARAM_ATT_OFFSET_Y];
	global_data.attitude_setpoint_offset.z = global_data.param[PARAM_ATT_OFFSET_Z];


	/// ATTITUDE PID PARAMETERS

	pid_set_parameters(&nick_controller,
			global_data.param[PARAM_PID_ATT_P],
			global_data.param[PARAM_PID_ATT_I],
			global_data.param[PARAM_PID_ATT_D],
			global_data.param[PARAM_PID_ATT_AWU]);
	pid_set_parameters(&roll_controller,
			global_data.param[PARAM_PID_ATT_P],
			global_data.param[PARAM_PID_ATT_I],
			global_data.param[PARAM_PID_ATT_D],
			global_data.param[PARAM_PID_ATT_AWU]);
	pid_set_parameters(&yaw_pos_controller,
			global_data.param[PARAM_PID_YAWPOS_P],
			global_data.param[PARAM_PID_YAWPOS_I],
			global_data.param[PARAM_PID_YAWPOS_D],
			global_data.param[PARAM_PID_YAWPOS_AWU]);
	pid_set_parameters(&yaw_speed_controller,
			global_data.param[PARAM_PID_YAWSPEED_P],
			global_data.param[PARAM_PID_YAWSPEED_I],
			global_data.param[PARAM_PID_YAWSPEED_D],
			global_data.param[PARAM_PID_YAWSPEED_AWU]);


	/// POSITION PID PARAMETERS

	pid_set_parameters(&x_axis_controller,
			global_data.param[PARAM_PID_POS_P],
			global_data.param[PARAM_PID_POS_I],
			global_data.param[PARAM_PID_POS_D],
			global_data.param[PARAM_PID_POS_AWU]);
	pid_set_parameters(&y_axis_controller,
			global_data.param[PARAM_PID_POS_P],
			global_data.param[PARAM_PID_POS_I],
			global_data.param[PARAM_PID_POS_D],
			global_data.param[PARAM_PID_POS_AWU]);
	if (global_data.state.fly != FLY_RAMP_UP)
	{
	pid_set_parameters(&z_axis_controller,
			global_data.param[PARAM_PID_POS_Z_P],
			global_data.param[PARAM_PID_POS_Z_I],
			global_data.param[PARAM_PID_POS_Z_D],
			global_data.param[PARAM_PID_POS_Z_AWU]);
	}
}

void update_controller_setpoints(void)
{
	if (global_data.param[PARAM_POSITIONSETPOINT_ACCEPT])
	{
		//ramp for setpoint changes
		float setpoint_step = 0.03;
		float setpoint_step_yaw = 0.05;// 0.01;// 0.004;
		if (global_data.position_setpoint.x + setpoint_step
				< global_data.param[PARAM_POSITION_SETPOINT_X])
		{
			global_data.position_setpoint.x += setpoint_step;
		}
		else if (global_data.position_setpoint.x - setpoint_step
				> global_data.param[PARAM_POSITION_SETPOINT_X])
		{
			global_data.position_setpoint.x -= setpoint_step;
		}

		if (global_data.position_setpoint.y + setpoint_step
				< global_data.param[PARAM_POSITION_SETPOINT_Y])
		{
			global_data.position_setpoint.y += setpoint_step;
		}
		else if (global_data.position_setpoint.y - setpoint_step
				> global_data.param[PARAM_POSITION_SETPOINT_Y])
		{
			global_data.position_setpoint.y -= setpoint_step;
		}

		if (global_data.position_setpoint.z + setpoint_step
				< global_data.param[PARAM_POSITION_SETPOINT_Z])
		{
			global_data.position_setpoint.z += setpoint_step;
		}
		else if (global_data.position_setpoint.z - setpoint_step
				> global_data.param[PARAM_POSITION_SETPOINT_Z])
		{
			global_data.position_setpoint.z -= setpoint_step;
		}

		if (global_data.yaw_pos_setpoint + setpoint_step_yaw
				< global_data.param[PARAM_POSITION_SETPOINT_YAW])
		{
			global_data.yaw_pos_setpoint += setpoint_step_yaw;
		}
		else if (global_data.yaw_pos_setpoint - setpoint_step_yaw
				> global_data.param[PARAM_POSITION_SETPOINT_YAW])
		{
			global_data.yaw_pos_setpoint -= setpoint_step_yaw;
		}

	}
}

#endif
