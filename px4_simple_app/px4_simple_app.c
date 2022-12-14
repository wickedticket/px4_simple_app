/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>


__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Selecting Control modes");

	/* subscribe to vehicle_attitude_setpoint topic */
	int sensor_sub_fg = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fg, 200);

	/* subscribe to vehicle_control_mode topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_control_mode));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* advertise vehicle_control_mode topic */
	struct vehicle_control_mode_s change;
	memset(&change, 0, sizeof(change));
	orb_advert_t change_pub = orb_advertise(ORB_ID(vehicle_control_mode), &change);

	/* advertise vehicle_attitude_setpoint topic */
	struct vehicle_attitude_setpoint_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 25; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				//Reading from Vehicle_control_status
				/* obtained data for the first file descriptor */
				struct vehicle_control_mode_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_control_mode), sensor_sub_fd, &raw);
				PX4_INFO("Vehicle_control_status:\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",
                    
					(bool)raw.flag_armed,			//# synonym for actuator_armed.armed

					(bool)raw.flag_multicopter_position_control_enabled,

					(bool)raw.flag_control_manual_enabled,		//# true if manual input is mixed in
					(bool)raw.flag_control_auto_enabled,			//# true if onboard autopilot should act
					(bool)raw.flag_control_offboard_enabled	,	//# true if offboard control should be used
					(bool)raw.flag_control_rates_enabled,			//# true if rates are stabilized
					(bool)raw.flag_control_attitude_enabled	,	//# true if attitude stabilization is mixed in
					(bool)raw.flag_control_acceleration_enabled	,	//# true if acceleration is controlled
					(bool)raw.flag_control_velocity_enabled	,	//# true if horizontal velocity (implies direction) is controlled
					(bool)raw.flag_control_position_enabled	,	//# true if position is controlled
					(bool)raw.flag_control_altitude_enabled	,	//# true if altitude is controlled
					(bool)raw.flag_control_climb_rate_enabled	,	//# true if climb rate is controlled
					(bool)raw.flag_control_termination_enabled	);	//# true if flighttermination is enabled

				/* obtained data for the first file descriptor */
				struct vehicle_attitude_setpoint_s att_read;
				/* copy sensors att_read data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude_setpoint), sensor_sub_fg, &att_read);
				//reading values present currently in vehicle_attitude_setpoint
				PX4_INFO("Attitude_setpoints\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
					 (double)att_read.q_d[0],
					 (double)att_read.q_d[1],
					 (double)att_read.q_d[2],
					 (double)att_read.q_d[3],
					 (double)att_read.thrust_body[2]);
				
				PX4_INFO("Updating values in vehicle control mode");
				
					change.flag_armed=1;			//# synonym for actuator_armed.armed
					change.flag_multicopter_position_control_enabled=1;
					change.flag_control_manual_enabled=0;	//# true if manual input is mixed in
					change.flag_control_auto_enabled=1;			//# true if onboard autopilot should act
					change.flag_control_offboard_enabled=0;		//# true if offboard control should be used
					change.flag_control_rates_enabled=1;			//# true if rates are stabilized
					change.flag_control_attitude_enabled=1;	//# true if attitude stabilization is mixed in
					change.flag_control_acceleration_enabled=0;	//# true if acceleration is controlled
					change.flag_control_velocity_enabled=1	;	//# true if horizontal velocity (implies direction) is controlled
					change.flag_control_position_enabled=1	;	//# true if position is controlled
					change.flag_control_altitude_enabled=1	;	//# true if altitude is controlled
					change.flag_control_climb_rate_enabled=1	;	//# true if climb rate is controlled
					change.flag_control_termination_enabled=0	;	//# true if flighttermination is enabled

				 orb_publish(ORB_ID(vehicle_control_mode), change_pub, &change);

				PX4_INFO("Publishing to attitude setpoint");
				att.q_d[0] = 0.0622648+0.001*i;
				att.q_d[1] = 0.1245296+0.001*i;
				att.q_d[2] = 0.1037746+0.001*i;
				att.q_d[3] = 0.9848078+0.001*i; 
				att.thrust_body[2]=-0.5+0.001*i;

				orb_publish(ORB_ID(vehicle_attitude_setpoint), att_pub, &att);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
