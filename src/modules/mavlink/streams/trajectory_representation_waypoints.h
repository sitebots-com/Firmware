#ifndef MAVLINK_STREAM_TRAJECTORY_REPRESENTATION_WAYPOINTS_H
#define MAVLINK_STREAM_TRAJECTORY_REPRESENTATION_WAYPOINTS_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>

class MavlinkStreamTrajectoryRepresentationWaypoints: public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamTrajectoryRepresentationWaypoints::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "TRAJECTORY_REPRESENTATION_WAYPOINTS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamTrajectoryRepresentationWaypoints(mavlink);
	}

	unsigned get_size() override
	{
		if (_traj_wp_avoidance_sub.advertised()) {
			return MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	uORB::Subscription _traj_wp_avoidance_sub{ORB_ID(vehicle_trajectory_waypoint_desired)};

	/* do not allow top copying this class */
	MavlinkStreamTrajectoryRepresentationWaypoints(MavlinkStreamTrajectoryRepresentationWaypoints &);
	MavlinkStreamTrajectoryRepresentationWaypoints &operator = (const MavlinkStreamTrajectoryRepresentationWaypoints &);

protected:
	explicit MavlinkStreamTrajectoryRepresentationWaypoints(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_trajectory_waypoint_s traj_wp_avoidance_desired;

		if (_traj_wp_avoidance_sub.update(&traj_wp_avoidance_desired)) {
			mavlink_trajectory_representation_waypoints_t msg{};

			msg.time_usec = traj_wp_avoidance_desired.timestamp;
			int number_valid_points = 0;

			for (int i = 0; i < vehicle_trajectory_waypoint_s::NUMBER_POINTS; ++i) {
				msg.pos_x[i] = traj_wp_avoidance_desired.waypoints[i].position[0];
				msg.pos_y[i] = traj_wp_avoidance_desired.waypoints[i].position[1];
				msg.pos_z[i] = traj_wp_avoidance_desired.waypoints[i].position[2];

				msg.vel_x[i] = traj_wp_avoidance_desired.waypoints[i].velocity[0];
				msg.vel_y[i] = traj_wp_avoidance_desired.waypoints[i].velocity[1];
				msg.vel_z[i] = traj_wp_avoidance_desired.waypoints[i].velocity[2];

				msg.acc_x[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[0];
				msg.acc_y[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[1];
				msg.acc_z[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[2];

				msg.pos_yaw[i] = traj_wp_avoidance_desired.waypoints[i].yaw;
				msg.vel_yaw[i] = traj_wp_avoidance_desired.waypoints[i].yaw_speed;

				switch (traj_wp_avoidance_desired.waypoints[i].type) {
				case position_setpoint_s::SETPOINT_TYPE_TAKEOFF:
					msg.command[i] = vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF;
					break;

				case position_setpoint_s::SETPOINT_TYPE_LOITER:
					msg.command[i] = vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM;
					break;

				case position_setpoint_s::SETPOINT_TYPE_LAND:
					msg.command[i] = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
					break;

				default:
					msg.command[i] = UINT16_MAX;
				}

				if (traj_wp_avoidance_desired.waypoints[i].point_valid) {
					number_valid_points++;
				}

			}

			msg.valid_points = number_valid_points;

			mavlink_msg_trajectory_representation_waypoints_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_TRAJECTORY_REPRESENTATION_WAYPOINTS_H */
