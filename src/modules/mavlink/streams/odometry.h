#ifndef MAVLINK_STREAM_ODOMETRY_H
#define MAVLINK_STREAM_ODOMETRY_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_odometry.h>

class MavlinkStreamOdometry : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamOdometry::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ODOMETRY";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ODOMETRY;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOdometry(mavlink);
	}

	unsigned get_size() override
	{
		if (_mavlink->odometry_loopback_enabled()) {
			return _vodom_sub.advertised() ? MAVLINK_MSG_ID_ODOMETRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;

		} else {
			return _odom_sub.advertised() ? MAVLINK_MSG_ID_ODOMETRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
		}
	}

private:
	uORB::Subscription _odom_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _vodom_sub{ORB_ID(vehicle_visual_odometry)};

	/* do not allow top copying this class */
	MavlinkStreamOdometry(MavlinkStreamOdometry &) = delete;
	MavlinkStreamOdometry &operator = (const MavlinkStreamOdometry &) = delete;

protected:
	explicit MavlinkStreamOdometry(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_odometry_s odom;
		// check if it is to send visual odometry loopback or not
		bool odom_updated = false;

		mavlink_odometry_t msg{};

		if (_mavlink->odometry_loopback_enabled()) {
			odom_updated = _vodom_sub.update(&odom);
			// frame matches the external vision system
			msg.frame_id = MAV_FRAME_VISION_NED;

		} else {
			odom_updated = _odom_sub.update(&odom);
			// frame matches the PX4 local NED frame
			msg.frame_id = MAV_FRAME_ESTIM_NED;
		}

		if (odom_updated) {
			msg.time_usec = odom.timestamp;
			msg.child_frame_id = MAV_FRAME_BODY_FRD;

			// Current position
			msg.x = odom.x;
			msg.y = odom.y;
			msg.z = odom.z;

			// Current orientation
			msg.q[0] = odom.q[0];
			msg.q[1] = odom.q[1];
			msg.q[2] = odom.q[2];
			msg.q[3] = odom.q[3];

			// Body-FRD frame to local NED frame Dcm matrix
			matrix::Dcmf R_body_to_local(matrix::Quatf(odom.q));

			// Rotate linear and angular velocity from local NED to body-NED frame
			matrix::Vector3f linvel_body(R_body_to_local.transpose() * matrix::Vector3f(odom.vx, odom.vy, odom.vz));

			// Current linear velocity
			msg.vx = linvel_body(0);
			msg.vy = linvel_body(1);
			msg.vz = linvel_body(2);

			// Current body rates
			msg.rollspeed = odom.rollspeed;
			msg.pitchspeed = odom.pitchspeed;
			msg.yawspeed = odom.yawspeed;

			// get the covariance matrix size

			// pose_covariance
			static constexpr size_t POS_URT_SIZE = sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0]);
			static_assert(POS_URT_SIZE == (sizeof(msg.pose_covariance) / sizeof(msg.pose_covariance[0])),
				      "Odometry Pose Covariance matrix URT array size mismatch");

			// velocity_covariance
			static constexpr size_t VEL_URT_SIZE = sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0]);
			static_assert(VEL_URT_SIZE == (sizeof(msg.velocity_covariance) / sizeof(msg.velocity_covariance[0])),
				      "Odometry Velocity Covariance matrix URT array size mismatch");

			// copy pose covariances
			for (size_t i = 0; i < POS_URT_SIZE; i++) {
				msg.pose_covariance[i] = odom.pose_covariance[i];
			}

			// copy velocity covariances
			//TODO: Apply rotation matrix to transform from body-fixed NED to earth-fixed NED frame
			for (size_t i = 0; i < VEL_URT_SIZE; i++) {
				msg.velocity_covariance[i] = odom.velocity_covariance[i];
			}

			mavlink_msg_odometry_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;

	}
};

#endif /* MAVLINK_STREAM_ODOMETRY_H */
