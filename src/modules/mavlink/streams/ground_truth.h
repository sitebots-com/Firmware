#ifndef MAVLINK_STREAM_GROUND_TRUTH_H
#define MAVLINK_STREAM_GROUND_TRUTH_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamGroundTruth : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGroundTruth::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GROUND_TRUTH";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIL_STATE_QUATERNION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGroundTruth(mavlink);
	}

	unsigned get_size() override
	{
		return (_att_sub.advertised()
			|| _gpos_sub.advertised()) ? MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	/* do not allow top copying this class */
	MavlinkStreamGroundTruth(MavlinkStreamGroundTruth &) = delete;
	MavlinkStreamGroundTruth &operator = (const MavlinkStreamGroundTruth &) = delete;

protected:
	explicit MavlinkStreamGroundTruth(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_angular_velocity_sub.updated() || _att_sub.updated() || _gpos_sub.updated() || _lpos_sub.updated()) {
			vehicle_attitude_s att{};
			_att_sub.copy(&att);

			vehicle_global_position_s gpos{};
			_gpos_sub.copy(&gpos);

			vehicle_local_position_s lpos{};
			_lpos_sub.copy(&lpos);

			vehicle_angular_velocity_s angular_velocity{};
			_angular_velocity_sub.copy(&angular_velocity);

			mavlink_hil_state_quaternion_t msg{};

			// vehicle_attitude -> hil_state_quaternion
			msg.attitude_quaternion[0] = att.q[0];
			msg.attitude_quaternion[1] = att.q[1];
			msg.attitude_quaternion[2] = att.q[2];
			msg.attitude_quaternion[3] = att.q[3];
			msg.rollspeed = angular_velocity.xyz[0];
			msg.pitchspeed = angular_velocity.xyz[1];
			msg.yawspeed = angular_velocity.xyz[2];

			// vehicle_global_position -> hil_state_quaternion
			// same units as defined in mavlink/common.xml
			msg.lat = gpos.lat * 1e7;
			msg.lon = gpos.lon * 1e7;
			msg.alt = gpos.alt * 1e3f;
			msg.vx = lpos.vx * 1e2f;
			msg.vy = lpos.vy * 1e2f;
			msg.vz = lpos.vz * 1e2f;
			msg.ind_airspeed = 0;
			msg.true_airspeed = 0;
			msg.xacc = lpos.ax;
			msg.yacc = lpos.ay;
			msg.zacc = lpos.az;

			mavlink_msg_hil_state_quaternion_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_GROUND_TRUTH_H */
