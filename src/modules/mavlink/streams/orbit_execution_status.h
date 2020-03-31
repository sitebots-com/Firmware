#ifndef MAVLINK_STREAM_ORBIT_EXECUTION_STATUS_H
#define MAVLINK_STREAM_ORBIT_EXECUTION_STATUS_H

#include "../mavlink_messages.h"

#include <uORB/topics/orbit_status.h>

class MavlinkStreamOrbitStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamOrbitStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ORBIT_EXECUTION_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOrbitStatus(mavlink);
	}

	unsigned get_size() override
	{
		return _orb_status_sub.advertised() ? MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _orb_status_sub{ORB_ID(orbit_status)};

	/* do not allow top copying this class */
	MavlinkStreamOrbitStatus(MavlinkStreamOrbitStatus &);
	MavlinkStreamOrbitStatus &operator = (const MavlinkStreamOrbitStatus &);

protected:
	explicit MavlinkStreamOrbitStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		orbit_status_s _orbit_status;

		if (_orb_status_sub.update(&_orbit_status)) {
			mavlink_orbit_execution_status_t _msg_orbit_execution_status{};

			_msg_orbit_execution_status.time_usec = _orbit_status.timestamp;
			_msg_orbit_execution_status.radius = _orbit_status.radius;
			_msg_orbit_execution_status.frame = _orbit_status.frame;
			_msg_orbit_execution_status.x = _orbit_status.x * 1e7;
			_msg_orbit_execution_status.y = _orbit_status.y * 1e7;
			_msg_orbit_execution_status.z = _orbit_status.z;

			mavlink_msg_orbit_execution_status_send_struct(_mavlink->get_channel(), &_msg_orbit_execution_status);
		}

		return true;
	}
};

#endif /* MAVLINK_STREAM_ORBIT_EXECUTION_STATUS_H */
