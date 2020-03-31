#ifndef MAVLINK_STREAM_GPS2_RAW_H
#define MAVLINK_STREAM_GPS2_RAW_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_gps_position.h>

#include <lib/matrix/matrix/math.hpp>

class MavlinkStreamGPS2Raw : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGPS2Raw::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GPS2_RAW";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS2_RAW;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPS2Raw(mavlink);
	}

	unsigned get_size() override
	{
		return _gps2_sub.advertised() ? (MAVLINK_MSG_ID_GPS2_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _gps2_sub{ORB_ID(vehicle_gps_position), 1};

	/* do not allow top copying this class */
	MavlinkStreamGPS2Raw(MavlinkStreamGPS2Raw &) = delete;
	MavlinkStreamGPS2Raw &operator = (const MavlinkStreamGPS2Raw &) = delete;

protected:
	explicit MavlinkStreamGPS2Raw(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_gps_position_s gps;

		if (_gps2_sub.update(&gps)) {
			mavlink_gps2_raw_t msg = {};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.eph = gps.eph * 1e3f;
			msg.epv = gps.epv * 1e3f;
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s);
			msg.cog = math::degrees(matrix::wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;
			//msg.dgps_numch = // Number of DGPS satellites
			//msg.dgps_age = // Age of DGPS info

			mavlink_msg_gps2_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_GPS2_RAW_H */
