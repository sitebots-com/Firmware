#ifndef MAVLINK_STREAM_GPS_RAW_INT_H
#define MAVLINK_STREAM_GPS_RAW_INT_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_gps_position.h>

#include <lib/matrix/matrix/math.hpp>

class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamGPSRawInt::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "GPS_RAW_INT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPSRawInt(mavlink);
	}

	unsigned get_size() override
	{
		return _gps_sub.advertised() ? MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _gps_sub{ORB_ID(vehicle_gps_position)};

	/* do not allow top copying this class */
	MavlinkStreamGPSRawInt(MavlinkStreamGPSRawInt &) = delete;
	MavlinkStreamGPSRawInt &operator = (const MavlinkStreamGPSRawInt &) = delete;

protected:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_gps_position_s gps;

		if (_gps_sub.update(&gps)) {
			mavlink_gps_raw_int_t msg{};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.alt_ellipsoid = gps.alt_ellipsoid;
			msg.eph = gps.hdop * 100;
			msg.epv = gps.vdop * 100;
			msg.h_acc = gps.eph * 1e3f;
			msg.v_acc = gps.epv * 1e3f;
			msg.vel_acc = gps.s_variance_m_s * 1e3f;
			msg.hdg_acc = gps.c_variance_rad * 1e5f / M_DEG_TO_RAD_F;
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s);
			msg.cog = math::degrees(matrix::wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_GPS_RAW_INT_H */
