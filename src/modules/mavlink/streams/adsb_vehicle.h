#ifndef MAVLINK_STREAM__H
#define MAVLINK_STREAM__H

#include "../mavlink_messages.h"

#include <uORB/topics/transponder_report.h>

class MavlinkStreamADSBVehicle : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamADSBVehicle::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ADSB_VEHICLE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ADSB_VEHICLE;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamADSBVehicle(mavlink);
	}

	bool const_rate() override
	{
		return true;
	}

	unsigned get_size() override
	{
		return _pos_sub.advertised() ? MAVLINK_MSG_ID_ADSB_VEHICLE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _pos_sub{ORB_ID(transponder_report)};

	/* do not allow top copying this class */
	MavlinkStreamADSBVehicle(MavlinkStreamADSBVehicle &) = delete;
	MavlinkStreamADSBVehicle &operator = (const MavlinkStreamADSBVehicle &) = delete;

protected:
	explicit MavlinkStreamADSBVehicle(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		transponder_report_s pos;
		bool sent = false;

		while (_pos_sub.update(&pos)) {

			if (!(pos.flags & transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE)) {
				continue;
			}

			mavlink_adsb_vehicle_t msg{};
			msg.ICAO_address = pos.icao_address;
			msg.lat = pos.lat * 1e7;
			msg.lon = pos.lon * 1e7;
			msg.altitude_type = pos.altitude_type;
			msg.altitude = pos.altitude * 1e3f;
			msg.heading = (pos.heading + M_PI_F) / M_PI_F * 180.0f * 100.0f;
			msg.hor_velocity = pos.hor_velocity * 100.0f;
			msg.ver_velocity = pos.ver_velocity * 100.0f;
			memcpy(&msg.callsign[0], &pos.callsign[0], sizeof(msg.callsign));
			msg.emitter_type = pos.emitter_type;
			msg.tslc = pos.tslc;
			msg.squawk = pos.squawk;

			msg.flags = 0;

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS) { msg.flags |= ADSB_FLAGS_VALID_COORDS; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE) { msg.flags |= ADSB_FLAGS_VALID_ALTITUDE; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING) { msg.flags |= ADSB_FLAGS_VALID_HEADING; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY) { msg.flags |= ADSB_FLAGS_VALID_VELOCITY; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) { msg.flags |= ADSB_FLAGS_VALID_CALLSIGN; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK) { msg.flags |= ADSB_FLAGS_VALID_SQUAWK; }

			mavlink_msg_adsb_vehicle_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

#endif /* MAVLINK_STREAM__H */
