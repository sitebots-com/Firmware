#ifndef MAVLINK_STREAM_VFR_HUD_H
#define MAVLINK_STREAM_VFR_HUD_H

#include "../mavlink_messages.h"

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_local_position.h>

#include <lib/matrix/matrix/math.hpp>

class MavlinkStreamVFRHUD : public MavlinkStream
{
public:

	const char *get_name() const override
	{
		return MavlinkStreamVFRHUD::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "VFR_HUD";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_VFR_HUD;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamVFRHUD(mavlink);
	}

	unsigned get_size() override
	{
		if (_lpos_sub.advertised() || _airspeed_validated_sub.advertised()) {
			return MAVLINK_MSG_ID_VFR_HUD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _act0_sub{ORB_ID(actuator_controls_0)};
	uORB::Subscription _act1_sub{ORB_ID(actuator_controls_1)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	/* do not allow top copying this class */
	MavlinkStreamVFRHUD(MavlinkStreamVFRHUD &) = delete;
	MavlinkStreamVFRHUD &operator = (const MavlinkStreamVFRHUD &) = delete;

protected:
	explicit MavlinkStreamVFRHUD(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_lpos_sub.updated() || _airspeed_validated_sub.updated()) {

			vehicle_local_position_s lpos{};
			_lpos_sub.copy(&lpos);

			actuator_armed_s armed{};
			_armed_sub.copy(&armed);

			airspeed_validated_s airspeed_validated{};
			_airspeed_validated_sub.copy(&airspeed_validated);

			mavlink_vfr_hud_t msg{};
			msg.airspeed = airspeed_validated.indicated_airspeed_m_s;
			msg.groundspeed = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy);
			msg.heading = math::degrees(matrix::wrap_2pi(lpos.yaw));

			if (armed.armed) {
				actuator_controls_s act0{};
				actuator_controls_s act1{};
				_act0_sub.copy(&act0);
				_act1_sub.copy(&act1);

				// VFR_HUD throttle should only be used for operator feedback.
				// VTOLs switch between actuator_controls_0 and actuator_controls_1. During transition there isn't a
				// a single throttle value, but this should still be a useful heuristic for operator awareness.
				//
				// Use ACTUATOR_CONTROL_TARGET if accurate states are needed.
				msg.throttle = 100 * math::max(
						       act0.control[actuator_controls_s::INDEX_THROTTLE],
						       act1.control[actuator_controls_s::INDEX_THROTTLE]);

			} else {
				msg.throttle = 0.0f;
			}

			if (lpos.z_valid && lpos.z_global) {
				/* use local position estimate */
				msg.alt = -lpos.z + lpos.ref_alt;

			} else {
				vehicle_air_data_s air_data{};
				_air_data_sub.copy(&air_data);

				/* fall back to baro altitude */
				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter;
				}
			}

			if (lpos.v_z_valid) {
				msg.climb = -lpos.vz;
			}

			mavlink_msg_vfr_hud_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_VFR_HUD_H */
