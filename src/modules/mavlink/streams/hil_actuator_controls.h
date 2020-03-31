#ifndef MAVLINK_STREAM_HIL_ACTUATOR_CONTROLS_H
#define MAVLINK_STREAM_HIL_ACTUATOR_CONTROLS_H

#include "../mavlink_messages.h"

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>

#include <drivers/drv_pwm_output.h>

class MavlinkStreamHILActuatorControls : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamHILActuatorControls::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "HIL_ACTUATOR_CONTROLS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHILActuatorControls(mavlink);
	}

	unsigned get_size() override
	{
		return _act_sub.advertised() ? MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _act_sub{ORB_ID(actuator_outputs)};

	/* do not allow top copying this class */
	MavlinkStreamHILActuatorControls(MavlinkStreamHILActuatorControls &) = delete;
	MavlinkStreamHILActuatorControls &operator = (const MavlinkStreamHILActuatorControls &) = delete;

protected:
	explicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		actuator_outputs_s act;

		if (_act_sub.update(&act)) {
			vehicle_status_s status{};
			_status_sub.copy(&status);

			if ((status.timestamp > 0) && (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
				/* translate the current system state to mavlink state and mode */
				uint8_t mavlink_state;
				uint8_t mavlink_base_mode;
				uint32_t mavlink_custom_mode;
				mavlink_hil_actuator_controls_t msg = {};

				get_mavlink_mode_state(&status, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

				const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

				unsigned system_type = _mavlink->get_system_type();

				/* scale outputs depending on system type */
				if (system_type == MAV_TYPE_QUADROTOR ||
				    system_type == MAV_TYPE_HEXAROTOR ||
				    system_type == MAV_TYPE_OCTOROTOR ||
				    system_type == MAV_TYPE_VTOL_DUOROTOR ||
				    system_type == MAV_TYPE_VTOL_QUADROTOR ||
				    system_type == MAV_TYPE_VTOL_RESERVED2) {

					/* multirotors: set number of rotor outputs depending on type */

					unsigned n;

					switch (system_type) {
					case MAV_TYPE_QUADROTOR:
						n = 4;
						break;

					case MAV_TYPE_HEXAROTOR:
						n = 6;
						break;

					case MAV_TYPE_VTOL_DUOROTOR:
						n = 2;
						break;

					case MAV_TYPE_VTOL_QUADROTOR:
						n = 4;
						break;

					case MAV_TYPE_VTOL_RESERVED2:
						n = 8;
						break;

					default:
						n = 8;
						break;
					}

					for (unsigned i = 0; i < 16; i++) {
						if (act.output[i] > PWM_DEFAULT_MIN / 2) {
							if (i < n) {
								/* scale PWM out 900..2100 us to 0..1 for rotors */
								msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

							} else {
								/* scale PWM out 900..2100 us to -1..1 for other channels */
								msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
							}

						} else {
							/* send 0 when disarmed and for disabled channels */
							msg.controls[i] = 0.0f;
						}
					}

				} else {
					/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */

					for (unsigned i = 0; i < 16; i++) {
						if (act.output[i] > PWM_DEFAULT_MIN / 2) {
							if (i != 3) {
								/* scale PWM out 900..2100 us to -1..1 for normal channels */
								msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);

							} else {
								/* scale PWM out 900..2100 us to 0..1 for throttle */
								msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
							}

						} else {
							/* set 0 for disabled channels */
							msg.controls[i] = 0.0f;
						}
					}
				}

				msg.time_usec = hrt_absolute_time();
				msg.mode = mavlink_base_mode;
				msg.flags = 0;

				mavlink_msg_hil_actuator_controls_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_HIL_ACTUATOR_CONTROLS_H */
