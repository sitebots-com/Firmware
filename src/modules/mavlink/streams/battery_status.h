#ifndef MAVLINK_STREAM_BATTERY_STATUS_H
#define MAVLINK_STREAM_BATTERY_STATUS_H

#include "../mavlink_messages.h"

#include <uORB/topics/battery_status.h>

// #include <lib/mathlib/mathlib.h>
#include <math.h>

class MavlinkStreamBatteryStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamBatteryStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "BATTERY_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_BATTERY_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamBatteryStatus(mavlink);
	}

	unsigned get_size() override
	{
		unsigned total_size = 0;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (_battery_status_sub[i].advertised()) {
				total_size += MAVLINK_MSG_ID_BATTERY_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
			}
		}

		return total_size;
	}

private:
	uORB::Subscription _battery_status_sub[ORB_MULTI_MAX_INSTANCES] {
		{ORB_ID(battery_status), 0}, {ORB_ID(battery_status), 1}, {ORB_ID(battery_status), 2}, {ORB_ID(battery_status), 3}
	};

	/* do not allow top copying this class */
	MavlinkStreamBatteryStatus(MavlinkStreamBatteryStatus &) = delete;
	MavlinkStreamBatteryStatus &operator = (const MavlinkStreamBatteryStatus &) = delete;

protected:
	explicit MavlinkStreamBatteryStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
	}

	bool send(const hrt_abstime t) override
	{
		bool updated = false;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			battery_status_s battery_status;

			if (_battery_status_sub[i].update(&battery_status)) {
				/* battery status message with higher resolution */
				mavlink_battery_status_t bat_msg{};
				// TODO: Determine how to better map between battery ID within the firmware and in MAVLink
				bat_msg.id = battery_status.id - 1;
				bat_msg.battery_function = MAV_BATTERY_FUNCTION_ALL;
				bat_msg.type = MAV_BATTERY_TYPE_LIPO;
				bat_msg.current_consumed = (battery_status.connected) ? battery_status.discharged_mah : -1;
				bat_msg.energy_consumed = -1;
				bat_msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100 : -1;
				bat_msg.battery_remaining = (battery_status.connected) ? ceilf(battery_status.remaining * 100.0f) : -1;

				switch (battery_status.warning) {
				case (battery_status_s::BATTERY_WARNING_NONE):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_OK;
					break;

				case (battery_status_s::BATTERY_WARNING_LOW):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_LOW;
					break;

				case (battery_status_s::BATTERY_WARNING_CRITICAL):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_CRITICAL;
					break;

				case (battery_status_s::BATTERY_WARNING_EMERGENCY):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_EMERGENCY;
					break;

				case (battery_status_s::BATTERY_WARNING_FAILED):
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_FAILED;
					break;

				default:
					bat_msg.charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED;
					break;
				}

				// check if temperature valid
				if (battery_status.connected && PX4_ISFINITE(battery_status.temperature)) {
					bat_msg.temperature = battery_status.temperature * 100.0f;

				} else {
					bat_msg.temperature = INT16_MAX;
				}

				static constexpr int mavlink_cells_max = (sizeof(bat_msg.voltages) / sizeof(bat_msg.voltages[0]));
				static constexpr int uorb_cells_max =
					(sizeof(battery_status.voltage_cell_v) / sizeof(battery_status.voltage_cell_v[0]));

				for (int cell = 0; cell < mavlink_cells_max; cell++) {
					if (battery_status.connected && (cell < battery_status.cell_count) && (cell < uorb_cells_max)) {
						bat_msg.voltages[cell] = battery_status.voltage_cell_v[cell] * 1000.0f;

					} else {
						bat_msg.voltages[cell] = UINT16_MAX;
					}
				}

				mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);

				updated = true;
			}

		}

		return updated;
	}
};

#endif /* MAVLINK_STREAM_BATTERY_STATUS_H */
