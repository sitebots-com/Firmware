#ifndef MAVLINK_STREAM_SYS_STATUS_H
#define MAVLINK_STREAM_SYS_STATUS_H

#include "../mavlink_messages.h"

#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/vehicle_status.h>

#include <math.h>

class MavlinkStreamSysStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamSysStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SYS_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYS_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSysStatus(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SYS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _cpuload_sub{ORB_ID(cpuload)};
	uORB::Subscription _battery_status_sub[ORB_MULTI_MAX_INSTANCES] {
		{ORB_ID(battery_status), 0}, {ORB_ID(battery_status), 1}, {ORB_ID(battery_status), 2}, {ORB_ID(battery_status), 3}
	};

	/* do not allow top copying this class */
	MavlinkStreamSysStatus(MavlinkStreamSysStatus &) = delete;
	MavlinkStreamSysStatus &operator = (const MavlinkStreamSysStatus &) = delete;

protected:
	explicit MavlinkStreamSysStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
	}

	bool send(const hrt_abstime t) override
	{
		bool updated_battery = false;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (_battery_status_sub[i].updated()) {
				updated_battery = true;
			}
		}

		if (_status_sub.updated() || _cpuload_sub.updated() || updated_battery) {
			vehicle_status_s status{};
			_status_sub.copy(&status);

			cpuload_s cpuload{};
			_cpuload_sub.copy(&cpuload);

			battery_status_s battery_status[ORB_MULTI_MAX_INSTANCES] {};

			for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
				_battery_status_sub[i].copy(&battery_status[i]);
			}

			int lowest_battery_index = 0;

			for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
				if (battery_status[i].connected && (battery_status[i].remaining < battery_status[lowest_battery_index].remaining)) {
					lowest_battery_index = i;
				}
			}

			mavlink_sys_status_t msg{};

			msg.onboard_control_sensors_present = status.onboard_control_sensors_present;
			msg.onboard_control_sensors_enabled = status.onboard_control_sensors_enabled;
			msg.onboard_control_sensors_health = status.onboard_control_sensors_health;

			msg.load = cpuload.load * 1000.0f;

			// TODO: Determine what data should be put here when there are multiple batteries.
			//  Right now, it uses the lowest battery. This is a safety decision, because if a client is only checking
			//  one battery using this message, it should be the lowest.
			//  In the future, this should somehow determine the "main" battery, or use the "type" field of BATTERY_STATUS
			//  to determine which battery is more important at a given time.
			const battery_status_s &lowest_battery = battery_status[lowest_battery_index];

			if (lowest_battery.connected) {
				msg.voltage_battery = lowest_battery.voltage_filtered_v * 1000.0f;
				msg.current_battery = lowest_battery.current_filtered_a * 100.0f;
				msg.battery_remaining = ceilf(lowest_battery.remaining * 100.0f);

			} else {
				msg.voltage_battery = UINT16_MAX;
				msg.current_battery = -1;
				msg.battery_remaining = -1;
			}

			mavlink_msg_sys_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_SYS_STATUS_H */
