#include "io/mavsdk_vehicle.hpp"

#include <mavsdk/mavlink/ardupilotmega/mavlink.h>  ///< MAVLink C headers bundled with MAVSDK

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <functional>
#include <mutex>
#include <numbers>
#include <vector>

using namespace std::chrono_literals;
using namespace path3d;
using Mode = mavsdk::Telemetry::FlightMode;

static_assert(to_flight_mode(Mode::Offboard) == FlightMode::External);
static_assert(to_flight_mode(Mode::Hold) == FlightMode::Hold);
static_assert(to_flight_mode(Mode::Manual) == FlightMode::Manual);
static_assert(to_flight_mode(Mode::Stabilized) == FlightMode::Manual);
static_assert(to_flight_mode(Mode::Altctl) == FlightMode::Manual);
static_assert(to_flight_mode(Mode::Posctl) == FlightMode::Manual);
static_assert(to_flight_mode(Mode::ReturnToLaunch) == FlightMode::ReturnHome);
static_assert(to_flight_mode(Mode::Land) == FlightMode::Land);
static_assert(to_flight_mode(Mode::Mission) == FlightMode::Unknown);
static_assert(to_flight_mode(Mode::Unknown) == FlightMode::Unknown);

namespace {

bool wait_for(const std::function<bool()>& done, std::chrono::milliseconds timeout) {
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	while (!done()) {
		if (std::chrono::steady_clock::now() > deadline) return false;
		std::this_thread::sleep_for(10ms);
	}
	return true;
}

std::uint16_t free_udp_port() {
	const int probe = ::socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in address{};
	address.sin_family = AF_INET;
	inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
	::bind(probe, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
	socklen_t length = sizeof(address);
	::getsockname(probe, reinterpret_cast<sockaddr*>(&address), &length);
	::close(probe);
	return ntohs(address.sin_port);
}

/// Plays ArduCopter: heartbeat every 50 ms, LOCAL_POSITION_NED every 10 ms, acknowledges commands
/// (and applies arm, mode, land and RTL commands like the real thing), records what it receives.
class FakeAutopilot {
public:
	static constexpr int kNoAnswer = -1;

	explicit FakeAutopilot(std::uint16_t vehicle_port) {
		m_socket = ::socket(AF_INET, SOCK_DGRAM, 0);
		sockaddr_in self{};
		self.sin_family = AF_INET;
		inet_pton(AF_INET, "127.0.0.1", &self.sin_addr);
		::bind(m_socket, reinterpret_cast<const sockaddr*>(&self), sizeof(self));
		m_vehicle.sin_family = AF_INET;
		m_vehicle.sin_port = htons(vehicle_port);
		inet_pton(AF_INET, "127.0.0.1", &m_vehicle.sin_addr);
		m_thread = std::thread([this] { loop(); });
	}
	~FakeAutopilot() {
		m_stop = true;
		m_thread.join();
		::close(m_socket);
	}

	std::atomic<std::uint32_t> custom_mode{5};
	std::atomic<bool> armed{false};
	std::atomic<int> ack_result{MAV_RESULT_ACCEPTED};  ///< kNoAnswer: never acknowledge

	std::vector<mavlink_command_long_t> commands(std::uint16_t command) const {
		std::lock_guard<std::mutex> lock(m_mutex);
		std::vector<mavlink_command_long_t> out;
		for (const auto& c : m_commands) {
			if (c.command == command) out.push_back(c);
		}
		return out;
	}
	std::vector<mavlink_set_position_target_local_ned_t> setpoints() const {
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_setpoints;
	}

private:
	void loop() {
		auto next_heartbeat = std::chrono::steady_clock::now();
		auto next_position = std::chrono::steady_clock::now();
		std::array<std::uint8_t, 2048> buffer{};
		while (!m_stop) {
			pollfd descriptor{.fd = m_socket, .events = POLLIN, .revents = 0};
			if (::poll(&descriptor, 1, 2) > 0) {
				const ssize_t received = ::recv(m_socket, buffer.data(), buffer.size(), 0);
				mavlink_message_t message;
				mavlink_status_t status;
				for (ssize_t i = 0; i < received; ++i) {
					if (mavlink_parse_char(MAVLINK_COMM_2, buffer[static_cast<std::size_t>(i)], &message, &status)) {
						handle(message);
					}
				}
			}
			const auto now = std::chrono::steady_clock::now();
			if (now >= next_heartbeat) {
				mavlink_message_t message;
				mavlink_msg_heartbeat_pack_chan(
				    1, 1, MAVLINK_COMM_3, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
				    static_cast<std::uint8_t>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
					                          (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0)),
				    custom_mode, MAV_STATE_STANDBY);
				transmit(message);
				next_heartbeat = now + 50ms;
			}
			if (now >= next_position) {
				mavlink_message_t message;
				mavlink_msg_local_position_ned_pack_chan(1, 1, MAVLINK_COMM_3, &message, 1000, 0, 0, 0, 0, 0, 0);
				transmit(message);
				next_position = now + 10ms;
			}
		}
	}

	void handle(const mavlink_message_t& message) {
		if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
			mavlink_command_long_t command;
			mavlink_msg_command_long_decode(&message, &command);
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				m_commands.push_back(command);
			}
			const int result = ack_result;
			if (result == MAV_RESULT_ACCEPTED) {
				if (command.command == MAV_CMD_DO_SET_MODE) custom_mode = static_cast<std::uint32_t>(command.param2);
				if (command.command == MAV_CMD_COMPONENT_ARM_DISARM) armed = command.param1 > 0.5f;
				if (command.command == MAV_CMD_NAV_LAND) custom_mode = 9;
				if (command.command == MAV_CMD_NAV_RETURN_TO_LAUNCH) custom_mode = 6;
			}
			if (result == kNoAnswer) return;
			mavlink_message_t ack;
			mavlink_msg_command_ack_pack_chan(1, 1, MAVLINK_COMM_3, &ack, command.command,
			                                  static_cast<std::uint8_t>(result), 0, 0, message.sysid,
			                                  message.compid);
			transmit(ack);
		} else if (message.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
			mavlink_set_position_target_local_ned_t setpoint;
			mavlink_msg_set_position_target_local_ned_decode(&message, &setpoint);
			std::lock_guard<std::mutex> lock(m_mutex);
			m_setpoints.push_back(setpoint);
		}
	}

	void transmit(const mavlink_message_t& message) {
		std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
		const std::uint16_t length = mavlink_msg_to_send_buffer(buffer.data(), &message);
		::sendto(m_socket, buffer.data(), length, 0, reinterpret_cast<const sockaddr*>(&m_vehicle),
		         sizeof(m_vehicle));
	}

	int m_socket = -1;
	sockaddr_in m_vehicle{};
	std::atomic<bool> m_stop{false};
	mutable std::mutex m_mutex;
	std::vector<mavlink_command_long_t> m_commands;
	std::vector<mavlink_set_position_target_local_ned_t> m_setpoints;
	std::thread m_thread;
};

MavsdkConfig test_config() {
	MavsdkConfig config;
	config.connection_url = "udpin://127.0.0.1:" + std::to_string(free_udp_port());
	config.origin_samples = 5;
	return config;
}

std::uint16_t port_of(const MavsdkConfig& config) {
	return static_cast<std::uint16_t>(std::stoi(config.connection_url.substr(config.connection_url.rfind(':') + 1)));
}

struct Fixture {
	MavsdkConfig config = test_config();
	Topic<StampedPose> world_pose;
	Topic<VehicleState> state;
	std::shared_ptr<LatestSlot<VehicleState>> latest_state = state.subscribe_latest();
	MavsdkVehicle vehicle{config, world_pose, state};

	Fixture() {
		StampedPose pose;  ///< the drone on the ground; the fake reports NED (0, 0, 0)
		pose.pose.position = Eigen::Vector3d(10, 20, 0.1);
		world_pose.publish(pose);
	}

	bool link_up() {
		return wait_for([&] {
			const auto s = latest_state->latest();
			return s && s->connected;
		},
		                15s);
	}
};

}

TEST(MavsdkVehicle, ReportsConnectionArmingAndMode) {
	Fixture f;
	FakeAutopilot autopilot(port_of(f.config));
	autopilot.custom_mode = 4;
	autopilot.armed = true;
	ASSERT_TRUE(wait_for([&] {
		const auto state = f.latest_state->latest();
		return state && state->connected && state->armed && state->mode == FlightMode::External;
	},
	                     15s));
}

TEST(MavsdkVehicle, FindsTheNedOriginFromTheWorldPose) {
	Fixture f;
	EXPECT_FALSE(f.vehicle.ned_origin().has_value());
	FakeAutopilot autopilot(port_of(f.config));
	ASSERT_TRUE(wait_for([&] {
		const auto state = f.latest_state->latest();
		return state && state->frames_ready;
	},
	                     15s));
	ASSERT_TRUE(f.vehicle.ned_origin().has_value());
	EXPECT_TRUE(f.vehicle.ned_origin()->isApprox(Eigen::Vector3d(10, 20, 0.1)));
}

TEST(MavsdkVehicle, CommandsReturnTheAutopilotsAnswer) {
	Fixture f;
	FakeAutopilot autopilot(port_of(f.config));
	ASSERT_TRUE(f.link_up());
	ASSERT_TRUE(wait_for([&] { return f.vehicle.ned_origin().has_value(); }, 15s));  ///< has a position

	EXPECT_TRUE(f.vehicle.arm(5s));
	ASSERT_FALSE(autopilot.commands(MAV_CMD_COMPONENT_ARM_DISARM).empty());
	EXPECT_FLOAT_EQ(autopilot.commands(MAV_CMD_COMPONENT_ARM_DISARM).back().param1, 1.0f);

	EXPECT_TRUE(f.vehicle.takeoff(5.0, 5s));
	ASSERT_FALSE(autopilot.commands(MAV_CMD_NAV_TAKEOFF).empty());
	EXPECT_FLOAT_EQ(autopilot.commands(MAV_CMD_NAV_TAKEOFF).back().param7, 5.0f);

	EXPECT_TRUE(f.vehicle.set_mode(FlightMode::External, 5s));  ///< offboard control starts GUIDED
	ASSERT_TRUE(wait_for([&] { return f.latest_state->latest()->mode == FlightMode::External; }, 5s));

	EXPECT_TRUE(f.vehicle.set_mode(FlightMode::Land, 5s));
	ASSERT_TRUE(wait_for([&] { return f.latest_state->latest()->mode == FlightMode::Land; }, 5s));

	EXPECT_FALSE(f.vehicle.set_mode(FlightMode::Manual, 100ms));  ///< cannot be commanded
	EXPECT_FALSE(f.vehicle.set_mode(FlightMode::Unknown, 100ms));

	autopilot.ack_result = MAV_RESULT_DENIED;  ///< e.g. pre-arm checks failing
	EXPECT_FALSE(f.vehicle.arm(5s));
	autopilot.ack_result = FakeAutopilot::kNoAnswer;
	const auto begin = std::chrono::steady_clock::now();
	EXPECT_FALSE(f.vehicle.arm(300ms));
	EXPECT_GE(std::chrono::steady_clock::now() - begin, 300ms);
}

TEST(MavsdkVehicle, SetpointsAreDroppedUntilTheFramesAreKnown) {
	MavsdkConfig config = test_config();
	Topic<StampedPose> world_pose;  ///< no world pose: the origin can never be found
	Topic<VehicleState> state;
	MavsdkVehicle vehicle(config, world_pose, state);
	FakeAutopilot autopilot(port_of(config));
	std::this_thread::sleep_for(300ms);
	vehicle.send(Setpoint{});
	EXPECT_EQ(vehicle.dropped_setpoint_count(), 1u);
	EXPECT_EQ(vehicle.sent_setpoint_count(), 0u);
	EXPECT_TRUE(autopilot.setpoints().empty());
}

TEST(MavsdkVehicle, SetpointsAreSentInTheAutopilotsNedFrame) {
	Fixture f;
	FakeAutopilot autopilot(port_of(f.config));
	ASSERT_TRUE(wait_for([&] { return f.vehicle.ned_origin().has_value(); }, 15s));
	ASSERT_TRUE(f.link_up());

	Setpoint setpoint;
	setpoint.position = Eigen::Vector3d(13, 24, 5.1);  ///< 3 m east, 4 m north, 5 m up of the origin
	setpoint.velocity = Eigen::Vector3d(1, 2, 3);
	setpoint.acceleration = Eigen::Vector3d(0.1, 0.2, 0.3);
	setpoint.yaw = 0.0;
	f.vehicle.send(setpoint);
	ASSERT_TRUE(wait_for([&] { return !autopilot.setpoints().empty(); }, 5s));
	EXPECT_EQ(f.vehicle.sent_setpoint_count(), 1u);

	const auto received = autopilot.setpoints().back();
	EXPECT_EQ(received.coordinate_frame, MAV_FRAME_LOCAL_NED);
	EXPECT_EQ(received.type_mask & 0x1FF, 0);  ///< position, velocity and acceleration are all used
	EXPECT_NEAR(received.x, 4.0, 1e-4);
	EXPECT_NEAR(received.y, 3.0, 1e-4);
	EXPECT_NEAR(received.z, -5.0, 1e-4);
	EXPECT_NEAR(received.vx, 2.0, 1e-5);
	EXPECT_NEAR(received.vy, 1.0, 1e-5);
	EXPECT_NEAR(received.vz, -3.0, 1e-5);
	EXPECT_NEAR(received.afx, 0.2, 1e-5);
	EXPECT_NEAR(received.afy, 0.1, 1e-5);
	EXPECT_NEAR(received.afz, -0.3, 1e-5);
	EXPECT_NEAR(received.yaw, std::numbers::pi / 2, 1e-4);  ///< east in NED
}
