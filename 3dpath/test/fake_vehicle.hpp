#pragma once

#include "io/vehicle.hpp"

#include <mutex>
#include <vector>

namespace path3d::test {

/**
 * @brief Vehicle that records the setpoints it is sent; its state is whatever the test publishes.
 */
class FakeVehicle : public Vehicle {
 public:
  explicit FakeVehicle(Topic<VehicleState>& state_out) : Vehicle(state_out) {}

  bool set_mode(FlightMode, std::chrono::milliseconds) override { return true; }
  bool arm(std::chrono::milliseconds) override { return true; }
  bool takeoff(double, std::chrono::milliseconds) override { return true; }
  bool land(std::chrono::milliseconds) override { return true; }
  void send(const Setpoint& setpoint) override {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_sent.push_back(setpoint);
  }

  void report(const VehicleState& state) { m_state_out.publish(state); }

  std::vector<Setpoint> sent() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_sent;
  }
  void clear() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_sent.clear();
  }

 private:
  mutable std::mutex m_mutex;
  std::vector<Setpoint> m_sent;
};

}  
