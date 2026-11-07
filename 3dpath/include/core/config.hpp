#pragma once

#include "core/types.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

/// Every tunable setting of the pipeline, in one nested structure. Each stage takes its own part
/// (e.g. MappingStage takes `config.mapping`), and apps/main.cpp fills the whole tree in one place.
/// Plain aggregates of std types only, so this header is free of ROS and of every other component.
///
/// Times: `TimeSec` fields are sim time (compared with message stamps); `std::chrono` fields are
/// wall time (thread waits and timeouts).
namespace path3d {

/**
 * @brief Gazebo reader (sim_reader.hpp): ROS topics and TF frame names.
 */
struct SimReaderConfig {
  std::string node_name = "sim_reader";
  std::string world_frame = "default";  ///< = Gazebo world name ("default", "city")
  std::string base_frame = "parrot_bebop_2/body";
  std::string tf_topic = "/tf";
  std::string color_image_topic = "/camera";
  std::string color_info_topic = "/camera/camera_info";
  std::string depth_image_topic = "/depth_camera";
  std::string depth_info_topic = "/depth_camera/camera_info";
};

/**
 * @brief Pairing of depth images with poses (FrameSynchronizer, sync.hpp).
 */
struct SyncConfig {
  /// Depth images waiting for the mapper; when full the oldest is dropped.
  std::size_t depth_queue = 2;
  /// Poses kept for interpolation. Must span the image latency: at a 100 Hz pose rate, 1024 poses
  /// are ~10 s, so an image is mapped as long as it arrives within that time of its stamp.
  std::size_t pose_history = 1024;
  /// How long to wait for a pose at or after an image's stamp before dropping the image.
  std::chrono::milliseconds pose_wait{100};
};

/**
 * @brief Map building and update publishing (MappingStage, stages.hpp).
 */
struct MappingConfig {
  SyncConfig sync;
  /// Diffs are published at most this often (sim time): building an update is not free.
  TimeSec publish_period = 0.2;
  /// A keyframe (the whole map) is published this often (wall time), even while the map is not
  /// changing, so late or out-of-date readers can catch up.
  std::chrono::milliseconds keyframe_period{5000};
  /// Changes still unpublished are flushed once no frame has come for this long (wall time).
  std::chrono::milliseconds idle_flush{100};
};

/**
 * @brief A planner's copy of the map (MapClient, clients.hpp).
 */
struct MapClientConfig {
  /// Updates held between two sync() calls; on overflow the oldest are dropped, which costs a wait
  /// for the next keyframe.
  std::size_t queue_capacity = 256;
};

/**
 * @brief Global planning (GlobalPlanningStage, stages.hpp).
 */
struct GlobalPlanningConfig {
  MapClientConfig map_client;
  /// Longest wait for a request before the loop rechecks for shutdown and syncs the map.
  std::chrono::milliseconds request_wait{50};
};

/**
 * @brief Local planning (LocalPlanningStage, stages.hpp).
 */
struct LocalPlanningConfig {
  MapClientConfig map_client;
  /// control cycles per second (wall time)
  double rate_hz = 20.0;
  /// A request without an answer for this long counts as lost: the next request is sent with a new
  /// id, and a late answer to the old one is dropped as stale.
  std::chrono::milliseconds request_timeout{2000};
};

/**
 * @brief Streaming of trajectories to the vehicle (CommanderStage, stages.hpp).
 */
struct CommanderConfig {
  /// setpoints sent per second (wall time)
  double rate_hz = 50.0;
  /// A trajectory older than this (sim time, from its stamp to now) means the planner stopped
  /// updating it: the drone holds its position instead of following it.
  TimeSec max_trajectory_age = 0.5;
  /// The pose is stale if its stamp has not advanced for this long (wall time): hold position.
  std::chrono::milliseconds max_pose_silence{200};
};

/**
 * @brief Autopilot connection through MAVSDK (MavsdkVehicle, mavsdk_vehicle.hpp).
 */
struct MavsdkConfig {
  /// MAVSDK connection URL. `udpin` listens: MAVProxy sends to it (`--out=udp:127.0.0.1:14560`).
  std::string connection_url = "udpin://127.0.0.1:14560";
  /// How long one attempt to find the autopilot waits (wall time); attempts repeat until found.
  std::chrono::milliseconds discovery_attempt{200};
  /// Rate requested for the autopilot's local position (used to find its frame origin), Hz.
  double position_rate_hz = 30.0;
  /// Position pairs averaged to find the autopilot's NED origin in the world frame (see NedOrigin).
  std::size_t origin_samples = 50;
};

/**
 * @brief Every setting of the pipeline: one part for each component.
 */
struct PipelineConfig {
  SimReaderConfig sim_reader;
  MappingConfig mapping;
  GlobalPlanningConfig global_planning;
  LocalPlanningConfig local_planning;
  CommanderConfig commander;
  MavsdkConfig mavsdk;
};

}
