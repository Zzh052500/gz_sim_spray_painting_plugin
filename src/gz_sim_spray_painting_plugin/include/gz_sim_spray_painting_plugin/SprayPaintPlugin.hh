#pragma once

// All third-party includes for this plugin are gathered here rather than
// split across the .cc, so there is a single place to see what the plugin
// depends on.  SprayPaintPlugin.cc includes only this header.

// Standard library
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// gz-math
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

// gz-msgs - spray trigger message, particle emitter proto, colour helper
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/convert/Color.hh>

// gz-sim - core types
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

// gz-sim - ECM components read/written by the plugin
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ParticleEmitter.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/World.hh>

// sdf - geometry/material description used to build patch visuals
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Material.hh>
#include <sdf/ParticleEmitter.hh>
#include <sdf/Visual.hh>

// gz-transport / gz-common / gz-plugin
#include <gz/transport/Node.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

namespace gz::sim::systems
{

/// \brief Spray paint plugin for Gazebo Sim 8 (Harmonic).
///
/// Attach it to any model and name a link to spray from; no per-robot code
/// is required.  Uses physics raycasting (gz::sim::components::RaycastData)
/// so it detects hits on any geometry type, including MESH.  On each active
/// spray tick the plugin:
///   1. Reads raycast results (hit point + normal in nozzle-local frame).
///   2. Transforms them to world frame via the nozzle link pose.
///   3. Finds the nearest non-own link to the hit point and parents a thin
///      coloured disc patch visual to it.
///
/// Patch sizing is derived once at Configure() rather than fixed: see
/// ComputePatchSizing(), which measures the ray pattern so that neighbouring
/// patches overlap (no interior gaps) and the outermost ones land on the
/// cone rim.
///
/// SDF parameters (all optional, defaults shown):
///   <nozzle_link>             spray_gun_nozzle_link </nozzle_link>
///   <cone_half_angle_deg>     15                    </cone_half_angle_deg>
///   <cone_max_range>          3.0                   </cone_max_range>
///   <spray_color>             1.0 0.2 0.1 1.0       </spray_color>
///   <spray_topic>             /spray_paint/trigger  </spray_topic>
///   <particle_rate>           100                   </particle_rate>
///   <num_rays>                16                    </num_rays>
///   <patch_spacing>           0.02                  </patch_spacing>
///   <paint_interval_steps>    10                    </paint_interval_steps>
///   <enable_particle_emitter> true                  </enable_particle_emitter>
///   <patch_overlap_factor>    1.30                  </patch_overlap_factor>
///   <rim_reach>               1.12                  </rim_reach>
///   <perf_log_path>           (unset - disabled)    </perf_log_path>
class SprayPaintPlugin
    : public System
    , public ISystemConfigure
    , public ISystemPreUpdate
{
public:
  SprayPaintPlugin();
  ~SprayPaintPlugin() override = default;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

private:
  void OnSprayMsg(const gz::msgs::Boolean &_msg);

  // Config
  std::string     nozzleLink_{"spray_gun_nozzle_link"};
  double          coneHalfAngle_{0.2618};    // 15° in radians
  double          coneMaxRange_{3.0};
  gz::math::Color sprayColor_{1.0f, 0.2f, 0.1f, 1.0f};
  std::string     sprayTopic_{"/spray_paint/trigger"};

  double   particleRate_{100.0};      // particles/s  (SDF: <particle_rate>)
  double   patchSpacing_{0.02};       // min patch centre gap in m (SDF: <patch_spacing>)
  uint32_t paintIntervalSteps_{10};   // paint scan every N steps (SDF: <paint_interval_steps>)
  int      numRays_{16};              // cone rays per scan (SDF: <num_rays>)
  bool     enableParticleEmitter_{true}; // SDF: <enable_particle_emitter>
  double   patchOverlapFactor_{1.30}; // target patch overlap (SDF: <patch_overlap_factor>)

  // How far the outermost patches reach, as a multiple of the cone radius
  // (SDF: <rim_reach>).  1.0 = never spill outside the cone, at the cost of
  // a scalloped, under-painted rim; >1 trades a little spill for markedly
  // less missed area.  Measured total mismatch (missed + spilled area) is
  // minimised near 1.10-1.15; see ComputePatchSizing().
  double   rimReach_{1.12};

  // Cone-coverage sizing, derived once in ComputePatchSizing().  Both are
  // fractions of the cone's cross-section radius at a given depth, chosen so
  // that (sampling disk) + (patch radius) == (cone radius): the union of
  // patches reaches the cone rim without any single patch spilling outside
  // it.  See ComputePatchSizing() for the covering-radius derivation.
  double   sampleDiskFraction_{1.0};   // S / R_cone  - used by GenerateConeRays
  double   patchRadiusFraction_{0.0};  // p / R_cone  - used by MakePatch

  // Runtime
  std::atomic<bool>      sprayActive_{false};
  gz::transport::Node    transportNode_;
  gz::sim::Entity        nozzleEntity_{gz::sim::kNullEntity};
  gz::sim::EventManager *eventMgr_{nullptr};

  gz::sim::Entity        robotModelEntity_{gz::sim::kNullEntity};

  gz::sim::Entity        emitterEntity_{gz::sim::kNullEntity};
  bool                   lastEmitterState_{false};
  uint32_t               emitterCounter_{0};   // unique suffix for each emitter name

  // Set once after nozzle entity is found and rays are attached.
  bool raysAttached_{false};

  // Own-robot link entities — excluded from nearest-link search.
  std::unordered_set<gz::sim::Entity> ownLinks_;

  uint64_t paintStepCounter_{0};

  // Perf logging (SDF: <perf_log_path>) - disabled unless a path is given.
  std::string  perfLogPath_;
  std::ofstream perfLog_;
  bool         perfLogEnabled_{false};

  // Per-link list of applied patch centres in link-local frame.
  // Keyed by the nearest link entity at the hit point.
  std::unordered_map<gz::sim::Entity,
                     std::vector<gz::math::Vector3d>> patchCenters_;

  // Patch geometry

  struct PaintPatch
  {
    bool valid{false};
    gz::math::Pose3d   worldPose;
    gz::math::Vector3d size;
  };

  /// Build a PaintPatch from a world-frame hit point + outward surface normal.
  /// _axialDepth is the hit's depth along the spray axis (nozzle-local +X),
  /// NOT the Euclidean range - the cone's cross-section radius at a hit is
  /// _axialDepth * tan(halfAngle).
  PaintPatch MakePatch(const gz::math::Vector3d &_hitWorld,
                       const gz::math::Vector3d &_normalWorld,
                       double _axialDepth) const;

  /// Derive sampleDiskFraction_ / patchRadiusFraction_ from the ray pattern.
  /// Called once from Configure(), after the SDF parameters are read.
  void ComputePatchSizing();

  /// Generate N cone rays in nozzle-local frame: {start, end} pairs.
  /// Center ray is always first; remaining rays use Fibonacci disk sampling.
  std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>>
  GenerateConeRays() const;

  /// Return the Link entity whose Collision is nearest to hitWorld,
  /// excluding own-robot links.  More accurate than link-origin proximity.
  gz::sim::Entity FindHitLink(
      const gz::math::Vector3d &hitWorld,
      EntityComponentManager &_ecm) const;

  // Debug logging
  bool          debugDumped_{false};

  std::string Timestamp() const;
  void Log(const std::string &level, const std::string &step,
           const std::string &msg);
  void Log(const std::string &step, const std::string &msg);
  void LogSection(const std::string &title);
};

}  // namespace gz::sim::systems
