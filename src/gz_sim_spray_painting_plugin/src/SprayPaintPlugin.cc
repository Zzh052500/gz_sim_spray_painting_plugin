// Every dependency this plugin needs is pulled in via the header, which is
// the single place the include list is maintained.
#include "gz_sim_spray_painting_plugin/SprayPaintPlugin.hh"

namespace gz::sim::systems
{

SprayPaintPlugin::SprayPaintPlugin() = default;

// Logging helpers

/**
 * @brief Returns the current wall-clock time as a formatted string.
 *
 * Format: HH:MM:SS.mmm - used as a prefix in log lines produced by Log().
 *
 * @return Formatted timestamp string.
 */
std::string SprayPaintPlugin::Timestamp() const
{
  using namespace std::chrono;
  const auto now    = system_clock::now();
  const auto now_t  = system_clock::to_time_t(now);
  const auto ms     = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

  std::ostringstream ts;
  ts << std::put_time(std::localtime(&now_t), "%H:%M:%S");
  ts << '.' << std::setfill('0') << std::setw(3) << ms.count();
  return ts.str();
}

/**
 * @brief Emits a structured log line to the Gazebo message stream.
 *
 * Each line is formatted as:
 *   [HH:MM:SS.mmm] [LEVEL  ] [step          ] message
 *
 * @param level  Severity label (e.g. "INFO", "WARN").
 * @param step   Short tag identifying the plugin stage (e.g. "Configure").
 * @param msg    Human-readable message body.
 */
void SprayPaintPlugin::Log(const std::string &level,
                           const std::string &step,
                           const std::string &msg)
{
  std::ostringstream line;
  line << '[' << Timestamp() << ']'
       << " [" << std::left << std::setw(7) << level << ']'
       << " [" << std::left << std::setw(14) << step << "] "
       << msg << '\n';

  const std::string out = line.str();
  gzmsg << out;
}

/**
 * @brief Convenience overload that logs at INFO level.
 *
 * @param step  Short tag identifying the plugin stage.
 * @param msg   Human-readable message body.
 */
void SprayPaintPlugin::Log(const std::string &step, const std::string &msg)
{
  Log("INFO", step, msg);
}

/**
 * @brief Emits a visual section header to the Gazebo message stream.
 *
 * Prints a 60-character '=' bar above and below the title, making it easy
 * to find major lifecycle transitions in a scrolling log.
 *
 * @param title  Label to display between the separator bars.
 */
void SprayPaintPlugin::LogSection(const std::string &title)
{
  const std::string bar(60, '=');
  const std::string entry = "\n" + bar + "\n  " + title + "\n" + bar + "\n";
  gzmsg << entry;
}

// MakePatch

/**
 * @brief Constructs a PaintPatch descriptor for a single raycast hit.
 *
 * The patch is a thin disc sized to `patchRadiusFraction_` of the cone's
 * cross-section radius at this hit's depth.  That fraction is derived once
 * in ComputePatchSizing() from the measured covering radius of the ray
 * pattern, so neighbouring patches always overlap (no interior gaps) while
 * the sampling disk is shrunk just enough that the outermost patches land
 * on the cone rim rather than crossing it.
 *
 * Note the scaling is driven by the hit's *axial depth*, not its Euclidean
 * range: the cone's cross-section radius at axial depth x is x*tan(theta).
 *
 * The disc is offset slightly along the surface normal so it sits proud of
 * the geometry and avoids z-fighting.
 *
 * @param _hitWorld    Hit point in world coordinates.
 * @param _normalWorld Outward surface normal at the hit point (world frame).
 * @param _axialDepth  Hit depth along the spray axis (nozzle-local +X), in m.
 * @return             A PaintPatch with worldPose, size, and valid=true set.
 */
SprayPaintPlugin::PaintPatch SprayPaintPlugin::MakePatch(
    const gz::math::Vector3d &_hitWorld,
    const gz::math::Vector3d &_normalWorld,
    double _axialDepth) const
{
  PaintPatch result;

  constexpr double kThickness = 0.003;
  constexpr double kMinRadius = 0.02;

  const double coneRadiusAtDepth = _axialDepth * std::tan(coneHalfAngle_);

  double radius = coneRadiusAtDepth * patchRadiusFraction_;

  // Visibility floor for very close hits, but never wider than the cone
  // itself at this depth - staying inside the cone wins over min size.
  radius = std::max(radius, std::min(kMinRadius, coneRadiusAtDepth));

  result.size = gz::math::Vector3d(radius * 2.0, radius * 2.0, kThickness);

  const gz::math::Vector3d centre = _hitWorld + _normalWorld * (kThickness * 0.5);

  const gz::math::Vector3d zAxis(0.0, 0.0, 1.0);
  const double dot = std::clamp(zAxis.Dot(_normalWorld), -1.0, 1.0);

  gz::math::Quaterniond rot;
  if (dot > 1.0 - 1e-6)
    rot = gz::math::Quaterniond::Identity;
  else if (dot < -(1.0 - 1e-6))
    rot = gz::math::Quaterniond(M_PI, 0.0, 0.0);
  else
  {
    const gz::math::Vector3d axis = zAxis.Cross(_normalWorld).Normalized();
    rot = gz::math::Quaterniond(axis, std::acos(dot));
  }

  result.worldPose = gz::math::Pose3d(centre, rot);
  result.valid = true;
  return result;
}

// GenerateConeRays

namespace
{
/// \brief Sample offsets on the *unit* sampling disk, index 0 = centre.
///
/// Shared by GenerateConeRays() (which scales them into the cone) and
/// ComputePatchSizing() (which measures their covering radius), so the
/// measurement always describes the pattern actually cast.
std::vector<std::pair<double, double>> UnitDiskSamples(int _numRays)
{
  std::vector<std::pair<double, double>> pts;
  pts.reserve(std::max(_numRays, 1));
  pts.emplace_back(0.0, 0.0);

  if (_numRays <= 1) return pts;

  // Fibonacci / sunflower sampling: uniform area density, no grid bias.
  const double goldenAngle = M_PI * (3.0 - std::sqrt(5.0));  // ≈ 2.3999 rad
  for (int i = 1; i < _numRays; ++i)
  {
    const double r     = std::sqrt(static_cast<double>(i) / (_numRays - 1));
    const double theta = i * goldenAngle;
    pts.emplace_back(r * std::cos(theta), r * std::sin(theta));
  }
  return pts;
}
}  // namespace

/**
 * @brief Generates ray origin-endpoint pairs spanning the spray cone.
 *
 * Rays are expressed in nozzle-local frame (+X is the spray axis).  The
 * first ray is always the centre axis.  Remaining rays are distributed
 * using Fibonacci/sunflower disk sampling.
 *
 * All end-points lie on a disk at x = coneMaxRange_, so for every ray the
 * hit's radial offset and the local cone radius scale together with
 * `fraction` - which is why the sampling disk is deliberately *smaller*
 * than the cone's own cross-section.  It is shrunk by sampleDiskFraction_
 * so that a sample at the sampling-disk rim, plus its patch radius, lands
 * exactly on the cone rim.  Without that shrink, rim samples would need
 * zero-radius patches to stay inside the cone.
 *
 * @return Vector of (origin, endpoint) pairs, each in nozzle-local frame.
 */
std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>>
SprayPaintPlugin::GenerateConeRays() const
{
  using Vec3 = gz::math::Vector3d;
  std::vector<std::pair<Vec3, Vec3>> rays;
  const Vec3 origin(0.0, 0.0, 0.0);

  const double diskRadius =
      coneMaxRange_ * std::tan(coneHalfAngle_) * sampleDiskFraction_;

  for (const auto &p : UnitDiskSamples(numRays_))
  {
    rays.emplace_back(origin,
        Vec3(coneMaxRange_, p.first * diskRadius, p.second * diskRadius));
  }
  return rays;
}

/**
 * @brief Derives the patch/sampling-disk sizing from the ray pattern.
 *
 * Two things have to hold at once for the painted union to match the cone
 * projection:
 *
 *  1. No interior gaps.  This is a disc-covering problem: the patch radius
 *     must be at least the pattern's *covering radius* - the largest
 *     distance from any point of the sampling disk to its nearest sample.
 *     The ideal hexagonal bound is 1.0997/sqrt(N), but Fibonacci sampling
 *     is only quasi-uniform, so that underestimates the real requirement.
 *     We measure it numerically instead of assuming it.
 *
 *  2. Control of the rim.  With patch radius p and sampling-disk radius S
 *     (both as fractions of the cone radius), the outermost patch reaches
 *     S + p.  `rimReach_` sets that reach: 1.0 puts it exactly on the cone
 *     rim so nothing ever spills outside the cone.
 *
 * Solving both gives S = rimReach_/(1+h) and p = h*S for an effective
 * covering fraction h.  Cost is a one-time grid sweep at Configure();
 * nothing here runs per scan.
 *
 * Caveat, by construction: a finite set of discs centred inside the rim can
 * never cover the rim completely, so full coverage is guaranteed only out to
 * radius S and the band beyond it is scalloped.  At rimReach_ = 1.0 and
 * N = 16 that leaves ~25% of the cone unpainted with zero spill; allowing a
 * little spill trades that down sharply, with total mismatch (missed +
 * spilled area) bottoming out near rimReach_ 1.10-1.15 at ~14% (N=16) and
 * ~10% (N=32).  Getting materially below that is not a sampling problem -
 * it needs a different patch representation (one footprint-shaped stamp, or
 * an alpha-masked decal), not more rays.
 */
void SprayPaintPlugin::ComputePatchSizing()
{
  const auto pts = UnitDiskSamples(numRays_);

  // Covering radius of the pattern over the unit disk.
  constexpr int kGrid = 256;
  double worstSq = 0.0;
  for (int gy = 0; gy <= kGrid; ++gy)
  {
    const double y = -1.0 + 2.0 * gy / kGrid;
    for (int gx = 0; gx <= kGrid; ++gx)
    {
      const double x = -1.0 + 2.0 * gx / kGrid;
      if (x * x + y * y > 1.0) continue;

      double nearestSq = std::numeric_limits<double>::max();
      for (const auto &p : pts)
      {
        const double dx = x - p.first;
        const double dy = y - p.second;
        nearestSq = std::min(nearestSq, dx * dx + dy * dy);
      }
      worstSq = std::max(worstSq, nearestSq);
    }
  }
  const double measured = std::sqrt(worstSq);

  // Take the larger of the measured requirement (with a small safety
  // margin) and the configured overlap target.
  const double target = patchOverlapFactor_ /
      std::sqrt(static_cast<double>(std::max(numRays_, 1)));
  const double h = std::max(measured * 1.02, target);

  sampleDiskFraction_  = rimReach_ / (1.0 + h);
  patchRadiusFraction_ = h * sampleDiskFraction_;
}

// FindHitLink

/**
 * @brief Finds the nearest non-robot link to a world-space hit point.
 *
 * Uses a two-pass search:
 *  1. Collision entity centres (accurate for multi-link / offset models).
 *  2. Link entity origins as a fallback for geometry types (e.g. PLANE)
 *     whose Collision components are absent from the ECM.
 *
 * Links belonging to the robot's own model are excluded via ownLinks_.
 *
 * @param hitWorld  Hit point in world coordinates.
 * @param _ecm      Reference to the Entity Component Manager.
 * @return          Entity ID of the nearest paintable link, or kNullEntity.
 */
gz::sim::Entity SprayPaintPlugin::FindHitLink(
    const gz::math::Vector3d &hitWorld,
    EntityComponentManager &_ecm) const
{
  // Closest candidate found so far, across whichever pass ends up running.
  gz::sim::Entity bestLink = gz::sim::kNullEntity;
  double minDist = std::numeric_limits<double>::max();

  // Primary pass: use Collision entity world-pose centres.
  // This is accurate for multi-link / offset-collision models (e.g. a car
  // where the chassis collision sits at the car body, not the model origin).
  // NOTE: _ecm.Each<>() calls this lambda once per matching entity; returning
  // true means "keep iterating" (not "found a hit") - only the final
  // bestLink value after the full scan represents the actual result.
  _ecm.Each<gz::sim::components::Collision>(
      [&](const gz::sim::Entity &colEnt,
          const gz::sim::components::Collision *) -> bool
      {
        // Collisions should always have a parent Link; skip defensively if
        // the ECM is in some transient/malformed state.
        const auto *parentComp =
            _ecm.Component<gz::sim::components::ParentEntity>(colEnt);
        if (!parentComp) return true;
        const gz::sim::Entity linkEnt = parentComp->Data();
        // Never let the spraying robot paint its own links.
        if (ownLinks_.count(linkEnt)) return true;

        // Keep the closest collision seen so far this scan.
        const double dist =
            gz::sim::worldPose(colEnt, _ecm).Pos().Distance(hitWorld);
        if (dist < minDist)
        {
          minDist  = dist;
          bestLink = linkEnt;
        }
        return true;
      });

  // If the primary pass found any paintable collision at all, use it -
  // no need to also run the (less precise) fallback pass below.
  if (bestLink != gz::sim::kNullEntity)
    return bestLink;

  // Fallback pass: use Link entity origin proximity.
  // This covers geometry types (e.g. PLANE) whose Collision entities are
  // not registered in the ECM as gz::sim::components::Collision.
  _ecm.Each<gz::sim::components::Link>(
      [&](const gz::sim::Entity &linkEnt,
          const gz::sim::components::Link *) -> bool
      {
        // Same self-exclusion rule as the primary pass.
        if (ownLinks_.count(linkEnt)) return true;
        const double dist =
            gz::sim::worldPose(linkEnt, _ecm).Pos().Distance(hitWorld);
        if (dist < minDist)
        {
          minDist  = dist;
          bestLink = linkEnt;
        }
        return true;
      });

  // kNullEntity here means neither pass found a paintable link (e.g. the ray
  // hit something with no registered Link/Collision at all).
  return bestLink;
}

// Configure

/**
 * @brief ISystemConfigure callback - reads SDF parameters and subscribes to
 *        the spray trigger topic.
 *
 * Called once by Gazebo when the plugin is loaded.  All SDF parameters are
 * optional; missing elements keep their default-initialised values.
 *
 * @param _entity   Entity this plugin is attached to (unused).
 * @param _sdf      SDF element containing plugin parameters.
 * @param _ecm      Entity Component Manager (unused at configure time).
 * @param _eventMgr Event Manager - cached for use in PreUpdate.
 */
void SprayPaintPlugin::Configure(
    const Entity & /*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  // 1. Read SDF parameters
  if (_sdf->HasElement("nozzle_link"))
    nozzleLink_ = _sdf->Get<std::string>("nozzle_link");

  if (_sdf->HasElement("cone_half_angle_deg"))
    coneHalfAngle_ = _sdf->Get<double>("cone_half_angle_deg") * M_PI / 180.0;

  if (_sdf->HasElement("cone_max_range"))
    coneMaxRange_ = _sdf->Get<double>("cone_max_range");

  if (_sdf->HasElement("spray_color"))
    sprayColor_ = _sdf->Get<gz::math::Color>("spray_color");

  if (_sdf->HasElement("spray_topic"))
    sprayTopic_ = _sdf->Get<std::string>("spray_topic");

  if (_sdf->HasElement("particle_rate"))
    particleRate_ = _sdf->Get<double>("particle_rate");

  if (_sdf->HasElement("patch_spacing"))
    patchSpacing_ = _sdf->Get<double>("patch_spacing");

  if (_sdf->HasElement("paint_interval_steps"))
    paintIntervalSteps_ = std::max(1u,
        static_cast<uint32_t>(_sdf->Get<int>("paint_interval_steps")));

  if (_sdf->HasElement("num_rays"))
    numRays_ = std::max(1, _sdf->Get<int>("num_rays"));

  if (_sdf->HasElement("enable_particle_emitter"))
    enableParticleEmitter_ = _sdf->Get<bool>("enable_particle_emitter");

  if (_sdf->HasElement("perf_log_path"))
    perfLogPath_ = _sdf->Get<std::string>("perf_log_path");

  if (_sdf->HasElement("patch_overlap_factor"))
    patchOverlapFactor_ = std::max(0.01, _sdf->Get<double>("patch_overlap_factor"));

  if (_sdf->HasElement("rim_reach"))
    rimReach_ = std::max(0.01, _sdf->Get<double>("rim_reach"));

  // 1a. Derive cone-coverage sizing from the (now known) ray pattern
  ComputePatchSizing();

  // 2. Cache EventManager pointer
  eventMgr_ = &_eventMgr;

  // 1b. Open perf log CSV, if requested
  if (!perfLogPath_.empty())
  {
    perfLog_.open(perfLogPath_, std::ios::out | std::ios::trunc);
    perfLogEnabled_ = perfLog_.is_open();
    if (perfLogEnabled_)
    {
      perfLog_ << "sim_time_s,paint_interval_steps,num_rays,"
                  "scan_us,valid_hits,patches_created\n";
      perfLog_.flush();
    }
  }

  // 3. Subscribe to trigger topic
  transportNode_.Subscribe(sprayTopic_, &SprayPaintPlugin::OnSprayMsg, this);

  // 4. Log startup banner
  LogSection("SprayPaintPlugin  –  Configure");
  Log("Configure", "nozzle_link", nozzleLink_);
  Log("Configure", "half_angle",
      std::to_string(coneHalfAngle_ * 180.0 / M_PI) + " deg");
  Log("Configure", "max_range",   std::to_string(coneMaxRange_) + " m");
  Log("Configure", "spray_color",
      "R=" + std::to_string(sprayColor_.R()) +
      " G=" + std::to_string(sprayColor_.G()) +
      " B=" + std::to_string(sprayColor_.B()));
  Log("Configure", "topic",          sprayTopic_);
  Log("Configure", "particle_rate",     std::to_string(particleRate_) + " /s");
  Log("Configure", "patch_spacing",     std::to_string(patchSpacing_) + " m");
  Log("Configure", "paint_interval",    std::to_string(paintIntervalSteps_) + " steps");
  Log("Configure", "num_rays",          std::to_string(numRays_) + " cone rays per scan");
  Log("Configure", "particle_emitter",  enableParticleEmitter_ ? "enabled" : "disabled");
  Log("Configure", "patch_overlap",     std::to_string(patchOverlapFactor_) + "x");
  Log("Configure", "rim_reach",         std::to_string(rimReach_) + " x cone radius");
  Log("Configure", "sample_disk",
      std::to_string(sampleDiskFraction_) + " x cone radius"
      " (union fully covered out to here; scalloped beyond)");
  Log("Configure", "patch_radius",
      std::to_string(patchRadiusFraction_) + " x cone radius");
  Log("Configure", "perf_log",
      perfLogEnabled_ ? ("writing to " + perfLogPath_) : "disabled");
  Log("Configure", "status",            "Plugin ready – waiting for nozzle entity");
}

// OnSprayMsg

/**
 * @brief Transport callback fired when a message arrives on the spray topic.
 *
 * Atomically updates sprayActive_ and logs the state transition.  Resets
 * the one-shot diagnostic flag so the next spray-ON edge re-dumps nozzle
 * state.
 *
 * @param _msg  Boolean message: true = spray ON, false = spray OFF.
 */
void SprayPaintPlugin::OnSprayMsg(const gz::msgs::Boolean &_msg)
{
  const bool newState = _msg.data();
  sprayActive_.store(newState);

  LogSection(std::string("Spray ") + (newState ? "ON" : "OFF"));
  Log("OnSprayMsg", "trigger",     newState ? "ACTIVE" : "INACTIVE");
  Log("OnSprayMsg", "patch_count",
      std::to_string(patchCenters_.size()) + " links have patches so far");

  if (newState)
  {
    debugDumped_ = false;
    Log("OnSprayMsg", "debug_dump", "Will log nozzle state on next PreUpdate");
  }
}

// PreUpdate

/**
 * @brief ISystemPreUpdate callback - core per-step logic.
 *
 * Executed every simulation step before physics.  Responsibilities:
 *  - Resolve the nozzle link entity on first appearance (STEP 2).
 *  - Manage the particle emitter lifecycle: create, reposition, remove
 *    (STEP 3 / 3b / 3c) - gated by enableParticleEmitter_.
 *  - Rate-limit ray scans to every paintIntervalSteps_ steps (STEP 6).
 *  - Read RaycastData results and deposit PaintPatch visuals on hit
 *    surfaces, with per-link deduplication (STEP 7-9).
 *
 * @param _info  Simulation update info (timestep, paused state, etc.).
 * @param _ecm   Entity Component Manager for querying and creating entities.
 */
void SprayPaintPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // STEP 1: Nozzle validity check
  if (nozzleEntity_ != kNullEntity && !_ecm.HasEntity(nozzleEntity_))
  {
    LogSection("PreUpdate – Nozzle Lost");
    Log("WARN", "PreUpdate",
        "nozzle entity " + std::to_string(nozzleEntity_) +
        " gone from ECM – re-resolving");
    nozzleEntity_     = kNullEntity;
    robotModelEntity_ = kNullEntity;
    raysAttached_     = false;
    ownLinks_.clear();
    patchCenters_.clear();
  }

  // STEP 2: Nozzle resolution
  if (nozzleEntity_ == kNullEntity)
  {
    _ecm.Each<components::Link, components::Name>(
        [&](const Entity &entity,
            const components::Link *,
            const components::Name *name) -> bool
        {
          if (name->Data() == nozzleLink_)
          {
            nozzleEntity_ = entity;
            return false;
          }
          return true;
        });

    if (nozzleEntity_ == kNullEntity)
      return;

    // Walk up to find parent model.
    {
      Entity e = nozzleEntity_;
      while (e != kNullEntity)
      {
        if (_ecm.Component<components::Model>(e))
        {
          robotModelEntity_ = e;
          break;
        }
        const auto *p = _ecm.Component<components::ParentEntity>(e);
        e = p ? p->Data() : kNullEntity;
      }
    }

    // Collect own-robot links to exclude from nearest-link search.
    if (robotModelEntity_ != kNullEntity)
    {
      _ecm.Each<components::Link>(
          [&](const Entity &lkEnt, const components::Link *) -> bool
          {
            Entity e = lkEnt;
            while (e != kNullEntity)
            {
              if (e == robotModelEntity_)
              {
                ownLinks_.insert(lkEnt);
                break;
              }
              const auto *p = _ecm.Component<components::ParentEntity>(e);
              e = p ? p->Data() : kNullEntity;
            }
            return true;
          });
    }

    LogSection("PreUpdate – Nozzle Resolved");
    Log("PreUpdate", "nozzle_entity",
        "Link '" + nozzleLink_ + "' -> entity " + std::to_string(nozzleEntity_));
    Log("PreUpdate", "own_links",
        std::to_string(ownLinks_.size()) + " own-robot links excluded");

    const gz::math::Pose3d p = gz::sim::worldPose(nozzleEntity_, _ecm);
    std::ostringstream ps;
    ps << "pos=(" << p.Pos().X() << ", " << p.Pos().Y() << ", " << p.Pos().Z()
       << ")  spray_axis_+X=(" << p.Rot().XAxis().X()
       << ", " << p.Rot().XAxis().Y() << ", " << p.Rot().XAxis().Z() << ")";
    Log("PreUpdate", "nozzle_world_pose", ps.str());

    // Attach RaycastData component
    // numRays_ rays spanning the cone solid angle (Fibonacci disk sampling).
    // Rays are in nozzle-local frame; the physics system transforms them by
    // the nozzle world pose each step, so they follow the moving nozzle.
    gz::sim::components::RaycastDataInfo rayData;
    for (const auto &ray : GenerateConeRays())
      rayData.rays.push_back({ray.first, ray.second});

    _ecm.CreateComponent(nozzleEntity_,
        gz::sim::components::RaycastData(rayData));
    raysAttached_ = true;

    Log("PreUpdate", "raycast",
        std::to_string(numRays_) + " cone rays attached to nozzle entity "
        + std::to_string(nozzleEntity_)
        + "  half_angle=" + std::to_string(coneHalfAngle_ * 180.0 / M_PI) + " deg"
        + "  max_range=" + std::to_string(coneMaxRange_) + " m");

    Log("PreUpdate", "emitter",
        "nozzle ready – emitter will be created on first spray-ON trigger"
        "  range=" + std::to_string(coneMaxRange_) + " m" +
        "  half_angle=" + std::to_string(coneHalfAngle_ * 180.0 / M_PI) + " deg");
  }

  // STEP 3 / 3b / 3c: Particle emitter (skipped when disabled in SDF)
  if (enableParticleEmitter_)
  {
    // STEP 3: Particle emitter toggle
    if (emitterEntity_ != kNullEntity)
    {
      const bool active = sprayActive_.load();
      if (active != lastEmitterState_)
      {
        if (!active)
        {
          // Hard stop: remove the emitter entity entirely so the rendering
          // side immediately clears all particles.  It is recreated on the
          // next spray-ON edge (see STEP 3b below).
          _ecm.RequestRemoveEntity(emitterEntity_);
          emitterEntity_ = kNullEntity;
          Log("PreUpdate", "emitter", "OFF – entity removed");
        }
        else
        {
          // Edge case: active flipped to ON while an emitter entity still
          // exists but lastEmitterState_ hadn't caught up yet. This path
          // should not occur via the normal OFF->ON cycle, since going OFF
          // always nulls emitterEntity_ first (see the branch above), which
          // routes recreation through STEP 3b instead. Log only - there is
          // no entity to create here.
          Log("PreUpdate", "emitter", "ON – recreating emitter");
        }
        lastEmitterState_ = active;
      }
    }

    // STEP 3b: (Re)create the emitter when spray is ON and no emitter entity
    // currently exists - covers both first-ever activation (never created)
    // and reactivation after STEP 3 removed it.
    if (sprayActive_ && emitterEntity_ == kNullEntity &&
        nozzleEntity_ != kNullEntity)
    {
      constexpr double kSprayVelocity = 2.0;
      const double effectiveRange =
          coneMaxRange_ / (1.0 + std::tan(coneHalfAngle_));
      const double kLifetime      = effectiveRange / kSprayVelocity;
      const double coneRadiusAtMax = effectiveRange * std::tan(coneHalfAngle_);
      constexpr double kInitSize  = 0.001;
      const double scaleRate =
          std::max((2.0 * coneRadiusAtMax - kInitSize) / kLifetime, 0.01);

      sdf::ParticleEmitter emitterSdf;
      emitterSdf.SetName("spray_emitter_" + std::to_string(++emitterCounter_));
      emitterSdf.SetType(sdf::ParticleEmitterType::POINT);
      emitterSdf.SetEmitting(true);
      emitterSdf.SetRate(particleRate_);
      emitterSdf.SetDuration(0.0);
      emitterSdf.SetLifetime(kLifetime);
      emitterSdf.SetMinVelocity(kSprayVelocity * 0.9);
      emitterSdf.SetMaxVelocity(kSprayVelocity * 1.1);
      emitterSdf.SetColorStart(sprayColor_);
      emitterSdf.SetColorEnd(
          gz::math::Color(sprayColor_.R(), sprayColor_.G(),
                          sprayColor_.B(), 0.0f));
      emitterSdf.SetParticleSize(
          gz::math::Vector3d(kInitSize, kInitSize, kInitSize));
      emitterSdf.SetScaleRate(scaleRate);
      emitterSdf.SetSize(gz::math::Vector3d(0.005, 0.005, 0.005));

      // Local pose zero relative to nozzle - SetParent keeps it as-is (LOCAL).
      // STEP 3c re-asserts zero every PreUpdate so any SetParent-induced drift
      // on 2nd+ activations is corrected before PostUpdate renders it.
      emitterSdf.SetRawPose(gz::math::Pose3d(0, 0, 0, 0, 0, 0));

      sdf::Material emitterMat;
      emitterMat.SetAmbient(sprayColor_);
      emitterMat.SetDiffuse(sprayColor_);
      emitterMat.SetEmissive(sprayColor_);
      emitterSdf.SetMaterial(emitterMat);

      gz::sim::SdfEntityCreator creator(_ecm, *eventMgr_);
      emitterEntity_ = creator.CreateEntities(&emitterSdf);
      creator.SetParent(emitterEntity_, nozzleEntity_);

      Log("PreUpdate", "emitter",
          "recreated entity=" + std::to_string(emitterEntity_));
    }

    // STEP 3c: Force emitter local Pose to zero every frame
    // Uses CreateComponent (not raw pointer write) so the ECM marks the Pose
    // as Changed, triggering the renderer to reposition the Ogre2 scene node.
    // World pose = nozzle_world + local(0,0,0) = nozzle_world every cycle.
    if (emitterEntity_ != kNullEntity)
    {
      _ecm.CreateComponent(emitterEntity_,
          gz::sim::components::Pose(gz::math::Pose3d::Zero));
    }
  }  // enableParticleEmitter_

  // STEP 4: Guard : do nothing if spray not active
  if (!sprayActive_)
    return;

  // STEP 5: One-shot diagnostic on spray-ON edge
  if (!debugDumped_)
  {
    debugDumped_ = true;
    LogSection("PreUpdate – Spray ON Diagnostic");
    const gz::math::Pose3d np = gz::sim::worldPose(nozzleEntity_, _ecm);
    std::ostringstream h;
    h << "pos=(" << np.Pos().X() << ", " << np.Pos().Y() << ", " << np.Pos().Z()
      << ")  axis_+X=(" << np.Rot().XAxis().X()
      << ", " << np.Rot().XAxis().Y() << ", " << np.Rot().XAxis().Z()
      << ")  half_angle=" << (coneHalfAngle_ * 180.0 / M_PI) << " deg"
      << "  max_range=" << coneMaxRange_ << " m"
      << "  rays_attached=" << (raysAttached_ ? "yes" : "no");
    Log("Dump", "nozzle", h.str());
    Log("Dump", "patch_count",
        std::to_string(patchCenters_.size()) + " links painted so far");
  }

  // STEP 6: Rate-limit paint scan.
  // Everything above this point is cheap bookkeeping; everything below is
  // the expensive part (ray results -> link lookup -> patch entities), so
  // it runs only every paintIntervalSteps_ ticks.
  //
  // Note this counts *simulation steps*, not elapsed time, so the effective
  // scan rate depends on the world's <max_step_size>.  Halving the step size
  // doubles how often this fires for the same setting.  Too large a value
  // also lets a fast-moving nozzle skip over surface between scans.
  if ((++paintStepCounter_ % paintIntervalSteps_) != 0)
    return;

  // STEP 7: Read physics raycast results.
  // The physics system fills this component in-place each step from the ray
  // set attached in STEP 2, so we just read back the latest hits.
  if (!raysAttached_) return;

  const auto *raycastComp =
      _ecm.Component<gz::sim::components::RaycastData>(nozzleEntity_);
  if (!raycastComp || raycastComp->Data().results.empty())
    return;

  // Per-scan counters, reported to the perf CSV in STEP 10.  Timing starts
  // here so it covers exactly the rate-limited work, not the bookkeeping.
  const auto scanStart = std::chrono::steady_clock::now();
  uint32_t validHits = 0;
  uint32_t patchesCreated = 0;

  // Raycast results are in the nozzle's own frame, so the nozzle world pose
  // is needed to lift each hit into world coordinates.
  const gz::math::Pose3d nozzlePose = gz::sim::worldPose(nozzleEntity_, _ecm);

  // STEP 8: Build spray material.
  // Shared by every patch created this scan.  Specular is a dimmed copy of
  // the paint colour so patches read as a slightly glossy coating rather
  // than flat unlit decals.
  sdf::Material sdfMat;
  sdfMat.SetAmbient(sprayColor_);
  sdfMat.SetDiffuse(sprayColor_);
  sdfMat.SetSpecular(gz::math::Color(
      sprayColor_.R() * 0.3f,
      sprayColor_.G() * 0.3f,
      sprayColor_.B() * 0.3f, 1.0f));

  // STEP 9: Create paint patches from raycast hits
  for (const auto &res : raycastComp->Data().results)
  {
    // res.fraction is the hit position along this ray, parameterised from
    // 0 (nozzle origin) to 1 (the ray's configured endpoint at
    // coneMaxRange_). It is not a real piece of geometry - fraction == 0
    // is a degenerate hit right at the nozzle, and fraction == 1 is the
    // physics engine's sentinel for "nothing was hit anywhere along this
    // ray", i.e. it ran out of ray to check. Both cases mean "no paintable
    // surface here" and are skipped identically.
    if (res.fraction <= 0.0 || res.fraction >= 1.0) continue;

    // Require a valid outward normal
    if (res.normal.Length() < 0.5) continue;

    // Every ray ends on the plane x = coneMaxRange_, so this is the hit's
    // depth *along the spray axis*, not its Euclidean range from the
    // nozzle (that would be res.point.Length()).  MakePatch wants the
    // axial depth, because the cone radius at depth x is x*tan(halfAngle).
    const double axialDepth = res.fraction * coneMaxRange_;
    if (axialDepth < 1e-3) continue;

    // Transform hit point and normal from nozzle-local to world frame.
    const gz::math::Vector3d hitWorld =
        nozzlePose.CoordPositionAdd(res.point);
    const gz::math::Vector3d normWorld =
        nozzlePose.Rot().RotateVector(res.normal);

    // Find the link whose collision shape is nearest to the hit point.
    const Entity patchParent = FindHitLink(hitWorld, _ecm);
    if (patchParent == kNullEntity)
      continue;  // no paintable surface found - skip silently

    ++validHits;

    const PaintPatch patch = MakePatch(hitWorld, normWorld, axialDepth);
    if (!patch.valid) continue;

    // Convert the hit into the target link's local frame before deduping.
    // Two reasons: the patch is parented to that link (so its pose must be
    // link-local anyway), and deduping in local coordinates keeps working
    // when the target itself moves - world-frame centres would drift.
    const gz::math::Pose3d parentPose  = gz::sim::worldPose(patchParent, _ecm);
    const gz::math::Pose3d localPatchPose = parentPose.Inverse() * patch.worldPose;
    const gz::math::Vector3d newCenter = localPatchPose.Pos();

    // Skip this hit if an existing patch on the same link is already within
    // patchSpacing_, so repeated scans over the same spot stop accumulating
    // entities (entity creation is by far the most expensive thing here).
    //
    // Known limitation: patchSpacing_ is a fixed distance and takes no
    // account of the patch radius or the surface normal.  It is therefore
    // far smaller than a patch at long range (heavy overdraw), and it
    // suppresses legitimate hits on the opposite face of a panel thinner
    // than patchSpacing_.
    auto &centers = patchCenters_[patchParent];
    bool tooClose = false;
    for (const auto &c : centers)
    {
      if ((c - newCenter).Length() < patchSpacing_)
      { tooClose = true; break; }
    }
    if (tooClose) continue;

    // Build the patch as a thin disc: a cylinder whose length is its
    // thickness, laid flat against the surface.
    sdf::Cylinder patchCylinder;
    patchCylinder.SetRadius(patch.size.X() / 2.0);
    patchCylinder.SetLength(patch.size.Z());
    sdf::Geometry patchGeom;
    patchGeom.SetType(sdf::GeometryType::CYLINDER);
    patchGeom.SetCylinderShape(patchCylinder);

    const std::string patchName =
        "paint_patch_" + std::to_string(patchParent) +
        "_" + std::to_string(centers.size());

    sdf::Visual patchVisualSdf;
    patchVisualSdf.SetName(patchName);
    patchVisualSdf.SetRawPose(localPatchPose);
    patchVisualSdf.SetGeom(patchGeom);
    patchVisualSdf.SetMaterial(sdfMat);
    // Paint is a surface marking, not an object - casting shadows would
    // both look wrong and add avoidable render cost per patch.
    patchVisualSdf.SetCastShadows(false);

    // Parenting to the hit link (rather than the world) is what makes paint
    // travel with the object if it is subsequently moved.
    gz::sim::SdfEntityCreator creator(_ecm, *eventMgr_);
    const Entity patchEntity = creator.CreateEntities(&patchVisualSdf);
    creator.SetParent(patchEntity, patchParent);

    ++patchesCreated;
    centers.push_back(newCenter);
  }

  // STEP 10: Record this scan's timing/throughput, if perf logging is on.
  if (perfLogEnabled_)
  {
    const auto scanEnd = std::chrono::steady_clock::now();
    const double scanUs = std::chrono::duration<double, std::micro>(
        scanEnd - scanStart).count();
    const double simTimeS = std::chrono::duration<double>(
        _info.simTime).count();

    perfLog_ << simTimeS << ',' << paintIntervalSteps_ << ',' << numRays_
             << ',' << scanUs << ',' << validHits << ',' << patchesCreated
             << '\n';
    perfLog_.flush();
  }
}

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::SprayPaintPlugin,
              gz::sim::System,
              gz::sim::systems::SprayPaintPlugin::ISystemConfigure,
              gz::sim::systems::SprayPaintPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::SprayPaintPlugin,
                    "gz::sim::systems::SprayPaintPlugin")
