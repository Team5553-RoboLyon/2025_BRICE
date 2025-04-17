//Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/SubsystemBase.h>
#include "photon/PhotonCamera.h"
#include "photon/PhotonUtils.h"
#include "frc/filter/MedianFilter.h"
#include "frc/filter/LinearFilter.h"
#include "lib/UtilsRBL.h"
#include "lib/pid_rbl.h"
#include "Constants.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include <vector>
// #include "lib/NRollingAverage.h"
#include <frc/geometry/Transform3d.h>
#include <span>
#include <array>

#include "Constants.h"


class Camera : public frc2::SubsystemBase
{
public:
  Camera();

  void Update();

  bool HasTargets();
  std::span<const photon::PhotonTrackedTarget> GetAllTargets();
  photon::PhotonTrackedTarget GetBestTarget();

  //The yaw of the target in degrees (positive right).
  double GetYaw(photon::PhotonTrackedTarget target);
  //The pitch of the target in degrees (positive up).
  double GetPitch(photon::PhotonTrackedTarget target);
  double GetHorizontalDistance(photon::PhotonTrackedTarget target);
  //The area (how much of the camera feed the bounding box takes up) as a percent (0-100).
  double GetArea(photon::PhotonTrackedTarget target);
  double GetDistance(photon::PhotonTrackedTarget target);
  // The skew of the target in degrees (counter-clockwise positive).
  double GetSkew(photon::PhotonTrackedTarget target);
  // The camera to target transform.
  frc::Transform3d GetCameraToTargetTransform(photon::PhotonTrackedTarget target);

  int GetAprilTagID(photon::PhotonTrackedTarget target);
  double GetAmbiguity(photon::PhotonTrackedTarget target);


  photon::PhotonCamera m_camera{"RBLcam"};
  photon::PhotonPipelineResult m_lastResult;

};