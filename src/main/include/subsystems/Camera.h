// Copyright (c) FIRST and other WPILib contributors.
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
#include "lib/NRollingAverage.h"

class Camera : public frc2::SubsystemBase
{
public:
  Camera();
  int getAprilId();
  double GetAngle();
  double GetPitch(int Id_1, int Id_2);
  double GetOutput();
  void SetSetpoint(double setpoint);
  double GetYaw(int Id);
  void Periodic() override;

  double Air;
  double lastAir;
  double diffAir;
  double yaw = 0.0;
  double yaw_dt;
  double yaw_speed; // V/s entre 0 et 1

  double m_output;
  bool drive_auto;

  photon::PhotonCamera m_camera{"RBLcam"};

  NdoubleRollingAverage m_verticalRollingAverage{10};
  NdoubleRollingAverage m_horizontalRollingAverage{10};
};