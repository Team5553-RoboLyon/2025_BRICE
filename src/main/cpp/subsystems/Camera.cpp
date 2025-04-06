// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Camera.h"

Camera::Camera(){};

// This method will be called once per scheduler run

int Camera::getAprilId()
{
    photon::PhotonPipelineResult result = m_camera.GetLatestResult();
    if (result.HasTargets())
    {
        return result.GetBestTarget().GetFiducialId();
    }
    else
    {
        return 0;
    }
}

// double Camera::GetAngle()
// {
//     if (m_camera.HasTargets())
//     {
//         double targetPitch = m_camera.GetLatestResult().GetBestTarget().GetPitch() + 0.01 * diffAir;
//         m_verticalMedian.Calculate(targetPitch);
//     }
//     else
//     {
//         m_verticalMedian.Calculate(m_verticalMedian.LastValue());
//     }
//     return m_verticalMedian.LastValue();
// }

double Camera::GetPitch(int Id_1, int Id_2)
{
    photon::PhotonPipelineResult result = m_camera.GetLatestResult();
    std::cout << m_camera.GetCameraName() << std::endl;
    if (result.HasTargets())
    {
        photon::PhotonPipelineResult result = m_camera.GetLatestResult();
        std::span<const photon::PhotonTrackedTarget> targetsList = result.GetTargets();
        std::vector<photon::PhotonTrackedTarget>::iterator it;
        photon::PhotonTrackedTarget target;

        for (int i = 0; i < result.targets.size(); i++)
        {
            target = result.targets[i];

            if (target.GetFiducialId() == Id_1 || target.GetFiducialId() == Id_2)
            {
                m_verticalRollingAverage.add(target.GetPitch());
                std::cout << target.GetFiducialId() << std::endl;
            }
        }
    }
    else
    {
        m_verticalRollingAverage.add(m_verticalRollingAverage.get());
    }
    return m_verticalRollingAverage.get();
}

double Camera::GetYaw(int Id)
{
    photon::PhotonPipelineResult result = m_camera.GetLatestResult();
    if (result.HasTargets())
    {
        photon::PhotonPipelineResult result = m_camera.GetLatestResult();
        std::span<const photon::PhotonTrackedTarget> targetsList = result.GetTargets();
        std::vector<photon::PhotonTrackedTarget>::iterator it;
        photon::PhotonTrackedTarget target;
        for (int i = 0; i < result.targets.size(); i++)
        {
            target = result.targets[i];
            if (target.GetFiducialId() == Id)
            {
                m_horizontalRollingAverage.add(target.GetYaw());
            }
        }
    }
    else
    {
        m_horizontalRollingAverage.add(m_horizontalRollingAverage.get());
    }
    return m_horizontalRollingAverage.get();
}

double Camera::GetHorizontalDistance()
{
    photon::PhotonPipelineResult result = m_camera.GetLatestResult();
    if (result.HasTargets())
    {
        return result.GetBestTarget().GetBestCameraToTarget().Y().value();
    }
    else
    {
        return 0.0;
    }
}

void Camera::Periodic()
{
    // photon::PhotonPipelineResult result = m_camera.GetLatestResult();
    // if (result.HasTargets())
    // {
    //     yaw = GetYaw(ID_APRILTAG_MIDDLE);
    //     yaw_dt = NABS(yaw) / YAW_DEG;
    //     yaw_speed = -NSIGN(yaw) * YAW_VELOCITY;
    // }

    // if (yaw_dt > 0.0)
    // {
    //     m_output = yaw_speed;
    //     yaw_dt -= 0.02;
    // }
    // else
    // {
    //     m_output = 0.0;
    // }
}