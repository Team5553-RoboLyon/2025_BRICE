#include "subsystems/Camera.h"

Camera::Camera() {};

void Camera::Update() 
{
    m_lastResult = m_camera.GetLatestResult();
}

bool Camera::HasTargets()
{
    return m_lastResult.HasTargets();
}

std::span<const photon::PhotonTrackedTarget> Camera::GetAllTargets()
{
    return m_lastResult.GetTargets();
}

photon::PhotonTrackedTarget Camera::GetBestTarget() 
{
    return m_lastResult.GetBestTarget();
}

// ------------------- DATA FROM TARGETS -----------------------
double Camera::GetYaw(photon::PhotonTrackedTarget target) 
{
    return target.GetYaw();
}
double Camera::GetPitch(photon::PhotonTrackedTarget target) 
{
    return target.GetPitch();
}
double Camera::GetArea(photon::PhotonTrackedTarget target) 
{
    
    return target.GetArea();
}
double Camera::GetSkew(photon::PhotonTrackedTarget target) 
{
    return target.GetSkew();
}
frc::Transform3d Camera::GetCameraToTargetTransform(photon::PhotonTrackedTarget target) 
{
    return target.GetBestCameraToTarget();
}
double Camera::GetHorizontalDistance(photon::PhotonTrackedTarget target) 
{
    return target.GetBestCameraToTarget().Y().value();
}


int Camera::GetAprilTagID(photon::PhotonTrackedTarget target) 
{
    return target.GetFiducialId();
}
double Camera::GetAmbiguity(photon::PhotonTrackedTarget target) 
{
    return target.GetPoseAmbiguity();
}

std::array<double, 2> Camera::GetErrorWithReefscapeBranch() 
{
    std::array<double, 2> output = {0.0, 0.0};
    if (m_lastResult.HasTargets()) {
        photon::PhotonTrackedTarget target = m_lastResult.GetBestTarget();
        if(target.GetFiducialId() == 1 /*TODO : list all ID en f(x) du coté*/)
        {
            output[0] = target.GetBestCameraToTarget().Y().value() - 1; // TODO : measure distance between APRIL TAG and the left reefscape branch
            output[1] = target.GetBestCameraToTarget().Y().value() +1; // TODO : measure distance between APRIL TAG and the left reefscape branch
            return output;
        }
    }
    return {0.0, 0.0};
}

