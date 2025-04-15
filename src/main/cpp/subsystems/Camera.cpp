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

