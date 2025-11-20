#pragma once

#include <memory>

#include <Eigen/Core>

namespace ORB_SLAM2
{
    
struct VoResult
{
    VoResult() 
    : bOK(false), Timestamp(0.0), nInliers(0)
    , Rwc(Eigen::Matrix3d::Identity())
    , Pwc(Eigen::Vector3d::Zero())
    , bIsKeyframe(false), nResetCount(0)
    {}

    bool bOK;
    double Timestamp;
    int nInliers;
    Eigen::Matrix3d Rwc;
    Eigen::Vector3d Pwc;
    bool bIsKeyframe;
    int nResetCount;
};
using VoResultPtr = std::shared_ptr<VoResult>;


} // namespace ORB_SLAM2
