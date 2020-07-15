#ifndef SPARSEGRAPHUTILS_H
#define SPARSEGRAPHUTILS_H

#include <camodocal/camera_models/Camera.h>
#include <camodocal/camera_systems/CameraSystem.h>
#include <Eigen/Dense>

namespace camodocal
{

void rectifyImagePoint(const CameraConstPtr& camera,
                       const cv::Point2f& src, cv::Point2f& dst);

void rectifyImagePoint(const CameraConstPtr& camera,
                       const Eigen::Vector2d& src, Eigen::Vector2d& dst);

void rectifyImagePoints(const CameraConstPtr& camera,
                        const std::vector<cv::Point2f>& src,
                        std::vector<cv::Point2f>& dst);

void pointsToPluckerLines(const CameraSystemConstPtr& cameraSystem, int camera_id,
                        const std::vector<cv::Point2f>& src,
                        std::vector<Eigen::Matrix<double, 6, 1> >& lines);

}

#endif
