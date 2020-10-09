#ifndef SOPHUS_VIEWER_COMMON_H
#define SOPHUS_VIEWER_COMMON_H

// std
#include <thread>
#include <map>

// define the commonly included file to avoid a long include list
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

// for Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

typedef Sophus::SE3d SE3d;
typedef Sophus::SO3d SO3d;

// glog
#include <glog/logging.h>

// not implemented exception
class NotImplemented : public std::logic_error
{
public:
    NotImplemented() : std::logic_error("Function not yet implemented"){};
};

#endif // SOPHUS_VIEWER_COMMON_H