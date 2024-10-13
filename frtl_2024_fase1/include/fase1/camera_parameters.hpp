// CameraParameters.hpp
#pragma once
#include <Eigen/Dense>

struct CameraParameters {
    float fx; // Focal length in pixels along x-axis
    float fy; // Focal length in pixels along y-axis
    float cx; // Principal point x-coordinate
    float cy; // Principal point y-coordinate
    float k1, k2, p1, p2, k3; // Distortion coefficients

    // Constructor with default values (simulation parameters)
    CameraParameters()
        : fx(320), fy(320), cx(320), cy(240),
          k1(0.0), k2(0.0), p1(0.0), p2(0.0), k3(0.0) {}
};
