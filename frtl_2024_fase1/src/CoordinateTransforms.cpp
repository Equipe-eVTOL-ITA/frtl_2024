#include "fase1/CoordinateTransforms.hpp"

Eigen::Vector3d CoordinateTransforms::ImageToWorld(
    const Eigen::Vector3d& drone_pos,
    const Eigen::Vector3d& drone_orientation,
    float center_x, float center_y)
{
    // Convert image coordinates to normalized coordinates
    std::vector<cv::Point2f> distorted_points = {
        cv::Point2f(center_x, center_y)
    };

    // Undistort points
    std::vector<cv::Point2f> undistorted_points;
    cv::undistortPoints(
        distorted_points,
        undistorted_points,
        camera_matrix_,
        dist_coeffs_);

    // Get the normalized undistorted coordinates
    float x_u = undistorted_points[0].x;
    float y_u = undistorted_points[0].y;

    // Direction of the ray in the camera frame
    Eigen::Vector3d ray_cam(x_u, y_u, 1.0);
    ray_cam.normalize();

    // Compute drone rotation matrix R_wd from roll, pitch, yaw
    double roll = drone_orientation[0];
    double pitch = drone_orientation[1];
    double yaw = drone_orientation[2];

    Eigen::Matrix3d R_wd =
        (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

    // Compute camera rotation matrix R_wc
    Eigen::Matrix3d R_wc = R_wd * R_dc_;

    // Compute camera position t_wc
    Eigen::Vector3d t_wc = drone_pos + R_wd * t_dc_;

    // Compute s
    double numerator = ground_z_ - t_wc[2];
    double denominator = R_wc.col(2).dot(ray_cam);
    double s = numerator / denominator;

    // Compute the world coordinates of the landing pad
    Eigen::Vector3d pad_world_coords = t_wc + s * (R_wc * ray_cam);

    return pad_world_coords;
}


bool CoordinateTransforms::WorldToImage(
    const Eigen::Vector3d& drone_pos,
    const Eigen::Vector3d& drone_orientation,
    const Eigen::Vector3d& pad_pos,
    float& center_x, float& center_y,
    float& size_x, float& size_y,
    const float pad_size)
{
    // Compute drone rotation matrix R_wd
    double roll = drone_orientation[0];
    double pitch = drone_orientation[1];
    double yaw = drone_orientation[2];

    Eigen::Matrix3d R_wd =
        (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

    // Compute camera rotation matrix R_wc and its inverse R_cw
    Eigen::Matrix3d R_wc = R_wd * R_dc_;
    Eigen::Matrix3d R_cw = R_wc.transpose();

    // Compute camera position t_wc
    Eigen::Vector3d t_wc = drone_pos + R_wd * t_dc_;

    // Vector from camera to pad in world coordinates
    Eigen::Vector3d vec_cw = pad_pos - t_wc;

    // Transform to camera frame
    Eigen::Vector3d vec_cc = R_cw * vec_cw;

    if (vec_cc[2] <= 0) {
        // Pad is behind the camera
        return false;
    }

    // Project onto image plane
    double x_u = vec_cc[0] / vec_cc[2];
    double y_u = vec_cc[1] / vec_cc[2];

    // Apply distortion
    double r2 = x_u * x_u + y_u * y_u;
    double radial_dist = 1 + dist_coeffs_.at<double>(0) * r2 +
                             dist_coeffs_.at<double>(1) * r2 * r2 +
                             dist_coeffs_.at<double>(4) * r2 * r2 * r2;
    double x_d = x_u * radial_dist +
                 2 * dist_coeffs_.at<double>(2) * x_u * y_u +
                 dist_coeffs_.at<double>(3) * (r2 + 2 * x_u * x_u);
    double y_d = y_u * radial_dist +
                 dist_coeffs_.at<double>(2) * (r2 + 2 * y_u * y_u) +
                 2 * dist_coeffs_.at<double>(3) * x_u * y_u;

    // Convert to pixel coordinates
    double u = camera_matrix_.at<double>(0, 0) * x_d + camera_matrix_.at<double>(0, 2);
    double v = camera_matrix_.at<double>(1, 1) * y_d + camera_matrix_.at<double>(1, 2);

    // Normalize pixel coordinates
    int image_width = static_cast<int>(camera_matrix_.at<double>(0, 2) * 2);
    int image_height = static_cast<int>(camera_matrix_.at<double>(1, 2) * 2);
    center_x = static_cast<float>(u) / image_width;
    center_y = static_cast<float>(v) / image_height;

    // Compute apparent size of the landing pad
    // Let's consider four corners of the landing pad
    float half_size = pad_size / 2.0f;
    Eigen::Vector3d pad_corners[4] = {
        pad_pos + Eigen::Vector3d(half_size, half_size, 0),
        pad_pos + Eigen::Vector3d(half_size, -half_size, 0),
        pad_pos + Eigen::Vector3d(-half_size, half_size, 0),
        pad_pos + Eigen::Vector3d(-half_size, -half_size, 0)
    };

    float min_u = std::numeric_limits<float>::max();
    float max_u = std::numeric_limits<float>::lowest();
    float min_v = std::numeric_limits<float>::max();
    float max_v = std::numeric_limits<float>::lowest();

    for (int i = 0; i < 4; ++i) {
        // Vector from camera to corner
        vec_cw = pad_corners[i] - t_wc;
        vec_cc = R_cw * vec_cw;

        if (vec_cc[2] <= 0) continue; // Skip if behind camera

        x_u = vec_cc[0] / vec_cc[2];
        y_u = vec_cc[1] / vec_cc[2];

        // Apply distortion
        r2 = x_u * x_u + y_u * y_u;
        radial_dist = 1 + dist_coeffs_.at<double>(0) * r2 +
                          dist_coeffs_.at<double>(1) * r2 * r2 +
                          dist_coeffs_.at<double>(4) * r2 * r2 * r2;
        x_d = x_u * radial_dist +
              2 * dist_coeffs_.at<double>(2) * x_u * y_u +
              dist_coeffs_.at<double>(3) * (r2 + 2 * x_u * x_u);
        y_d = y_u * radial_dist +
              dist_coeffs_.at<double>(2) * (r2 + 2 * y_u * y_u) +
              2 * dist_coeffs_.at<double>(3) * x_u * y_u;

        // Convert to pixel coordinates
        u = camera_matrix_.at<double>(0, 0) * x_d + camera_matrix_.at<double>(0, 2);
        v = camera_matrix_.at<double>(1, 1) * y_d + camera_matrix_.at<double>(1, 2);

        // Update min and max
        min_u = std::min(min_u, static_cast<float>(u));
        max_u = std::max(max_u, static_cast<float>(u));
        min_v = std::min(min_v, static_cast<float>(v));
        max_v = std::max(max_v, static_cast<float>(v));
    }

    if (min_u == std::numeric_limits<float>::max()) {
        // All corners are behind the camera
        return false;
    }

    // Compute size in pixels
    size_x = (max_u - min_u) / image_width;
    size_y = (max_v - min_v) / image_height;

    return true;
}
