#include "include/path_type.h"
#include "include/helpers.h"
#include <math.h>

namespace path_type {

/*
    Line functions
*/

void Line::computeGuidanceInputs(const Eigen::Vector2d &veh_pos, const Eigen::Vector2d& veh_vel,
    Eigen::Vector2d& closest_point_on_path, Eigen::Vector2d& unit_path_tangent, double& track_error, double& path_curvature)
{
    /*
        Calculate guidance inputs for current vehicle state relative to line path

        input:
        veh_pos (2,1): current vehicle position, NE [m]
        veh_vel (2,1): current vehicle velocity, NE [m/s]

        output:
        closest_pt_on_path (2,1): closest point on line path [m]
        unit_path_tangent (2,1): unit path tangent vector [~]
        track_error: track error [m]
        curvature: path curvature [m^-1]
     */

    const Eigen::Vector2d rel_pos = veh_pos - path_pos_;

    closest_point_on_path = path_pos_ + unit_path_tangent_.dot(rel_pos) * unit_path_tangent_;

    unit_path_tangent = unit_path_tangent_;

    const Eigen::Vector2d unit_path_normal(-unit_path_tangent_(1), unit_path_tangent_(0)); // 90 deg clockwise rotation
    track_error = rel_pos.dot(unit_path_normal);

    path_curvature = 0.0;
} // computeGuidanceInputs

/*
    Loiter functions
*/

void Loiter::computeGuidanceInputs(const Eigen::Vector2d &veh_pos, const Eigen::Vector2d& veh_vel,
    Eigen::Vector2d& closest_point_on_path, Eigen::Vector2d& unit_path_tangent, double& track_error, double& path_curvature)
{
    /*
        Calculate guidance inputs for current vehicle state relative to loiter path

        input:
        veh_pos (2,1): current vehicle position, NE [m]
        veh_vel (2,1): current vehicle velocity, NE [m/s]

        output:
        closest_pt_on_path (2,1): closest point on loiter path [m]
        unit_path_tangent (2,1): unit path tangent vector [~]
        track_error: track error [m]
        curvature: path curvature [m^-1]
     */

    const Eigen::Vector2d rel_pos = veh_pos - path_pos_;
    const double dist_to_center = rel_pos.norm();

    Eigen::Vector2d unit_vec_to_closest_pt;
    if (dist_to_center < MIN_DIST_TO_CENTER) {
        if (veh_vel.norm() < MIN_SPEED) {
            // arbitrarily set the point in the northern top of the circle
            unit_vec_to_closest_pt = rel_pos.normalized();
        }
        else {
            // set the point in the direction we are moving
            unit_vec_to_closest_pt = veh_vel.normalized();
        }
    }
    else {
        // set the point in the direction of the aircraft
        unit_vec_to_closest_pt = rel_pos.normalized();
    }

    closest_point_on_path = unit_vec_to_closest_pt * path_radius_ + path_pos_;

    unit_path_tangent = loiter_dir_ * Eigen::Vector2d(-unit_vec_to_closest_pt(1), unit_vec_to_closest_pt(0));

    const Eigen::Vector2d unit_path_normal(-unit_path_tangent(1), unit_path_tangent(0)); // 90 deg clockwise rotation
    track_error = (veh_pos - closest_point_on_path).dot(unit_path_normal);

    path_curvature = path_curvature_;
} // computeGuidanceInputs

} // namespace path