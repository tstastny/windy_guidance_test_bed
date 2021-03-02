#ifndef PATH_TYPE_H_
#define PATH_TYPE_H_

// TODO: need a better name.. path_lib? (if keeping everything here..), path_segment? (if separating into derived class headers)

#include <Eigen/Eigen>
#include <math.h>

namespace path_type {

// constants TODO: would be better to stick these individually into derived classes.. but need to find a way to initialize static const eigen vector
const Eigen::Vector2d DEFAULT_POS(0.0, 0.0);
const Eigen::Vector2d DEFAULT_UPT(1.0, 0.0);
const double UNIT_VEC_THRES = 0.1;
const double MIN_LOITER_RADIUS = 1.0; // m
const double MIN_DIST_TO_CENTER = 0.1; // m
const double MIN_SPEED = 0.1; // m/s

/*
    (base) PathSegment class (abstract)
    Manage operations related to path object
*/
class PathSegment {

    protected:

        // pybind wrappers - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // NOTE: would have been nicer to wrap with single function outputting a tuple from computeGuidanceInputs
        // but pybind wasn't keen on doing this from an overridden class member for some reason.
        // REMEMBER: it wont be necessary to store these variables on each path object when not using pybind
        Eigen::Vector2d pb_closest_point_on_path_;
        Eigen::Vector2d  pb_unit_path_tangent_;
        double pb_track_error_;
        double pb_path_curvature_;
        // pybind wrappers - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    public:

        // functions
        virtual void computeGuidanceInputs(const Eigen::Vector2d &veh_pos, const Eigen::Vector2d& veh_vel,
            Eigen::Vector2d& closest_point_on_path, Eigen::Vector2d& unit_path_tangent, double& track_error, double& path_curvature) = 0; // XXX: better name for this?

        // pybind wrappers - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // NOTE: again, these are just to play nice with pybind
        // REMEMBER: updateState() wrapper needs to be called first
        void updateState(const Eigen::Vector2d &veh_pos, const Eigen::Vector2d& veh_vel) {
            computeGuidanceInputs(veh_pos, veh_vel, pb_closest_point_on_path_, pb_unit_path_tangent_, pb_track_error_, pb_path_curvature_);
        }
        Eigen::Vector2d getClosestPoint() { return pb_closest_point_on_path_; }
        Eigen::Vector2d getUnitTangent() { return pb_unit_path_tangent_; }
        double getTrackError() { return pb_track_error_; }
        double getCurvature() { return pb_path_curvature_; }
        // pybind wrappers - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

}; // class PathSegment

/*
    Line class (derived from Path)
    Manage operations related to line path
*/
class Line : public PathSegment {

    private:

        // parameters
        Eigen::Vector2d path_pos_;
        Eigen::Vector2d unit_path_tangent_;

        // functions
        Eigen::Vector2d safePathTangent(const Eigen::Vector2d& vec) {
            if (vec.norm() < UNIT_VEC_THRES) {
                return DEFAULT_UPT;
            }
            return vec.normalized();
        }

    public:

        // constructors
        Line() : path_pos_(DEFAULT_POS), unit_path_tangent_(DEFAULT_UPT) {}
        Line(const Eigen::Vector2d& pos, const Eigen::Vector2d& unit_path_tangent) :
            path_pos_(pos),
            unit_path_tangent_(safePathTangent(unit_path_tangent))
            {}

        // functions
        void computeGuidanceInputs(const Eigen::Vector2d &veh_pos, const Eigen::Vector2d& veh_vel,
            Eigen::Vector2d& closest_point_on_path, Eigen::Vector2d& unit_path_tangent, double& track_error, double& path_curvature) override;

        // setters
        void setPosition(const Eigen::Vector2d& pos) { path_pos_ = pos; }
        void setUnitTangent(const Eigen::Vector2d& unit_path_tangent) { unit_path_tangent_ = safePathTangent(unit_path_tangent); }

        // getters
        Eigen::Vector2d pos() { return path_pos_; }
        Eigen::Vector2d upt() { return unit_path_tangent_; }

}; // class Line

/*
    Loiter class (derived from Path)
    Manage operations related to loiter path
*/
class Loiter : public PathSegment {

    private:

        // parameters
        Eigen::Vector2d path_pos_;
        Eigen::Vector2d unit_path_tangent_;
        double path_radius_;
        double path_curvature_;
        double loiter_dir_;

        // functions
        double safeRadius(const double& radius) {
            return (radius < MIN_LOITER_RADIUS) ? MIN_LOITER_RADIUS : radius;
        }
        double typeCastDir(const int& dir) {
            return (dir < 0) ? -1.0 : 1.0; // XXX: should use an enum? (e.g. CW, CCW)
        }

    public:

        // constructors
        Loiter() :
            path_pos_(DEFAULT_POS),
            unit_path_tangent_(DEFAULT_UPT),
            path_radius_(MIN_LOITER_RADIUS),
            path_curvature_(MIN_LOITER_RADIUS),
            loiter_dir_(1.0)
            {}
        Loiter(const Eigen::Vector2d& pos, const double& radius, const int& dir) :
            path_pos_(pos),
            unit_path_tangent_(DEFAULT_UPT),
            path_radius_(safeRadius(radius)),
            path_curvature_(1.0 / path_radius_),
            loiter_dir_(typeCastDir(dir))
            {}

        // functions
        void computeGuidanceInputs(const Eigen::Vector2d &veh_pos, const Eigen::Vector2d& veh_vel,
            Eigen::Vector2d& closest_point_on_path, Eigen::Vector2d& unit_path_tangent, double& track_error, double& path_curvature) override;

        // setters
        void setPosition(const Eigen::Vector2d& pos) { path_pos_ = pos; }
        void setRadius(const double& radius) {
            path_radius_ = safeRadius(radius);
            path_curvature_ = 1.0 / path_radius_;
        }
        void setLoiterDir(const int& dir) { loiter_dir_ = typeCastDir(dir); }

        // getters
        Eigen::Vector2d pos() { return path_pos_; }
        double radius() { return path_radius_; }
        double dir() { return loiter_dir_; }

}; // class Loiter

} // namespace path_type

#endif // PATH_TYPE_H_