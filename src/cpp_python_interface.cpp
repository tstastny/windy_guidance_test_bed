#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "include/npfg.h"
#include "include/path_type.h"
#include <tuple>

/*
    Wrappers / trampolines
*/

namespace path_type {

class PyPathSegment : public PathSegment {

    public:

        using PathSegment::PathSegment; // Inherit constructors

        /* Trampoline (need one for each virtual function) */
        void computeGuidanceInputs(const Eigen::Vector2d &veh_pos, const Eigen::Vector2d& veh_vel,
            Eigen::Vector2d& closest_point_on_path, Eigen::Vector2d& unit_path_tangent,
            double& track_error, double& path_curvature) override {
            PYBIND11_OVERRIDE_PURE(void, PathSegment, computeGuidanceInputs, veh_pos, veh_vel);
        }
};

/*
    Python interface
*/

namespace py = pybind11;

PYBIND11_MODULE(cpp_functions, m)
{
    py::class_<NPFG>(m, "NPFG")
        .def(py::init<>())
        .def("getAirspeedRef", &NPFG::getAirspeedRef)
        .def("getBearingFeas", &NPFG::getBearingFeas)
        .def("getBearingVec", &NPFG::getBearingVec)
        .def("getLateralAccel", &NPFG::getLateralAccel)
        .def("getPGainAdj", &NPFG::getPGainAdj)
        .def("getTrackErrorBound", &NPFG::getTrackErrorBound)
        .def("getGroundSpMinE", &NPFG::getGroundSpMinE)
        .def("enableBackwardsSolution", &NPFG::enableBackwardsSolution)
        .def("enableFeedForwardAirVelRef", &NPFG::enableFeedForwardAirVelRef)
        .def("enableMinGroundSpeed", &NPFG::enableMinGroundSpeed)
        .def("enableTrackKeeping", &NPFG::enableTrackKeeping)
        .def("enableWindExcessRegulation", &NPFG::enableWindExcessRegulation)
        .def("setAirspeedMax", &NPFG::setAirspeedMax)
        .def("setAirspeedNom", &NPFG::setAirspeedNom)
        .def("setMinGroundSpeed", &NPFG::setMinGroundSpeed)
        .def("setMinGroundSpeedEMax", &NPFG::setMinGroundSpeedEMax)
        .def("setNominalHeadingRate", &NPFG::setNominalHeadingRate)
        .def("setNTEFraction", &NPFG::setNTEFraction)
        .def("setPathCurvature", &NPFG::setPathCurvature)
        .def("setPeriod", &NPFG::setPeriod)
        .def("setDamping", &NPFG::setDamping)
        .def("setWindRatioBuf", &NPFG::setWindRatioBuf)
        .def("evaluate", &NPFG::evaluate);

    py::class_<PathSegment, PyPathSegment>(m, "PathSegment")
        .def(py::init<>());

    py::class_<Line, PathSegment>(m, "Line")
        .def(py::init<const Eigen::Vector2d&, const Eigen::Vector2d&>())
        .def("pos", &Line::pos)
        .def("upt", &Line::upt)
        .def("updateState", &Line::updateState)
        .def("getClosestPoint", &Line::getClosestPoint)
        .def("getUnitTangent", &Line::getUnitTangent)
        .def("getTrackError", &Line::getTrackError)
        .def("getCurvature", &Line::getCurvature)
        .def("setPosition", &Line::setPosition)
        .def("setUnitTangent", &Line::setUnitTangent);

    py::class_<Loiter, PathSegment>(m, "Loiter")
        .def(py::init<const Eigen::Vector2d&, const double&, const int&>())
        .def("pos", &Loiter::pos)
        .def("radius", &Loiter::radius)
        .def("dir", &Loiter::dir)
        .def("updateState", &Loiter::updateState)
        .def("getClosestPoint", &Loiter::getClosestPoint)
        .def("getUnitTangent", &Loiter::getUnitTangent)
        .def("getTrackError", &Loiter::getTrackError)
        .def("getCurvature", &Loiter::getCurvature)
        .def("setPosition", &Loiter::setPosition)
        .def("setRadius", &Loiter::setRadius)
        .def("setLoiterDir", &Loiter::setLoiterDir);
}

} // namespace path_type