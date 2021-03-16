#include "include/npfg.h"
#include "include/helpers.h"
#include <math.h>

NPFG::NPFG() :
    feas_(1.0),
    feas0_(0.0),
    lateral_accel_(0.0),
    lateral_accel_curv_(0.0),
    path_curvature_(0.0),
    track_error_bound_(1.0),
    bearing_vec_(Eigen::Vector2d(1.0, 0.0)),
    p_gain_(0.11),
    time_const_(7.0),
    period_(20.0),
    damping_(0.7071),
    airspeed_nom_(14.0),
    airspeed_max_(20.0),
    airspeed_ref_(14.0),
    wind_ratio_buf_(0.1),
    inv_nte_fraction_(0.5),
    min_ground_speed_e_(0.0),
    min_ground_speed_e_max_(0.0),
    min_ground_speed_g_(0.0),
    en_wind_excess_regulation_(true),
    en_track_keeping_(false),
    en_min_ground_speed_(false),
    feed_forward_air_vel_(false),
    en_backwards_solution_(false),
    nom_heading_rate_(0.6),
    en_period_lower_bound_(false),
    en_period_upper_bound_(false),
    track_proximity_fraction_(1.0)
{} // NPFG

double NPFG::periodUB(const double airspeed, const double wind_ratio) {
    if (wind_ratio * path_curvature_ > 0.0) {
        const double turn_rate = airspeed * path_curvature_;
        const double crit_wind_factor = 2.0 * (1.0 - sqrt(1.0 - wind_ratio));
        return (damping_ < 0.5) ? 4.0 * M_PI * damping_ / (turn_rate * crit_wind_factor) : M_PI / (damping_ * turn_rate * crit_wind_factor);
    }
    return INFINITY;
}

double NPFG::periodUB(const double airspeed, const double wind_ratio, const double wind_cross_upt, const double wind_dot_upt, const double wind_speed) {
    if (wind_ratio * path_curvature_ > 0.0) {
        const double turn_rate = airspeed * path_curvature_;
        const double sin_cross_wind_angle = wind_cross_upt / wind_speed;
        const double cos_cross_wind_angle = wind_dot_upt / wind_speed;
        const double wr_s_cwa = wind_ratio * sin_cross_wind_angle;
        const double wind_factor = std::max(wind_ratio * wr_s_cwa * cos_cross_wind_angle / sqrt(1.0 - wr_s_cwa * wr_s_cwa) + wr_s_cwa, 0.0);
        return (damping_ < 0.5) ? 4.0 * M_PI * damping_ / (turn_rate * wind_factor) : M_PI / (damping_ * turn_rate * wind_factor);
    }
    return INFINITY;
}

double NPFG::periodLB(const double airspeed, const double wind_ratio) {
    const double tc_roll = 1.0;
    const double turn_rate = airspeed * path_curvature_;
    const double crit_wind_factor = 2.0 * (1.0 - sqrt(1.0 - wind_ratio));
    const double omf = turn_rate * crit_wind_factor;
    const double omft = omf * tc_roll;
    const double omft_m_1 = omft - 1;
    return (omf != 0.0) ? 2.0 * M_PI / omf * ( sqrt(omft_m_1 * omft_m_1 * damping_ * damping_ + omft) + omft_m_1 * damping_ ) : M_PI * tc_roll / damping_;
}

void NPFG::calcGains(const double ground_speed, const double airspeed, const double wind_ratio, const double track_error, const Eigen::Vector2d& wind_vel, const Eigen::Vector2d& upt)
{
    double period = period_;
    if (en_period_lower_bound_) {
        // lower bound the period for stability w.r.t. roll time constant and current flight condition
        const double period_lb = periodLB(airspeed, wind_ratio);
        period = std::max(period_lb, period);

        // only allow upper bounding if lower bounding is enabled
        if (en_period_upper_bound_) {
            // NOTE: if the roll time constant is not accurately known, reducing the period here can destabilize the system!
            //       enable this feature at your own risk!

            // upper bound the period (track keeping stability), prefer lower bound if enabled and violated
            const double period_ub = periodUB(airspeed, wind_ratio);
            const double period_adapted = (period_ub > period_lb) ? std::min(period, period_ub) : period_lb;
//            double wind_cross_upt, wind_dot_upt;
//            trigWindToBearing(wind_cross_upt, wind_dot_upt, wind_vel, upt);
//            const double period_ub = constrain(period, period_lb, periodUB(airspeed, wind_ratio, wind_cross_upt, wind_dot_upt, wind_vel.norm())); // note: constrain() checks min first

            // recalculate time constant (needed for track proximity calculation)
            calcTimeConst(period);

            const double normalized_track_error = constrain(track_error / calcTrackErrorBound(ground_speed), 0.0, 1.0);

            // calculate nominal track proximity with lower bounded time constant
            // (only a num. sol. can find corresponding track proximity and adapted gains simultaneously)
            const double track_proximity = std::max(calcTrackProximityOnly(normalized_track_error) + track_proximity_fraction_ - 1.0, 0.0) / track_proximity_fraction_;

            // transition from the nominal period to the adapted period as we get closer to the track
            period = period_adapted * track_proximity + (1.0 - track_proximity) * period;
        }
    }
    calcPGain(period);
    calcTimeConst(period);
} // calcGains

void NPFG::calcPGain(const double period)
{
    p_gain_ = 4.0 * M_PI * damping_ / period;
}

void NPFG::calcTimeConst(const double period)
{
    time_const_ = period * damping_;
}

double NPFG::calcTrackErrorBound(const double ground_speed)
{
    if (ground_speed > 1.0) {
      return ground_speed * time_const_;
    }
    else {
      // limit bound to some minimum ground speed to avoid singularities in track error normalization
      // the following equation assumes ground speed minimum = 1.0
      return 0.5 * time_const_ * (ground_speed * ground_speed + 1.0);
    }
} // calcTrackErrorBound

double NPFG::calcLookAheadAngle(const double normalized_track_error)
{
    return M_PI_2 * (normalized_track_error - 1.0) * (normalized_track_error - 1.0);
} // calcLookAheadAngle

double NPFG::calcTrackProximityOnly(const double normalized_track_error) {
    const double sin_look_ahead_angle = sin(calcLookAheadAngle(normalized_track_error));
    return sin_look_ahead_angle * sin_look_ahead_angle;
}

Eigen::Vector2d NPFG::calcBearingVec(double &track_proximity, double &inv_track_proximity,
    const Eigen::Vector2d &unit_track_error, const Eigen::Vector2d &unit_path_tangent, const double look_ahead_angle)
{
    /*
        calculate the bearing vector and track proximity smoother from the look-ahead angle mapping
    */

    const double cos_look_ahead_angle = cos(look_ahead_angle);
	const double sin_look_ahead_angle = sin(look_ahead_angle);

    track_proximity = sin_look_ahead_angle * sin_look_ahead_angle;
    inv_track_proximity = cos_look_ahead_angle * cos_look_ahead_angle;

    return cos_look_ahead_angle * unit_track_error + sin_look_ahead_angle * unit_path_tangent;
} // calcBearingVec

double NPFG::calcMinGroundSpeed(const double normalized_track_error)
{
    min_ground_speed_e_ = 0.0;
    if (en_track_keeping_ * en_wind_excess_regulation_) {
        min_ground_speed_e_ = min_ground_speed_e_max_ * constrain(normalized_track_error * inv_nte_fraction_, 0.0, 1.0);
    }

    double min_ground_speed_g = 0.0;
    if (en_min_ground_speed_ * en_wind_excess_regulation_) {
        min_ground_speed_g = min_ground_speed_g_;
    }

    return std::max(min_ground_speed_e_, min_ground_speed_g);
} // calcMinGroundSpeed

void NPFG::trigWindToBearing(double &wind_cross_bearing, double &wind_dot_bearing, const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec)
{
    wind_cross_bearing = wind_vel(0) * bearing_vec(1) - wind_vel(1) * bearing_vec(0);
	wind_dot_bearing = wind_vel(0) * bearing_vec(0) + wind_vel(1) * bearing_vec(1);
} // trigWindToBearing

double NPFG::projectAirspOnBearing(const double airspeed, const double wind_cross_bearing)
{
    return sqrt(std::max(airspeed * airspeed - wind_cross_bearing * wind_cross_bearing, 0.0));
} // projectAirspOnBearing

int NPFG::bearingIsFeasible(const double wind_cross_bearing, const double wind_dot_bearing, const double airspeed, const double wind_speed)
{
    return (abs(wind_cross_bearing) < airspeed) && ((wind_dot_bearing > 0.0) || (wind_speed < airspeed));
} // bearingIsFeasible

double NPFG::calcBearingFeas(const double wind_cross_bearing, const double wind_dot_bearing, const double wind_speed, const double wind_ratio)
{
    double sin_lambda; // in [0, 1]
    if (wind_dot_bearing <= 0.0) {
        sin_lambda = 1.0;
    }
    else {
        sin_lambda = fabs(wind_cross_bearing / wind_speed);
    }

	// upper and lower feasibility barriers
	double wind_ratio_ub, wind_ratio_lb;
	if (sin_lambda < LAMBDA_CO) { // small angle approx.
		// linear finite cut-off
		const double mx = M_CO * (LAMBDA_CO - sin_lambda);
		const double wind_ratio_ub_co = ONE_OVER_S_LAMBDA_CO;
		wind_ratio_ub = wind_ratio_ub_co + mx;
		const double wind_ratio_lb_co = (ONE_OVER_S_LAMBDA_CO - 2.0) * wind_ratio_buf_ + 1.0;
		wind_ratio_lb = wind_ratio_lb_co + wind_ratio_buf_ * mx;
	}
    else {
        const double one_over_s_lambda = 1.0 / sin_lambda;
        wind_ratio_ub = one_over_s_lambda;
        wind_ratio_lb = (one_over_s_lambda - 2.0) * wind_ratio_buf_ + 1.0;
    }

	// calculate bearing feasibility
	double feas;
    if (wind_ratio > wind_ratio_ub) {
        // infeasible
        feas = 0.0;
    }
    else if (wind_ratio > wind_ratio_lb) {
        // partially feasible
        // smoothly transition from fully feasible to infeasible
        feas = cos(M_PI_2 * constrain((wind_ratio - wind_ratio_lb) / (wind_ratio_ub - wind_ratio_lb), 0.0, 1.0));
        feas *= feas;
	}
    else {
        // feasible
        feas = 1.0;
    }

    return feas;
} // calcBearingFeas

Eigen::Vector2d NPFG::calcRefAirVelocityFF(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec,
    const double wind_cross_bearing, const double wind_dot_bearing, const double wind_speed, const double min_ground_speed,
    bool use_backwards_solution)
{
    /*
        determine air velocity reference without curvature compensation
        :: with "optimal" airspeed reference compensation
    */

    Eigen::Vector2d air_vel_ref;

    if (min_ground_speed > wind_dot_bearing && (en_min_ground_speed_ || en_track_keeping_) * en_wind_excess_regulation_) {
        // minimum ground speed and/or track keeping

        // airspeed required to achieve min. ground speed along bearing vector
        const double airspeed_min = sqrt((min_ground_speed - wind_dot_bearing) * (min_ground_speed - wind_dot_bearing)
            + wind_cross_bearing * wind_cross_bearing);

        if (airspeed_min > airspeed_max_) {
            if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_max_, wind_speed)) {
                const double airsp_dot_bearing = projectAirspOnBearing(airspeed_max_, wind_cross_bearing);
                air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
            }
            else {
                air_vel_ref = calcInfeasibleAirVelRef(wind_vel, bearing_vec, wind_speed, airspeed_max_);
            }
        }
        else if (airspeed_min > airspeed_nom_) {
            const double airsp_dot_bearing = projectAirspOnBearing(airspeed_min, wind_cross_bearing);
            air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
        }
        else {
            const double airsp_dot_bearing = projectAirspOnBearing(airspeed_nom_, wind_cross_bearing);
            if (use_backwards_solution) {
                air_vel_ref = rotate2d(wind_cross_bearing, -airsp_dot_bearing, bearing_vec);
            }
            else {
                air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
            }
        }
    }
    else {
        // wind excess regulation and/or mitigation

        if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_nom_, wind_speed)) {
            const double airsp_dot_bearing = projectAirspOnBearing(airspeed_nom_, wind_cross_bearing);
            if (use_backwards_solution) {
                air_vel_ref = rotate2d(wind_cross_bearing, -airsp_dot_bearing, bearing_vec);
            }
            else {
                air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
            }
        }
        else if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_max_, wind_speed) && en_wind_excess_regulation_) {
            if (wind_dot_bearing <= 0.0) {
                air_vel_ref = wind_vel;
            }
            else {
                const double airsp_dot_bearing = 0.0; // right angle to the bearing line gives minimum airspeed usage, zero forward ground speed
                air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
            }
        }
        else {
            // infeasible
            const double airspeed_input = (en_wind_excess_regulation_) ? airspeed_max_ : airspeed_nom_;
            air_vel_ref = calcInfeasibleAirVelRef(wind_vel, bearing_vec, wind_speed, airspeed_input);
        }
    }

    return air_vel_ref;
} // calcRefAirVelocityFF

Eigen::Vector2d NPFG::calcRefAirVelocityFB(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec,
    const double wind_cross_bearing, const double wind_dot_bearing, const double wind_speed, const double airspeed,
    bool use_backwards_solution)
{
    /*
        determine air velocity reference without curvature compensation
        :: this is the feedback formulation (at current airspeed), no airspeed reference compensation
    */

    Eigen::Vector2d air_vel_ref;

    if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed, wind_speed)) {
        const double airsp_dot_bearing = projectAirspOnBearing(airspeed, wind_cross_bearing);
        if (use_backwards_solution) {
            air_vel_ref = rotate2d(wind_cross_bearing, -airsp_dot_bearing, bearing_vec);
        }
        else {
            air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
        }
    }
    else {
        air_vel_ref = calcInfeasibleAirVelRef(wind_vel, bearing_vec, wind_speed, airspeed);
    }

    return air_vel_ref;
} // calcRefAirVelocityFB

Eigen::Vector2d NPFG::calcInfeasibleAirVelRef(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec, const double wind_speed, const double airspeed)
{
    Eigen::Vector2d air_vel_ref = sqrt(std::max(wind_speed * wind_speed - airspeed * airspeed, 0.0)) * bearing_vec - wind_vel;
    return air_vel_ref.normalized() * airspeed;
} // calcInfeasibleAirVelRef

double NPFG::calcRefAirspeed(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec,
    const double wind_cross_bearing, const double wind_dot_bearing, const double wind_speed, const double min_ground_speed)
{
    /*
        determine airspeed reference without curvature compensation
        :: with "optimal" airspeed reference compensation
    */

    double airspeed_ref;

    if (min_ground_speed > wind_dot_bearing && (en_min_ground_speed_ || en_track_keeping_) * en_wind_excess_regulation_) {
        // minimum ground speed and/or track keeping

        // airspeed required to achieve min. ground speed along bearing vector
        const double airspeed_min = sqrt((min_ground_speed - wind_dot_bearing) * (min_ground_speed - wind_dot_bearing)
            + wind_cross_bearing * wind_cross_bearing);

        airspeed_ref = constrain(airspeed_min, airspeed_nom_, airspeed_max_);
    }
    else {
        // wind excess regulation and/or mitigation

        if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_nom_, wind_speed)) {
            airspeed_ref = airspeed_nom_;
        }
        else if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_max_, wind_speed) && en_wind_excess_regulation_) {

            if (wind_dot_bearing <= 0.0) {
                airspeed_ref = wind_speed;
            }
            else {
                const double airsp_dot_bearing = 0.0; // right angle to the bearing line gives minimum airspeed usage, zero forward ground speed
                airspeed_ref = abs(wind_cross_bearing);
            }
        }
        else {
            // infeasible
            airspeed_ref = (en_wind_excess_regulation_) ? airspeed_max_ : airspeed_nom_;
        }
    }

    return airspeed_ref;
} // calcRefAirspeed

bool NPFG::backwardsSolutionOK(const double wind_speed, const double airspeed, const double min_ground_speed, const double track_error)
{
    if (en_backwards_solution_ && airspeed < wind_speed) {
        // must satisfy:
        // 1) we are within the turn distance
        bool cond1 = wind_speed * M_PI / nom_heading_rate_ > track_error;
        // 2) the backwards solution satisfies the minimum ground speed at ALL bearings
        //    worst case: wind_dot_bearing = wind_speed; airsp_dot_bearing = -airspeed; compare to min_ground_speed_e_max_
        //    ^if allowing airspeed or min ground speed to vary, could cause oscillations in roll ref. if wind_speed is not stably in excess, similar problem
//        bool cond2 = wind_speed - airspeed >= std::max(min_ground_speed_g_, min_ground_speed_e_max_);
        // NOTE: since condition 2 is disabled, min ground sp may not always be maintained
        return cond1; // && cond2;
    }
    else {
        return false;
    }
} // backwardsSolutionOK

double NPFG::calcLateralAccelForCurvature(const Eigen::Vector2d& unit_path_tangent, const Eigen::Vector2d& ground_vel, const Eigen::Vector2d& wind_vel,
    const double airspeed, const double wind_speed, const double wind_ratio, const double signed_track_error, const double track_proximity)
{
    double wind_cross_upt, wind_dot_upt;
    trigWindToBearing(wind_cross_upt, wind_dot_upt, wind_vel, unit_path_tangent);

    feas0_ = calcBearingFeas(wind_cross_upt, wind_dot_upt, wind_speed, wind_ratio);

    double pc_ste = path_curvature_ * signed_track_error;

    double path_frame_rate = path_curvature_ * std::max(ground_vel.dot(unit_path_tangent), 0.0) / (1 - ((pc_ste < 1 - EPSILON) ? pc_ste : 1 - EPSILON));

    double speed_ratio = (1 + wind_dot_upt / projectAirspOnBearing(airspeed, wind_cross_upt));

    return airspeed * track_proximity * feas0_ * feas_ * speed_ratio * path_frame_rate;
} // calcLateralAccelForCurvature

double NPFG::calcLateralAccel(const Eigen::Vector2d &air_vel, const Eigen::Vector2d &air_vel_ref, const double airspeed)
{
    const double dot_air_vel_err = air_vel(0) * air_vel_ref(0) + air_vel(1) * air_vel_ref(1);
    const double cross_air_vel_err = air_vel(0) * air_vel_ref(1) - air_vel(1) * air_vel_ref(0);
    if (dot_air_vel_err < 0.0) {
        return p_gain_ * ((cross_air_vel_err < 0.0) ? -airspeed * airspeed : airspeed * airspeed);
    }
    else {
        return p_gain_ * cross_air_vel_err;
    }
} // calcLateralAccel

void NPFG::evaluate(const Eigen::Vector2d &aircraft_pos, const Eigen::Vector2d &ground_vel, const Eigen::Vector2d &wind_vel,
    const Eigen::Vector2d& closest_point_on_path, const Eigen::Vector2d& unit_path_tangent, const double& signed_track_error)
{
    const Eigen::Vector2d track_error_vec = closest_point_on_path - aircraft_pos;
    const double track_error = abs(signed_track_error);
    const double ground_speed = ground_vel.norm();

    Eigen::Vector2d air_vel = ground_vel - wind_vel;
    const double airspeed = std::max(air_vel.norm(), 0.1);//MIN_AIRSPEED);

    const double wind_speed = wind_vel.norm();
    const double wind_ratio = wind_speed / airspeed;

    // calculate gains considering upper and lower bounds (if enabled)
    calcGains(ground_speed, airspeed, wind_ratio, track_error, wind_vel, unit_path_tangent);

    Eigen::Vector2d unit_track_error;
    if (track_error < EPSILON) {
        unit_track_error = track_error_vec;
    }
    else {
        unit_track_error = track_error_vec / track_error;
    }

    track_error_bound_ = calcTrackErrorBound(ground_speed);

    const double normalized_track_error = constrain(track_error / track_error_bound_, 0.0, 1.0);

    const double look_ahead_angle = calcLookAheadAngle(normalized_track_error);

    double track_proximity, inv_track_proximity;
    bearing_vec_ = calcBearingVec(track_proximity, inv_track_proximity, unit_track_error, unit_path_tangent, look_ahead_angle);

    double wind_cross_bearing, wind_dot_bearing;
    trigWindToBearing(wind_cross_bearing, wind_dot_bearing, wind_vel, bearing_vec_);

    feas_ = calcBearingFeas(wind_cross_bearing, wind_dot_bearing, wind_speed, wind_ratio);

    // adjust along-bearing minimum ground speed for constant set point and/or track error
    double min_ground_speed = calcMinGroundSpeed(normalized_track_error);

    bool use_backwards_solution = backwardsSolutionOK(wind_speed, airspeed, min_ground_speed, track_error);

    Eigen::Vector2d air_vel_ref;
    if (feed_forward_air_vel_) {
        // check backwards solution for *nominal airspeed
        bool use_backwards_solution_nom = backwardsSolutionOK(wind_speed, airspeed_nom_, min_ground_speed, track_error);
        // get the heading reference assuming any airspeed incrementing is instantaneously achieved
        air_vel_ref = calcRefAirVelocityFF(wind_vel, bearing_vec_, wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed, use_backwards_solution_nom);
        airspeed_ref_ = air_vel_ref.norm();
        // re-scale with current airspeed for following feedback logic
        air_vel_ref = air_vel_ref * airspeed / airspeed_ref_;
    }
    else {
        air_vel_ref = calcRefAirVelocityFB(wind_vel, bearing_vec_, wind_cross_bearing, wind_dot_bearing, wind_speed, airspeed, use_backwards_solution);
        airspeed_ref_ = calcRefAirspeed(wind_vel, bearing_vec_, wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed);
    }

    lateral_accel_curv_ = calcLateralAccelForCurvature(unit_path_tangent, ground_vel, wind_vel, airspeed, wind_speed, wind_ratio, signed_track_error, track_proximity);
    lateral_accel_ = calcLateralAccel(air_vel, air_vel_ref, airspeed) + lateral_accel_curv_;
} // evaluate