#include "include/helpers.h"
#include <math.h>

int constrain(const int x, const int xmin, const int xmax)
{
    return (x < xmin) ? xmin : ((x > xmax) ? xmax : x);
} // constrain

double constrain(const double x, const double xmin, const double xmax)
{
    return (x < xmin) ? xmin : ((x > xmax) ? xmax : x);
} // constrain

double cross2d(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
{
    return v1(0) * v2(1) - v1(1) * v2(0);
} // cross2d

Eigen::Vector2d rotate2d(const Eigen::Vector2d &vec, const double angle)
{
    /*
        NOTE: SHOULD be able to use --
        Eigen::Rotation2Dd r(angle);
        Eigen::Vector2d vec2 = r * vec;
        -- but for some reason it doesn't compile.
    */
    const double cos_angle = cos(angle);
    const double sin_angle = sin(angle);
    return Eigen::Vector2d(vec(0) * cos_angle + vec(1) * -sin_angle, vec(0) * sin_angle + vec(1) * cos_angle);
} // rotate2d

Eigen::Vector2d rotate2d(const double sinx, const double cosx, const Eigen::Vector2d &vec)
{
    /*
        NOTE: SHOULD be able to use --
        Eigen::Rotation2Dd r(angle);
        Eigen::Vector2d vec2 = r * vec;
        -- but for some reason it doesn't compile.
    */
    Eigen::Matrix2d rot;
    rot << cosx, -sinx, sinx, cosx;
    return rot * vec;
} // rotate2d