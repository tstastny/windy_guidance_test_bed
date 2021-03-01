#ifndef HELPERS_H_
#define HELPERS_H_

#include <Eigen/Eigen>

static const double DEG_TO_RAD = 0.017453292519943;    // degrees to radians XXX: is there really no other function already built in for this?

/*
    some helpful common functions
*/

int constrain(const int x, const int xmin, const int xmax);
double constrain(const double x, const double xmin, const double xmax);
double cross2d(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);
Eigen::Vector2d rotate2d(const Eigen::Vector2d &vec, const double angle);
Eigen::Vector2d rotate2d(const double sinx, const double cosx, const Eigen::Vector2d &vec);

#endif // HELPERS_H_
