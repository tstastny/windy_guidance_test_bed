#include "include/path_type.h"
#include <gtest/gtest.h>
#include <string>

// TODO: clean this up.. lots of repeated lines for e.g. setup and allocation across individual tests

/*
    Line Path Tests
*/

TEST(LineTest, Constructor1) {

    /*
        test nominal inputs to Line class
        default constructor, setters
    */

    // default constructor
    path_type::Line line_;

    EXPECT_TRUE(path_type::DEFAULT_POS.isApprox(line_.pos()));
    EXPECT_TRUE(path_type::DEFAULT_UPT.isApprox(line_.upt()));

    // arbitrary vehicle pos/vel
    Eigen::Vector2d veh_pos(5.0, 5.0);
    Eigen::Vector2d veh_vel(10.0, 0.0);

    // dummy outputs
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;

    line_.computeGuidanceInputs(veh_pos, veh_vel,
        closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    // calculations for expected output
    Eigen::Vector2d rel_pos = veh_pos - path_type::DEFAULT_POS;
    Eigen::Vector2d unit_path_normal(-path_type::DEFAULT_UPT(1), path_type::DEFAULT_UPT(0)); // 90 deg clockwise rotation
    Eigen::Vector2d expected_closest_pt = path_type::DEFAULT_POS + path_type::DEFAULT_UPT.dot(rel_pos) * path_type::DEFAULT_UPT;

    // TEST outputs
    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE((path_type::DEFAULT_UPT).isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(rel_pos.dot(unit_path_normal), track_error);
    EXPECT_DOUBLE_EQ(0.0, path_curvature);

    // set new path params / modify vehicle pos/vel
    Eigen::Vector2d path_pos = {-15.0,10.0};
    Eigen::Vector2d upt = -veh_pos.normalized();
    veh_pos += path_pos;
    line_.setPosition(path_pos);
    line_.setUnitTangent(upt);

    line_.computeGuidanceInputs(veh_pos, veh_vel,
        closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    // calculations for expected output
    rel_pos = veh_pos - path_pos;
    unit_path_normal = Eigen::Vector2d(-upt(1), upt(0)); // 90 deg clockwise rotation
    expected_closest_pt = path_pos + upt.dot(rel_pos) * upt;

    // TEST outputs
    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(rel_pos.dot(unit_path_normal), track_error);
    EXPECT_DOUBLE_EQ(0.0, path_curvature);
}

TEST(LineTest, Constructor2) {

    /*
        test nominal inputs to Line class
        argin constructor, setters
    */

    // arbitrary vehicle pos/vel
    Eigen::Vector2d veh_pos(5.0-15.0, 5.0+10.0);
    Eigen::Vector2d veh_vel(10.0, 0.0);

    // dummy outputs
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;

    // input path params in constructor
    Eigen::Vector2d path_pos = {-15.0, 10.0};
    Eigen::Vector2d upt = -Eigen::Vector2d(1, 1).normalized();
    path_type::Line line_(path_pos, upt);

    EXPECT_TRUE(path_pos.isApprox(line_.pos()));
    EXPECT_TRUE(upt.isApprox(line_.upt()));

    line_.computeGuidanceInputs(veh_pos, veh_vel,
        closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    // calculations for expected output
    const Eigen::Vector2d rel_pos = veh_pos - path_pos;
    const Eigen::Vector2d unit_path_normal(-upt(1), upt(0)); // 90 deg clockwise rotation
    const Eigen::Vector2d expected_closest_pt = path_pos + upt.dot(rel_pos) * upt;

    // TEST outputs
    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(rel_pos.dot(unit_path_normal), track_error);
    EXPECT_DOUBLE_EQ(0.0, path_curvature);
}

TEST(LineTest, PathTangentNotUnit) {

    /*
        tests for a non-unit, non-zero path tangent vector input
    */

    // arbitrary vehicle pos/vel
    Eigen::Vector2d veh_pos(5.0, 5.0);
    Eigen::Vector2d veh_vel(10.0, 0.0);

    // dummy outputs
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;

    // set path params in constructor
    Eigen::Vector2d path_pos(-15.0, 10.0);
    Eigen::Vector2d upt = Eigen::Vector2d(-path_type::UNIT_VEC_THRES, -path_type::UNIT_VEC_THRES) * 2.0; // something over the threshold
    path_type::Line line_(path_pos, upt);

    line_.computeGuidanceInputs(veh_pos, veh_vel,
        closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    // calculations for expected output
    Eigen::Vector2d rel_pos = veh_pos - path_pos;
    upt.normalize(); // non unit path tangent should get normalized when input to the line object
    Eigen::Vector2d unit_path_normal(-upt(1), upt(0)); // 90 deg clockwise rotation
    Eigen::Vector2d expected_closest_pt = path_pos + upt.dot(rel_pos) * upt;

    // TEST outputs
    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(rel_pos.dot(unit_path_normal), track_error);
    EXPECT_DOUBLE_EQ(0.0, path_curvature);

    // reset path params, rerun after using setters XXX: duplicate code
    upt(0) = -path_type::UNIT_VEC_THRES * 2.0;
    upt(1) = -path_type::UNIT_VEC_THRES * 2.0;
    path_type::Line line2_;
    line2_.setPosition(path_pos);
    line2_.setUnitTangent(upt);

    line2_.computeGuidanceInputs(veh_pos, veh_vel,
        closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    // calculations for expected output
    rel_pos = veh_pos - path_pos;
    upt.normalize(); // non unit path tangent should get normalized when input to the line object
    unit_path_normal(-upt(1), upt(0)); // 90 deg clockwise rotation
    expected_closest_pt = path_pos + upt.dot(rel_pos) * upt;

    // TEST outputs
    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(rel_pos.dot(unit_path_normal), track_error);
    EXPECT_DOUBLE_EQ(0.0, path_curvature);
}

TEST(LineTest, PathTangentZero) {

    /*
        tests for a zero path tangent vector input (below threshold)
    */

    // set arbitrary vehicle pos/vel
    Eigen::Vector2d veh_pos(5.0, 5.0);
    Eigen::Vector2d veh_vel(10.0, 0.0);

    // dummy outputs
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;

    // set path params in constructor
    Eigen::Vector2d path_pos(-15.0, 10.0);
    Eigen::Vector2d upt(0.0, 0.0); // something below the threshold
    path_type::Line line_(path_pos, upt);

    line_.computeGuidanceInputs(veh_pos, veh_vel,
        closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    // calculations for expected output
    Eigen::Vector2d rel_pos = veh_pos - path_pos;
    Eigen::Vector2d unit_path_normal(-path_type::DEFAULT_UPT(1), path_type::DEFAULT_UPT(0)); // 90 deg clockwise rotation
    Eigen::Vector2d expected_closest_pt = path_pos + (path_type::DEFAULT_UPT).dot(rel_pos) * path_type::DEFAULT_UPT;

    // TEST outputs
    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE((path_type::DEFAULT_UPT).isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(rel_pos.dot(unit_path_normal), track_error);
    EXPECT_DOUBLE_EQ(0.0, path_curvature);

    // run additional case using setters
    path_type::Line line2_;
    line2_.setPosition(path_pos);
    line2_.setUnitTangent(upt);

    line2_.computeGuidanceInputs(veh_pos, veh_vel,
        closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    // TEST outputs
    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE((path_type::DEFAULT_UPT).isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(rel_pos.dot(unit_path_normal), track_error);
    EXPECT_DOUBLE_EQ(0.0, path_curvature);
}

/*
    Loiter Path Tests
*/

TEST(LoiterTest, DefaultConstructor) {

    path_type::Loiter loit_;

    EXPECT_TRUE(path_type::DEFAULT_POS.isApprox(loit_.pos()));
    EXPECT_DOUBLE_EQ(path_type::MIN_LOITER_RADIUS, loit_.radius());
    EXPECT_DOUBLE_EQ(1.0, loit_.dir());

    // calculate expected outputs
    const Eigen::Vector2d veh_pos(100.0, 100.0);
    const Eigen::Vector2d veh_vel(0.0, 15.0);
    Eigen::Vector2d unit_vec_to_closest_pt = (veh_pos - path_type::DEFAULT_POS).normalized();
    Eigen::Vector2d expected_closest_pt = unit_vec_to_closest_pt * path_type::MIN_LOITER_RADIUS + path_type::DEFAULT_POS;
    Eigen::Vector2d expected_upt = 1 * Eigen::Vector2d(-unit_vec_to_closest_pt(1), unit_vec_to_closest_pt(0));
    Eigen::Vector2d unit_path_normal(-expected_upt(1), expected_upt(0)); // 90 deg clockwise rotation
    double expected_track_error = (veh_pos - expected_closest_pt).dot(unit_path_normal);

    // function outputs
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;
    loit_.computeGuidanceInputs(veh_pos, veh_vel, closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(expected_upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(expected_track_error, track_error);
    EXPECT_DOUBLE_EQ(1.0 / path_type::MIN_LOITER_RADIUS, path_curvature);
}

TEST(LoiterTest, Constructor2) {

    Eigen::Vector2d path_pos(20.0,20.0);
    double radius = 100.0;
    int dir = -1;
    path_type::Loiter loit_(path_pos, radius, dir);

    EXPECT_TRUE(path_pos.isApprox(loit_.pos()));
    EXPECT_DOUBLE_EQ(radius, loit_.radius());
    EXPECT_DOUBLE_EQ(double(dir), loit_.dir());

    // calculate expected outputs
    const Eigen::Vector2d veh_pos(100.0, 100.0);
    const Eigen::Vector2d veh_vel(0.0, 15.0);
    Eigen::Vector2d unit_vec_to_closest_pt = (veh_pos - path_pos).normalized();
    Eigen::Vector2d expected_closest_pt = unit_vec_to_closest_pt * radius + path_pos;
    Eigen::Vector2d expected_upt = dir * Eigen::Vector2d(-unit_vec_to_closest_pt(1), unit_vec_to_closest_pt(0));
    Eigen::Vector2d unit_path_normal(-expected_upt(1), expected_upt(0)); // 90 deg clockwise rotation
    double expected_track_error = (veh_pos - expected_closest_pt).dot(unit_path_normal);

    // function outputs
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;
    loit_.computeGuidanceInputs(veh_pos, veh_vel, closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(expected_upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(expected_track_error, track_error);
    EXPECT_DOUBLE_EQ(1.0 / radius, path_curvature);
}

TEST(LoiterTest, Setters) {

    path_type::Loiter loit_;

    // set members
    Eigen::Vector2d path_pos(20.0,20.0);
    double radius = 200.0;
    int dir = -1;
    loit_.setPosition(path_pos);
    loit_.setRadius(radius);
    loit_.setLoiterDir(dir);

    // calculate expected outputs

    const Eigen::Vector2d veh_pos(100.0, 100.0);
    const Eigen::Vector2d veh_vel(0.0, 15.0);
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;

    Eigen::Vector2d unit_vec_to_closest_pt = (veh_pos - path_pos).normalized();
    Eigen::Vector2d expected_closest_pt = unit_vec_to_closest_pt * radius + path_pos;
    Eigen::Vector2d expected_upt = dir * Eigen::Vector2d(-unit_vec_to_closest_pt(1), unit_vec_to_closest_pt(0));
    Eigen::Vector2d unit_path_normal(-expected_upt(1), expected_upt(0)); // 90 deg clockwise rotation
    double expected_track_error = (veh_pos - expected_closest_pt).dot(unit_path_normal);

    // function outputs
    loit_.computeGuidanceInputs(veh_pos, veh_vel, closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(expected_upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(expected_track_error, track_error);
    EXPECT_DOUBLE_EQ(1.0 / radius, path_curvature);
}

TEST(LoiterTest, SettersInvalid1) {

    path_type::Loiter loit_;

    // set members
    Eigen::Vector2d path_pos(20.0,20.0);
    loit_.setPosition(path_pos);
    loit_.setRadius(0.0);
    loit_.setLoiterDir(0);

    double radius = path_type::MIN_LOITER_RADIUS;
    int dir = 1;

    // calculate expected outputs

    const Eigen::Vector2d veh_pos(100.0, 100.0);
    const Eigen::Vector2d veh_vel(0.0, 15.0);
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;

    Eigen::Vector2d unit_vec_to_closest_pt = (veh_pos - path_pos).normalized();
    Eigen::Vector2d expected_closest_pt = unit_vec_to_closest_pt * radius + path_pos;
    Eigen::Vector2d expected_upt = dir * Eigen::Vector2d(-unit_vec_to_closest_pt(1), unit_vec_to_closest_pt(0));
    Eigen::Vector2d unit_path_normal(-expected_upt(1), expected_upt(0)); // 90 deg clockwise rotation
    double expected_track_error = (veh_pos - expected_closest_pt).dot(unit_path_normal);

    // function outputs
    loit_.computeGuidanceInputs(veh_pos, veh_vel, closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(expected_upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(expected_track_error, track_error);
    EXPECT_DOUBLE_EQ(1.0 / radius, path_curvature);
}

TEST(LoiterTest, SettersInvalid2) {

    path_type::Loiter loit_;

    // set members
    Eigen::Vector2d path_pos(20.0,20.0);
    double radius = 150.0;
    loit_.setPosition(path_pos);
    loit_.setRadius(radius);
    loit_.setLoiterDir(-30);

    int dir = -1;

    // calculate expected outputs

    const Eigen::Vector2d veh_pos(100.0, 100.0);
    const Eigen::Vector2d veh_vel(0.0, 15.0);
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;

    Eigen::Vector2d unit_vec_to_closest_pt = (veh_pos - path_pos).normalized();
    Eigen::Vector2d expected_closest_pt = unit_vec_to_closest_pt * radius + path_pos;
    Eigen::Vector2d expected_upt = dir * Eigen::Vector2d(-unit_vec_to_closest_pt(1), unit_vec_to_closest_pt(0));
    Eigen::Vector2d unit_path_normal(-expected_upt(1), expected_upt(0)); // 90 deg clockwise rotation
    double expected_track_error = (veh_pos - expected_closest_pt).dot(unit_path_normal);

    // function outputs
    loit_.computeGuidanceInputs(veh_pos, veh_vel, closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(expected_upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(expected_track_error, track_error);
    EXPECT_DOUBLE_EQ(1.0 / radius, path_curvature);
}

TEST(LoiterTest, InCircleCenter1) {

    /*
        Test vehicle position in circle center with nominal speed
        (uses velocity vector to determine the orientation of the closest point on circle)
    */

    Eigen::Vector2d path_pos(0.0,0.0);
    double radius = 100.0;
    int dir = 1;
    path_type::Loiter loit_(path_pos, radius, dir);

    // calculate expected outputs
    const Eigen::Vector2d veh_pos(path_type::MIN_DIST_TO_CENTER/2.0, 0.0); // vehicle position violating min dist to center
    const Eigen::Vector2d veh_vel(0.0, 15.0);

    Eigen::Vector2d unit_vec_to_closest_pt = veh_vel.normalized();
    Eigen::Vector2d expected_closest_pt = unit_vec_to_closest_pt * radius + path_pos;
    Eigen::Vector2d expected_upt = dir * Eigen::Vector2d(-unit_vec_to_closest_pt(1), unit_vec_to_closest_pt(0));
    Eigen::Vector2d unit_path_normal(-expected_upt(1), expected_upt(0)); // 90 deg clockwise rotation
    double expected_track_error = (veh_pos - expected_closest_pt).dot(unit_path_normal);

    // function outputs
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;
    loit_.computeGuidanceInputs(veh_pos, veh_vel, closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(expected_upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(expected_track_error, track_error);
    EXPECT_DOUBLE_EQ(1.0 / radius, path_curvature);
}

TEST(LoiterTest, InCircleCenter2) {

    /*
        Test vehicle position in circle center with low speed
        (closest point placed at northern most point on circle)
    */

    Eigen::Vector2d path_pos(0.0,0.0);
    double radius = 100.0;
    int dir = 1;
    path_type::Loiter loit_(path_pos, radius, dir);

    // calculate expected outputs
    const Eigen::Vector2d veh_pos(path_type::MIN_DIST_TO_CENTER/2.0, 0.0); // vehicle position violating min dist to center
    const Eigen::Vector2d veh_vel(0.0, path_type::MIN_SPEED/2.0);

    Eigen::Vector2d unit_vec_to_closest_pt(1.0, 0.0); // north
    Eigen::Vector2d expected_closest_pt = unit_vec_to_closest_pt * radius + path_pos;
    Eigen::Vector2d expected_upt = dir * Eigen::Vector2d(-unit_vec_to_closest_pt(1), unit_vec_to_closest_pt(0));
    Eigen::Vector2d unit_path_normal(-expected_upt(1), expected_upt(0)); // 90 deg clockwise rotation
    double expected_track_error = (veh_pos - expected_closest_pt).dot(unit_path_normal);

    // function outputs
    Eigen::Vector2d closest_point_on_path;
    Eigen::Vector2d unit_path_tangent;
    double track_error;
    double path_curvature;
    loit_.computeGuidanceInputs(veh_pos, veh_vel, closest_point_on_path, unit_path_tangent, track_error, path_curvature);

    EXPECT_TRUE(expected_closest_pt.isApprox(closest_point_on_path));
    EXPECT_TRUE(expected_upt.isApprox(unit_path_tangent));
    EXPECT_DOUBLE_EQ(expected_track_error, track_error);
    EXPECT_DOUBLE_EQ(1.0 / radius, path_curvature);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}