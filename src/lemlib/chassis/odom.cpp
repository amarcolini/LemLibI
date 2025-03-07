#include "odom.hpp"
#include "lemlib/util.hpp"


// void lemlib::setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain) {
//     odomSensors = sensors;
//     drive = drivetrain;
// }

lemlib::Pose lemlib::Odometry::getPose(bool radians) {
    Pose odomPose = _getPose();
    if (radians) return odomPose;
    else return lemlib::Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}

void lemlib::Odometry::setPose(lemlib::Pose pose, bool radians) {
    if (radians) _setPose(pose);
    else _setPose(lemlib::Pose(pose.x, pose.y, degToRad(pose.theta)));
}

lemlib::Pose lemlib::Odometry::getSpeed(bool radians) {
    Pose odomSpeed = _getSpeed();
    if (radians) return odomSpeed;
    else return lemlib::Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

lemlib::Pose lemlib::Odometry::getLocalSpeed(bool radians) {
    Pose odomLocalSpeed = _getLocalSpeed();
    if (radians) return odomLocalSpeed;
    else return lemlib::Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.theta));
}

// lemlib::Pose lemlib::estimatePose(float time, bool radians) {
//     // get current position and speed
//     Pose curPose = getPose(true);
//     Pose localSpeed = getLocalSpeed(true);
//     // calculate the change in local position
//     Pose deltaLocalPose = localSpeed * time;

//     // calculate the future pose
//     float avgHeading = curPose.theta + deltaLocalPose.theta / 2;
//     Pose futurePose = curPose;
//     futurePose.x += deltaLocalPose.y * sin(avgHeading);
//     futurePose.y += deltaLocalPose.y * cos(avgHeading);
//     futurePose.x += deltaLocalPose.x * -cos(avgHeading);
//     futurePose.y += deltaLocalPose.x * sin(avgHeading);
//     if (!radians) futurePose.theta = radToDeg(futurePose.theta);

//     return futurePose;
// }

void lemlib::Odometry::initTask() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task {[=, this] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}


// void lemlib::init() {
//     if (trackingTask == nullptr) {
//         trackingTask = new pros::Task {[=] {
//             while (true) {
//                 update();
//                 pros::delay(10);
//             }
//         }};
//     }
// }