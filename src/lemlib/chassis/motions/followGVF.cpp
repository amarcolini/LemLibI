#include <cmath>
#include <optional>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/geometry/path.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"
#include "lemlib/gvf/gvf.hpp"

void lemlib::Chassis::followGVF(std::function<void(Pose)> setPower, lemlib::GVFFollower follower, lemlib::Path path, int timeout, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { followGVF(setPower, follower, path, timeout, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    follower.followPath(path);
    follower.init();

    // initialize vars used between iterations
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();

    // main loop
    while (!timer.isDone() && (!follower.isFollowing()) &&
           this->motionRunning) {
        // update position
        const Pose pose = getPose(true, true);

        // update distance traveled
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        // calculate distance to the target point
        auto output = follower.update(pose, std::nullopt);
        setPower(output);
        // delay to save resources
        pros::delay(10);
    }

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}