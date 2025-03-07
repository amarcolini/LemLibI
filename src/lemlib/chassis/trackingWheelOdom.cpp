// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <math.h>
#include "lemlib/logger/logger.hpp"
#include "pros/rtos.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/trackingWheelOdom.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/chassis/abstractTrackingWheel.hpp"

void lemlib::TrackingWheelOdom::_setPose(Pose pose) {
    odomPose = pose;
}

lemlib::Pose lemlib::TrackingWheelOdom::_getPose() { return odomPose; }

lemlib::Pose lemlib::TrackingWheelOdom::_getSpeed() { return odomSpeed; }

lemlib::Pose lemlib::TrackingWheelOdom::_getLocalSpeed() { return odomLocalSpeed; }

lemlib::TrackingWheelOdom::TrackingWheelOdom(AbstractTrackingWheel* vertical1, AbstractTrackingWheel* vertical2,
                                             AbstractTrackingWheel* horizontal1, AbstractTrackingWheel* horizontal2,
                                             pros::Imu* imu, HeadingSource headingSource)
    : vertical1(vertical1),
      vertical2(vertical2),
      horizontal1(horizontal1),
      horizontal2(horizontal2),
      imu(imu),
      headingSource(headingSource),
      odomPose({0.0, 0.0, 0.0}),
      odomSpeed({0.0, 0.0, 0.0}),
      odomLocalSpeed({0.0, 0.0, 0.0}) {}

void lemlib::TrackingWheelOdom::calibrate(bool calibrateIMU) {
    // calibrate the IMU if it exists and the user doesn't specify otherwise
    if (imu != nullptr && calibrateIMU) this->calibrateIMU();
    // initialize odom
    // if (vertical1 == nullptr && vertical2 == nullptr) {
    //     vertical1 = new lemlib::TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter,
    //                                                   -(drivetrain.trackWidth / 2), drivetrain.rpm);
    //     vertical2 = new lemlib::TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter,
    //                                                   drivetrain.trackWidth / 2, drivetrain.rpm);
    // }
    if (vertical1 != nullptr) vertical1->reset();
    if (vertical2 != nullptr) vertical2->reset();
    if (horizontal1 != nullptr) horizontal1->reset();
    if (horizontal2 != nullptr) horizontal2->reset();
}

/**
 * @brief calibrate the IMU given a sensors struct
 *
 * @param sensors reference to the sensors struct
 */
void lemlib::TrackingWheelOdom::calibrateIMU() {
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    while (attempt <= 5) {
        imu->reset();
        // wait until IMU is calibrated
        do pros::delay(10);
        while (imu->get_status() != pros::ImuStatus::error && imu->is_calibrating());
        // exit if imu has been calibrated
        if (!isnanf(imu->get_heading()) && !isinf(imu->get_heading())) {
            calibrated = true;
            break;
        }
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        lemlib::infoSink()->warn("IMU failed to calibrate! Attempt #{}", attempt);
        attempt++;
    }
    // check if calibration attempts were successful
    if (attempt > 5) {
        imu = nullptr;
        lemlib::infoSink()->error("IMU calibration failed, defaulting to tracking wheels / motor encoders");
    }
}

void lemlib::TrackingWheelOdom::update() {
    // TODO: add particle filter
    // get the current sensor values
    float vertical1Raw = 0;
    float vertical2Raw = 0;
    float horizontal1Raw = 0;
    float horizontal2Raw = 0;
    float imuRaw = 0;
    if (vertical1 != nullptr) vertical1Raw = vertical1->getDistanceTraveled();
    if (vertical2 != nullptr) vertical2Raw = vertical2->getDistanceTraveled();
    if (horizontal1 != nullptr) horizontal1Raw = horizontal1->getDistanceTraveled();
    if (horizontal2 != nullptr) horizontal2Raw = horizontal2->getDistanceTraveled();
    if (imu != nullptr) imuRaw = degToRad(imu->get_rotation());

    // calculate the change in sensor values
    float deltaVertical1 = vertical1Raw - prevVertical1;
    float deltaVertical2 = vertical2Raw - prevVertical2;
    float deltaHorizontal1 = horizontal1Raw - prevHorizontal1;
    float deltaHorizontal2 = horizontal2Raw - prevHorizontal2;
    float deltaImu = imuRaw - prevImu;

    // update the previous sensor values
    prevVertical1 = vertical1Raw;
    prevVertical2 = vertical2Raw;
    prevHorizontal1 = horizontal1Raw;
    prevHorizontal2 = horizontal2Raw;
    prevImu = imuRaw;

    // calculate the heading of the robot
    // Priority:
    // 1. Horizontal tracking wheels
    // 2. Vertical tracking wheels
    // 3. Inertial Sensor
    // 4. Drivetrain
    float heading = odomPose.theta;

    bool vertical1IsDrivetrain =
        (typeid(vertical1) == typeid(TrackingWheel)) ? (static_cast<TrackingWheel*>(vertical1))->getType() == 1 : false;
    bool vertical2IsDrivetrain =
        (typeid(vertical2) == typeid(TrackingWheel)) ? (static_cast<TrackingWheel*>(vertical2))->getType() == 1 : false;

    // calculate the heading using the horizontal tracking wheels
    if (headingSource == TrackingWheelOdom::HeadingSource::Horizontal && horizontal1 != nullptr &&
        horizontal2 != nullptr)
        heading -= (deltaHorizontal1 - deltaHorizontal2) / (horizontal1->getOffset() - horizontal2->getOffset());
    // else, if both vertical tracking wheels aren't substituted by the drivetrain, use the vertical tracking wheels
    else if (headingSource == TrackingWheelOdom::HeadingSource::Vertical && vertical1 != nullptr &&
             vertical2 != nullptr)
        heading -= (deltaVertical1 - deltaVertical2) / (vertical1->getOffset() - vertical2->getOffset());
    // else, if the inertial sensor exists, use it
    else if (headingSource == TrackingWheelOdom::HeadingSource::IMU && imu != nullptr) heading += deltaImu;

    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2;

    // choose tracking wheels to use
    // Prioritize non-powered tracking wheels
    lemlib::AbstractTrackingWheel* verticalWheel = nullptr;
    lemlib::AbstractTrackingWheel* horizontalWheel = nullptr;
    if (!vertical1IsDrivetrain) verticalWheel = vertical1;
    else if (!vertical2IsDrivetrain) verticalWheel = vertical2;
    else verticalWheel = vertical1;
    if (horizontal1 != nullptr) horizontalWheel = horizontal1;
    else if (horizontal2 != nullptr) horizontalWheel = horizontal2;
    float rawVertical = 0;
    float rawHorizontal = 0;
    if (verticalWheel != nullptr) rawVertical = verticalWheel->getDistanceTraveled();
    if (horizontalWheel != nullptr) rawHorizontal = horizontalWheel->getDistanceTraveled();
    float horizontalOffset = 0;
    float verticalOffset = 0;
    if (verticalWheel != nullptr) verticalOffset = verticalWheel->getOffset();
    if (horizontalWheel != nullptr) horizontalOffset = horizontalWheel->getOffset();

    // calculate change in x and y
    float deltaX = 0;
    float deltaY = 0;
    if (verticalWheel != nullptr) deltaY = rawVertical - prevVertical;
    if (horizontalWheel != nullptr) deltaX = rawHorizontal - prevHorizontal;
    prevVertical = rawVertical;
    prevHorizontal = rawHorizontal;

    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { // prevent divide by 0
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    // save previous pose
    lemlib::Pose prevPose = odomPose;

    // calculate global x and y
    odomPose.x += localY * sin(avgHeading);
    odomPose.y += localY * cos(avgHeading);
    odomPose.x += localX * -cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);
    odomPose.theta = heading;

    // calculate speed
    odomSpeed.x = ema((odomPose.x - prevPose.x) / 0.01, odomSpeed.x, 0.95);
    odomSpeed.y = ema((odomPose.y - prevPose.y) / 0.01, odomSpeed.y, 0.95);
    odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / 0.01, odomSpeed.theta, 0.95);

    // calculate local speed
    odomLocalSpeed.x = ema(localX / 0.01, odomLocalSpeed.x, 0.95);
    odomLocalSpeed.y = ema(localY / 0.01, odomLocalSpeed.y, 0.95);
    odomLocalSpeed.theta = ema(deltaHeading / 0.01, odomLocalSpeed.theta, 0.95);
}
