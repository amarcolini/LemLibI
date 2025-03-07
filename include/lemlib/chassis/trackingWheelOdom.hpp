#pragma once

#include "odom.hpp"
#include "pros/imu.hpp"
#include "lemlib/chassis/abstractTrackingWheel.hpp"

namespace lemlib {

/**
 * @brief class containing the sensors used for odometry
 */
class TrackingWheelOdom : public Odometry {
    public:
        enum class HeadingSource {
            Vertical,
            Horizontal,
            IMU,
        };
        /**
         * The sensors are stored in a class so that they can be easily passed to the chassis class
         * The variables are pointers so that they can be set to nullptr if they are not used
         * Otherwise the chassis class would have to have a constructor for each possible combination of sensors
         *
         * @param vertical1 pointer to the first vertical tracking wheel
         * @param vertical2 pointer to the second vertical tracking wheel
         * @param horizontal1 pointer to the first horizontal tracking wheel
         * @param horizontal2 pointer to the second horizontal tracking wheel
         * @param imu pointer to the IMU
         *
         * @b Example
         * @code {.cpp}
         * pros::Rotation vertical_rotation(1); // rotation sensor on port 1
         * pros::Imu imu(2); // IMU on port 2
         * // tracking wheel using a new 2.75" wheel, 0.5 inches to the right of the tracking center
         * lemlib::TrackingWheel vertical1(&vertical_rotation, lemlib::Omniwheel::NEW_275, 0.5);
         * lemlib::OdomSensors sensors(&vertical1, // vertical tracking wheel
         *                     nullptr, // no second vertical tracking wheel, set to nullptr
         *                     nullptr, // no horizontal tracking wheels, set to nullptr
         *                     nullptr, // no second horizontal tracking wheel, set to nullptr
         *                     &imu); // IMU
         * @endcode
         */
        TrackingWheelOdom(AbstractTrackingWheel* vertical1, AbstractTrackingWheel* vertical2,
                          AbstractTrackingWheel* horizontal1, AbstractTrackingWheel* horizontal2, pros::Imu* imu,
                          HeadingSource headingSource);

        void calibrateIMU();

        AbstractTrackingWheel* vertical1;
        AbstractTrackingWheel* vertical2;
        AbstractTrackingWheel* horizontal1;
        AbstractTrackingWheel* horizontal2;
        pros::Imu* imu;
        HeadingSource headingSource;

        void update();
        void calibrate(bool calibrateIMU = true);
    protected:
        Pose _getPose();
        void _setPose(Pose pose);
        Pose _getSpeed();
        Pose _getLocalSpeed();
    private:
        lemlib::Pose odomPose; // the pose of the robot
        lemlib::Pose odomSpeed; // the speed of the robot
        lemlib::Pose odomLocalSpeed; // the local speed of the robot

        float prevVertical = 0;
        float prevVertical1 = 0;
        float prevVertical2 = 0;
        float prevHorizontal = 0;
        float prevHorizontal1 = 0;
        float prevHorizontal2 = 0;
        float prevImu = 0;
};

} // namespace lemlib