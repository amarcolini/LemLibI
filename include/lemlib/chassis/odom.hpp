#pragma once
#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"

namespace lemlib {
class Odometry {
    public:
        /**
         * @brief Get the pose of the robot
         *
         * @param radians true for theta in radians, false for degrees. False by default
         * @return Pose
         */
        Pose getPose(bool radians = false);
        /**
         * @brief Set the Pose of the robot
         *
         * @param pose the new pose
         * @param radians true if theta is in radians, false if in degrees. False by default
         */
        void setPose(Pose pose, bool radians = false);
        /**
         * @brief Get the speed of the robot
         *
         * @param radians true for theta in radians, false for degrees. False by default
         * @return lemlib::Pose
         */
        Pose getSpeed(bool radians = false);
        /**
         * @brief Get the local speed of the robot
         *
         * @param radians true for theta in radians, false for degrees. False by default
         * @return lemlib::Pose
         */
        Pose getLocalSpeed(bool radians = false);
        // /**
        //  * @brief Estimate the pose of the robot after a certain amount of time
        //  *
        //  * @param time time in seconds
        //  * @param radians False for degrees, true for radians. False by default
        //  * @return lemlib::Pose
        //  */
        // Pose estimatePose(float time, bool radians = false);

        void initTask();

        /**
         * @brief Update the pose of the robot
         *
         */
        virtual void update() = 0;
        /**
         * @brief Initialize the odometry system
         *
         */
        virtual void calibrate(bool calibrateIMU = true) = 0;
    protected:
        virtual Pose _getPose() = 0;
        virtual void _setPose(Pose pose) = 0;
        virtual Pose _getSpeed() = 0;
        virtual Pose _getLocalSpeed() = 0;
    private:
        pros::Task* trackingTask = nullptr;
};
} // namespace lemlib