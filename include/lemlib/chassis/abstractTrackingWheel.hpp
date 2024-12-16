#pragma once

namespace lemlib {

class AbstractTrackingWheel {
    public:
        virtual ~AbstractTrackingWheel() {}

        /**
         * @brief Reset the tracking wheel position to 0
         *
         * If you are using odometry provided by LemLib, this will automatically be called when
         * the chassis is calibrated
         *
         * @b Example
         * @code {.cpp}
         * void initialize() {
         *     exampleTrackingWheel.reset();
         *     // this will now return 0 inches traveled
         *     exampleTrackingWheel.getDistanceTraveled();
         * }
         * @endcode
         */
        virtual void reset() = 0;
        /**
         * @brief Get the distance traveled by the tracking wheel
         *
         * @return float distance traveled in inches
         *
         * @b Example
         * @code {.cpp}
         * void initialize() {
         *     while (true) {
         *         // print the distance traveled by the tracking wheel to the terminal
         *         std::cout << "distance: " << exampleTrackingWheel.getDistanceTraveled() << std::endl;
         *         pros::delay(10);
         *     }
         * }
         */
        virtual float getDistanceTraveled() = 0;
        /**
         * @brief Get the offset of the tracking wheel from the center of rotation
         *
         * @return float offset in inches
         *
         * @b Example
         * @code {.cpp}
         * void initialize() {
         *     // create a tracking wheel with an offset of 0.5 inches
         *     lemlib::AbstractTrackingWheel exampleTrackingWheel(&exampleEncoder, lemlib::Omniwheel::NEW_275, 0.5);
         *     // this prints 0.5 to the terminal, the offset of the tracking wheel
         *     std::cout << "offset: " << exampleTrackingWheel.getOffset() << std::endl;
         * }
         * @endcode
         */
        virtual float getOffset() = 0;
};
} // namespace lemlib