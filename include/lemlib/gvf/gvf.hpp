#pragma once
#include <algorithm>
#include <functional>
#include <cmath>
#include <optional>
#include "lemlib/geometry/path.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"

// static int sign(float x) { return (x > 0) - (x < 0); }

namespace lemlib {
class PathGVF {
    public:
        const Path& path;
        const float kN;

        PathGVF(
            const Path& path, const float kN,
            const std::function<float(float)> errorMapFunc = [](float x) { return x; });

        struct Phi {
                Vector target;
                Vector tangent;
                Vector pathToPoint;
                float orientation;
                float error;
                Vector normal;

                Phi(const Vector& point, const Vector& target, const Vector& tangent);
        };

        Vector compute(Phi phi);

        void reset();
        const std::function<float(float)> errorMapFunc;
        float lastProjectDisplacement = 0.0;

        Phi internalGet(const Vector& point);
};

class GVFFollower {
    public:
        float maxVel;
        float maxAccel;
        float maxDecel;
        float maxAngVel;
        float maxAngAccel;
        float kN;
        float kOmega;
        PID headingPID;
        Pose admissibleError;
        float correctionDistance = 5.0;
        bool useCurvatureControl = false;
        std::function<float(float)> errorMapFunc;

        GVFFollower(
            float maxVel, float maxAccel, float maxDecel, float maxAngVel, float maxAngAccel,
            const Pose& admissibleError, float kN, float kOmega, const PID& headingPID, float correctionDistance = 5.0,
            bool useCurvatureControl = false, std::function<float(float)> errorMapFunc = [](float x) { return x; });

        void followPath(const Path& path);
        Pose lastError = Pose(0.0, 0.0);
        float lastUpdateTimestamp = 0.0;
        float lastVel = 0.0;
        float lastAngVel = 0.0;
        float lastProjectedDisplacement = 0.0;
        std::optional<PathGVF> gvf;
        std::optional<float> lastDesiredHeading;

        void init();

        Pose update(const Pose& currentPose, const std::optional<Pose>& currentRobotVel);

        bool isFollowing();

        // static Pose calculateRobotPoseError(Pose targetFieldPose, Pose currentFieldPose)
        // static float normDelta(float angle) { return wrap(angle, -M_PI, M_PI); }
};
} // namespace lemlib
