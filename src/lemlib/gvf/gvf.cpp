#include <functional>
#include <cmath>
#include "lemlib/geometry/path.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"
#include "lemlib/util.hpp"

namespace lemlib {
class PathGVF {
    public:
        PathGVF(
            Path& path, float kN, std::function<float(float)> errorMapFunc = [](float x) { return x; })
            : path(path),
              kN(kN),
              errorMapFunc(std::move(errorMapFunc)) {}

        struct Phi {
                const Vector point;
                const Vector target;
                const Vector tangent;
                const Vector pathToPoint;
                const float orientation;
                const float error;
                const Vector normal;

                Phi(const Vector& point, const Vector& target, const Vector& tangent)
                    : point(point),
                      target(target),
                      tangent(tangent),
                      pathToPoint(point - target),
                      orientation(-sign(pathToPoint.x * tangent.y - pathToPoint.y * tangent.x)),
                      error(pathToPoint.norm()),
                      normal(Vector(-tangent.y, tangent.x)) {}
        };

        Vector compute(const Vector& point) {
            Phi phi = internalGet(point);
            bool isFollowingPath = !((epsilonEquals(phi.target, path.start) && !epsilonEquals(phi.error, 0.0)) ||
                                     epsilonEquals(phi.target, path.end));

            float signedError = phi.orientation * phi.error;

            if (isFollowingPath) {
                return phi.tangent - phi.normal * kN * errorMapFunc(signedError);
            } else {
                Vector normal = epsilonEquals(phi.error, 0.0) ? Vector() : (phi.pathToPoint / phi.error);
                return normal * kN * errorMapFunc(-phi.error);
            }
        }

        void reset() { lastProjectDisplacement = 0.0; }
    private:
        Path& path;
        float kN;
        std::function<float(float)> errorMapFunc;
        float lastProjectDisplacement = 0.0;

        Phi internalGet(const Vector& point) {
            float displacement = path.fastProject(point, lastProjectDisplacement, 10);
            auto pathPoint = path[displacement];
            Vector tangent = path.deriv(displacement);
            lastProjectDisplacement = displacement;
            return Phi(point, pathPoint, tangent);
        }
};

// C++ translation of the Kotlin GVFFollower class

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
        float correctionDistance = 5.0;
        bool useCurvatureControl = false;
        std::function<float(float)> errorMapFunc;

        GVFFollower(
            float maxVel, float maxAccel, float maxDecel, float maxAngVel, float maxAngAccel,
            const Pose& admissibleError, float kN, float kOmega, const PID pidCoeffs, float correctionDistance = 5.0,
            bool useCurvatureControl = false, std::function<float(float)> errorMapFunc = [](float x) { return x; })
            : maxVel(maxVel),
              maxAccel(maxAccel),
              maxDecel(maxDecel),
              maxAngVel(maxAngVel),
              maxAngAccel(maxAngAccel),
              kN(kN),
              kOmega(kOmega),
              headingPID(pidCoeffs),
              correctionDistance(correctionDistance),
              useCurvatureControl(useCurvatureControl),
              errorMapFunc(errorMapFunc),
              headingController(pidCoeffs) {
            headingController.setInputBounds(-M_PI, M_PI);
        }

        void followPath(Path& path) {
            gvf.emplace(PathGVF(path, kN, errorMapFunc));
            init();
        }
    protected:
        Pose lastError = Pose(0.0, 0.0);
        float lastUpdateTimestamp = 0.0;
        float lastVel = 0.0;
        float lastAngVel = 0.0;
        std::optional<PathGVF> gvf;
        PID headingController;
        std::optional<float> lastDesiredHeading;

        void init() {
            lastUpdateTimestamp = pros::millis() / 1000.0f;
            lastVel = 0.0;
            lastAngVel = 0.0;
            lastDesiredHeading = std::nullopt;
        }

        DriveSignal internalUpdate(const Pose& currentPose, const std::optional<Pose>& currentRobotVel,
                                   const Pose& projectedPose, float projectedDisplacement) override {
            auto output = GuidingVectorField::Query(currentPose.vec())
                              .Phi(currentPose.vec(), projectedPos, gvf.path.deriv(projectedDisplacement));
            auto gvfResult = gvf->compute(output);

            float initialDesiredHeading = std::atan2(gvfResult.y, gvfResult.x);
            float initialHeadingError = (initialDesiredHeading - currentPose.theta).normDelta();

            bool atTarget = (output.target epsilonEquals gvf->endPosition());
            auto pathEnd = path.end();
            float remainingDistance = currentPose.vec().distTo(pathEnd.vec());

            bool reversed =
                atTarget && output.error < correctionDistance && initialHeadingError.abs() > Angle::quarterCircle();
            Angle desiredHeading = initialDesiredHeading + (reversed ? Angle::halfCircle() : 0_rad);
            Angle headingError = (desiredHeading - currentPose.heading).normDelta();
            headingController.setTargetPosition(desiredHeading.radians());

            float timestamp = clock->seconds();
            float dt = timestamp - lastUpdateTimestamp;

            std::optional<Angle> desiredOmega =
                lastDesiredHeading ? std::make_optional((desiredHeading - *lastDesiredHeading).normDelta() / dt)
                                   : std::nullopt;

            Angle omega = desiredOmega ? ((remainingDistance > correctionDistance ? *desiredOmega * kOmega : 0_rad) +
                                          headingController.update(currentPose.heading.radians()).rad())
                                       : 0_rad;

            lastDesiredHeading = desiredHeading;

            float maxVelToStop = std::sqrt(2 * maxDecel * remainingDistance);
            float maxVelFromLast = lastVel + maxAccel * dt;
            Angle maxAngVelFromLast =
                desiredOmega ? lastAngVel + maxAngAccel * dt * sign(*desiredOmega - lastAngVel) : 0_rad;
            Angle angVel = maxAngVelFromLast >= 0_rad ? std::min(maxAngVelFromLast, maxAngVel)
                                                      : std::max(maxAngVelFromLast, -maxAngVel);

            float maxVelForCurvature = Float::INFINITY;
            if (useCurvatureControl && output.error < correctionDistance && lastVel > 5.0 && lastDesiredHeading) {
                auto seg = path.segment(gvf->lastProjectDisplacement());
                Angle targetOmega = seg.first.curve.tangentAngleDeriv(seg.second);
                if (!(targetOmega epsilonEquals 0_rad)) {
                    maxVelForCurvature = lastVel * std::abs(angVel / targetOmega);
                }
            }

            float velocity = std::min({maxVelFromLast, maxVelToStop, maxVel, maxVelForCurvature});
            velocity = std::max(velocity, 0.0);

            lastUpdateTimestamp = timestamp;
            lastVel = velocity;
            lastAngVel = angVel;

            lastError = Kinematics::calculateRobotPoseError(Pose(output.target, output.tangent.angle()), currentPose);

            bool inhibitMotion =
                (remainingDistance < correctionDistance && headingError.abs() > Angle::degrees(15)) ||
                (remainingDistance < admissibleError.vec().norm() && headingError.abs() > admissibleError.heading);

            return DriveSignal(Pose(velocity * (reversed ? -1.0 : 1.0) * (inhibitMotion ? 0.0 : 1.0), 0.0, omega));
        }
};
} // namespace lemlib
