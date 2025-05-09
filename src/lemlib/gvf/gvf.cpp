#include <algorithm>
#include <functional>
#include <cmath>
#include <optional>
#include "lemlib/geometry/path.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "lemlib/gvf/gvf.hpp"

static int sign(float x) { return (x > 0) - (x < 0); }

namespace lemlib {
static float normDelta(float angle) { return wrap(angle, -M_PI, M_PI); }

static Pose calculateRobotPoseError(Pose targetFieldPose, Pose currentFieldPose) {
    auto errorInFieldFrame =
        Pose((targetFieldPose - currentFieldPose).vec(), normDelta(targetFieldPose.theta - currentFieldPose.theta));
    return Pose(errorInFieldFrame.vec().rotate(-currentFieldPose.theta), errorInFieldFrame.theta);
}

PathGVF::PathGVF(const Path& path, const float kN, const std::function<float(float)> errorMapFunc)
    : path(path),
      kN(kN),
      errorMapFunc(errorMapFunc) {}

PathGVF::Phi::Phi(const Vector& point, const Vector& target, const Vector& tangent)
    : target(target),
      tangent(tangent),
      pathToPoint(point - target),
      orientation(-sign(pathToPoint.x * tangent.y - pathToPoint.y * tangent.x)),
      error(pathToPoint.norm()),
      normal(Vector(-tangent.y, tangent.x)) {}

Vector PathGVF::compute(Phi phi) {
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

void PathGVF::reset() { lastProjectDisplacement = 0.0; }

PathGVF::Phi PathGVF::internalGet(const Vector& point) {
    float displacement = path.fastProject(point, lastProjectDisplacement, 10);
    auto pathPoint = path[displacement];
    Vector tangent = path.deriv(displacement);
    lastProjectDisplacement = displacement;
    return Phi(point, pathPoint, tangent);
}

GVFFollower::GVFFollower(float maxVel, float maxAccel, float maxDecel, float maxAngVel, float maxAngAccel,
                         const Pose& admissibleError, float kN, float kOmega, const PID& headingPID,
                         float correctionDistance, bool useCurvatureControl, std::function<float(float)> errorMapFunc)
    : maxVel(maxVel),
      maxAccel(maxAccel),
      maxDecel(maxDecel),
      maxAngVel(maxAngVel),
      maxAngAccel(maxAngAccel),
      kN(kN),
      kOmega(kOmega),
      headingPID(headingPID),
      correctionDistance(correctionDistance),
      useCurvatureControl(useCurvatureControl),
      admissibleError(admissibleError),
      errorMapFunc(errorMapFunc) {}

void GVFFollower::followPath(const Path& path) {
    gvf.emplace(PathGVF(path, kN, errorMapFunc));
    init();
}

void GVFFollower::init() {
    lastUpdateTimestamp = pros::millis() / 1000.0;
    lastVel = 0.0;
    lastAngVel = 0.0;
    lastDesiredHeading = 0.0;
    headingPID.reset();
    lastProjectedDisplacement = 0.0;
}

Pose GVFFollower::update(const Pose& currentPose, const std::optional<Pose>& currentRobotVel) {
    if (!gvf.has_value()) return Pose(0.0, 0.0);
    float projectedDisplacement = gvf->path.fastProject(currentPose.vec(), lastProjectedDisplacement, 10);
    auto projectedPos = gvf->path[projectedDisplacement];
    auto output = PathGVF::Phi(currentPose.vec(), projectedPos, gvf->path.deriv(projectedDisplacement));
    auto gvfResult = gvf->compute(output);

    float initialDesiredHeading = std::atan2(gvfResult.y, gvfResult.x);
    float initialHeadingError = normDelta(initialDesiredHeading - currentPose.theta);

    bool atTarget = epsilonEquals(output.target, gvf->path.end);
    float remainingDistance = currentPose.vec().distTo(gvf->path.end);

    bool reversed = atTarget && output.error < correctionDistance && std::abs(initialHeadingError) > M_PI / 2.0;
    float desiredHeading = initialDesiredHeading + (reversed ? M_PI : 0.0);
    float headingError = normDelta(desiredHeading - currentPose.theta);

    float timestamp = pros::millis() / 1000.0;
    float dt = timestamp - lastUpdateTimestamp;

    std::optional<float> desiredOmega =
        lastDesiredHeading ? std::make_optional(normDelta(desiredHeading - *lastDesiredHeading) / dt) : std::nullopt;

    float omega = desiredOmega ? ((remainingDistance > correctionDistance ? *desiredOmega * kOmega : 0.0) +
                                  headingPID.update(headingError))
                               : 0.0;

    lastDesiredHeading = desiredHeading;

    float maxVelToStop = std::sqrt(2 * maxDecel * remainingDistance);
    float maxVelFromLast = lastVel + maxAccel * dt;
    float maxAngVelFromLast = desiredOmega ? lastAngVel + maxAngAccel * dt * sign(*desiredOmega - lastAngVel) : 0.0;
    float angVel =
        maxAngVelFromLast >= 0.0 ? std::min(maxAngVelFromLast, maxAngVel) : std::max(maxAngVelFromLast, -maxAngVel);

    float maxVelForCurvature = INFINITY;
    if (useCurvatureControl && output.error < correctionDistance && lastVel > 5.0 && lastDesiredHeading) {
        float targetOmega = gvf->path.tangentAngleDeriv(projectedDisplacement);
        if (!epsilonEquals(targetOmega, 0.0)) { maxVelForCurvature = lastVel * std::abs(angVel / targetOmega); }
    }

    float velocity = std::min({maxVelFromLast, maxVelToStop, maxVel, maxVelForCurvature});
    velocity = std::max(velocity, 0.0f);

    lastUpdateTimestamp = timestamp;
    lastVel = velocity;
    lastAngVel = angVel;

    lastError = calculateRobotPoseError(Pose(output.target.x, output.target.y, output.tangent.angle()), currentPose);

    bool inhibitMotion =
        (remainingDistance < correctionDistance && std::abs(headingError) > 15 * M_PI / 180.0) ||
        (remainingDistance < admissibleError.vec().norm() && std::abs(headingError) > admissibleError.theta);

    return Pose(velocity * (reversed ? -1.0 : 1.0) * (inhibitMotion ? 0.0 : 1.0), 0.0, omega);
}

bool GVFFollower::isFollowing() {
    return std::abs(lastError.x) < admissibleError.x && std::abs(lastError.y) < admissibleError.y &&
           std::abs(normDelta(lastError.theta)) < admissibleError.theta;
}
}; // namespace lemlib
