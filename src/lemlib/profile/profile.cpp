// C++ version of the Kotlin MotionProfileGenerator
// Dependencies such as MotionState, MotionProfile, MotionSegment, VelocityConstraint, and AccelerationConstraint
// are assumed to be implemented similarly in C++.

#include <vector>
#include <cmath>
#include <algorithm>
#include <functional>
#include <cassert>
#include "lemlib/util.hpp"

namespace lemlib {
struct MotionState {
        float x, v, a;

        MotionState(float x = 0, float v = 0, float a = 0)
            : x(x),
              v(v),
              a(a) {}

        MotionState flipped() const { return MotionState(-x, -v, -a); }

        MotionState get(float t) { return MotionState(x + v * t + a / 2.0f * powf(t, 2.0f), v + a * t, a); }
};

struct MotionSegment {
        MotionState state;
        float dt;

        MotionSegment(MotionState state, float dt)
            : state(state),
              dt(dt) {}
};

struct MotionProfile {
        std::vector<MotionSegment> segments;

        MotionProfile(const std::vector<MotionSegment>& segments)
            : segments(segments) {}

        MotionProfile flipped() const {
            std::vector<MotionSegment> flippedSegments;
            for (const auto& seg : segments) { flippedSegments.emplace_back(seg.state.flipped(), seg.dt); }
            return MotionProfile(flippedSegments);
        }

        MotionState get(float t) const {
            if (t < 0.0) return segments[0].state;
            auto remainingTime = t;
            for (auto segment : segments) {
                if (remainingTime <= segment.dt) { return segment.state.get(remainingTime); }
                remainingTime -= segment.dt;
            }
            return segments.back().state;
        }
};

using VelocityConstraint = std::function<float(float, float)>;
using AccelerationConstraint = std::function<float(float, float, float)>;

MotionState afterDisplacement(const MotionState& state, float ds) {
    float discriminant = state.v * state.v + 2 * state.a * ds;
    float newV = (epsilonEquals(discriminant, 0.0)) ? 0.0 : std::sqrt(discriminant);
    return MotionState(state.x + ds, newV, state.a);
}

float intersection(const MotionState& s1, const MotionState& s2) {
    return (s1.v * s1.v - s2.v * s2.v) / (2 * s2.a - 2 * s1.a);
}

std::vector<std::pair<MotionState, float>> forwardPass(const MotionState& start,
                                                       const std::vector<float>& displacements,
                                                       const std::vector<float>& velocityConstraints,
                                                       const AccelerationConstraint& accelerationConstraint) {
    std::vector<std::pair<MotionState, float>> forwardStates;
    float ds = displacements[1] - displacements[0];
    MotionState lastState = start;

    for (size_t i = 0; i < displacements.size() - 1; ++i) {
        float s = displacements[i];
        float maxVel = velocityConstraints[i];

        if (lastState.v >= maxVel) {
            MotionState state(s, maxVel, 0.0);
            forwardStates.emplace_back(state, ds);
            lastState = afterDisplacement(state, ds);
        } else {
            float finalVel = accelerationConstraint(s, std::abs(ds), lastState.v);
            float accel = (finalVel * finalVel - lastState.v * lastState.v) / (2 * ds);

            if (finalVel <= maxVel) {
                MotionState state(s, lastState.v, accel);
                forwardStates.emplace_back(state, ds);
                lastState = afterDisplacement(state, ds);
            } else {
                float accelDx = (maxVel * maxVel - lastState.v * lastState.v) / (2 * accel);
                MotionState accelState(s, lastState.v, accel);
                MotionState coastState(s + accelDx, maxVel, 0.0);
                forwardStates.emplace_back(accelState, accelDx);
                forwardStates.emplace_back(coastState, ds - accelDx);
                lastState = afterDisplacement(coastState, ds - accelDx);
            }
        }
    }
    return forwardStates;
}

MotionProfile generateMotionProfile(const MotionState& start, const MotionState& goal,
                                    const VelocityConstraint& velocityConstraint,
                                    const AccelerationConstraint& accelerationConstraint,
                                    const AccelerationConstraint& decelerationConstraint, float resolution = 0.25) {
    if (goal.x < start.x) {
        return generateMotionProfile(
                   start.flipped(), goal.flipped(), [&](float s, float ds) { return velocityConstraint(-s, -ds); },
                   [&](float s, float ds, float v) { return accelerationConstraint(-s, -ds, -v); },
                   [&](float s, float ds, float v) { return decelerationConstraint(-s, -ds, -v); }, resolution)
            .flipped();
    }

    float length = goal.x - start.x;
    int samples = std::max(2, static_cast<int>(std::ceil(length / resolution)));

    std::vector<float> s(samples);
    float step = length / (samples - 1);
    for (int i = 0; i < samples; ++i) s[i] = i * step;

    std::vector<float> velocityConstraints(samples);
    for (int i = 0; i < samples; ++i) velocityConstraints[i] = velocityConstraint(start.x + s[i], step);

    auto forwardStates = forwardPass(start, s, velocityConstraints, accelerationConstraint);
    auto backwardStates = forwardPass(goal, std::vector<float>(s.rbegin(), s.rend()),
                                      std::vector<float>(velocityConstraints.rbegin(), velocityConstraints.rend()),
                                      decelerationConstraint);

    std::vector<std::pair<MotionState, float>> finalStates;

    size_t i = 0;
    while (i < forwardStates.size() && i < backwardStates.size()) {
        auto [fStart, fDx] = forwardStates[i];
        auto [bStart, bDx] = backwardStates[i];

        if (!epsilonEquals(fDx, bDx)) {
            if (fDx > bDx) {
                forwardStates.insert(forwardStates.begin() + i + 1, {afterDisplacement(fStart, bDx), fDx - bDx});
                fDx = bDx;
            } else {
                backwardStates.insert(backwardStates.begin() + i + 1, {afterDisplacement(bStart, fDx), bDx - fDx});
                bDx = fDx;
            }
        }

        MotionState fEnd = afterDisplacement(fStart, fDx);
        MotionState bEnd = afterDisplacement(bStart, bDx);

        if (fStart.v <= bStart.v) {
            if (fEnd.v <= bEnd.v) {
                finalStates.emplace_back(fStart, fDx);
            } else {
                float intersect = intersection(fStart, bStart);
                finalStates.emplace_back(fStart, intersect);
                finalStates.emplace_back(afterDisplacement(bStart, intersect), bDx - intersect);
            }
        } else {
            if (fEnd.v >= bEnd.v) {
                finalStates.emplace_back(bStart, bDx);
            } else {
                float intersect = intersection(fStart, bStart);
                finalStates.emplace_back(bStart, intersect);
                finalStates.emplace_back(afterDisplacement(fStart, intersect), fDx - intersect);
            }
        }
        ++i;
    }

    std::vector<MotionSegment> segments;
    for (const auto& [state, dx] : finalStates) {
        float dt;
        if (epsilonEquals(state.a, 0.0)) {
            dt = dx / state.v;
        } else {
            float discriminant = state.v * state.v + 2 * state.a * dx;
            if (epsilonEquals(discriminant, 0.0)) dt = -state.v / state.a;
            else {
                float sqrtDisc = std::sqrt(discriminant);
                float p = (sqrtDisc - state.v) / state.a;
                float n = (-sqrtDisc - state.v) / state.a;
                dt = (p >= 0) ? p : n;
            }
        }
        segments.emplace_back(state, dt);
    }

    return MotionProfile(segments);
}
} // namespace lemlib