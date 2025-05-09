#include "lemlib/geometry/path.hpp"
#include "lemlib/geometry/quinticSpline.hpp"
#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <cmath>

namespace lemlib {
Path::Path(const std::vector<QuinticSpline> segments)
    : segments(segments),
      start(segments[0].internalGet(0)),
      end(segments.back().internalGet(1.0)) {
    if (segments.empty()) { throw std::invalid_argument("A Path cannot be initialized without segments."); }
}

Path::Path(const QuinticSpline segment)
    : segments {segment} {}

float Path::length() const {
    float totalLength = 0.0;
    for (const auto segment : segments) { totalLength += segment.length(); }
    return totalLength;
}

std::pair<QuinticSpline, float> Path::getSegment(float s) const {
    if (s <= 0.0) { return {segments.front(), 0.0}; }
    float remainingDisplacement = s;
    for (const auto& segment : segments) {
        if (remainingDisplacement <= segment.length()) { return {segment, remainingDisplacement}; }
        remainingDisplacement -= segment.length();
    }
    return {segments.back(), segments.back().length()};
}

Vector Path::get(float s) const {
    auto [segment, remainingDisplacement] = getSegment(s);
    return segment.get(segment.reparam(remainingDisplacement));
}

Vector Path::deriv(float s) const {
    auto [segment, remainingDisplacement] = getSegment(s);
    return segment.deriv(segment.reparam(remainingDisplacement));
}

float Path::tangentAngleDeriv(float s) const {
    auto [segment, remainingDisplacement] = getSegment(s);
    auto t = segment.reparam(remainingDisplacement);
    auto d = segment.deriv(t);
    auto d2 = segment.secondDeriv(t);
    return (d.x * d2.y - d.y * d2.x);
}

Vector Path::secondDeriv(float s) const {
    auto [segment, remainingDisplacement] = getSegment(s);
    return segment.secondDeriv(segment.reparam(remainingDisplacement));
}

float Path::curvature(float s) const {
    auto [segment, remainingDisplacement] = getSegment(s);
    return segment.curvature(segment.reparam(remainingDisplacement));
}

float Path::fastProject(const Vector queryPoint, float projectGuess, int iterations) const {
    if (projectGuess == -1.0) { projectGuess = length() / 2.0; }

    float s = projectGuess;
    for (int i = 0; i < iterations; ++i) {
        auto [segment, remainingDisplacement] = getSegment(s);
        auto t = segment.reparam(remainingDisplacement);
        Vector pathPoint = segment.internalGet(t);
        Vector deriv = segment.deriv(t);
        float ds = (queryPoint - pathPoint) * (deriv);
        s = std::clamp(s + ds, 0.0f, length());
    }
    return s;
}
} // namespace lemlib
