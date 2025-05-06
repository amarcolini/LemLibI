#pragma once
#include <vector>
#include <stdexcept>
#include "lemlib/pose.hpp"
#include "lemlib/geometry/quinticSpline.hpp"
#include "quinticSpline.hpp"

namespace lemlib {
class Path {
    public:
        const std::vector<QuinticSpline> segments;

        // Constructor from a list of path segments
        Path(const std::vector<QuinticSpline> segments);

        // Constructor from a single path segment
        explicit Path(const QuinticSpline segment);

        // Returns the length of the path
        float length() const;

        // Reparameterize all curves in the path
        void reparameterize();

        // Returns the segment at s along the path
        std::pair<QuinticSpline, float> getSegment(float s) const;

        // Returns the pose at s units along the path
        Vector get(float s) const;

        // Returns the derivative of the pose at s units along the path
        Vector deriv(float s) const;

        // Returns the second derivative of the pose at s units along the path
        Vector secondDeriv(float s) const;

        float curvature(float s) const;

        float fastProject(const Vector queryPoint, float projectGuess, int iterations) const;
};
} // namespace lemlib