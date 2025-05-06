#pragma once
#include "lemlib/pose.hpp"
#include "lemlib/geometry/quinticPolynomial.hpp"
#include <optional>
#include <vector>
#include <cmath>
#include <cassert>

namespace lemlib {
class QuinticSpline {
    public:
        struct Knot {
                float x, y, dx, dy, d2x, d2y;

                Knot(float x, float y, float dx = 0.0, float dy = 0.0, float d2x = 0.0, float d2y = 0.0)
                    : x(x),
                      y(y),
                      dx(dx),
                      dy(dy),
                      d2x(d2x),
                      d2y(d2y) {}

                Knot(const Vector& pos, const Vector& deriv = Vector(), const Vector& secondDeriv = Vector())
                    : x(pos.x),
                      y(pos.y),
                      dx(deriv.x),
                      dy(deriv.y),
                      d2x(secondDeriv.x),
                      d2y(secondDeriv.y) {}

                Vector pos() const { return Vector(x, y); }

                Vector deriv() const { return Vector(dx, dy); }

                Vector secondDeriv() const { return Vector(d2x, d2y); }
        };

        QuinticSpline(const Knot start, const Knot end);

        float length() const;
        float reparam(float s) const;

        Vector get(float t) const;
        Vector internalDeriv(float t) const;
        Vector internalSecondDeriv(float t) const;
        Vector internalThirdDeriv(float t) const;
        Vector deriv(float t) const;
        Vector secondDeriv(float t) const;
        Vector thirdDeriv(float t) const;

        lemlib::Vector internalGet(float t) const;
        float curvature(float s) const;

        class ArcLengthParameterization {
            public:
                ArcLengthParameterization(const QuinticSpline& spline, float tLo, float tHi,
                                          float maxSegmentLength = 0.25, int maxDepth = 15, float maxDeltaK = 0.01);

                float reparam(float s) const;

                float getLength() const { return length; }
            private:
                const QuinticSpline& spline;
                float tLo;
                float tHi;
                std::vector<float> tSamples;
                std::vector<float> sSamples;
                float length = 0.0;

                void parameterize(float tLo, float tHi, int depth);
        };
    private:
        QuinticPolynomial x, y;
        ArcLengthParameterization param;
};
} // namespace lemlib