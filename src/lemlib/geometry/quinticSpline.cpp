#include "lemlib/geometry/quinticSpline.hpp"

namespace lemlib {
QuinticSpline::QuinticSpline(const Knot start, const Knot end)
    : x(start.x, start.dx, start.d2x, end.x, end.dx, end.d2x),
      y(start.y, start.dy, start.d2y, end.y, end.dy, end.d2y),
      param(*this, 0.0, 1.0) {};

Vector QuinticSpline::internalGet(float t) const {
    // Evaluate polynomials at t
    return Vector {x(t), y(t)}; // Replace with actual evaluation
}

float QuinticSpline::curvature(float s) const {
    auto t = reparam(s);
    auto deriv = internalDeriv(t);
    auto secondDeriv = internalSecondDeriv(s);
    return (deriv.cross(secondDeriv)) / powf(deriv.squaredNorm(), 1.5);
}

float QuinticSpline::length() const { return param.getLength(); }

float QuinticSpline::reparam(float s) const { return param.reparam(s); }

Vector QuinticSpline::get(float s) const { return internalGet(param.reparam(s)); };

Vector QuinticSpline::internalDeriv(float t) const { return Vector {x.deriv(t), y.deriv(t)}; };

Vector QuinticSpline::internalSecondDeriv(float t) const { return Vector {x.secondDeriv(t), y.secondDeriv(t)}; };

Vector QuinticSpline::internalThirdDeriv(float t) const { return Vector {x.thirdDeriv(t), y.thirdDeriv(t)}; };

Vector QuinticSpline::deriv(float t) const {
    auto deriv = internalDeriv(t);
    return deriv / deriv.norm();
};

Vector QuinticSpline::secondDeriv(float t) const {
    auto deriv = internalDeriv(t);
    auto secondDeriv = internalSecondDeriv(t);

    return ((secondDeriv * (deriv * deriv)) - (deriv * (secondDeriv * deriv))) / powf(deriv.norm(), 4);
};

Vector QuinticSpline::thirdDeriv(float t) const {
    auto deriv = internalDeriv(t);
    auto secondDeriv = internalSecondDeriv(t);
    auto thirdDeriv = internalThirdDeriv(t);

    auto pt1 = (thirdDeriv * (deriv * deriv)) - (deriv * (deriv * thirdDeriv));
    auto pt2 = (secondDeriv * (secondDeriv * deriv)) - (deriv * (secondDeriv * secondDeriv));
    return (pt1 + pt2) / powf(deriv.norm(), 9);
};

// ArcLengthParameterization implementation

QuinticSpline::ArcLengthParameterization::ArcLengthParameterization(const QuinticSpline& spline, float tLo, float tHi,
                                                                    float maxSegmentLength, int maxDepth,
                                                                    float maxDeltaK)
    : spline(spline),
      tLo(tLo),
      tHi(tHi) {
    tSamples.push_back(tLo);
    sSamples.push_back(0.0);
    parameterize(tLo, tHi, 0);
}

void QuinticSpline::ArcLengthParameterization::parameterize(float tLo, float tHi, int depth) {
    float tMid = (tLo + tHi) * 0.5;
    Vector vLo = spline.internalGet(tLo);
    Vector vMid = spline.internalGet(tMid);
    Vector vHi = spline.internalGet(tHi);

    float deltaK = std::abs(spline.curvature(0.0, tLo) - spline.curvature(0.0, tHi));
    float segmentLength = vLo.distTo(vMid) + vMid.distTo(vHi);

    if (depth < 15 && (deltaK > 0.01 || segmentLength > 0.25)) {
        parameterize(tLo, tMid, depth + 1);
        parameterize(tMid, tHi, depth + 1);
    } else {
        length += segmentLength;
        sSamples.push_back(length);
        tSamples.push_back(tHi);
    }
}

float QuinticSpline::ArcLengthParameterization::reparam(float s) const {
    if (s <= 0.0) return tLo;
    if (s >= length) return tHi;

    int lo = 0;
    int hi = static_cast<int>(sSamples.size()) - 1;

    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (s < sSamples[mid]) {
            hi = mid - 1;
        } else if (s > sSamples[mid]) {
            lo = mid + 1;
        } else {
            return tSamples[mid];
        }
    }

    // Linear interpolate
    float sLo = sSamples[lo - 1];
    float sHi = sSamples[lo];
    float tLoVal = tSamples[lo - 1];
    float tHiVal = tSamples[lo];

    return tLoVal + (s - sLo) * (tHiVal - tLoVal) / (sHi - sLo);
}

} // namespace lemlib
