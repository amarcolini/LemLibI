#include "lemlib/geometry/quinticPolynomial.hpp"
#include <cmath>

namespace lemlib {
QuinticPolynomial::QuinticPolynomial(float start, float startDeriv, float startSecondDeriv, float end, float endDeriv,
                                     float endSecondDeriv) {
    p0 = start;
    p1 = 0.2 * startDeriv + p0;
    p2 = 0.05 * startSecondDeriv + 2 * p1 - p0;
    p4 = end - 0.2 * endDeriv;
    p5 = end;
    p3 = 0.05 * endSecondDeriv + 2 * p4 - p5;

    coeffs = {p5 - 5 * p4 + 10 * p3 - 10 * p2 + 5 * p1 - p0,
              5 * (p4 - 4 * p3 + 6 * p2 - 4 * p1 + p0),
              10 * (p3 - 3 * p2 + 3 * p1 - p0),
              10 * (p2 - 2 * p1 + p0),
              5 * (p1 - p0),
              p0};

    dcoeffs = {5 * coeffs[0], 4 * coeffs[1], 3 * coeffs[2], 2 * coeffs[3], coeffs[4]};

    d2coeffs = {20 * coeffs[0], 12 * coeffs[1], 6 * coeffs[2], 2 * coeffs[3]};
}

float QuinticPolynomial::operator()(float t) const {
    return coeffs[0] * t * t * t * t * t + coeffs[1] * t * t * t * t + coeffs[2] * t * t * t + coeffs[3] * t * t +
           coeffs[4] * t + coeffs[5];
}

float QuinticPolynomial::deriv(float t) const {
    return dcoeffs[0] * t * t * t * t + dcoeffs[1] * t * t * t + dcoeffs[2] * t * t + dcoeffs[3] * t + dcoeffs[4];
}

float QuinticPolynomial::secondDeriv(float t) const {
    return d2coeffs[0] * t * t * t + d2coeffs[1] * t * t + d2coeffs[2] * t + d2coeffs[3];
}

float QuinticPolynomial::thirdDeriv(float t) const {
    return 60 * ((p5 - 5 * p4 + 10 * p3 - 10 * p2 + 5 * p1 - p0) * t * t +
                 (2 * p4 - 8 * p3 + 12 * p2 - 8 * p1 + 2 * p0) * t + p3 - 3 * p2 + 3 * p1 - p0);
}

// std::string QuinticPolynomial::toString() const {
//     std::ostringstream oss;
//     oss << std::fixed << std::setprecision(3);
//     oss << "(1-t)^5*" << p0
//         << " + 5(1-t)^4*t*" << p1
//         << " + 10(1-t)^3*t^2*" << p2
//         << " + 10(1-t)^2*t^3*" << p3
//         << " + 5(1-t)*t^4*" << p4
//         << " + t^5*" << p5;
//     return oss.str();
// }
} // namespace lemlib