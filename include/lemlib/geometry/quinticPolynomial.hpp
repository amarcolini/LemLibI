#pragma once
#include <array>
#include <string>

namespace lemlib {
class QuinticPolynomial {
    public:
        QuinticPolynomial(float start, float startDeriv, float startSecondDeriv, float end, float endDeriv,
                          float endSecondDeriv);

        float operator()(float t) const;
        float deriv(float t) const;
        float secondDeriv(float t) const;
        float thirdDeriv(float t) const;

        std::string toString() const;
    private:
        float p0, p1, p2, p3, p4, p5;
        std::array<float, 6> coeffs;
        std::array<float, 5> dcoeffs;
        std::array<float, 4> d2coeffs;
};
} // namespace lemlib
