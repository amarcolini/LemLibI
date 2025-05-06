#include <array>
// #include <iostream>
// #include <sstream>
// #include <string>

class QuinticPolynomial {
private:
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double p5;

    std::array<double, 6> coeffs;
    std::array<double, 5> dcoeffs;
    std::array<double, 4> d2coeffs;

public:
    QuinticPolynomial(double start, double startDeriv, double startSecondDeriv, 
                      double end, double endDeriv, double endSecondDeriv) 
        : p0(start),
          p1(0.2 * startDeriv + p0),
          p2(0.05 * startSecondDeriv + 2 * p1 - p0),
          p3(0.05 * endSecondDeriv + 2 * (end - 0.2 * endDeriv) - end),
          p4(end - 0.2 * endDeriv),
          p5(end),
          coeffs{(p5 - 5 * p4 + 10 * p3 - 10 * p2 + 5 * p1 - p0),
                 5 * (p4 - 4 * p3 + 6 * p2 - 4 * p1 + p0),
                 10 * (p3 - 3 * p2 + 3 * p1 - p0),
                 10 * (p2 - 2 * p1 + p0),
                 (5 * p1 - 5 * p0),
                 p0},
          dcoeffs{5 * (p5 - 5 * p4 + 10 * p3 - 10 * p2 + 5 * p1 - p0),
                   5 * (4 * p4 - 16 * p3 + 24 * p2 - 16 * p1 + 4 * p0),
                   5 * (6 * p3 - 18 * p2 + 18 * p1 - 6 * p0),
                   5 * (4 * p2 - 8 * p1 + 4 * p0),
                   5 * (p1 - p0)},
          d2coeffs{20 * (p5 - 5 * p4 + 10 * p3 - 10 * p2 + 5 * p1 - p0),
                    20 * (3 * p4 - 12 * p3 + 18 * p2 - 12 * p1 + 3 * p0),
                    20 * (3 * p3 - 9 * p2 + 9 * p1 - 3 * p0),
                    20 * (p2 - 2 * p1 + p0)} {}

    double operator[](double t) const {
        return coeffs[0] * t * t * t * t * t +
               coeffs[1] * t * t * t * t +
               coeffs[2] * t * t * t +
               coeffs[3] * t * t +
               coeffs[4] * t + coeffs[5];
    }

    double deriv(double t) const {
        return dcoeffs[0] * t * t * t * t +
               dcoeffs[1] * t * t * t +
               dcoeffs[2] * t * t +
               dcoeffs[3] * t + dcoeffs[4];
    }

    double secondDeriv(double t) const {
        return d2coeffs[0] * t * t * t +
               d2coeffs[1] * t * t +
               d2coeffs[2] * t +
               d2coeffs[3];
    }

    double thirdDeriv(double t) const {
        return 60 * ((p5 - 5 * p4 + 10 * p3 - 10 * p2 + 5 * p1 - p0) * t * t +
                      (2 * p4 - 8 * p3 + 12 * p2 - 8 * p1 + 2 * p0) * t +
                      p3 - 3 * p2 + 3 * p1 - p0);
    }

    // std::string toString() const {
    //     std::ostringstream oss;
    //     oss << "(1-t)^5*" << p0 << " + 5(1-t)^4*t*" << p1 
    //         << " + 10(1-t)^3*t^2*" << p2 << " + 10(1-t)^2*t^3*" 
    //         << p3 << " + 5(1-t)*t^4*" << p4 << " + t^5*" << p5;
    //     return oss.str();
    // }
};

