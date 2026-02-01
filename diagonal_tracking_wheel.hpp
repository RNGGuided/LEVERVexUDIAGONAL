#pragma once
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include <cmath>

class DiagonalTrackingWheel : public lemlib::TrackingWheel {
public:
    DiagonalTrackingWheel(
        pros::adi::Encoder* diagA,
        pros::adi::Encoder* diagB,
        bool vertical,          // true = forward, false = lateral
        double wheelDiameter,
        double offset
    )
    : lemlib::TrackingWheel((pros::adi::Encoder*)nullptr, wheelDiameter, offset),
      a(diagA),
      b(diagB),
      isVertical(vertical),
      wheelDiameter(wheelDiameter),
      lastA(0),
      lastB(0) {}

    double getDistance() {
        // Read encoder ticks
        double currA = a->get_value();
        double currB = b->get_value();

        double dA = currA - lastA;
        double dB = currB - lastB;

        lastA = currA;
        lastB = currB;

    
        constexpr double TICKS_PER_REV = 8192.0;

        double inchesPerTick =
            (M_PI * wheelDiameter) / TICKS_PER_REV;

        dA *= inchesPerTick;
        dB *= inchesPerTick;


        if (isVertical) {
            return (dA + dB) / M_SQRT2;   // forward
        } else {
            return (dA - dB) / M_SQRT2;   // lateral
        }
    }

private:
    pros::adi::Encoder* a;
    pros::adi::Encoder* b;
    bool isVertical;
    double wheelDiameter;
    double lastA;
    double lastB;
};
