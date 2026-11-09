/*********************************************************************************
 * Copyright (c) 2026, Dr. Daniel L. Lau -- All rights reserved.                 *
 *********************************************************************************/
//
// LAURailPredictor -- where one rail probably is between the controller's position reports, which only come about
// twice a second.  A commanded move has a known target, speed and acceleration, so its trapezoidal profile is
// followed in time, and every report slides the profile to agree with the rail.  For drawing only: never for
// anything that is measured or commanded.
//
#ifndef LAURAILPREDICTOR_H
#define LAURAILPREDICTOR_H

#include <algorithm>
#include <cmath>

class LAURailPredictor
{
public:
    bool isKnown() const
    {
        return known;
    }
    bool isMoving() const
    {
        return moving;
    }

    double estimate(double now) const
    {
        return moving ? start + direction * along(now - startTime) : last;
    }

    void command(double target, double speed, double accel, double now)
    {
        if (!known || speed <= 0.0 || accel <= 0.0) {
            return;
        }
        start = estimate(now);
        goal = target;
        direction = (goal >= start) ? 1.0 : -1.0;
        distance = std::abs(goal - start);
        accelerating = accel;
        if (distance < speed * speed / accel) {      // too short to reach full speed
            rampTime = std::sqrt(distance / accel);
            peak = accel * rampTime;
            cruiseTime = 0.0;
        } else {
            rampTime = speed / accel;
            peak = speed;
            cruiseTime = (distance - speed * speed / accel) / speed;
        }
        startTime = commandTime = now;
        lastReport = start;
        moving = distance > ARRIVED_MM;
        if (!moving) {
            last = goal;
        }
    }

    void report(double mm, double now)
    {
        known = true;
        if (moving) {
            const bool arrived = std::abs(mm - goal) < ARRIVED_MM;
            const bool stalled = now - commandTime > STALL_AFTER_S && std::abs(mm - lastReport) < STILL_MM;   // aborted, or blocked
            if (!arrived && !stalled) {
                // slide the profile in time so that it is at the rail's reported place now
                const double done = std::min(std::max(direction * (mm - start), 0.0), distance);
                double low = 0.0, high = duration();
                for (int n = 0; n < 40; n++) {
                    const double middle = 0.5 * (low + high);
                    (along(middle) < done ? low : high) = middle;
                }
                startTime = now - 0.5 * (low + high);
                lastReport = mm;
                return;
            }
            moving = false;
        }
        last = mm;
    }

private:
    static constexpr double ARRIVED_MM = 0.05;
    static constexpr double STILL_MM = 0.01;
    static constexpr double STALL_AFTER_S = 0.3;  // a rail that has just been told to move may not have started

    bool known = false, moving = false;
    double last = 0.0, lastReport = 0.0;
    double start = 0.0, goal = 0.0, direction = 1.0, distance = 0.0;
    double accelerating = 1.0, rampTime = 0.0, cruiseTime = 0.0, peak = 0.0;
    double startTime = 0.0, commandTime = 0.0;

    double duration() const
    {
        return 2.0 * rampTime + cruiseTime;
    }

    double along(double t) const              // distance covered after t seconds of the profile
    {
        const double total = duration();
        if (t <= 0.0) {
            return 0.0;
        }
        if (t >= total) {
            return distance;
        }
        if (t < rampTime) {
            return 0.5 * accelerating * t * t;
        }
        if (t <= rampTime + cruiseTime) {
            return 0.5 * accelerating * rampTime * rampTime + peak * (t - rampTime);
        }
        const double left = total - t;
        return distance - 0.5 * accelerating * left * left;
    }
};

#endif // LAURAILPREDICTOR_H
