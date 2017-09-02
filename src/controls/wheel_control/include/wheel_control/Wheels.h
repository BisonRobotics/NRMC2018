#ifndef PROJECT_WHEELS_H
#define PROJECT_WHEELS_H

#include <vector>

namespace wheel_control {
    class JointState
    {
        public:
            JointState(double position, double velocity, double effort);

            double position;
            double velocity;
            double effort;
    };

    class Wheel {
        public:
            Wheel(std::string name);
            Wheel(std::string name, double x_pos, double y_pos, double radius);

            std::string name;
            int id;
            double x_pos, y_pos, radius;
            JointState *current_state, *desired_state;
    };

    class Wheels {
        public:
            Wheels();
            Wheels(double radius, double x_dist, double y_dist);
            ~Wheels();

            Wheel* get_wheel(std::string name);
            std::vector<Wheel*> get();
            Wheel *right_front, *right_back, *left_front, *left_back;
    };
}

#endif //PROJECT_WHEELS_H
