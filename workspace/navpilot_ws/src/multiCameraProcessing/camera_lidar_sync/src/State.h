
#ifndef STATE_H
#define STATE_H

class State
{

public:
    // Constructors
    State() = default;
    State(double x_, double y_, double heading_ = 0)
        : x(x_), y(y_), heading(heading_){}

    /* data */
    double x{};
    double y{};
    double z{};
    double roll{};
    double pitch{};
    double heading{};

};

#endif // STATE_H