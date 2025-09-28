#ifndef NODE_H
#define NODE_H

#include <vector>
#include <memory>
#include "State.h"

using namespace std;

namespace planner
{
    class Node
    {
    public:
        // Constructor
        Node(const State &Current_state, const vector<State> &Trajectory, double Cost_path, double steeringAngle, int direction, std::weak_ptr<Node> Parent)
            : Current_state(Current_state), Trajectory(Trajectory), Cost_path(Cost_path), steeringAngle(steeringAngle), direction(direction), Parent(Parent)
        {
        }

        // Variables
        State Current_state;
        vector<State> Trajectory;
        double Cost_path;
        double steeringAngle; // heading
        int direction;
        std::weak_ptr<Node> Parent; // Use weak_ptr to prevent cyclic references
    };

} // namespace planner

#endif // NODE_H