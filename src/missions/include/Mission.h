#ifndef MISSION_H_
#define MISSION_H_

#include <optional>
#include <string>
#include <vector>

namespace missions {

struct Goal {
    double x;
    double y;
};

struct State {
    std::string name;
    std::optional<Goal> goal;
    std::optional<bool> liftEnabled;
};

struct Mission {
    std::string name;
    std::vector<State> states;
};

}

#endif
