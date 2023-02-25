#include <queue>
#include <iostream>
#include <array>
#include <set>
#include <memory>
#include <algorithm>
#include <math.h>       /* round, floor, ceil, trunc */
#include "uahrk_navigation/GridNode.hpp"
#include "uahrk_navigation/pose2d.hpp"


struct AstarCell {
    int cell;
    unsigned int cost;
    unsigned int function;

    std::shared_ptr<AstarCell> father;


    AstarCell() : cell{0}, cost{0}, function{0} {};
    AstarCell(const unsigned int _cell) : cell{_cell}, cost{0}, function{0}, father{nullptr} {};
    AstarCell(const unsigned int _cell, AstarCell _father) : cell{_cell}, cost{0}, function{0}, father{std::make_shared<AstarCell>(_father)} {};
    AstarCell(const unsigned int _cell, const unsigned int _cost, const unsigned int _function) : cell{_cell}, cost{_cost}, function{_function} {};

    friend inline bool operator==(const AstarCell& lhs, const AstarCell& rhs) { 
        return (lhs.cell == rhs.cell); 
    }
    friend inline bool operator!=(const AstarCell& lhs, const AstarCell& rhs) {
        return !(lhs == rhs);
    }
    friend inline bool operator<(const AstarCell& lhs, const AstarCell& rhs) {
        return (lhs.function > rhs.function);
    }
};


std::vector<int> Astar(const AstarCell &start, const AstarCell &goal_cell, const nav_msgs::msg::OccupancyGrid &grid);
int pose2dintcell(Pose2d p, const nav_msgs::msg::OccupancyGrid &grid);
geometry_msgs::msg::PoseArray intpath2rospath(std::vector<int> intpath, const nav_msgs::msg::OccupancyGrid &grid);
geometry_msgs::msg::Pose intcell2rospose(int cell, const nav_msgs::msg::OccupancyGrid &grid);
std::vector<int> discretize_path(std::vector<int> path);
std::vector<int> reduce_by_slopes(std::vector<int> path, const nav_msgs::msg::OccupancyGrid &grid);


