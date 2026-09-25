#ifndef ASTARPLANNER_H_
#define ASTARPLANNER_H_

// inspired by https://www.datacamp.com/tutorial/a-star-algorithm

#include <limits>
#include <memory>
#include <utility>
#include <vector>

struct AStarNode {
    int col;
    int row;
    double g;
    double h;
    double f;
    std::shared_ptr<AStarNode> parent;

    AStarNode(int col = 0, int row = 0,
              double g = std::numeric_limits<double>::infinity(),
              double h = 0.0,
              std::shared_ptr<AStarNode> parent = nullptr)
        : col(col), row(row), g(g), h(h), f(g + h), parent(parent) {}
};

class AStarPlanner {
    public:
        AStarPlanner(int8_t costThresh = 65);
        ~AStarPlanner();
        void setCosts(const std::vector<int8_t>& costs,
                      double resolution,
                      int width,
                      int height,
                      double originX,
                      double originY);

    private:
        double distanceHeuristic(int firstCol, int firstRow, int secondCol, int secondRow) const;
        std::vector<std::pair<int, int>> getNeighbors(int col, int row);
        std::vector<std::pair<int, int>> reconstructPath(std::shared_ptr<AStarNode> goalNode) const;
        bool inBounds(int col, int row) const;
        int index(int col, int row) const { return row * width + col; }

        double resolution;
        int width;
        int height;
        double originX;
        double originY;
        std::vector<int8_t> costs;
        int8_t costThresh;
};

#endif
