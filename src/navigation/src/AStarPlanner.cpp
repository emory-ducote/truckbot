#include "AStarPlanner.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

AStarPlanner::AStarPlanner(int8_t costThresh)
    : costThresh(costThresh)
{
}

AStarPlanner::~AStarPlanner() {}


void AStarPlanner::setCosts(const std::vector<int8_t>& costs,
                            double resolution,
                            int width,
                            int height,
                            double originX,
                            double originY)
{
    this->costs = costs;
    this->resolution = resolution;
    this->width = width;
    this->height = height;
    this->originX = originX;
    this->originY = originY;
}


double AStarPlanner::distanceHeuristic(int firstCol, int firstRow, int secondCol, int secondRow) const
{
    double dx = static_cast<double>(firstCol - secondCol);
    double dy = static_cast<double>(firstRow - secondRow);
    return std::hypot(dx, dy) * resolution;
}

bool AStarPlanner::inBounds(int col, int row) const
{
    return (col >= 0) && (col < width) && (row >= 0) && (row < height);
}

std::vector<std::pair<int, int>> AStarPlanner::getNeighbors(int col, int row)
{
    static const int offsets[8][2] = {
        {1, 0}, {-1, 0},
        {0, 1}, {0, -1},
        {1, 1}, {-1, -1},
        {1, -1}, {-1, 1}
    };

    std::vector<std::pair<int, int>> neighbors;
    for (const auto& offset : offsets) {
        int neighborCol = col + offset[0];
        int neighborRow = row + offset[1];
        if (inBounds(neighborCol, neighborRow) && costs[index(neighborCol, neighborRow)] < costThresh) {
            neighbors.push_back({neighborCol, neighborRow});
        }
    }
    return neighbors;
}


std::vector<std::pair<int, int>> AStarPlanner::reconstructPath(std::shared_ptr<AStarNode> goalNode) const
{
    std::vector<std::pair<int, int>> path;
    std::shared_ptr<AStarNode> current = goalNode;

    while (current != nullptr) {
        path.push_back({current->col, current->row});
        current = current->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::pair<int, int>> AStarPlanner::findPath(const std::pair<int, int>& start,
                                                        const std::pair<int, int>& goal)
{
    if (!inBounds(start.first, start.second) || !inBounds(goal.first, goal.second)) {
        std::cerr << "AStarPlanner: start or goal out of bounds\n";
        return {};
    }

    const int startIdx = index(start.first, start.second);
    const int goalIdx = index(goal.first, goal.second);

    auto startNode = std::make_shared<AStarNode>(start.first, start.second, 0.0,
        distanceHeuristic(start.first, start.second, goal.first, goal.second));

    std::priority_queue<std::pair<double, int>,
                        std::vector<std::pair<double, int>>,
                        std::greater<>> openList;
    openList.emplace(startNode->f, startIdx);

    std::unordered_map<int, std::shared_ptr<AStarNode>> openDict;
    openDict[startIdx] = startNode;
    std::unordered_set<int> closedSet;

    while (!openList.empty()) {
        auto [f, currentIdx] = openList.top();
        openList.pop();

        if (closedSet.count(currentIdx)) {
            continue;
        }

        if (currentIdx == goalIdx) {
            return reconstructPath(openDict[currentIdx]);
        }

        closedSet.insert(currentIdx);

        auto currentNode = openDict[currentIdx];

        for (const auto& [neighborCol, neighborRow] : getNeighbors(currentNode->col, currentNode->row)) {
            const int neighborIdx = index(neighborCol, neighborRow);
            if (closedSet.count(neighborIdx))
            {
                continue;
            }

            auto tempG = currentNode->g + 
                           distanceHeuristic(currentNode->col, currentNode->row, neighborCol, neighborRow);
            

            if (!openDict.count(neighborIdx)) 
            {
                auto neighborNode = std::make_shared<AStarNode>(neighborCol, neighborRow, tempG,
                                    distanceHeuristic(neighborCol, neighborRow, goal.first, goal.second), currentNode);
                openDict[neighborIdx] = neighborNode;
                openList.emplace(neighborNode->f, neighborIdx);
            }
            else if (tempG < openDict[neighborIdx]->g)
            {
                auto neighborNode = openDict[neighborIdx];
                neighborNode->g = tempG;
                neighborNode->f = tempG + neighborNode->h;
                neighborNode->parent = currentNode;
                openList.emplace(neighborNode->f, neighborIdx);
            }
        }
    }


    return {};
}