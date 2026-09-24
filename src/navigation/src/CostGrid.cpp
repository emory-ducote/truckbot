#include "CostGrid.h"

#include <algorithm>
#include <cmath>

CostGrid::CostGrid(double collisionRadius,
                   double clearanceRadius,
                   int occupiedThreshold)
    : collisionRadius(collisionRadius),
      clearanceRadius(clearanceRadius),
      occupiedThreshold(occupiedThreshold),
      resolution(0.0),
      width(0),
      height(0),
      originX(0.0),
      originY(0.0)
{
}

CostGrid::~CostGrid() {}

void CostGrid::setOccupancy(const std::vector<int8_t>& occupancy,
                            double resolution,
                            int width,
                            int height,
                            double originX,
                            double originY)
{
    this->occupancy = occupancy;
    this->resolution = resolution;
    this->width = width;
    this->height = height;
    this->originX = originX;
    this->originY = originY;
    this->costs.assign(static_cast<size_t>(width) * static_cast<size_t>(height), 0.0);
}

void CostGrid::computeCosts()
{
    std::fill(costs.begin(), costs.end(), 0.0);
    for (int row = 0; row < height; row++)
    {
        for (int col = 0; col < width; col++)
        {
            int i = index(col, row);
            if (occupancy[i] >= occupiedThreshold) {
                growObstacle(col, row);
            }
        }
    }
}

void CostGrid::growObstacle(int col, int row)
{

   
}

double CostGrid::costFromDistance(double distance) const
{
    if (distance <= collisionRadius)
    {
        return BLOCKED;
    }
    else if ((distance > collisionRadius) && (distance <= clearanceRadius))
    {
        return 1.0 - (distance - collisionRadius) / (clearanceRadius - collisionRadius);
    }
    else {
        return 0.0;
    }
}

std::vector<int8_t> CostGrid::toOccupancyData() const
{
    std::vector<int8_t> rosOccupancy(costs.size());
    for (size_t i = 0; i < costs.size(); i++)
    {
        double clamped = std::clamp(costs[i], 0.0, 1.0);
        rosOccupancy[i] = static_cast<int8_t>(std::round(clamped * 100.0));
    }
    return rosOccupancy;
}

bool CostGrid::inBounds(int col, int row) const
{
    return (col >= 0) && (col < width) && (row >= 0) && (row < height);
}
