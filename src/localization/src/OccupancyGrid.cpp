#include "OccupancyGrid.h"

#include <algorithm>
#include <cmath>

OccupancyGrid::OccupancyGrid(double resolution,
                             int width,
                             int height,
                             double origin_x,
                             double origin_y,
                             double log_odds_occupied,
                             double log_odds_free,
                             double log_odds_min,
                             double log_odds_max,
                             double max_range)
    : resolution(resolution),
      width(width),
      height(height),
      originX(origin_x),
      originY(origin_y),
      logOddsOccupied(log_odds_occupied),
      logOddsFree(log_odds_free),
      logOddsMin(log_odds_min),
      logOddsMax(log_odds_max),
      maxRange(max_range),
      logOdds(static_cast<size_t>(width) * static_cast<size_t>(height), 0.0)
{
}

OccupancyGrid::~OccupancyGrid() {}

void OccupancyGrid::integrateScan(const Vector3d& sensor_pose,
                                  const std::vector<float>& ranges,
                                  double angle_min,
                                  double angle_increment)
{
    float angle = angle_min;
    for (size_t i = 0; i < ranges.size(); ++i, angle += angle_increment)
    {
        float r = ranges[i];
        int endRow; int endCol; int egoRow; int egoCol;
        bool valid = !std::isnan(r) && !std::isinf(r)
                     && r > 0.0 && r <= maxRange;
        if (!valid) continue;

        double beam_angle = sensor_pose.z() + angle;   // yaw + beam bearing
        if (!worldToCell(sensor_pose.x() + r * std::cos(beam_angle),
                         sensor_pose.y() + r * std::sin(beam_angle),
                         endCol, endRow)) continue;
        if (!worldToCell(sensor_pose.x(), sensor_pose.y(), egoCol, egoRow)) continue;

        int dx = abs(endRow - egoRow);
        int sx = egoRow < endRow ? 1 : -1;
        int dy = abs(endCol - egoCol);
        int sy = egoCol < endCol ? 1 : -1;
        int err = dx - dy;   

        int x = egoRow; int y = egoCol;
        while (x != endRow || y != endCol) {
            updateCell(y, x, logOddsFree);
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
        updateCell(endCol, endRow, logOddsOccupied);
    }
}

std::vector<int8_t> OccupancyGrid::toOccupancyData() const
{
    std::vector<int8_t> rosOccupancy(logOdds.size());
    for (size_t i = 0; i < logOdds.size(); i++)
    {
        if (logOdds[i] == 0.0)
        {
            rosOccupancy[i] = -1;
            continue;
        }
        double p = (1.0 - 1.0 / (1.0 + std::exp(logOdds[i]))) * 100.0;
        rosOccupancy[i] = static_cast<int8_t>(std::round(p));
    }
    return rosOccupancy;
}

bool OccupancyGrid::worldToCell(double x, double y, int& col, int& row) const
{
    int colCalc = static_cast<int>(std::floor((x - originX) / resolution));
    int rowCalc = static_cast<int>(std::floor((y - originY) / resolution));
    if ((colCalc < 0) || (colCalc >= width) || (rowCalc < 0) || (rowCalc >= height))
    {
        col = -1;
        row = -1;
        return false;
    }
    col = colCalc;
    row = rowCalc;
    return true;
}

void OccupancyGrid::updateCell(int col, int row, double delta)
{
    int index = row * width + col;
    double updated_val = logOdds[index] + delta;
    logOdds[index] = std::clamp(updated_val, logOddsMin, logOddsMax);
}
