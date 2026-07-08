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
    : resolution_(resolution),
      width_(width),
      height_(height),
      origin_x_(origin_x),
      origin_y_(origin_y),
      log_odds_occupied_(log_odds_occupied),
      log_odds_free_(log_odds_free),
      log_odds_min_(log_odds_min),
      log_odds_max_(log_odds_max),
      max_range_(max_range),
      log_odds_(static_cast<size_t>(width) * static_cast<size_t>(height), 0.0)
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
                     && r > 0.0 && r <= max_range_;
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
            updateCell(y, x, log_odds_free_);
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
        updateCell(endCol, endRow, log_odds_occupied_);
    }
}

std::vector<int8_t> OccupancyGrid::toOccupancyData() const
{
    std::vector<int8_t> rosOccupancy(log_odds_.size());
    for (size_t i = 0; i < log_odds_.size(); i++)
    {
        if (log_odds_[i] == 0.0)
        {
            rosOccupancy[i] = -1;
            continue;
        }
        double p = (1.0 - 1.0 / (1.0 + std::exp(log_odds_[i]))) * 100.0;
        rosOccupancy[i] = static_cast<int8_t>(std::round(p));
    }
    return rosOccupancy;
}

bool OccupancyGrid::worldToCell(double x, double y, int& col, int& row) const
{
    int colCalc = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    int rowCalc = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    if ((colCalc < 0) || (colCalc >= width_) || (rowCalc < 0) || (rowCalc >= height_))
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
    int index = row * width_ + col;
    double updated_val = log_odds_[index] + delta;
    log_odds_[index] = std::clamp(updated_val, log_odds_min_, log_odds_max_);    
}
