#ifndef OCCUPANCY_GRID_H_
#define OCCUPANCY_GRID_H_

#include <cstdint>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

class OccupancyGrid {
    public:
        OccupancyGrid(double resolution = 0.05,
                      int width = 200,
                      int height = 200,
                      double origin_x = -5.0,
                      double origin_y = -5.0,
                      double log_odds_occupied = 0.85,
                      double log_odds_free = -0.4,
                      double log_odds_min = -2.0,
                      double log_odds_max = 3.5,
                      double max_range = 9.0);
        ~OccupancyGrid();

        void integrateScan(const Vector3d& sensor_pose,
                           const std::vector<float>& ranges,
                           double angle_min,
                           double angle_increment);

        std::vector<int8_t> toOccupancyData() const;

        double getResolution() const { return resolution; }
        int getWidth() const { return width; }
        int getHeight() const { return height; }
        double getOriginX() const { return originX; }
        double getOriginY() const { return originY; }

    private:
        // Map (meters) -> cell index. Returns false if outside the grid.
        bool worldToCell(double x, double y, int& col, int& row) const;
        // Clamped log-odds update of a single cell.
        void updateCell(int col, int row, double delta);

        double resolution;
        int width;
        int height;
        double originX;
        double originY;
        double logOddsOccupied;
        double logOddsFree;
        double logOddsMin;
        double logOddsMax;
        double maxRange;

        // Row-major log-odds storage, size width * height.
        std::vector<double> logOdds;
};

#endif
