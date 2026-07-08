#ifndef OCCUPANCY_GRID_H_
#define OCCUPANCY_GRID_H_

#include <cstdint>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

// Pure (ROS-free) occupancy grid map. Accumulates LiDAR returns into a
// log-odds grid anchored in the map frame. The ROS node lives in
// OccupancyGridMiddleware.cpp; this class holds only the map + update math.
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

        // Fold one LiDAR scan into the grid. `sensor_pose` is the sensor's pose
        // in the map frame (x, y, yaw). `ranges` are the raw beam distances with
        // the given angular layout (radians), matching sensor_msgs/LaserScan.
        void integrateScan(const Vector3d& sensor_pose,
                           const std::vector<float>& ranges,
                           double angle_min,
                           double angle_increment);

        // Occupancy in ROS convention: 0-100 probability, -1 for unknown.
        std::vector<int8_t> toOccupancyData() const;

        double getResolution() const { return resolution_; }
        int getWidth() const { return width_; }
        int getHeight() const { return height_; }
        double getOriginX() const { return origin_x_; }
        double getOriginY() const { return origin_y_; }

    private:
        // Map (meters) -> cell index. Returns false if outside the grid.
        bool worldToCell(double x, double y, int& col, int& row) const;
        // Clamped log-odds update of a single cell.
        void updateCell(int col, int row, double delta);

        double resolution_;
        int width_;
        int height_;
        double origin_x_;
        double origin_y_;
        double log_odds_occupied_;
        double log_odds_free_;
        double log_odds_min_;
        double log_odds_max_;
        double max_range_;

        // Row-major log-odds storage, size width_ * height_.
        std::vector<double> log_odds_;
};

#endif
