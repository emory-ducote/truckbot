#ifndef COSTGRID_H_
#define COSTGRID_H_

#include <cstdint>
#include <vector>

class CostGrid {
    public:
        CostGrid(double collisionRadius = 0.23,
                 double clearanceRadius = 0.45,
                 int occupiedThreshold = 65);
        ~CostGrid();

        void setOccupancy(const std::vector<int8_t>& occupancy,
                          double resolution,
                          int width,
                          int height,
                          double originX,
                          double originY);

        void computeCosts();
        std::vector<int8_t> toOccupancyData() const;
        double getResolution() const { return resolution; }
        int getWidth() const { return width; }
        int getHeight() const { return height; }
        double getOriginX() const { return originX; }
        double getOriginY() const { return originY; }

        static constexpr double BLOCKED = 1.0;

    private:
        double costFromDistance(double distance) const;
        void growObstacle(int col, int row);

        bool inBounds(int col, int row) const;
        int index(int col, int row) const { return row * width + col; }

        double collisionRadius;
        double clearanceRadius;
        int occupiedThreshold;

        double resolution;
        int width;
        int height;
        double originX;
        double originY;
        std::vector<int8_t> occupancy;
        std::vector<double> costs;
};

#endif
