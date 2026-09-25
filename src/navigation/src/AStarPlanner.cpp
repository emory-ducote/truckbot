#include "AStarPlanner.h"

#include <algorithm>
#include <cmath>

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



/* TODO - translate this

def find_path(grid: np.ndarray, start: Tuple[int, int], 
              goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    Find the optimal path using A* algorithm.
    
    Args:
        grid: 2D numpy array (0 = free space, 1 = obstacle)
        start: Starting position (x, y)
        goal: Goal position (x, y)
    
    Returns:
        List of positions representing the optimal path
    """
    # Initialize start node
    start_node = create_node(
        position=start,
        g=0,
        h=calculate_heuristic(start, goal)
    )
    
    # Initialize open and closed sets
    open_list = [(start_node['f'], start)]  # Priority queue
    open_dict = {start: start_node}         # For quick node lookup
    closed_set = set()                      # Explored nodes
    
    while open_list:
        # Get node with lowest f value
        _, current_pos = heapq.heappop(open_list)
        current_node = open_dict[current_pos]
        
        # Check if we've reached the goal
        if current_pos == goal:
            return reconstruct_path(current_node)
            
        closed_set.add(current_pos)
        
        # Explore neighbors
        for neighbor_pos in get_valid_neighbors(grid, current_pos):
            # Skip if already explored
            if neighbor_pos in closed_set:
                continue
                
            # Calculate new path cost
            tentative_g = current_node['g'] + calculate_heuristic(current_pos, neighbor_pos)
            
            # Create or update neighbor
            if neighbor_pos not in open_dict:
                neighbor = create_node(
                    position=neighbor_pos,
                    g=tentative_g,
                    h=calculate_heuristic(neighbor_pos, goal),
                    parent=current_node
                )
                heapq.heappush(open_list, (neighbor['f'], neighbor_pos))
                open_dict[neighbor_pos] = neighbor
            elif tentative_g < open_dict[neighbor_pos]['g']:
                # Found a better path to the neighbor
                neighbor = open_dict[neighbor_pos]
                neighbor['g'] = tentative_g
                neighbor['f'] = tentative_g + neighbor['h']
                neighbor['parent'] = current_node
    
    return []  # No path found

*/