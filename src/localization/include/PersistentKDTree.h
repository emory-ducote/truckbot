#include <memory>
#include <algorithm>
#include <limits>
#include <climits>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

const int k = 2;

struct Node {
    double point[k];
    Matrix2d P;
    std::shared_ptr<const Node> left;
    std::shared_ptr<const Node> right;

    Node(const double arr[k],
         Matrix2d P,
         std::shared_ptr<const Node> L = nullptr,
         std::shared_ptr<const Node> R = nullptr)
        : P(P), left(L), right(R)
    {
        for (int i = 0; i < k; i++)
            point[i] = arr[i];
    }
};


std::shared_ptr<const Node> insertRec(std::shared_ptr<const Node> root, 
                                      double point[], 
                                      Matrix2d P,
                                      unsigned depth)
{
    // Tree is empty?
    if (root == nullptr) {
        return std::make_shared<const Node>(point, P);
    }

    // Calculate current dimension (cd) of comparison
    unsigned cd = depth % k;

    // Compare the new point with root on current dimension 'cd'
    // and decide the left or right subtree
    if (point[cd] < (root->point[cd])) {
        auto newLeft = insertRec(root->left, point, P, depth + 1);
        return std::make_shared<const Node>(root->point, root->P, newLeft, root->right);
    }
    else {
        auto newRight = insertRec(root->right, point, P, depth + 1);
        return std::make_shared<const Node>(root->point, root->P, root->left, newRight);
    }

    return root;
}

std::shared_ptr<const Node> insert(std::shared_ptr<const Node> root,
                                   double point[],
                                   Matrix2d P)
{
    return insertRec(root, point, P, 0);
}

std::shared_ptr<const Node> minNode(std::shared_ptr<const Node> x, 
                                    std::shared_ptr<const Node> y, 
                                    std::shared_ptr<const Node> z, 
                                    int d)
{
    std::shared_ptr<const Node> res = x;
    if (y != NULL && y->point[d] < res->point[d])
       res = y;
    if (z != NULL && z->point[d] < res->point[d])
       res = z;
    return res;
}

std::shared_ptr<const Node> findMinRec(std::shared_ptr<const Node> root, 
                                       int d, 
                                       unsigned depth)
{
    if (!root) {
        return nullptr;
    }

    unsigned cd = depth % k;
    if (cd == d) {
        if (!root->left) {
            return root;
        }
        return findMinRec(root->left, d, depth + 1);
    }
    return minNode(root,
                   findMinRec(root->left, d, depth + 1),
                   findMinRec(root->right, d, depth + 1), d);
}

std::shared_ptr<const Node> findMin(std::shared_ptr<const Node> root, int d)
{
    // Pass current level or depth as 0
    return findMinRec(root, d, 0);
}

bool arePointsSame(const double point1[], const double point2[])
{
    for (int i = 0; i < k; ++i)
        if (point1[i] != point2[i])
            return false;

    return true;
}

void copyPoint(double p1[],const double p2[])
{
   for (int i=0; i<k; i++)
       p1[i] = p2[i];
}

std::shared_ptr<const Node> deleteNodeRec(std::shared_ptr<const Node> root,
                                       const double point[],
                                       unsigned depth)
{
    if (!root) {
        return nullptr;
    }

    unsigned cd = depth % k;

    if (arePointsSame(root->point, point)) {
        // Leaf node
        if (!root->left && !root->right) {
            return nullptr;
        }

        // Node with children
        std::cout << "NODE:" << root->point[0] << "," << root->point[1] << std::endl;
        if (root->right) {
            auto minNode = findMin(root->right, cd);
            double newPoint[k];
            copyPoint(newPoint, minNode->point);
            auto newRight = deleteNodeRec(root->right, minNode->point, depth + 1);
            return std::make_shared<const Node>(newPoint, minNode->P, root->left, newRight);
        } else { // only left child
            auto minNode = findMin(root->left, cd);
            double newPoint[k];
            copyPoint(newPoint, minNode->point);
            auto newRight = deleteNodeRec(root->left, minNode->point, depth + 1);
            return std::make_shared<const Node>(newPoint, minNode->P, nullptr, newRight);
        }
    }

    // Recurse left or right
    if (point[cd] < root->point[cd]) {
        auto newLeft = deleteNodeRec(root->left, point, depth + 1);
        if (newLeft == root->left) {
          return root; // no change
        }   
        return std::make_shared<const Node>(root->point, root->P, newLeft, root->right);
    } else {
        auto newRight = deleteNodeRec(root->right, point, depth + 1);
        if (newRight == root->right) {
            return root; // no change
        }
        return std::make_shared<const Node>(root->point, root->P, root->left, newRight);
    }
}

std::shared_ptr<const Node> deleteNode(std::shared_ptr<const Node> root, double point[])
{
   // Pass depth as 0
   return deleteNodeRec(root, point, 0);
}

void printKDTree(std::shared_ptr<const Node> root, int depth = 0)
{
    if (!root) return;

    printKDTree(root->right, depth + 1);

    std::cout << std::string(depth * 4, ' ');

    std::cout << "(";
    for (int i = 0; i < k; i++) {
        std::cout << root->point[i];
        if (i < k - 1) std::cout << ",";
    }
    std::cout << ")\n";

    printKDTree(root->left, depth + 1);
}

double distanceSquared(const double a[], const double b[]) {
    double dist = 0.0;
    for (size_t i = 0; i < k; ++i) {
        double diff = a[i] - b[i];
        dist += diff * diff;
    }
    return dist;
}

void nearestNeighbor(
    std::shared_ptr<const Node> root,
    const double target[k],
    int depth,
    const Node*& best,
    double& bestDist)
{
    if (!root) return;

    double d = distanceSquared(root->point, target);
    if (d < bestDist) {
        bestDist = d;
        best = root.get();
    }

    int cd = depth % k;
    bool goLeft = target[cd] < root->point[cd];

    nearestNeighbor(goLeft ? root->left : root->right, target, depth + 1, best, bestDist);

    double diff = static_cast<double>(target[cd] - root->point[cd]);
    if (diff * diff < bestDist) {
        nearestNeighbor(goLeft ? root->right : root->left, target, depth + 1, best, bestDist);
    }
}

const Node* findNearest(std::shared_ptr<const Node> root, const double target[k]) {
    const Node* best = nullptr;
    double bestDist = std::numeric_limits<double>::infinity();
    nearestNeighbor(root, target, 0, best, bestDist);
    return best;
}

// int main()
// {
//     int points[][k] = {{30, 40}, {5, 25}, {70, 70},
//                         {10, 12}, {50, 30}, {35, 45}};
//     std::shared_ptr<const Node> root = nullptr;

//     int n = sizeof(points)/sizeof(points[0]);

//     for (int i=0; i<n; i++)
//         root = insert(root, points[i]);

//     // Delete (30, 40);
//     std::shared_ptr<const Node> root2 = deleteNode(root, points[0]);

//     printKDTree(root);
//     std::cout << "\n\n\n" << std::endl;
//     const int target[2] = {5, 25};
//     const int targetNew[2] = {9, 11};
//     printKDTree(root2);
//     std::cout << "\n\n\n" << std::endl;

//     std::cout << findNearest(root, target)->point[0] << std::endl;

//     return 0;
// }
