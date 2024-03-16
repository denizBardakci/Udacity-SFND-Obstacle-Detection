/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
        : point(arr), id(setId), left(nullptr), right(nullptr)
    {}
};

struct KdTree
{
    Node* root;

    KdTree()
        : root(nullptr)
    {}

    void insertHelper(Node*& node, unsigned int depth, const std::vector<float>& point, int id)
    {
        // is tree empty
        if (node == nullptr) {
            node = new Node(point, id);
        } else {
            unsigned int current_depth = depth % 3;
            if (point[current_depth] < node->point[current_depth])
                insertHelper(node->left, depth + 1, point, id);
            else
                insertHelper(node->right, depth + 1, point, id);
        }
    }

    void insert(const std::vector<float>& point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place it correctly within the root
        insertHelper(root, 0, point, id);
    }

    // check if the node point is within a boxed square that is 2x distanceTol for length, centered around the target point
    bool isInBox(const std::vector<float>& target, const std::vector<float>& point, float distanceTol)
    {
        if ((point[0] < (target[0] - distanceTol)) || (point[0] > (target[0] + distanceTol)) ||
            (point[1] < (target[1] - distanceTol)) || (point[1] > (target[1] + distanceTol)) ||
            (point[2] < (target[2] - distanceTol)) || (point[2] > (target[2] + distanceTol))) {
            return false;
        }
        return true;
    }

    // calculate if the distance between node point and target point is within tolerance
    bool isInDistanceTol(const std::vector<float>& target, const std::vector<float>& point, float distanceTol)
    {
        float d = sqrtf(pow((target[0] - point[0]), 2.0) + pow((target[1] - point[1]), 2.0) + + pow((target[2] - point[2]), 2.0));

        return (d < distanceTol);
    }

    void searchHelper(const std::vector<float>& target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
    {
        if (node != nullptr) {
            if (isInBox(target, node->point, distanceTol)) {
                if (isInDistanceTol(target, node->point, distanceTol)) {
                    // add point within distanceTol to the return list
                    ids.push_back(node->id);
                }
            }

            // branch to the next node depending on if the boxed square crosses over the divided x or y region
            if ((target[depth % 3] - distanceTol) < node->point[depth % 3]) {
                searchHelper(target, node->left, depth + 1, distanceTol, ids);
            }
            if ((target[depth % 3] + distanceTol) >= node->point[depth % 3]) {
                searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const std::vector<float>& target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }
};
