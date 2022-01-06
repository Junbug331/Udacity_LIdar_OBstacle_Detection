/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <unordered_set>
#include <vector>
#include <thread>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree {
    Node *root;

    KdTree()
            : root(NULL) {}

    ~KdTree() {
        delete root;
    }

    bool comparePoints(std::vector<float> &p1, const std::vector<float> &p2) {
        if (p1.size() != p2.size()) return false;
        for (int i = 0; i < p1.size(); i++) {
            if (p1[i] != p2[i]) return false;
        }
        return true;
    }

    Node *findNode(Node *currNode, const std::vector<float> &pt, int depth) {
        if (currNode == nullptr || comparePoints(currNode->point, pt)) return currNode;
        const std::vector<float> &currPt = currNode->point;
        int dim = depth % currPt.size();
        if (pt[dim] < currPt[dim])
            return currNode->left == nullptr ? currNode : findNode(currNode->left, pt, depth + 1);
        else
            return currNode->right == nullptr ? currNode : findNode(currNode->right, pt, depth + 1);
    }

    void insertHelper(Node **node, uint depth, std::vector<float> point, int id) {
        if (*node == nullptr)
            *node = new Node(point, id);
        else {
            uint cd = depth % point.size();

            if (point[cd] < ((*node)->point[cd]))
                insertHelper(&((*node)->left), depth + 1, point, id);
            else
                insertHelper(&((*node)->right), depth + 1, point, id);
        }
    }

    void insert(std::vector<float> point, int id) {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        Node *newNode = new Node(point, id);

        if (root == nullptr) {
            root = newNode;
            return;
        }

        Node *currNode = root;
        Node *prevNode = currNode;
        int depth = 0;
        bool isleft = false;

        while (currNode != nullptr) {
            int dim = depth % point.size();
            prevNode = currNode;
            if (point[dim] <= currNode->point[dim]) {
                isleft = true;
                currNode = currNode->left;
            } else if (point[dim] > currNode->point[dim]) {
                isleft = false;
                currNode = currNode->right;
            }
            depth++;
        }

        if (isleft) prevNode->left = newNode;
        else prevNode->right = newNode;
    }


    void searchHelper(Node *node, int depth, std::vector<int> &ids, const std::vector<float> &target, float distanceTol) {
        if (node != nullptr) {
            bool flag = false;
            for (int i = 0; i < target.size(); i++) {
                flag = (node->point[i] >= (target[i] - distanceTol)) && (node->point[i] < (target[i] + distanceTol));
                if (!flag)
                    break;
            }
            if (flag) {
                float distance = 0.0;
                for (int i = 0; i < target.size(); i++)
                    distance += pow(node->point[i] - target[i], 2);
                if (distance <= distanceTol * distanceTol) ids.push_back(node->id);
            }

            // check across boundary
            int dim = depth % target.size();
            if ((target[dim] - distanceTol) < node->point[dim])
                searchHelper(node->left, depth + 1, ids, target, distanceTol);
            if ((target[dim] + distanceTol) > node->point[dim])
                searchHelper(node->right, depth + 1, ids, target, distanceTol);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol) {
        std::vector<int> ids;
        searchHelper(root, 0, ids, target, distanceTol);
        return ids;
    }

};




