/* \author Aaron Brown */
// Quiz on implementing kd tree

// #include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;
	uint kDim = 3; // dimension of the cloud

	KdTree()
		: root(NULL)
	{
	}

	void insertHelper(Node **node, uint depth, std::vector<float> point, int id)
	{
		// Tree is empty
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			// calculate current dim
			uint cd = depth % kDim;
			float metric, threshold;

			metric = point[cd];
			threshold = (*node)->point[cd];

			if (metric < threshold)
			{
				insertHelper(&((*node)->left), depth + 1, point, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(std::vector<float> target, Node *node, uint depth, float distanceTol, std::vector<int> &ids)
	{
		if (node != NULL)
		{
			float x_dist = fabs(target[0] - node->point[0]);
			float y_dist = fabs(target[1] - node->point[1]);
			float z_dist = fabs(target[2] - node->point[2]);

			if (x_dist <= distanceTol && y_dist <= distanceTol && z_dist <= distanceTol)
			{
				float distance = sqrt(x_dist * x_dist + y_dist * y_dist + z_dist * z_dist);

				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			// check across boundary
			if ((target[depth % kDim] + distanceTol) > node->point[depth % kDim])
			{
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
			if ((target[depth % kDim] - distanceTol) < node->point[depth % kDim])
			{
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			}
		}
		return;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};