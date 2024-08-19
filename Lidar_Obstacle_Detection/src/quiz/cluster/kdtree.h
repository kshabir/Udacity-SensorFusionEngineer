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
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node* &node, std::vector<float> point, uint depth, int id )
	{
		if(node == NULL)
		{
			node = new Node(point, id);
		}
		else
		{
			for(int i = 0; i < point.size(); i++)
			{
				// since point.size() always equal 2: i can be 0 or 1

				if(point.at(i) > node->point[i])
				{
					// insert in right of root
					insertHelper(node->right, point, depth + 1, id);
				}
				else
				{
					insertHelper(node->left, point, depth + 1, id);
				}

			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		insertHelper(root, point, 0, id);
	}

	bool isInBox(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		if( point[0] < (target[0] - distanceTol) || point[0] > (target[0] + distanceTol) ||
			point[1] < (target[1] - distanceTol) || point[1] > (target[1] - distanceTol)
		)
		{
			return false;
		}
		return true;
	}

	bool isIndistanceTol(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		float d = sqrtf(pow((target[0], point[0]), 2.0) + pow((target[1], point[1]), 2.0));

		if(d < distanceTol) return true;

		return false;

	}

	void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol)
	{
		// If target inside a box
		if(isInBox(target, node->point, distanceTol))
		{
			// if pont and target within threshold, push that point index into ids 
			if(isIndistanceTol(target, node->point, distanceTol))
			{
				ids->push_back(point->id);
			}
		}

		// If not in box, either move left or right to the kd tree
		// It's a nearest neighbor search
		if( (target[depth % 2] - distanceTol) < node->point[depth % 2])
		{
			searchHelper(target, root->left, depth + 1, distanceTol);
		}
		if( (target[depth % 2] + distanceTol) > node->point[depth % 2])
		{
			searchHelper(target, root->right, depth + 1, distanceTol);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol);
		return ids;
	}
	

};





