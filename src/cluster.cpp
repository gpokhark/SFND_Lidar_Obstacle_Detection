
#include "cluster.h"


void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for (int id : nearest)
	{
		if (!processed[id])
		{
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while (i < points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}
		else
		{
			std::vector<int> cluster;
			clusterHelper(i, points, cluster, processed, tree, distanceTol);
			clusters.push_back(cluster);
			i++;
		}
	}

	return clusters;
}