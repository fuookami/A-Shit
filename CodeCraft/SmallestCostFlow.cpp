#include "SmallestCostFlow.h"
#include <iostream>

using namespace std;

namespace SmallestCostFlow
{
	static GraphMatrix graph;
	static size_t nodeSize;
	static int minCost = 0;
	const Graph * pGraph;
}

FlowSolution SmallestCostFlow::getSmallestCostFlow(BoolTable servers, const Graph & g)
{
	FlowSolution flowSolution;
	SubFun::initGraph(flowSolution, servers, g);
	SubFun::minCostMaxFlow(flowSolution);
	return std::move(flowSolution);
}

void SmallestCostFlow::SubFun::initGraph(FlowSolution &flowSolution, BoolTable & server, const Graph & g)
{
	minCost = 0;
	graph = g.graphMatrix;
	pGraph = &g;
	nodeSize = graph.size();

	for (int i = 0; i < nodeSize - 1; ++i)
	{
		if (server[i])
		{
			graph[0][i + 1].first = INT_MAX;
			graph[0][i + 1].second = 0;
			for (int j = 1; j < nodeSize - 1; ++j)
			{
				if (graph[i][j].first != 0)
				{
					graph[i][j].first = INT_MAX;
				}
			}
			cout << i << " ";
		}
	}
	cout << endl;
}

// find a path
void SmallestCostFlow::SubFun::dijkstra(std::vector<int> &distance, std::vector<int> &path)
{
	distance.assign(nodeSize, INT_MAX);
	vector<bool> isVisited;
	isVisited.assign(nodeSize, false);
	vector<int> pre;
	pre.assign(nodeSize, -1);

	int v = 0;
	for (v = 0; v < nodeSize; ++v)
	{
		distance[v] = graph[0][v].second;
		if (distance[v] != INT_MAX)
		{
			pre[v] = 0;
		}
	}
	distance[0] = 0;
	isVisited[0] = true;

	for (int i = 0; i < nodeSize; ++i)
	{
		int minDistance = INT_MAX;
		for (int w = 0; w < nodeSize; ++w)
		{
			if (!isVisited[w] && distance[w] < minDistance)
			{
				v = w;
				minDistance = distance[w];
			}
		}
		isVisited[v] = true;
		for (int w = 0; w < nodeSize; ++w)
		{
			if (!isVisited[w] && graph[v][w].second != INT_MAX &&
				minDistance + graph[v][w].second < distance[w])
			{
				distance[w] = minDistance + graph[v][w].second;
				pre[w] = v;
			}
		}
	}
	v = nodeSize - 1;
	if (pre[v] == -1)
	{
		return;
	}
	path.clear();
	while (v != 0)
	{
		path.push_back(v);
		v = pre[v];
	}
	reverse(path.begin(), path.end());
}

void SmallestCostFlow::SubFun::minCostMaxFlow(FlowSolution &flowSolution)
{
	Flow flow;

	std::vector<int> distance;

	std::vector<int> path;

	while (true)
	{
		dijkstra(distance, path);
		if (distance[nodeSize - 1] >= INT_MAX)
		{
			break;
		}

		Flow flow;
		flow.serverNodeId = path[0] - 1;

		int increase = INT_MAX;
		for (int i = 0; i < path.size() - 1; ++i)
		{
			if (graph[path[i]][path[i + 1]].first < increase)
			{
				increase = graph[path[i]][path[i + 1]].first;
			}
			flow.edges.push_back(pGraph->nodes[path[i]-1]->edges[path[i + 1]-1]);
			cout << path[i]-1 << " ";
		}
		flow.edges.pop_back();

		flow.flow = increase;
		cout << "  \t" <<increase<< endl;

		for (int i = 0; i < path.size() - 1; ++i)
		{
			if (graph[path[i]][path[i + 1]].first != INT_MAX)
			{
				graph[path[i]][path[i + 1]].first -= increase;
			}
			if (graph[path[i + 1]][path[i]].first != INT_MAX)
			{
				graph[path[i + 1]][path[i]].first += increase;
			}
			if (graph[path[i + 1]][path[i]].second == INT_MAX)
			{
				graph[path[i + 1]][path[i]].second = -graph[path[i]][path[i + 1]].second;
			}
			if (graph[path[i]][path[i + 1]].first == 0)
			{
				graph[path[i]][path[i + 1]].second = INT_MAX;
			}
		}
		minCost += distance[nodeSize - 1] * increase;
		flow.totalCost = distance[nodeSize - 1] * increase;
		flowSolution.flows.push_back(flow);
	}
	flowSolution.totalCost = minCost;
}
