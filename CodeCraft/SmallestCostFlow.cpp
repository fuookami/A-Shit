#include "SmallestCostFlow.h"
#include <iostream>
#include <algorithm>
#include <queue>

using namespace std;

namespace SmallestCostFlow
{
	static GraphMatrix graph;
	static size_t nodeSize;
	static int minCost = 0;
	const Graph * pGraph;
	static FlowSolution flowSolution;
	static vector<int> _servers;
}

FlowSolution SmallestCostFlow::getSmallestCostFlow(BoolTable servers, const Graph & g)
{
	SubFun::initGraph(servers, g);
	SubFun::minCostMaxFlow();
	return flowSolution;
}

void SmallestCostFlow::SubFun::initGraph(BoolTable & servers, const Graph & g)
{
	minCost = 0;
	flowSolution.flows.clear();
	graph = g.graphMatrix;
	pGraph = &g;
	nodeSize = graph.size();

	for (unsigned int i = 0; i != nodeSize - 1; ++i)
	{
		if (servers[i])
		{
			graph[0][i + 1].first = INT_MAX;
			graph[0][i + 1].second = 0;
			_servers.push_back(i);
		}
	}
}

// find a path
void SmallestCostFlow::SubFun::dijkstra(std::vector<int> &distance, std::vector<int> &path)
{
	distance.assign(nodeSize, INT_MAX);
	vector<bool> isVisited;
	isVisited.assign(nodeSize, false);
	vector<int> pre;
	pre.assign(nodeSize, -1);

	unsigned int v = 0;
	for (v = 0; v != nodeSize; ++v)
	{
		distance[v] = graph[0][v].second;
		if (distance[v] != INT_MAX)
		{
			pre[v] = 0;
		}
	}
	distance[0] = 0;
	isVisited[0] = true;

	for (unsigned int i = 0; i != nodeSize; ++i)
	{
		int minDistance = INT_MAX;
		for (unsigned int w(0); w != nodeSize; ++w)
		{
			if (!isVisited[w] && distance[w] < minDistance)
			{
				v = w;
				minDistance = distance[w];
			}
		}
		isVisited[v] = true;
		for (unsigned int w(0); w != nodeSize; ++w)
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

void SmallestCostFlow::SubFun::priorityQueueDijkstra(std::vector<int>& distance, std::vector<int>& path)
{
	distance.assign(nodeSize, INT_MAX);
	vector<int> nextv;
	nextv.assign(pGraph->edges.size() + _servers.size() + 1, -1);
	vector<int> tail;
	tail.assign(pGraph->edges.size() + _servers.size() + 1, -1);
	vector<int> val;
	val.assign(pGraph->edges.size() + _servers.size() + 1, -1);
	vector<int> head;
	head.assign(nodeSize, 0);
	vector<int> prev;
	prev.assign(nodeSize, -1);

	path.clear();

	for (size_t i = 0; i < _servers.size(); ++i)
	{
		int _size = i + 1;
		int u = pGraph->edges[i]->nodes.first->id + 1;
		int v = pGraph->edges[i]->nodes.second->id + 1;
		nextv[_size] = head[0];
		head[0] = _size;
		tail[_size] = _servers[i];
		val[_size] = 0;
	}

	for (unsigned int i = 0; i < pGraph->edges.size(); ++i)
	{
		int _size = i + 1 + _servers.size();
		int u = pGraph->edges[i]->nodes.first->id + 1;
		int v = pGraph->edges[i]->nodes.second->id + 1;
		if (graph[u][v].second == INT_MAX)
		{
			continue;
		}
		nextv[_size] = head[u];
		head[u] = _size;
		tail[_size] = v;
		val[_size] = pGraph->edges[i]->costPerFlow;
	}

	priority_queue<int> priorityQueue;
	priorityQueue.push(0);
	distance[0] = 0;
	while (!priorityQueue.empty())
	{
		int topNode = priorityQueue.top();
		priorityQueue.pop();
		for (int i = head[topNode]; i != 0; i = nextv[i])
		{
			int j = tail[i];
			if (distance[j] > distance[topNode] + val[i])
			{
				distance[j] = distance[topNode] + val[i];
				priorityQueue.push(j);
				prev[j] = topNode;
			}
		}
	}
	int k = nodeSize - 1;
	if (prev[k] == -1)
	{
		return;
	}
	while (k != 0)
	{
		path.push_back(k);
		k = prev[k];
	}
	reverse(path.begin(), path.end());
}

void SmallestCostFlow::SubFun::minCostMaxFlow()
{
	Flow flow;

	std::vector<int> distance;

	std::vector<int> path;

	while (true)
	{
		priorityQueueDijkstra(distance, path);
		if (distance[nodeSize - 1] >= INT_MAX)
		{
			break;
		}

		Flow flow;
		flow.serverNodeId = path[0] - 1;

		int increase = INT_MAX;
		for (unsigned int i(0); i != path.size() - 1; ++i)
		{
			if (graph[path[i]][path[i + 1]].first < increase)
			{
				increase = graph[path[i]][path[i + 1]].first;
			}
			flow.edges.push_back(pGraph->nodes[path[i] - 1]->edges[path[i + 1] - 1]);
		}
		flow.edges.pop_back();

		flow.flow = increase;

		for (unsigned int i(0); i != path.size() - 1; ++i)
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

bool FlowSolution::isValid(const Graph & g)
{
	UInt2UIntTable needs(g.getNeeds());

	for (std::vector<Flow>::const_iterator currIt(flows.cbegin()), edIt(flows.cend()); currIt != edIt; ++currIt)
	{
		if (currIt->edges.empty())
			needs[currIt->serverNodeId] -= currIt->flow;
		else
			needs[currIt->edges.back()->nodes.second->id] -= currIt->flow;
	}

	for (UInt2UIntTable::const_iterator currIt(needs.cbegin()), edIt(needs.cend()); currIt != edIt; ++currIt)
		if (currIt->second != 0)
			return false;
	return true;
}
