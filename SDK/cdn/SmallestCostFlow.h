#pragma once

#include "graph.h"

struct Flow
{
	unsigned int serverNodeId;
	std::vector<Edge *> edges;
	unsigned int flow;
	unsigned int totalCost;
};

struct FlowSolution
{
	std::vector<Flow> flows;
	unsigned int totalCost;

	bool isValid(const Graph &g);
};

namespace SmallestCostFlow
{
	FlowSolution getSmallestCostFlow(BoolTable servers, const Graph &g);

	namespace SubFun
	{
		void initGraph(BoolTable &server, const Graph &g);

		void dijkstra(std::vector<int> &distance, std::vector<int> &path);

		void minCostMaxFlow();

		//void removeSourceNode();

	};
};
