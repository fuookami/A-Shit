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
};

namespace SmallestCostFlow
{
	FlowSolution getSmallestCostFlow(BoolTable servers, Graph &g);

	namespace SubFun
	{
		void initGraph(BoolTable &server, Graph &g);

		void dijkstra(std::vector<int> &distance, std::vector<int> &nextNode);

		void minCostMaxFlow();

		void removeSourceNode();

		static Graph *graph;
		static size_t nodeSize;
		static int minCost = 0;
		static FlowSolution flowSolution;
	};
};