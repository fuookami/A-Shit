#pragma once

#include "graph.h"

namespace GenerateIntialSolutions
{
	static unsigned int solutionNum = 10;

	std::vector<BoolTable> getIntialSolutions(Graph &g);

	namespace SubFun
	{
		void pretreat(Graph &g);
		void BFSPretreatFrom(Node *bgNode, Graph &g);
		double calNodeScore(const Node * const pNode);

		std::vector<BoolTable> generateIntialSolutions(const Graph &g);
		std::vector<BoolTable> selectServers(const std::vector<Node *> &nodes, const Graph &g);
		BoolTable generateServerCombination(const std::vector<Node *> &nodes, const Graph &g);

		double calSolutionCost(BoolTable &currSolution, const Graph &g);
	};
};