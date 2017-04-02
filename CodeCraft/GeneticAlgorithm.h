#pragma once

#include "SmallestCostFlow.h"
#include "GenerateIntialSolutions.h"


struct Solution
{
	FlowSolution flowSolution;
	std::vector<unsigned int> servers; // node id

	std::string to_string(const Graph &g) const;
};

namespace GeneticAlgorithm
{
	Solution generateSolution(std::vector<BoolTable> intialSolutions, const Graph &g);
	namespace SubFun
	{
		// to do
		static const unsigned short limit_time = 85;
		static const double mutation_rate = 0.1;
		static const double cross_rate = 0.9;
		static const int pop_size = 50;
		static const int iteration = 10;
		static const int opt = 5;
	};
};