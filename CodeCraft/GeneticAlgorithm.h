#pragma once

#include "SmallestCostFlow.h"
#include "GenerateIntialSolutions.h"

struct Solution
{
	FlowSolution flowSolution;
	UIntTable servers; // node id

	std::string to_string(const Graph &g) const;
};

namespace GeneticAlgorithm
{
	Solution generateSolution(std::vector<BoolTable> intialSolutions, unsigned long interval_ms, const Graph &g);

	namespace SubFun
	{
		// to do
		static const unsigned long limit_time = 85000;
		static const double mutation_rate = 0.1;
		static const double cross_rate = 0.9;
		static const int pop_size = 15;
		static const int iteration = 30;
		static const int opt = 5;
	};
};