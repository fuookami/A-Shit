#include "GeneticAlgorithm.h"
#include <iostream>
#include <sstream>
#include <sys/timeb.h>
#include <random>


std::vector<BoolTable> old_population;
std::vector<BoolTable> new_population;
std::vector<double> fitness_rate;
double fitness_sum = 0;
double Rand()
{
	static std::random_device rd;
	return rd() /(double) rd.max();
}
int RWS()
{
	double m = 0;
	double  r = Rand();
	for (int i = 0; i < fitness_rate.size(); i++)
	{
		m = m + fitness_rate.at(i) / fitness_sum;
		if (r <= m)
			return i;
	}
	return -1;
}
std::string Solution::to_string(const Graph &g) const
{
	if (servers.empty())
	{
		return std::string("NA");
	}
	else
	{
		std::ostringstream out;
		const std::vector<Flow> &flows(flowSolution.flows);

		out << flows.size() << std::endl;
		out << std::endl;

		for (std::vector<Flow>::const_iterator currIt(flows.cbegin()), 
			edIt(flows.cend()); currIt != edIt; ++currIt)
		{
			out << currIt->serverNodeId << ' ';

			if (!currIt->edges.empty())
			{
				for (std::vector<Edge *>::const_iterator currEdgeIt(currIt->edges.cbegin()),
					edEdgeIt(currIt->edges.cend()); currEdgeIt != edEdgeIt; ++currEdgeIt)
					out << (*currEdgeIt)->nodes.second->id << ' ';
				out << currIt->edges.back()->nodes.second->needOrder << ' ';
			}
			else 
			{
				const Node * const currNode(g.nodes[currIt->serverNodeId].get());
				out << currNode->needOrder << ' ';
			}
			out << currIt->flow << std::endl;
		}

		return std::move(out.str());
	}
}

Solution GeneticAlgorithm::generateSolution(std::vector<BoolTable> intialSolutions, unsigned long interval_ms, const Graph & g)
{
	static std::random_device rd;

	if (intialSolutions.empty())
		return std::move(Solution());
	
	timeb rawtime;
	ftime(&rawtime);
	unsigned long last_interval_ms(interval_ms);
	unsigned long lastTime(0);
	unsigned long maxTime(0);
	unsigned int raw_count_it(0);
	unsigned int raw_count_it_sum(0);

	std::vector<BoolTable>::iterator init_it;
	new_population.assign(intialSolutions.begin(),intialSolutions.end());
	Solution minCost;
	minCost.flowSolution.totalCost = LONG_MAX;
	
	for (std::vector<BoolTable>::const_iterator it = new_population.cbegin(); it != new_population.cend(); it++)
	{
		FlowSolution temp = SmallestCostFlow::getSmallestCostFlow(*it, g);
		UIntTable servers;
		for (int i = 0; i < it->size(); i++)
			if (it->at(i))
				servers.push_back(i);
		temp.totalCost += g.costPerServer * servers.size();

		if (temp.isValid(g) && temp.totalCost < minCost.flowSolution.totalCost)
		{
			minCost.flowSolution = std::move(temp);
			minCost.servers = std::move(servers);
		}
	}

	int count_it = 0;
	do{
		int count = 0;
		int father_id;
		int mother_id;
		BoolTable father;
		BoolTable mother;
		old_population.clear();
		old_population.assign(new_population.cbegin(), new_population.cend());
		new_population.clear();
		fitness_rate.clear();
		for (init_it = old_population.begin(); init_it != old_population.end(); init_it++)
		{
			double fit_rate = GenerateIntialSolutions::SubFun::calSolutionCost(*init_it, g);
			fitness_sum += fit_rate;
			fitness_rate.push_back(fit_rate);
		}
		
		while (count < SubFun::pop_size)
		{
			while ((father_id = RWS()) == -1);
			while ((mother_id = RWS() )== -1 || mother_id == father_id);
			father.assign(old_population.at(father_id).cbegin(), old_population.at(father_id).cend());
			mother.assign(old_population.at(mother_id).cbegin(), old_population.at(mother_id).cend());
			if (Rand() < SubFun::cross_rate)
			{
				int front;
				while ((front = (rd() % father.size()) - SubFun::opt) < 0);
				for (int i = front; i <= front + SubFun::opt; i++)
				{
					bool temp = father.at(i);
					father.at(i) = mother.at(i);
					mother.at(i) = temp;
				}
			}
			if (Rand() < SubFun::mutation_rate)
			{

				while(g.nodes.at(father_id = (rd() % (father.size() - 1)))->isNeed);
				while(g.nodes.at(mother_id = (rd() % (father.size() - 1)))->isNeed);
				if (father.at(father_id))
					father.at(father_id) = false;
				else father.at(father_id) = true;
				if (mother.at(mother_id))
					father.at(mother_id) = false;
				else father.at(mother_id) = true;
			}
			new_population.push_back(mother);
			new_population.push_back(father);
			father.clear();
			mother.clear();
			count++;
		}
		++raw_count_it;
		if (++count_it == SubFun::iteration)
		{
			//调用精确算法；
			for (std::vector<BoolTable>::const_iterator it = new_population.cbegin(); it != new_population.cend(); it++)
			{
				FlowSolution temp = SmallestCostFlow::getSmallestCostFlow(*it, g);
				UIntTable servers;
				for (int i = 0; i < it->size(); i++)
					if (it->at(i))
						servers.push_back(i);
				temp.totalCost += g.costPerServer * servers.size();

				if (temp.isValid(g) && temp.totalCost < minCost.flowSolution.totalCost)
				{
					minCost.flowSolution = std::move(temp);
					minCost.servers = std::move(servers);
				}
			}
			//Solution = SmallestCostFlow::getSmallestCostFlow(new_population);
			count_it = 0;
		}

		ftime(&rawtime);
		interval_ms += SubFun::timeb2ms(rawtime);
		if (interval_ms - last_interval_ms > maxTime)
			maxTime = interval_ms - last_interval_ms;
		lastTime *= raw_count_it_sum;
		++raw_count_it;
		lastTime += (interval_ms - last_interval_ms) * raw_count_it;
		raw_count_it_sum += raw_count_it;
		lastTime /= raw_count_it_sum;
		last_interval_ms = interval_ms;

		if ((count_it + 1) == SubFun::iteration && interval_ms + maxTime > SubFun::limit_time)
			break;

	} while (interval_ms + lastTime < SubFun::limit_time);
	
	maxTime = 0;
	for (std::vector<BoolTable>::const_iterator it = new_population.cbegin(); 
		it != new_population.cend() && interval_ms + maxTime < SubFun::limit_time; it++)
	{
		FlowSolution temp = SmallestCostFlow::getSmallestCostFlow(*it, g);
		UIntTable servers;
		for (int i = 0; i < it->size(); i++)
			if (it->at(i))
				servers.push_back(i);
		temp.totalCost += g.costPerServer * servers.size();

		if (temp.isValid(g) && temp.totalCost < minCost.flowSolution.totalCost)
		{
			minCost.flowSolution = std::move(temp);
			minCost.servers = std::move(servers);
		}

		ftime(&rawtime);
		interval_ms += SubFun::timeb2ms(rawtime);
		if (interval_ms - last_interval_ms > maxTime)
			maxTime = interval_ms - last_interval_ms;
		last_interval_ms = interval_ms;
	}

	return std::move(minCost);
}

unsigned long GeneticAlgorithm::SubFun::timeb2ms(const timeb & t)
{
	static int ms = t.millitm;
	static unsigned long s = t.time;
	int out_ms = t.millitm - ms;
	unsigned long out_s = t.time - s;
	ms = t.millitm;
	s = t.time;

	return out_s * 1000 + out_ms;
}
