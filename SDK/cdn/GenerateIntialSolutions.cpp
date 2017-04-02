#include "GenerateIntialSolutions.h"
#include <deque>
#include <algorithm>
#include <random>

std::vector<BoolTable> GenerateIntialSolutions::getIntialSolutions(Graph & g)
{
	SubFun::pretreat(g);

	return std::move(SubFun::generateIntialSolutions(g));
}

void GenerateIntialSolutions::SubFun::pretreat(Graph & g)
{
	for (std::unordered_set<unsigned int>::const_iterator currIt(g.needPoints.cbegin()),
		edIt(g.needPoints.cend()); currIt != edIt; ++currIt)
		BFSPretreatFrom(g.nodes[*currIt].get(), g);

	for (std::vector<std::shared_ptr<Node>>::iterator currIt(g.nodes.begin()), edIt(g.nodes.end() - 1);
		currIt != edIt; ++currIt)
		currIt->get()->preTreatScore = calNodeScore(currIt->get());
}

void GenerateIntialSolutions::SubFun::BFSPretreatFrom(Node *bgNode, Graph &g)
{
	unsigned int bgNodeId(bgNode->id);
	unsigned int edNodeId(g.nodes.size() - 1);
	unsigned int maxNeed(bgNode->need);
	UIntTable nodeDepth(g.getNodesUIntTable());
	IntTable flowDistance(g.getNodesIntTable());

	nodeDepth[bgNodeId] = 2;
	nodeDepth[edNodeId] = 1;
	bgNode->preTreatInfo.insert(std::make_pair(bgNodeId, NodeToNeedPointInfo()));
	NodeToNeedPointInfo &bgNodePreTreatInfo(bgNode->preTreatInfo.find(bgNodeId)->second);
	bgNodePreTreatInfo.maxFlowToNeedPoint = maxNeed;
	bgNodePreTreatInfo.minCostOfMaxFlowToNeedPoint = 0;

	std::deque<Node *> nodes;
	unsigned int flowIn(0);
	for (std::unordered_map<unsigned int, Edge *>::const_iterator currIt(bgNode->edges.cbegin()),
		edIt(bgNode->edges.cend()); currIt != edIt; ++currIt)
	{
		Edge *reverseEdge(currIt->second->reverseEdge);
		if (reverseEdge == nullptr)
			continue;

		Node *anoNode(currIt->second->nodes.second);
		nodeDepth[anoNode->id] = 3;
		nodes.push_back(anoNode);
		
		reverseEdge->preTreatInfo.insert(std::make_pair(bgNodeId, EdgeToNeedPointInfo()));
		EdgeToNeedPointInfo &reverseEdgeToNeedPointInfo(reverseEdge->preTreatInfo.find(bgNodeId)->second);

		reverseEdgeToNeedPointInfo.flag = true;
		reverseEdgeToNeedPointInfo.maxFlowToNeedPoint = maxNeed > reverseEdge->flow ? reverseEdge->flow : maxNeed;
		reverseEdgeToNeedPointInfo.minCostOfMaxFlowToNeedPoint =
			reverseEdgeToNeedPointInfo.maxFlowToNeedPoint * reverseEdge->costPerFlow;

		anoNode->preTreatInfo.insert(std::make_pair(bgNodeId, NodeToNeedPointInfo()));
		NodeToNeedPointInfo &anoNodeToNeedPointInfo(anoNode->preTreatInfo.find(bgNodeId)->second);
		anoNodeToNeedPointInfo.BFSDepth = 3;
		anoNodeToNeedPointInfo.maxFlowToNeedPoint = reverseEdgeToNeedPointInfo.maxFlowToNeedPoint;
		anoNodeToNeedPointInfo.minCostOfMaxFlowToNeedPoint = reverseEdgeToNeedPointInfo.minCostOfMaxFlowToNeedPoint;

		flowIn += reverseEdgeToNeedPointInfo.maxFlowToNeedPoint;
	}
	flowDistance[bgNodeId] = flowIn;
	
	unsigned int depth(3);
	while (!nodes.empty())
	{
		unsigned int thisDepth(nodes.size());

		/******************************************************************************/
		/* First Step:                                                                */
		/* Calculate the flow distance and find, mark and add the nodes in next depth */
		/******************************************************************************/
		for (unsigned int i(0); i != thisDepth; ++i)
		{
			Node &currNode(*nodes[i]);
			NodeToNeedPointInfo &currNodeToNeedPointInfo(currNode.preTreatInfo.find(bgNodeId)->second);

			unsigned int flowIn(0);
			for (std::unordered_map<unsigned int, Edge *>::const_iterator currIt(currNode.edges.cbegin()),
				edIt(currNode.edges.cend()); currIt != edIt; ++currIt)
			{
				Node *anoNode(currIt->second->nodes.second);
				if (nodeDepth[anoNode->id] == 0)
				{
					// unvisited next depth point
					// mark and add it
					nodeDepth[anoNode->id] = depth + 1;
					nodes.push_back(anoNode);

					anoNode->preTreatInfo.insert(std::make_pair(bgNodeId, NodeToNeedPointInfo()));
					NodeToNeedPointInfo &anoNodeToNeedPointInfo(anoNode->preTreatInfo.find(bgNodeId)->second);
					anoNodeToNeedPointInfo.BFSDepth = depth + 1;
				}

				if (nodeDepth[anoNode->id] > depth)
				{
					// visited next depth point
					// set its reverse edge enabled
					Edge *reverseEdge(currIt->second->reverseEdge);
					reverseEdge->preTreatInfo.insert(std::make_pair(bgNodeId, EdgeToNeedPointInfo()));
					EdgeToNeedPointInfo &reverseEdgeToNeedPointInfo(reverseEdge->preTreatInfo.find(bgNodeId)->second);
					reverseEdgeToNeedPointInfo.flag = true;

					// cal the flow in
					flowIn += reverseEdge->flow;
				}
			}
			// FlowOut is same as the nodeToNeedPointInfo.maxFlowToNeedPoint
			flowDistance[currNode.id] = flowIn - currNodeToNeedPointInfo.maxFlowToNeedPoint;
		}

		/************************************************************************/
		/* Second Step:                                                         */
		/* Judge the edges is needed between same depth                         */
		/************************************************************************/
		for (unsigned int i(0); i != thisDepth; ++i)
		{
			Node &currNode(*nodes[i]);
			NodeToNeedPointInfo &currNodeToNeedPointInfo(currNode.preTreatInfo.find(bgNodeId)->second);

			if (flowDistance[currNode.id] < 0)
			{
				for (std::unordered_map<unsigned int, Edge *>::const_iterator currIt(currNode.edges.cbegin()),
					edIt(currNode.edges.cend()); currIt != edIt; ++currIt)
				{
					Node *anoNode(currIt->second->nodes.second);
					if (nodeDepth[anoNode->id] == depth && flowDistance[anoNode->id] > 0)
					{
						Edge *reverseEdge(currIt->second->reverseEdge);
						reverseEdge->preTreatInfo.insert(std::make_pair(bgNodeId, EdgeToNeedPointInfo()));
						EdgeToNeedPointInfo &reverseEdgeToNeedPointInfo(reverseEdge->preTreatInfo.find(bgNodeId)->second);

						unsigned int flowDistanceUInt(-flowDistance[currNode.id]);
						reverseEdgeToNeedPointInfo.flag = true;
						reverseEdgeToNeedPointInfo.maxFlowToNeedPoint = std::min(std::initializer_list<unsigned int>
							{flowDistanceUInt, reverseEdge->flow, (unsigned int)flowDistance[anoNode->id]});
						reverseEdgeToNeedPointInfo.minCostOfMaxFlowToNeedPoint =
							currNodeToNeedPointInfo.minCostOfMaxFlowToNeedPoint
							+ reverseEdgeToNeedPointInfo.maxFlowToNeedPoint * reverseEdge->costPerFlow;
					}
				}
			}
		}

		for (unsigned int i(0); i != thisDepth; ++i)
		{
			Node &currNode(*nodes[i]);
			NodeToNeedPointInfo &currNodeToNeedPointInfo(currNode.preTreatInfo.find(bgNodeId)->second);

			if (flowDistance[currNode.id] > 0)
			{
				std::vector<Edge *> flowOutEdgeOfSameDepth;
				for (std::unordered_map<unsigned int, Edge *>::const_iterator currIt(currNode.edges.cbegin()),
					edIt(currNode.edges.cend()); currIt != edIt; ++currIt)
				{
					Node *anoNode(currIt->second->nodes.second);

					if (nodeDepth[anoNode->id] == depth && flowDistance[anoNode->id] < 0)
					{
						EdgeToNeedPointInfo &thisEdgeToNeedPointInfo(currIt->second->preTreatInfo.find(bgNodeId)->second);
						if (thisEdgeToNeedPointInfo.flag)
							flowOutEdgeOfSameDepth.push_back(currIt->second);
					}
				}

				if (!flowOutEdgeOfSameDepth.empty())
				{
					std::sort(flowOutEdgeOfSameDepth.begin(), flowOutEdgeOfSameDepth.end(),
						[bgNodeId](const Edge * const pEdgeA, const Edge * const pEdgeB)->bool
					{
						return pEdgeA->preTreatInfo.find(bgNodeId)->second.minCostOfMaxFlowToNeedPoint
							< pEdgeB->preTreatInfo.find(bgNodeId)->second.minCostOfMaxFlowToNeedPoint;
					});

					unsigned int restFlow(flowDistance[currNode.id]), addedCost(0);
					for (std::vector<Edge *>::const_iterator currIt(flowOutEdgeOfSameDepth.cbegin()),
						edIt(flowOutEdgeOfSameDepth.cend()); currIt != edIt; ++currIt)
					{
						Edge *thisEdge(*currIt);
						EdgeToNeedPointInfo &thisEdgeToNeedPointInfo(thisEdge->preTreatInfo.find(bgNodeId)->second);

						unsigned int usedFlow(0);
						if (restFlow > thisEdgeToNeedPointInfo.maxFlowToNeedPoint)
						{
							usedFlow = thisEdgeToNeedPointInfo.maxFlowToNeedPoint;
							restFlow -= usedFlow;
							addedCost += thisEdge->costPerFlow * usedFlow;
						}
						else 
						{
							usedFlow = restFlow;
							restFlow = 0;
							addedCost += thisEdge->costPerFlow * usedFlow;
							break;
						}
					}
					currNodeToNeedPointInfo.maxFlowToNeedPoint += flowDistance[currNode.id] - restFlow;
					currNodeToNeedPointInfo.minCostOfMaxFlowToNeedPoint += addedCost;
				}
			}
		}

		/************************************************************************/
		/* Third Step:                                                          */
		/* Judge the reverse edges                                              */
		/************************************************************************/
		for (unsigned int i(0); i != thisDepth; ++i)
		{
			Node &currNode(*nodes[i]);
			NodeToNeedPointInfo &currNodeToNeedPointInfo(currNode.preTreatInfo.find(bgNodeId)->second);

			for (std::unordered_map<unsigned int, Edge *>::const_iterator currIt(currNode.edges.cbegin()),
				edIt(currNode.edges.cend()); currIt != edIt; ++currIt)
			{
				Node *anoNode(currIt->second->nodes.second);

				if (nodeDepth[anoNode->id] > depth)
				{
					Edge *reverseEdge(currIt->second->reverseEdge);
					EdgeToNeedPointInfo &reverseEdgeToNeedPointInfo(reverseEdge->preTreatInfo.find(bgNodeId)->second);

					unsigned int flow = currNodeToNeedPointInfo.maxFlowToNeedPoint > reverseEdge->flow ? 
						reverseEdge->flow : currNodeToNeedPointInfo.maxFlowToNeedPoint;
					unsigned int cost = currNodeToNeedPointInfo.minCostOfMaxFlowToNeedPoint * flow /
						currNodeToNeedPointInfo.maxFlowToNeedPoint + flow * reverseEdge->costPerFlow;

					reverseEdgeToNeedPointInfo.maxFlowToNeedPoint = flow;
					reverseEdgeToNeedPointInfo.minCostOfMaxFlowToNeedPoint = cost;
				}
			}
		}

		for (unsigned int i(thisDepth), j(nodes.size()); i != j; ++i)
		{
			Node &currNode(*nodes[i]);
			NodeToNeedPointInfo &currNodeToNeedPointInfo(currNode.preTreatInfo.find(bgNodeId)->second);
			std::vector<Edge *> edges;

			for (std::unordered_map<unsigned int, Edge *>::const_iterator currIt(currNode.edges.cbegin()),
				edIt(currNode.edges.cend()); currIt != edIt; ++currIt)
			{
				Node *anoNode(currIt->second->nodes.second);

				if (nodeDepth[anoNode->id] == depth)
					edges.push_back(currIt->second);
			}

			std::sort(edges.begin(), edges.end(), [bgNodeId](const Edge * const pEdgeA, const Edge * const pEdgeB)
			{
				const EdgeToNeedPointInfo &edgeAToNeedPointInfo(pEdgeA->preTreatInfo.find(bgNodeId)->second);
				const EdgeToNeedPointInfo &edgeBToNeedPointInfo(pEdgeB->preTreatInfo.find(bgNodeId)->second);
				return edgeAToNeedPointInfo.minCostOfMaxFlowToNeedPoint / edgeAToNeedPointInfo.maxFlowToNeedPoint <
					edgeBToNeedPointInfo.minCostOfMaxFlowToNeedPoint / edgeBToNeedPointInfo.maxFlowToNeedPoint;
			});

			unsigned int restFlow(maxNeed), totalCost(0);
			for (unsigned int i(0), j(edges.size()); i != j; ++i)
			{
				const EdgeToNeedPointInfo &edgeToNeedPointInfo(edges[i]->preTreatInfo.find(bgNodeId)->second);
				if (restFlow < edgeToNeedPointInfo.maxFlowToNeedPoint)
				{
					totalCost += edgeToNeedPointInfo.minCostOfMaxFlowToNeedPoint 
						* restFlow / edgeToNeedPointInfo.maxFlowToNeedPoint;
					restFlow = 0;
					break;
				}
				totalCost += edgeToNeedPointInfo.minCostOfMaxFlowToNeedPoint;
				restFlow -= edgeToNeedPointInfo.maxFlowToNeedPoint;
			}
			currNodeToNeedPointInfo.maxFlowToNeedPoint = maxNeed - restFlow;
			currNodeToNeedPointInfo.minCostOfMaxFlowToNeedPoint = totalCost;
		}

		nodes.erase(nodes.begin(), nodes.begin() + thisDepth);
		++depth;
	}

	for (std::vector<std::shared_ptr<Node>>::iterator currIt(g.nodes.begin()), edIt(g.nodes.end() - 1);
		currIt != edIt; ++currIt)
	{
		NodeToNeedPointInfo &currNodeToNeedPointInfo((*currIt)->preTreatInfo.find(bgNodeId)->second);

		if (currNodeToNeedPointInfo.minCostOfMaxFlowToNeedPoint >= g.costPerServer)
			currNodeToNeedPointInfo.flag = false;
	}
}

double GenerateIntialSolutions::SubFun::calNodeScore(const Node * const pNode)
{
	const NodePreTreatmentInfo &preTreatmentInfo(pNode->preTreatInfo);

	unsigned int enabledNodeNumber(0);
	double score(0);
	for (std::unordered_map<unsigned int, NodeToNeedPointInfo>::const_iterator currIt(preTreatmentInfo.cbegin()),
		edIt(preTreatmentInfo.cend()); currIt != edIt; ++currIt)
	{
		const NodeToNeedPointInfo &nodeToNeedPointInfo(currIt->second);
		if (nodeToNeedPointInfo.flag)
		{
			++enabledNodeNumber;
			if (nodeToNeedPointInfo.minCostOfMaxFlowToNeedPoint != 0)
				score += pow(nodeToNeedPointInfo.maxFlowToNeedPoint, 2)
					/ nodeToNeedPointInfo.minCostOfMaxFlowToNeedPoint;
		}
	}

	return score * enabledNodeNumber;
}

std::vector<BoolTable> GenerateIntialSolutions::SubFun::generateIntialSolutions(const Graph & g)
{
	std::vector<Node *> nodes(g.nodes.size() - 1, nullptr);
	for (unsigned int i(0), j(nodes.size()); i != j; ++i)
		nodes[i] = g.nodes[i].get();

	std::sort(nodes.begin(), nodes.end(), [](const Node * const pNodeA, const Node * const pNodeB)
	{
		return pNodeA->preTreatScore > pNodeB->preTreatScore;
	});

	return std::move(selectServers(nodes, g));
}

std::vector<BoolTable> GenerateIntialSolutions::SubFun::selectServers(const std::vector<Node *> &nodes, const Graph &g)
{
	std::vector<std::pair<BoolTable, double>> intialSolutionWithScore;
	unsigned int targetSolutionNumber(solutionNum * g.nodes.size() / g.needPoints.size());

	while (intialSolutionWithScore.size() != targetSolutionNumber)
	{
		BoolTable solution(generateServerCombination(nodes, g));
		double score(calSolutionCost(solution, g));

		bool solutionIsExisted(std::find_if(intialSolutionWithScore.cbegin(),
			intialSolutionWithScore.cend(), [solution](const std::pair<BoolTable, double> &solutionPair)->bool
		{
			return solutionPair.first == solution;
		}) != intialSolutionWithScore.cend());
		if (!solutionIsExisted)
			intialSolutionWithScore.push_back(std::make_pair(std::move(solution), score));
	}

	std::sort(intialSolutionWithScore.begin(), intialSolutionWithScore.end(),
		[](const std::pair<BoolTable, double> &solutionA, const std::pair<BoolTable, double> &solutionB)->bool
	{
		return solutionA.second < solutionB.second;
	});

	std::vector<BoolTable> intialSolutions;
	for (unsigned int i(0); i != solutionNum; ++i)
	{
		intialSolutions.push_back(std::move(intialSolutionWithScore[i].first));
	}
	return std::move(intialSolutions);
}

BoolTable GenerateIntialSolutions::SubFun::generateServerCombination(const std::vector<Node*>& nodes, const Graph &g)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::weibull_distribution<> dis(1, g.needPoints.size());
	static auto judgeNeedIsSatisfied([](const std::unordered_map<unsigned int, unsigned int> &flowTable)->bool
	{
		for (std::unordered_map<unsigned int, unsigned int>::const_iterator currIt(flowTable.cbegin()), edIt(flowTable.cend());
			currIt != edIt; ++currIt)
		{
			if (currIt->second != 0)
				return false;
		}
		return true;
	});

	static auto judgeNodeIsNeeded([&g](const std::unordered_map<unsigned int, unsigned int> &flowTable, const Node * const pNode)->bool
	{
		for (std::unordered_map<unsigned int, unsigned int>::const_iterator currIt(flowTable.cbegin()), edIt(flowTable.cend());
			currIt != edIt; ++currIt)
		{
			if (currIt->second != 0 && pNode->preTreatInfo.find(currIt->first)->second.flag)
				return true;
		}
		return false;
	});

	std::unordered_map<unsigned int, unsigned int> flowTable;
	for (std::unordered_set<unsigned int>::const_iterator currIt(g.needPoints.begin()), edIt(g.needPoints.end());
		currIt != edIt; ++currIt)
		flowTable.insert(std::make_pair(*currIt, g.nodes[*currIt].get()->need));
	BoolTable solution(g.getNodesBoolTable());

	for(;;)
	{
		unsigned int thisNode(std::round(dis(gen)));
		if (thisNode > 0 && thisNode <= nodes.size())
		{
			const Node * const currNode(nodes[thisNode - 1]);
			thisNode = currNode->id;

			if (!solution[thisNode] && judgeNodeIsNeeded(flowTable, currNode))
			{
				solution[thisNode] = true;

				for (std::unordered_map<unsigned int, NodeToNeedPointInfo>::const_iterator currIt(currNode->preTreatInfo.cbegin()),
					edIt(currNode->preTreatInfo.cend()); currIt != edIt; ++currIt)
				{
					if (currIt->second.flag)
					{
						if (flowTable.find(currIt->first)->second > currIt->second.maxFlowToNeedPoint)
							flowTable.find(currIt->first)->second -= currIt->second.maxFlowToNeedPoint;
						else
							flowTable.find(currIt->first)->second = 0;
					}
				}

				if (judgeNeedIsSatisfied(flowTable))
					break;
			}
		}
	}

	return std::move(solution);
}

double GenerateIntialSolutions::SubFun::calSolutionCost(BoolTable & currSolution, const Graph & g)
{
	double cost(0);

	std::unordered_map<unsigned int, std::vector<std::pair<unsigned int, const NodeToNeedPointInfo *>>> costInfos;
	std::unordered_map<unsigned int, bool> flowUsed;

	for (std::unordered_set<unsigned int>::const_iterator currIt(g.needPoints.begin()), edIt(g.needPoints.end());
		currIt != edIt; ++currIt)
		costInfos.insert(std::make_pair(*currIt, std::vector<std::pair<unsigned int, const NodeToNeedPointInfo *>>()));

	for (unsigned int i(0), j(currSolution.size()); i != j; ++i)
	{
		if (currSolution[i])
		{
			const Node * const currNode(g.nodes[i].get());
			flowUsed.insert(std::make_pair(currNode->id, false));
			for (std::unordered_map<unsigned int, NodeToNeedPointInfo>::const_iterator currIt(currNode->preTreatInfo.cbegin()),
				edIt(currNode->preTreatInfo.cend()); currIt != edIt; ++currIt)
			{
				costInfos.find(currIt->first)->second.push_back(std::make_pair(currNode->id, &(currIt->second)));
			}
		}
	}
	
	for (std::unordered_map<unsigned int, std::vector<std::pair<unsigned int, const NodeToNeedPointInfo *>>>::iterator
		currIt(costInfos.begin()), edIt(costInfos.end()); currIt != edIt; ++currIt)
	{
		unsigned int currNeedPointId(currIt->first);
		unsigned int need(g.nodes[currNeedPointId]->need);
		std::vector<std::pair<unsigned int, const NodeToNeedPointInfo *>> &costInfos(currIt->second);

		std::sort(costInfos.begin(), costInfos.end(),
			[](const std::pair<unsigned int, const NodeToNeedPointInfo *> &costInfoA, 
				const std::pair<unsigned int, const NodeToNeedPointInfo *> &costInfoB) -> bool
		{
			const NodeToNeedPointInfo * const infoA(costInfoA.second);
			const NodeToNeedPointInfo * const infoB(costInfoB.second);
			return infoA->flag &&
				(double)infoA->minCostOfMaxFlowToNeedPoint / infoA->maxFlowToNeedPoint <
				(double)infoB->minCostOfMaxFlowToNeedPoint / infoB->maxFlowToNeedPoint;
		});

		for (unsigned int i(0), j(costInfos.size()); i != j; ++i)
		{
			const NodeToNeedPointInfo * const currInfo(costInfos[i].second);
			flowUsed.find(costInfos[i].first)->second = true;

			if (need > currInfo->maxFlowToNeedPoint)
			{
				need -= currInfo->maxFlowToNeedPoint;
				cost += currInfo->minCostOfMaxFlowToNeedPoint;
			}
			else 
			{
				cost += need * (double)currInfo->minCostOfMaxFlowToNeedPoint / currInfo->maxFlowToNeedPoint;
				need = 0;
				break;
			}
		}
	}
	
	unsigned int serverNum(0);
	for (std::unordered_map<unsigned int, bool>::const_iterator currIt(flowUsed.cbegin()),
		edIt(flowUsed.cend()); currIt != edIt; ++currIt)
	{
		if (currIt->second)
			++serverNum;
		else
			currSolution[currIt->first] = false;
	}

	return cost + serverNum * g.costPerServer;
}