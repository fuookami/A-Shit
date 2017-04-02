#pragma once

#include "lib_io.h"
#include <vector>
#include <utility>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <memory>

struct Edge;
struct Node;
struct Graph;

using BoolTable = std::vector<bool>;
using UIntTable = std::vector<unsigned int>;
using IntTable = std::vector<int>;

struct EdgeToNeedPointInfo
{
	unsigned int maxFlowToNeedPoint = 0;
	unsigned int minCostOfMaxFlowToNeedPoint = -1;
	bool flag = false;
};

using EdgePreTreatmentInfo = std::unordered_map<unsigned int, EdgeToNeedPointInfo>;

struct Edge
{
	Edge() {}
	Edge(unsigned int _flow, unsigned int _costPerFlow, Node * _node1, Node * _node2) :
		flow(_flow), costPerFlow(_costPerFlow), nodes(std::make_pair(_node1, _node2)) {}
	unsigned int id = 0;
	unsigned int flow = 0;
	int costPerFlow = 0;
	Edge * reverseEdge;
	std::pair<Node *, Node *> nodes = std::make_pair(nullptr, nullptr);

	EdgePreTreatmentInfo preTreatInfo;
};

struct NodeToNeedPointInfo
{
	unsigned int maxFlowToNeedPoint = 0;
	unsigned int minCostOfMaxFlowToNeedPoint = -1;

	unsigned int BFSDepth = 0;

	bool flag = true;
};

using NodePreTreatmentInfo = std::unordered_map<unsigned int, NodeToNeedPointInfo>;

struct Node
{
	Node() {}
	Node(unsigned int _id) : id(_id) {}
	unsigned int id = 0;
	std::unordered_map<unsigned int, Edge *> edges;
	
	bool isNeed = false;
	unsigned int needOrder = 0;
	unsigned int need = 0;

	NodePreTreatmentInfo preTreatInfo;
	// the score bigger, the node better
	// score = flow(bigger) * costPerFlow(smaller) * enabledNeedNodeNumber(bigger)
	// score = flow * flow / cost * enabledNodeNumber
	double preTreatScore = 0; 
};

struct Graph
{
	Graph() {}
	Graph(char * topo[MAX_EDGE_NUM]);
	BoolTable getNodesBoolTable() const;
	UIntTable getNodesUIntTable() const;
	IntTable getNodesIntTable() const;
	BoolTable getEdgesBoolTable() const;

	std::vector<std::shared_ptr<Node>> nodes;
	std::vector<std::shared_ptr<Edge>> edges;
	unsigned int costPerServer = 0;

	std::unordered_set<unsigned int> needPoints;
};