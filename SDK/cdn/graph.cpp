#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif // !_CRT_SECURE_NO_WARNINGS

#include "graph.h"
#include <iostream>
#include <sstream>

Graph::Graph(char * topo[MAX_EDGE_NUM])
{
	unsigned int nodeNum, edgeNum, needNodeNum;
	sscanf(topo[0], "%u %u %u", &nodeNum, &edgeNum, &needNodeNum);
	sscanf(topo[2], "%u", &(this->costPerServer));

	// 创建点集
	++nodeNum; // 多目的地转单目的地的结点
	for (unsigned int i(0); i != nodeNum; ++i)
	{
		nodes.push_back(std::shared_ptr<Node>(new Node(i)));
	}

	graphMatrix.insert(graphMatrix.begin(), nodeNum + 1,
		std::vector<std::pair<int, int>>(nodeNum + 1, std::make_pair(0, INT_MAX)));

	// 创建边集
	unsigned edgeCounter(-1);
	for (unsigned int i(4), j(i + edgeNum); i != j; ++i)
	{
		unsigned int node1, node2, flow, costPerFlow;
		sscanf(topo[i], "%u %u %u %u", &node1, &node2, &flow, &costPerFlow);

		Node *pNode1(nodes[node1].get()), *pNode2(nodes[node2].get());
		Edge *edge1(nullptr), *edge2(nullptr);

		edges.push_back(std::shared_ptr<Edge>(new Edge(flow, costPerFlow, pNode1, pNode2)));
		edge1 = edges.back().get();
		edges.push_back(std::shared_ptr<Edge>(new Edge(flow, costPerFlow, pNode2, pNode1)));
		edge2 = edges.back().get();
		
		edge1->reverseEdge = edge2;
		edge1->id = ++edgeCounter;
		edge2->reverseEdge = edge1;
		edge2->id = ++edgeCounter;
		pNode1->edges.insert(std::make_pair(node2, edge1));
		pNode2->edges.insert(std::make_pair(node1, edge2));

		graphMatrix[node1 + 1][node2 + 1].first = flow;
		graphMatrix[node1 + 1][node2 + 1].second = costPerFlow;
		graphMatrix[node2 + 1][node1 + 1].first = flow;
		graphMatrix[node2 + 1][node1 + 1].second = costPerFlow;
	}

	// 读取需求点集
	Node * edNode(nodes.back().get());
	for (unsigned int i(4 + edgeNum + 1), j(i + needNodeNum); i != j; ++i)
	{
		unsigned int order, node, need;
		sscanf(topo[i], "%u %u %u", &order, &node, &need);
		Node * thisNode(nodes[node].get());
		thisNode->isNeed = true;
		thisNode->needOrder = order;
		thisNode->need = need;

		needPoints.insert(node);

		// 将多目的地修正为单目的地
		Edge * thisEdge(nullptr);
		edges.push_back(std::shared_ptr<Edge>(new Edge(need, 0, thisNode, edNode)));
		thisEdge = edges.back().get();
		thisEdge->reverseEdge = nullptr;
		thisNode->edges.insert(std::make_pair(edNode->id, thisEdge));

		graphMatrix[thisNode->id + 1][nodeNum].first = need;
		graphMatrix[thisNode->id + 1][nodeNum].second = 0;

		needs[node] = need;
	}
}

std::vector<bool> Graph::getNodesBoolTable() const
{
	static BoolTable nodeBoolTable(nodes.size(), false);
	return nodeBoolTable;
}

UIntTable Graph::getNodesUIntTable() const
{
	static UIntTable nodesUIntTable(nodes.size(), 0);
	return nodesUIntTable;
}

IntTable Graph::getNodesIntTable() const
{
	static IntTable nodesIntTable(nodes.size(), 0);
	return nodesIntTable;
}

std::vector<bool> Graph::getEdgesBoolTable() const
{
	static BoolTable edgesBoolTable(edges.size(), false);
	return edgesBoolTable;
}

UInt2UIntTable Graph::getNeeds() const
{
	return needs;
}