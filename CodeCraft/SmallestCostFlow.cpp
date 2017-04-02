#include "SmallestCostFlow.h"

using namespace std;

FlowSolution SmallestCostFlow::getSmallestCostFlow(BoolTable servers, Graph & g)
{
	SubFun::initGraph(servers, g);
	SubFun::minCostMaxFlow();
	SubFun::removeSourceNode();
	return SubFun::flowSolution;
}

void SmallestCostFlow::SubFun::initGraph(BoolTable & server, Graph & g)
{
	minCost = 0;
	flowSolution.flows.clear();
	nodeSize = g.nodes.size();
	graph = &g;
	unsigned int startNodexIndex = -1;

	graph->nodes.insert(graph->nodes.begin(), shared_ptr<Node>(new Node(startNodexIndex)));
	Node &startNode(*graph->nodes[0].get());

	// insert the source node
	for (int i = 0; i < nodeSize; ++i)
	{
		if (server[i])
		{
			Edge *e = new Edge(INT_MAX, 0, &startNode, g.nodes[i].get());
			graph->edges.insert(graph->edges.begin(), shared_ptr<Edge>(e));
			startNode.edges.insert(make_pair(i, e));
		}
	}
	
	nodeSize++;
}

// find a path
void SmallestCostFlow::SubFun::dijkstra(std::vector<int> &distance, std::vector<int> &nextNode)
{
	distance.assign(nodeSize, INT_MAX);
	nextNode.assign(nodeSize, 0);
	distance[nodeSize - 1] = 0;
	nextNode[nodeSize - 1] = -1;

	for (vector<shared_ptr<Node>>::reverse_iterator nodeRit = graph->nodes.rbegin();
		nodeRit != graph->nodes.rend(); ++nodeRit)
	{
		for (unordered_map<unsigned int, Edge*>::iterator edgeIt = nodeRit->get()->edges.begin();
			edgeIt != nodeRit->get()->edges.end(); ++edgeIt)
		{
			if (distance[edgeIt->first + 1] > distance[nodeRit->get()->id + 1] +
				edgeIt->second->costPerFlow)
			{
				distance[edgeIt->first + 1] = distance[nodeRit->get()->id + 1] + edgeIt->second->costPerFlow;
				nextNode[edgeIt->first + 1] = nodeRit->get()->id + 1;
			}
		}
	}
}

void SmallestCostFlow::SubFun::minCostMaxFlow()
{
	Flow flow;
	// sum of cost per flow
	static std::vector<int> distance;
	// linked list for recording flow
	static std::vector<int> nextNode;

	while (true)
	{
		dijkstra(distance, nextNode);
		if (distance[0] == 0)
		{
			break;
		}
		int increase = INT_MAX;
		for (int i = 0; nextNode[i] != -1; i = nextNode[i])
		{
			if (increase > graph->nodes[i]->edges[nextNode[i]]->flow)
			{
				increase = graph->nodes[i]->edges[nextNode[i]]->flow;
			}
		}

		for (int i = 0; nextNode[i] != -1; i = nextNode[i])
		{
			graph->nodes[i]->edges[nextNode[i]]->flow -= increase;
			graph->nodes[nextNode[i]]->edges[i]->flow += increase;
			if (graph->nodes[nextNode[i]]->edges[i]->costPerFlow == INT_MAX)
			{
				graph->nodes[nextNode[i]]->edges[i]->costPerFlow =
					-graph->nodes[i]->edges[nextNode[i]]->costPerFlow;
			}
			if (graph->nodes[i]->edges[nextNode[i]] == 0)
			{
				graph->nodes[i]->edges[nextNode[i]]->costPerFlow = INT_MAX;
			}
			flow.edges.push_back(new Edge(increase,
				graph->nodes[i]->edges[nextNode[i]]->costPerFlow,
				graph->nodes[i].get(), graph->nodes[nextNode[i]].get()));
		}
		flow.edges.erase(flow.edges.begin());
		minCost += distance[0] * increase;
		flow.serverNodeId = nextNode[0];
		flow.totalCost = distance[0] * increase;
		flow.flow = increase;
	}
	flowSolution.totalCost = minCost;
	flowSolution.flows.push_back(flow);
}

void SmallestCostFlow::SubFun::removeSourceNode()
{
	while (graph->edges[0]->nodes.first->id == -1)
	{
		graph->edges.erase(graph->edges.begin());
	}
	graph->nodes.erase(graph->nodes.begin());
}
