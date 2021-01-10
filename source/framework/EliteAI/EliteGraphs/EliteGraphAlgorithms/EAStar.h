#pragma once
#include <set>

namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class AStar
	{
	public:
		AStar(GridGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

		// stores the optimal connection to a node and its total costs related to the start and end node of the path
		struct NodeRecord
		{
			T_NodeType* pNode = nullptr;
			T_ConnectionType* pConnection = nullptr;
			float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
			float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& pConnection == other.pConnection
					&& costSoFar == other.costSoFar
					&& estimatedTotalCost == other.estimatedTotalCost;
			};

			bool operator<(const NodeRecord& other) const
			{
				return estimatedTotalCost < other.estimatedTotalCost;
			};
		};

		using VisitedUMap = std::unordered_map<T_NodeType*, NodeRecord, size_t(*)(T_NodeType*)>;

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);

	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;
		NodeRecord GetClosestToDestination(T_NodeType* pStartNode, T_NodeType* pDestinationNode, const VisitedUMap& visited) const;

		GridGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;
	};

	template <class T_NodeType, class T_ConnectionType>
	AStar<T_NodeType, T_ConnectionType>::AStar(GridGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> AStar<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode)
	{
		std::vector<NodeRecord> openList;
		NodeRecord currentNodeRecord;

		auto hash = [](T_NodeType* node) { return std::hash<int>()(node->GetIndex()); };
		VisitedUMap closedList{ unsigned int(m_pGraph->GetNrOfNodes()), hash };
		vector<T_NodeType*> path{};
		bool pathFound = false;

		openList.push_back(NodeRecord{ pStartNode, nullptr, 0.f, GetHeuristicCost(pStartNode, pDestinationNode) });
		while (!openList.empty() && !pathFound)
		{
			//use partial sort with reverse operator to get the min_element at the back of the vector
			std::partial_sort(openList.rbegin(), openList.rbegin() + 1, openList.rend());
			currentNodeRecord = std::move(openList.back());
			openList.pop_back();
			closedList.emplace(currentNodeRecord.pNode, currentNodeRecord);

			if (currentNodeRecord.pNode != pDestinationNode)
			{
				auto connections{ m_pGraph->GetNodeConnections(currentNodeRecord.pNode->GetIndex()) };
				for (T_ConnectionType* conn : connections)
				{
					T_NodeType* nextNode{ m_pGraph->GetNode(conn->GetTo()) };
					float costSoFar{ currentNodeRecord.costSoFar + conn->GetCost() };
					if (closedList.find(nextNode) == closedList.end())
					{
						auto existingOpenRecord{ std::find_if(openList.begin(), openList.end(), [nextNode](const NodeRecord& nr) { return nr.pNode == nextNode; }) };
						if (existingOpenRecord == openList.end())
						{
							openList.push_back(NodeRecord{ nextNode, conn, costSoFar, costSoFar + GetHeuristicCost(nextNode, pDestinationNode) });
						}
						else if (existingOpenRecord->costSoFar > costSoFar)
						{
							(*existingOpenRecord) = NodeRecord{ nextNode, conn, costSoFar, costSoFar + GetHeuristicCost(nextNode, pDestinationNode) };
						}
					}
				}
			}
			else
				pathFound = true;
		}

		//If the path isn't find (inaccessible), look for the closest node from the end node
		currentNodeRecord = pathFound ? currentNodeRecord : GetClosestToDestination(pStartNode, pDestinationNode, closedList);
		while (currentNodeRecord.pNode != pStartNode)
		{
			path.push_back(currentNodeRecord.pNode);
			T_NodeType* fromNode{ m_pGraph->GetNode(currentNodeRecord.pConnection->GetFrom()) };
			currentNodeRecord = closedList[fromNode];
		}
		path.push_back(pStartNode);
		std::reverse(path.begin(), path.end());

		return path;
	}

	template <class T_NodeType, class T_ConnectionType>
	float Elite::AStar<T_NodeType, T_ConnectionType>::GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const
	{
		Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
		return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
	}

	template <typename T_NodeType, typename T_ConnectionType>
	typename AStar<T_NodeType, T_ConnectionType>::NodeRecord Elite::AStar<T_NodeType, T_ConnectionType>::GetClosestToDestination(T_NodeType* pStartNode, T_NodeType* pDestinationNode, const VisitedUMap& visited) const
	{
		if (visited.size() > 0)
		{
			auto lambdaClosest = [&](const std::pair<T_NodeType* const, NodeRecord>& nodePair1, const std::pair<T_NodeType* const, NodeRecord>& nodePair2)
			{
				float h1{ GetHeuristicCost(nodePair1.first, pDestinationNode) };
				float h2{ GetHeuristicCost(nodePair2.first, pDestinationNode) };
				return h1 < h2;
			};

			auto cIt = std::min_element(visited.cbegin(), visited.cend(), lambdaClosest);

			return (*cIt).second;
		}

		return NodeRecord{ pStartNode, nullptr, 0.f, GetHeuristicCost(pStartNode, pDestinationNode) };
	}
}