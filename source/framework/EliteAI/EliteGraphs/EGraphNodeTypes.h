/*=============================================================================*/
// Copyright 2020-2021 Elite Engine
// Authors: Yosha Vandaele
/*=============================================================================*/
// EGraphNodeTypes.h: Various node types for graphs
/*=============================================================================*/

#pragma once

#include "EGraphEnums.h"
#include "EliteGraphUtilities/EGraphVisuals.h"

namespace Elite
{
	class GraphNode
	{
	public:
		GraphNode() : m_Index(invalid_node_index) {}
		explicit GraphNode(int idx) : m_Index(idx) {}

		virtual ~GraphNode() = default;

		int GetIndex() const { return m_Index; }
		void SetIndex(int newIdx) { m_Index = newIdx; }

		bool operator==(const GraphNode& rhs) { return m_Index == rhs.m_Index; }
		
	protected:
		int m_Index;
	};

	class GridTerrainNode : public GraphNode
	{
	public:
		GridTerrainNode(int index)
			: GraphNode(index), m_Terrain(TerrainType::Ground)
		{
		}
		virtual ~GridTerrainNode() = default;


		TerrainType GetTerrainType() const { return m_Terrain; }
		void SetTerrainType(TerrainType terrain) { m_Terrain = terrain; }

	protected:
		TerrainType m_Terrain;
	};
}