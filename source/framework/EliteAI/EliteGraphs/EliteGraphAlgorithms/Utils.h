#pragma once
#include "../EGridGraph.h"

namespace PathFindingUtils
{
	template<typename T_NodeType, typename T_ConnectionType>
	bool HasLineOfSight(const Elite::GridGraph<T_NodeType, T_ConnectionType>* pGridGraph, T_NodeType* pStartNode, T_NodeType* pTargetNode);

	template<typename T_NodeType, typename T_ConnectionType>
	std::vector<T_NodeType*> SmoothPath(const Elite::GridGraph<T_NodeType, T_ConnectionType>* pGridGraph, const std::vector<T_NodeType*>& path);
}

template<typename T_NodeType, typename T_ConnectionType>
bool PathFindingUtils::HasLineOfSight(const Elite::GridGraph<T_NodeType, T_ConnectionType>* pGridGraph, T_NodeType* pStartNode, T_NodeType* pTargetNode)
{
	//Determine if the targetNode is in line of sight of StartNode, if the stariught line traced from start to target doesn't intersect a wall
	//If no connection exist between 2 adjacent cells, it means the cell is a wall (isolated)
	//Based on Bresenham's line algorithm: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	//Acuuracy improved using Bresenham-based supercover line algorithm: http://eugen.dedu.free.fr/projects/bresenham/
	//The modified algorithm provide more accuracy as Bresenham's might miss cells when steping both horizontally and vertically
	Vector2 startPos{ pGridGraph->GetNodePos(pStartNode) };
	Vector2 endPos{ pGridGraph->GetNodePos(pTargetNode) };

	int x{ int(startPos.x) };
	int y{ int(startPos.y) };
	int dx{ int(endPos.x) - x };
	int dy{ int(endPos.y) - y };
	int stepX, stepY, error, errorprev, ddy, ddx;
	int prevX{ x }, prevY{ y };

	if (dy < 0) {
		stepY = -1;
		dy = -dy;
	}
	else
		stepY = 1;

	if (dx < 0) {
		stepX = -1;
		dx = -dx;
	}
	else
		stepX = 1;

	ddy = 2 * dy;
	ddx = 2 * dx;
	if (ddx >= ddy) {
		errorprev = error = dx;
		for (int i = 0; i < dx; i++) {
			x += stepX;
			error += ddy;
			if (error > ddx) {
				y += stepY;
				error -= ddx;
				if ((error + errorprev < ddy && pGridGraph->IsWithinBounds(x, y - stepY) && !pGridGraph->GetConnection(pGridGraph->GetIndex(prevX, prevY), pGridGraph->GetIndex(x, y - stepY)))
					|| (error + errorprev > ddy && pGridGraph->IsWithinBounds(x - stepX, y) && !pGridGraph->GetConnection(pGridGraph->GetIndex(prevX, prevY), pGridGraph->GetIndex(x - stepX, y))))
					return false;
			}

			if (pGridGraph->IsWithinBounds(x, y) && !pGridGraph->GetConnection(pGridGraph->GetIndex(prevX, prevY), pGridGraph->GetIndex(x, y)))
				return false;
			errorprev = error;
			prevX = x;
			prevY = y;
		}
	}
	else {
		errorprev = error = dy;
		for (int i = 0; i < dy; i++) {
			y += stepY;
			error += ddx;
			if (error > ddy) {
				x += stepX;
				error -= ddy;
				if ((error + errorprev < ddy && pGridGraph->IsWithinBounds(x - stepX, y) && !pGridGraph->GetConnection(pGridGraph->GetIndex(prevX, prevY), pGridGraph->GetIndex(x - stepX, y)))
					|| (error + errorprev > ddy && pGridGraph->IsWithinBounds(x, y - stepY) && !pGridGraph->GetConnection(pGridGraph->GetIndex(prevX, prevY), pGridGraph->GetIndex(x, y - stepY))))
					return false;
			}

			if (pGridGraph->IsWithinBounds(x, y) && !pGridGraph->GetConnection(pGridGraph->GetIndex(prevX, prevY), pGridGraph->GetIndex(x, y)))
				return false;
			errorprev = error;
			prevX = x;
			prevY = y;
		}
	}

	return true;
}

template<typename T_NodeType, typename T_ConnectionType>
std::vector<T_NodeType*> PathFindingUtils::SmoothPath(const Elite::GridGraph<T_NodeType, T_ConnectionType>* pGridGraph, const std::vector<T_NodeType*>& path)
{
	if (path.empty())
		return path;

	std::vector<T_NodeType*> smoothedPath{};
	size_t anchorIdx{ 0 };
	smoothedPath.push_back(path[anchorIdx]);

	size_t pathSize{ path.size() };
	for (size_t idx{ 1 }; idx < pathSize;)
	{
		if (!HasLineOfSight(pGridGraph, path[anchorIdx], path[idx]))
		{
			anchorIdx = idx - 1;
			smoothedPath.push_back(path[anchorIdx]);
		}

		++idx;
	}
	smoothedPath.push_back(path[pathSize - 1]);

	return smoothedPath;
}