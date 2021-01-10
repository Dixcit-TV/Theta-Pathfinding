//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "App_Pathfinding.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAstar.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\ThetaStar.h"
#include <numeric>

using namespace Elite;

//Destructor
App_Pathfinding::~App_Pathfinding()
{
	SAFE_DELETE(m_pGridGraph);
}

//Functions
void App_Pathfinding::Start()
{
	//Set Camera
	DEBUGRENDERER2D->GetActiveCamera()->SetZoom(39.0f);
	DEBUGRENDERER2D->GetActiveCamera()->SetCenter(Elite::Vector2(73.0f, 35.0f));

	//Create Graph
	MakeGridGraph();

	startPathIdx = 0;
	endPathIdx = 4;
}

void App_Pathfinding::Update(float deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);

	//INPUT
	bool const middleMousePressed = INPUTMANAGER->IsMouseButtonUp(InputMouseButton::eMiddle);
	if (middleMousePressed)
	{
		MouseData mouseData = { INPUTMANAGER->GetMouseData(Elite::InputType::eMouseButton, Elite::InputMouseButton::eMiddle) };
		Elite::Vector2 mousePos = DEBUGRENDERER2D->GetActiveCamera()->ConvertScreenToWorld({ (float)mouseData.X, (float)mouseData.Y });

		//Find closest node to click pos
		int closestNode = m_pGridGraph->GetNodeFromWorldPos(mousePos);
		if (m_StartSelected)
		{
			startPathIdx = closestNode;
			m_UpdatePath = true;
		}
		else
		{
			endPathIdx = closestNode;
			m_UpdatePath = true;
		}
	}

	//GRID INPUT
	bool hasGridChanged = m_GraphEditor.UpdateGraph(m_pGridGraph);
	if (hasGridChanged)
	{
		m_UpdatePath = true;
	}

	//IMGUI
	UpdateImGui();


	//CALCULATEPATH
	//If we have nodes and the target is not the startNode, find a path!
	if (m_UpdatePath
		&& startPathIdx != invalid_node_index
		&& endPathIdx != invalid_node_index
		&& startPathIdx != endPathIdx)
	{
		const int iterations{ 100 };
		std::chrono::high_resolution_clock::time_point t1, t2;
		float min = FLT_MAX, max = 0.f, total = 0.f, duration;

		auto startNode = m_pGridGraph->GetNode(startPathIdx);
		auto endNode = m_pGridGraph->GetNode(endPathIdx);

		if (m_PathFindingAlgo != PathfindingAlgorithm::THETASTAR)
		{
			auto pathfinder = AStar<GridTerrainNode, GraphConnection>(m_pGridGraph, m_pHeuristicFunction);

			min = FLT_MAX;
			max = 0.f;
			total = 0.f;
			duration = 0.f;

			if (m_SmoothAstar) //Take smoothing into account for benchmarking
			{
				for (int count{}; count < iterations; ++count)
				{
					t1 = std::chrono::high_resolution_clock::now();
					auto path = pathfinder.FindPath(startNode, endNode);
					path = PathFindingUtils::SmoothPath(m_pGridGraph, path);
					t2 = std::chrono::high_resolution_clock::now();
					duration = std::chrono::duration<float>(t2 - t1).count();

					if (duration < min)
						min = duration;

					if (duration > max)
						max = duration;

					total += duration;
				}

				m_vPathAStar = pathfinder.FindPath(startNode, endNode);
				m_vPathAStar = PathFindingUtils::SmoothPath(m_pGridGraph, m_vPathAStar);
			}
			else
			{
				for (int count{}; count < iterations; ++count)
				{
					t1 = std::chrono::high_resolution_clock::now();
					auto path = pathfinder.FindPath(startNode, endNode);
					t2 = std::chrono::high_resolution_clock::now();
					duration = std::chrono::duration<float>(t2 - t1).count();

					if (duration < min)
						min = duration;

					if (duration > max)
						max = duration;

					total += duration;
				}

				m_vPathAStar = pathfinder.FindPath(startNode, endNode);
			}

			m_ExecutionTimeAStar = (total - min - max) / (iterations - 2);

			size_t pathSize{ m_vPathAStar.size() };
			m_PathLengthAStar = 0.f;
			for (size_t idx{ 1 }; idx < pathSize; ++idx)
			{
				m_PathLengthAStar += Elite::Distance(m_pGridGraph->GetNodeWorldPos(m_vPathAStar[idx-1]), m_pGridGraph->GetNodeWorldPos(m_vPathAStar[idx]));
			}
		}

		if (m_PathFindingAlgo != PathfindingAlgorithm::ASTAR)
		{
			auto pathfinder = ThetaStar<GridTerrainNode, GraphConnection>(m_pGridGraph, m_pHeuristicFunction);

			min = FLT_MAX;
			max = 0.f;
			total = 0.f;
			duration = 0.f;
			for (int count{}; count < iterations; ++count) 
			{
				t1 = std::chrono::high_resolution_clock::now();
				pathfinder.FindPath(startNode, endNode);
				t2 = std::chrono::high_resolution_clock::now();
				duration = std::chrono::duration<float>(t2 - t1).count();

				if (duration < min)
					min = duration;

				if (duration > max)
					max = duration;

				total += duration;
			}

			m_ExecutionTimeThetaStar = (total - min - max) / (iterations - 2);
			m_vPathThetaStar = pathfinder.FindPath(startNode, endNode);

			size_t pathSize{ m_vPathThetaStar.size() };
			m_PathLengthThetaStar = 0.f;
			for (size_t idx{ 1 }; idx < pathSize; ++idx)
			{
				m_PathLengthThetaStar += Elite::Distance(m_pGridGraph->GetNodeWorldPos(m_vPathThetaStar[idx - 1]), m_pGridGraph->GetNodeWorldPos(m_vPathThetaStar[idx]));
			}
		}

		m_UpdatePath = false;
		std::cout << "New Path Calculated" << std::endl;
	}
}

void App_Pathfinding::Render(float deltaTime) const
{
	UNREFERENCED_PARAMETER(deltaTime);
	//Render grid
	m_GraphRenderer.RenderGraph(
		m_pGridGraph, 
		m_bDrawGrid, 
		false, 
		false,
		false
	);
	
	//Render start node on top if applicable
	if (startPathIdx != invalid_node_index)
	{
		m_GraphRenderer.RenderHighlightedGrid(m_pGridGraph, { m_pGridGraph->GetNode(startPathIdx) }, START_NODE_COLOR);
	}

	//Render end node on top if applicable
	if (endPathIdx != invalid_node_index)
	{
		m_GraphRenderer.RenderHighlightedGrid(m_pGridGraph, { m_pGridGraph->GetNode(endPathIdx) }, END_NODE_COLOR);
	}
	
	//render path below if applicable
	if (m_PathFindingAlgo != PathfindingAlgorithm::ASTAR && m_vPathThetaStar.size() > 0)
	{
		Elite::Color pathColor{ 0.f, 1.f, 0.f };
		m_GraphRenderer.RenderHighlightedGrid(m_pGridGraph, m_vPathThetaStar, pathColor);

		size_t pathSize{ m_vPathThetaStar.size() };
		for (size_t idx{ 1 }; idx < pathSize; ++idx)
		{
			DEBUGRENDERER2D->DrawSegment(m_pGridGraph->GetNodeWorldPos(m_vPathThetaStar[idx - 1]), m_pGridGraph->GetNodeWorldPos(m_vPathThetaStar[idx]), pathColor, 0.f);
		}
	}

	if (m_PathFindingAlgo != PathfindingAlgorithm::THETASTAR && m_vPathAStar.size() > 0)
	{
		Elite::Color pathColor{ 0.f, 0.f, 1.f };
		m_GraphRenderer.RenderHighlightedGrid(m_pGridGraph, m_vPathAStar, pathColor);

		size_t pathSize{ m_vPathAStar.size() };
		for (size_t idx{ 1 }; idx < pathSize; ++idx)
		{
			DEBUGRENDERER2D->DrawSegment(m_pGridGraph->GetNodeWorldPos(m_vPathAStar[idx - 1]), m_pGridGraph->GetNodeWorldPos(m_vPathAStar[idx]), pathColor, 0.f);
		}
	}
}

void App_Pathfinding::MakeGridGraph()
{
	m_pGridGraph = new GridGraph<GridTerrainNode, GraphConnection>(COLUMNS, ROWS, m_SizeCell, false, true, 1.f, 1.5f);
}

void App_Pathfinding::UpdateImGui()
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		int menuWidth = 150;
		int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
		int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
		bool windowActive = true;
		ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
		ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::PushAllowKeyboardFocus(false);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: target");
		ImGui::Text("RMB: start");
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing();ImGui::Separator();ImGui::Spacing();ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("Pathfinding");
		ImGui::Spacing();

		ImGui::Text("Middle Mouse");
		ImGui::Text("controls");
		std::string buttonText{""};
		if (m_StartSelected)
			buttonText += "Start Node";
		else
			buttonText += "End Node";

		if (ImGui::Button(buttonText.c_str()))
		{
			m_StartSelected = !m_StartSelected;
		}

		ImGui::Checkbox("Grid", &m_bDrawGrid);
		if (ImGui::Combo("Algorithm", &m_SelectedAlgorithm, "A* & Theta*\0A*\0Theta*\0", 2))
		{
			switch (m_SelectedAlgorithm)
			{
			case 0:
				m_PathFindingAlgo = PathfindingAlgorithm::BOTH;
				break;
			case 1:
				m_PathFindingAlgo = PathfindingAlgorithm::ASTAR;
				break;
			case 2:
				m_PathFindingAlgo = PathfindingAlgorithm::THETASTAR;
				break;
			default:
				m_PathFindingAlgo = PathfindingAlgorithm::BOTH;
				break;
			}

			m_UpdatePath = true;
		}

		if (m_PathFindingAlgo == PathfindingAlgorithm::BOTH || m_PathFindingAlgo == PathfindingAlgorithm::ASTAR)
		{
			if (ImGui::Checkbox("Smooth A*", &m_SmoothAstar))
				m_UpdatePath = true;

			ImGui::Spacing();
			ImGui::Text("A*: (Blue)");
			ImGui::Text("	Time: %.2fms", m_ExecutionTimeAStar * 1000.f);
			ImGui::Text("	Length: %.2f", m_PathLengthAStar);
			ImGui::Spacing();
		}

		if (m_PathFindingAlgo == PathfindingAlgorithm::BOTH || m_PathFindingAlgo == PathfindingAlgorithm::THETASTAR)
		{
			ImGui::Text("Theta*: (Green)");
			ImGui::Text("	Time: %.2fms", m_ExecutionTimeThetaStar * 1000.f);
			ImGui::Text("	Length: %.2f", m_PathLengthThetaStar);
			ImGui::Spacing();
		}

		if (ImGui::Combo("Heuristic", &m_SelectedHeuristic, "Manhattan\0Euclidean\0SqrtEuclidean\0Octile\0Chebyshev\0", 4))
		{
			switch (m_SelectedHeuristic)
			{
			case 0:
				m_pHeuristicFunction = HeuristicFunctions::Manhattan;
				break;
			case 1:
				m_pHeuristicFunction = HeuristicFunctions::Euclidean;
				break;
			case 2:
				m_pHeuristicFunction = HeuristicFunctions::SqrtEuclidean;
				break;
			case 3:
				m_pHeuristicFunction = HeuristicFunctions::Octile;
				break;
			case 4:
				m_pHeuristicFunction = HeuristicFunctions::Chebyshev;
				break;
			default:
				m_pHeuristicFunction = HeuristicFunctions::Chebyshev;
				break;
			}

			m_UpdatePath = true;
		}
		ImGui::Spacing();

		//End
		ImGui::PopAllowKeyboardFocus();
		ImGui::End();
	}
#pragma endregion
#endif
}
