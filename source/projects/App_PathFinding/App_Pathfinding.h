#ifndef ASTAR_APPLICATION_H
#define ASTAR_APPLICATION_H
//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "framework/EliteInterfaces/EIApp.h"
#include "framework\EliteAI\EliteGraphs\EGridGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphUtilities\EGraphEditor.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphUtilities\EGraphRenderer.h"

//-----------------------------------------------------------------
// Application
//-----------------------------------------------------------------

enum class PathfindingAlgorithm
{
	BOTH, ASTAR, THETASTAR
};

class App_Pathfinding final : public IApp
{
public:
	//Constructor & Destructor
	App_Pathfinding() = default;
	virtual ~App_Pathfinding();

	//App Functions
	void Start() override;
	void Update(float deltaTime) override;
	void Render(float deltaTime) const override;

private:
	//Datamembers
	const bool ALLOW_DIAGONAL_MOVEMENT = true;
	Elite::Vector2 m_StartPosition = Elite::ZeroVector2;
	Elite::Vector2 m_TargetPosition = Elite::ZeroVector2;

	//Grid datamembers
	static const int COLUMNS = 25;
	static const int ROWS = 25;
	unsigned int m_SizeCell = 50;
	Elite::GridGraph<Elite::GridTerrainNode, Elite::GraphConnection>* m_pGridGraph;

	//Pathfinding datamembers
	int startPathIdx = invalid_node_index;
	int endPathIdx = invalid_node_index;
	std::vector<Elite::GridTerrainNode*> m_vPathAStar;
	std::vector<Elite::GridTerrainNode*> m_vPathThetaStar;
	float m_PathLengthAStar;
	float m_PathLengthThetaStar;
	float m_ExecutionTimeAStar;
	float m_ExecutionTimeThetaStar;
	bool m_UpdatePath = true;

	//Editor and Visualisation
	Elite::EGraphEditor m_GraphEditor{};
	Elite::EGraphRenderer m_GraphRenderer{};

	//Debug rendering information
	int m_SelectedHeuristic = 4;
	int m_SelectedAlgorithm = 0;
	Elite::Heuristic m_pHeuristicFunction = Elite::HeuristicFunctions::Chebyshev;
	PathfindingAlgorithm m_PathFindingAlgo;
	bool m_bDrawGrid = true;
	bool m_bDebugRenderPathSearch = false;
	bool m_StartSelected = true;
	bool m_SmoothAstar = false;

	//Functions
	void MakeGridGraph();
	void UpdateImGui();

	//C++ make the class non-copyable
	App_Pathfinding(const App_Pathfinding&) = delete;
	App_Pathfinding& operator=(const App_Pathfinding&) = delete;
};
#endif