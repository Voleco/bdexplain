//
//  SubTest.cpp
//  hog2 glut
//
//

#include "SubTest.h"
#include "PancakePuzzle.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "PancakeInstances.h"
#include "WeightedVertexGraph.h"
#include "HeuristicError.h"
#include "HeavyPancakePuzzle.h"
#include "NonlinearWeightedAStar.h"
#include "MyAStar.h"


#include <vector>
#include <fstream>
#include <numeric>
#include "ScenarioLoader.h"
#include "Map2DEnvironment.h"
#include "MapGenerators.h"
#include "MapOverlay.h"
#include "TemplateAStar.h"
#include "SVGUtil.h"
#include "Timer.h"

void TestSub1(const char *scenario, double wt, int reopen)
{
	Map *map = 0;
	MapEnvironment *me = 0;
	MapOverlay *mo;
	xyLoc start, goal;

	TemplateAStar<xyLoc, tDirection, MapEnvironment> oldAStar;
	oldAStar.SetWeight(wt);
	if (reopen != 0)
		oldAStar.SetReopenNodes(true);
	MyAStar<xyLoc, tDirection, MapEnvironment> newAStar;
	newAStar.SetWeight(wt);
	if (reopen != 0)
		newAStar.SetReopenNodes(true);

	printf("Loading scenario %s\n", scenario);
	ScenarioLoader s(scenario);

	std::string base = "/home/jingwei/Desktop/Shared/nbs_ad/hog2/";
	Timer t;
	ZeroHeuristic<xyLoc> z;

	std::string mapName = "";
	for (int x = s.GetNumExperiments() - 1; x >= 0; x--)
	{
		if (fequal(s.GetNthExperiment(x).GetDistance(), 0))
			continue;
		std::string currentMapName(s.GetNthExperiment(x).GetMapName());
		if (mapName.compare(currentMapName) != 0)
		{
			mapName = currentMapName;
			std::string fullPath = base + mapName;
			Map *m = new Map(fullPath.c_str());
			me = new MapEnvironment(m);
			me->SetDiagonalCost(1.5);
		}
		xyLoc start, goal;
		start.x = s.GetNthExperiment(x).GetStartX();
		start.y = s.GetNthExperiment(x).GetStartY();
		goal.x = s.GetNthExperiment(x).GetGoalX();
		goal.y = s.GetNthExperiment(x).GetGoalY();
		printf("Problem %d of %d from ", x, s.GetNumExperiments());
		std::cout << start << " to " << goal << " " << mapName << "\n";
		std::vector<xyLoc> oldAStarPath;
		std::vector<xyLoc> newAStarPath;
		double h_i = me->HCost(start, goal);
		

		oldAStar.SetHeuristic(me);

		newAStar.SetHeuristic(me);

		double t1, t2, t3, t4, t5, t6;


		oldAStar.InitializeSearch(me, start, goal, oldAStarPath);
		newAStar.InitializeSearch(me, start, goal, newAStarPath);

		t.StartTimer();
		oldAStar.GetPath(me, start, goal, oldAStarPath);
		t.EndTimer();
		t1 = t.GetElapsedTime();


		t.StartTimer();
		newAStar.GetPath(me, start, goal, newAStarPath);
		t.EndTimer();
		t2 = t.GetElapsedTime();

		std::cout << "nodes:\t"
			<< oldAStar.GetNodesExpanded() << "\t"
			<< oldAStar.GetNecessaryExpansions() << "\t"
			<< oldAStar.GetNecessaryExpansions2() << "\t"
			<< oldAStar.GetNodesExplored() << "\t"
			<< newAStar.GetNodesExpanded() << "\t"
			<< newAStar.GetNecessaryExpansions() << "\t"
			<< newAStar.GetNecessaryExpansions2() << "\t"
			<< newAStar.GetNodesExplored() << "\t"
			<< "\n";


	}
	printf("Exiting with no errors\n");
	exit(0);
}
