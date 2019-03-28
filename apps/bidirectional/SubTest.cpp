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
#include "MyOptimisticSearch.h"
#include "DynamicPotentialSearch.h"

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

void TestSub0(const char *scenario, double wt, int reopen)
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

void TestSub1(const char *scenario, double wt, int phi_type, int reopen)
{
	Map *map = 0;
	MapEnvironment *me = 0;
	MapOverlay *mo;
	xyLoc start, goal;

	MyOptimisticSearch<xyLoc, tDirection, MapEnvironment> OS(Phi_XUP);
	OS.SetWeight(2*wt-1.0);
	OS.SetOptimalityBound(wt);
	std::cout<<"OS:"<<OS.GetName()<<"\n";

	DynamicPotentialSearch<xyLoc, tDirection, MapEnvironment> dps;
	dps.SetOptimalityBound(wt);
	//if (reopen != 0)
	//	oldAStar.SetReopenNodes(true);
	MyAStar<xyLoc, tDirection, MapEnvironment> newAStar(Phi_XUP);
	newAStar.SetWeight(wt);
	//if (reopen != 0)
	//	newAStar.SetReopenNodes(true);

	if (phi_type == 0)
	{
		OS.SetPhi(Phi_WA);
		newAStar.SetPhi(Phi_WA);
	}
	if (phi_type == 1)
	{
		OS.SetPhi(Phi_XDP);
		newAStar.SetPhi(Phi_XDP);
	}
	if (phi_type == 2)
	{
		OS.SetPhi(Phi_XUP);
		newAStar.SetPhi(Phi_XUP);
	}

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
		std::vector<xyLoc> OSPath;
		std::vector<xyLoc> dpsPath;
		std::vector<xyLoc> newAStarPath;
		double h_i = me->HCost(start, goal);



		OS.SetHeuristic(me);
		dps.SetHeuristic(me);

		newAStar.SetHeuristic(me);

		double t1, t2, t3, t4, t5, t6;


		OS.InitializeSearch(me, start, goal, OSPath);
		OS.SetEnhancement1(true);
		newAStar.InitializeSearch(me, start, goal, newAStarPath);
		dps.InitializeSearch(me, start, goal, dpsPath);

		for (int j = 0; j < 3; j++)
		{
			if (j == 0)
				OS.SetPhi(Phi_WA);

			if (j == 1)
				OS.SetPhi(Phi_XDP);

			if (phi_type == 2)
				OS.SetPhi(Phi_XUP);
			for (int i = 0; i < 3; i++)
			{
				//update cost and reopen
				if (i == 0)
				{
					OS.SetEnhancement2(true);
					OS.SetReopenStage1(true);

				}
				//update cost and no reopen
				if (i == 1)
				{
					OS.SetEnhancement2(true);
					OS.SetReopenStage1(false);

				}
				if (i == 2)
				{
					OS.SetEnhancement2(false);
					OS.SetReopenStage1(false);
				}
				OS.InitializeSearch(me, start, goal, OSPath);
				OS.GetPath(me, start, goal, OSPath);

				std::cout << "i,j" << i << j << "\n";
				std::cout << "nodes:" << i << "\t"
					<< OS.GetNodesExpanded() << "\t"
					<< OS.GetUniqueNodesExpanded() << "\t"
					<< OS.GetMaxFCost() << "\t"
					<< OS.GetFirstSolutionCost() << "\t"
					<< me->GetPathLength(OSPath) << "\t"
					<< "\n";
			}

		}

		/*
		if (1)
		{
			t.StartTimer();
			OS.GetPath(me, start, goal, OSPath);
			t.EndTimer();
			t1 = t.GetElapsedTime();
		}


		if (0)
		{
			t.StartTimer();
			newAStar.GetPath(me, start, goal, newAStarPath);
			t.EndTimer();
			t2 = t.GetElapsedTime();
		}

		if (0)
		{
			t.StartTimer();
			dps.GetPath(me, start, goal, OSPath);
			t.EndTimer();
			t3 = t.GetElapsedTime();
		}

		std::cout << "nodes:\t"
			<< OS.GetNodesExpanded() << "\t"
			<< OS.GetUniqueNodesExpanded() << "\t"
			<< OS.GetMaxFCost() << "\t"
			<< OS.GetFirstSolutionCost() << "\t"
			<< me->GetPathLength(OSPath) << "\t"
			//<< newAStar.GetNodesExpanded() << "\t"
			////<< newAStar.GetNecessaryExpansions() << "\t"
			////<< newAStar.GetNodesExplored() << "\t"
			//<< newAStar.GetMaxFCost() << "\t"
			//<< me->GetPathLength(newAStarPath) << "\t"
			//<< dps.GetNodesExpanded() << "\t"
			//<< me->GetPathLength(dpsPath) << "\t"
			<< "\n";
			*/
	}
	printf("Exiting with no errors\n");
	exit(0);
}
