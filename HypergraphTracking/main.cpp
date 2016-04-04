/******************************************************************************
 *                        HYPERGRAPH FOR MCMTT
 ******************************************************************************
 *               .__                           __.
 *                \ `\~~---..---~~~~~~--.---~~| /   
 *                 `~-.   `                   .~         _____ 
 *                     ~.                .--~~    .---~~~    /
 *                      / .-.      .-.      |  <~~        __/
 *                     |  |_|      |_|       \  \     .--'
 *                    /-.      -       .-.    |  \_   \_
 *                    \-'   -..-..-    `-'    |    \__  \_ 
 *                     `.                     |     _/  _/
 *                     ~-                .,-\   _/  _/
 *                      /                 -~~~~\ /_  /_
 *                     |               /   |    \  \_  \_ 
 *                     |   /          /   /      | _/  _/
 *                     |  |          |   /    .,-|/  _/ 
 *                     )__/           \_/    -~~~| _/
 *                       \                      /  \
 *                        |           |        /_---` 
 *                        \    .______|      ./
 *                        (   /        \    /
 *                        `--'          /__/
 *
 ******************************************************************************/

#include <stdio.h>

#include "HyperGraphTracker.h"
#include "Evaluator.h"
#include "hjlib.h"

int main(int argc, char* argv[])
{ 
	CSetting SET("settings.txt");
	const std::string strCurTime = hj::currentDateTime(1);
	std::string filename = "hofmann_" + strCurTime + ".txt";
	std::string strTrackingResultPath = hj::fullfile(SET.GetResultPath(), filename);

	CHyperGraphTracker hgTracker;
	hgTracker.Initialize(SET);
	hgTracker.Run();
	hgTracker.SaveTrackingResultToFile(strTrackingResultPath);
	//hgTracker.Visualization();

	std::string strEvaluationFileName = "hofmann_" + strCurTime + "_eval.txt";
	CEvaluator evaluator;
	evaluator.Initialize(hj::fullfile(SET.GetDatasetPath(), SET.GetGTPath())); 
	evaluator.LoadTrackingResultFromText(strTrackingResultPath);
	evaluator.Evaluate();
	evaluator.PrintResultToConsole();
	evaluator.PrintResultToFile(hj::fullfile(SET.GetResultPath(), strEvaluationFileName));

	return 0;
}

//()()
//('')HAANJU.YOO

