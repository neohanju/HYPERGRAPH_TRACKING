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
	std::string filename = hj::sprintf("hofmann_%s.txt", hj::currentDateTime(1));
	std::string strTrackingResultPath = hj::fullfile(SET.GetResultPath(), filename);

	CHyperGraphTracker hgTracker;
	hgTracker.Initialize(SET);
	hgTracker.Run();
	hgTracker.SaveTrackingResultToFile(strTrackingResultPath);

	CEvaluator evaluator;
	evaluator.Initialize(hj::fullfile(SET.GetDatasetPath(), SET.GetGTPath())); 
	evaluator.LoadTrackingResultFromText(strTrackingResultPath);
	evaluator.Evaluate();
	evaluator.PrintResultToConsole();
	evaluator.PrintResultToFile(hj::fullfile(SET.GetResultPath(), hj::sprintf("eval_%s", filename)));

	return 0;
}

//()()
//('')HAANJU.YOO

