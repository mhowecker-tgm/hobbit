#include "ros/ros.h"
#include "ros/package.h"
#include "DecisionTool.h"
#include <string>

using namespace dt;

int main(int argc, char **argv)
{
	/* Initialize ROS */
	ros::init(argc, argv, "DecisionToolTester");

	/* Initialize ROS node - useful for ROS_INFO etc. */
	ros::NodeHandle n;

	/* Get path to testfiles (path is for DecisionService node) */
	std::string path = ros::package::getPath("decision_service").append("/testfiles/");

	DecisionTool *dta = new DecisionTool;

	/* Create a new engine for a particular component and load the respective ruleset */
	if(dta->EngineExists("firstid")) {
		dta->DisposeEngine("firstid");
	}
	if(!dta->CreateEngine("firstid", path +"example1.txt")) {
		delete dta;
		return 1;
	}

	/* Create a new engine for another component and load the respective ruleset */
	if(dta->EngineExists("secondid")) {
		dta->DisposeEngine("secondid");
	}
	if(!dta->CreateEngine("secondid", path+"example2.txt")) {
		delete dta;
		return 1;
	}

	/* ================================================================================ */
	/* Engine 1 - Access to The Profile */

	/* Load profile values from a file for the first engine */
	if(!dta->LoadProfile("firstid", path+ "profile.txt")) {
		delete dta;
		return 1;
	}

	/* Evaluate a rule that depends on user parameters */
	printf("\n== Eval using Engine 1 ==\n");
	printf("ErrorReporting 1: %s\n", dta->Evaluate("firstid", "ErrorReporting").c_str());

	/* Set a different value for user vision */
	dta->SetConditional("firstid", "user.vision", true);

	/* Evaluate a rule using the new value */
	printf("ErrorReporting 2: %s\n", dta->Evaluate("firstid", "ErrorReporting").c_str());

	/* Print a profile attribute */
	printf("User name: %s\n", dta->GetProfileStringAttribute("firstid", "user.name").c_str());

	/* Set another value for user name */
	dta->SetConditional("firstid", "user.name", "John");

	/* Store the profile to disk */
	dta->SaveProfile("firstid", "saved_profile.txt");

	/* Dispose the first engine */
	dta->DisposeEngine("firstid");

	/* ================================================================================ */
	/* Engine 2 */

	/* Set conditional at the second engine */
	dta->SetConditional("secondid", "screen.brightness", 5);

	/* Evaluate a rule that depends on the above conditional */
	printf("\n== Eval using Engine 2 ==\n");
	printf("ScreenContrast: %s\n", dta->Evaluate("secondid", "ScreenContrast").c_str());

	/* Set another conditional at the second engine */
	/* These values cause an error (see example2.txt) */
	dta->SetConditional("secondid", "screen.width", 1920);
	dta->SetConditional("secondid", "screen.height", 1080);
	printf("ScreenSize: %s\n", dta->Evaluate("secondid", "ScreenSize").c_str());

	/* Dispose the second engine */
	dta->DisposeEngine("secondid");

	delete dta;
	/* Shut down ROS */
	ros::shutdown();

	return 0;
}
