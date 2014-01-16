#include "ros/ros.h"
#include "ros/package.h"
#include "DecisionTool.h"
#include <string>

using namespace dt;
using namespace std;

void print_vector(vector<string> &v, bool type);

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
	if(!dta->CreateEngine("firstid", path +"example3.txt")) {
		delete dta;
		return 1;
	}

	/* Create a new engine for another component and load the SAME ruleset */
	if(dta->EngineExists("secondid")) {
		dta->DisposeEngine("secondid");
	}
	if(!dta->CreateEngine("secondid", path+"example3.txt")) {
		delete dta;
		return 1;
	}

	/* Set global parameter (male/female) */
	dta->SetGlobalConditional("user.male", true);

	/* The first engine will use capital letters, while the second will not */
	dta->SetConditional("firstid", "response.capslock", true);
	dta->SetConditional("secondid", "response.capslock", false);

	/* Evaluate WhatAmI for each engine */
	printf("What am I, first engine?  : %s\n", dta->Evaluate("firstid", "WhatAmI").c_str());
	printf("What am I, second engine? : %s\n", dta->Evaluate("secondid", "WhatAmI").c_str());

	/* Dispose the first engine */
	dta->DisposeEngine("firstid");

	/* Dispose the second engine */
	dta->DisposeEngine("secondid");
	
	/* Playing with global attributes (names do not follow conventions for demo purposes) */
	dta->SetGlobalConditional("hobbit.voice.volume", 7);
	dta->SetGlobalConditional("hobbit.voice.male", false);
	dta->SetGlobalConditional("hobbit.voice.pitch", "high");

	/* Just test attributes to check if the prefix services work */
	dta->SetGlobalConditional("hobbit.name", "George");
	dta->SetGlobalConditional("house.temperature", 27.2);

	/* Test engine */
	if(dta->EngineExists("test")) {
		dta->DisposeEngine("test");
	}
	if(!dta->CreateEngine("test", path +"example1.txt")) {
		delete dta;
		return 1;
	}
	dta->SetConditional("test", "hobbit.nickname", "Bob");
	dta->SetConditional("test", "house.size", "Big");

	/* Print what each profile contains at this point */
	printf("\nGlobal profile:\nhobbit.voice.volume = 7\nhobbit.voice.male = false\nhobbit.voice.pitch = high\n");
	printf("hobbit.name = George\nhouse.temperature = 27.2\n\n");
	printf("Local parameters for engine \"test\":\nhobbit.nickname = Bob\nhouse.size = Big\n\n");

	/* Get attributes based on prefix */
	printf("Getting local attributes for \"test\" and global attributes beginning with \"house.\" (attribute type included):\n");
	vector<string> v = dta->GetProfileAttributesWithPrefix("test", "house.*", true);
	print_vector(v, true);

	printf("Getting local attributes for \"test\" and global attributes beginning with \"ho\" (attribute type included):\n");
	v = dta->GetProfileAttributesWithPrefix("test", "ho*", true);
	print_vector(v, true);

	printf("Getting global attributes beginning with \"hobbit.\"(attribute type NOT included):\n");
	v = dta->GetProfileAttributesWithPrefix("", "hobbit.*", false);
	print_vector(v, false);

	printf("Getting global attributes beginning with \"hobbit.\" (attribute type NOT included):\n");
	//alternative way to achieve the same result as before
	v = dta->GetGlobalProfileAttributesWithPrefix("hobbit.*");
	print_vector(v, false);

	printf("Getting global attributes beginning with \"ho\" (attribute type NOT included):\n");
	v = dta->GetGlobalProfileAttributesWithPrefix("ho*");
	print_vector(v, false);

	dta->DisposeEngine("test");

	delete dta;

	/* Shut down ROS */
	ros::shutdown();

	return 0;
}

void print_vector(vector<string> &v, bool type) {
	vector<string>::iterator it;
	for(it = v.begin(); it != v.end(); ++it) {
		printf("%s\n", (*it).c_str());
		++it;
		if(type) {
			printf("\tType:\t%s\n", (*it).c_str());
			++it;
		}
		printf("\tValue:\t%s\n", (*it).c_str());
	}
	printf("\n==========\n");
	v.clear();
}
