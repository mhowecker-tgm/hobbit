#include "ros/ros.h"
#include "ros/package.h"
#include "DecisionTool.h"
#include <string>

using namespace dt;
using namespace std;

// String tokenizer template
// Tokenizes string 'in' with delimiter 'delimiters' and stores the result in the container (list)
template <typename TContainer, class TString>
void stringtokenizer (
		TContainer& container,
		const TString& in,
		const typename TString::value_type* const delimiters
) {
	container.clear();
	const typename TString::size_type len = in.length();
	typename TString::size_type i = 0;

	while (i < len) {
		i = in.find_first_not_of(delimiters, i);

		if (i == TString::npos)
			return;
		typename TString::size_type j = in.find_first_of(delimiters, i);
		if (j == TString::npos) {
			container.push_back(in.substr(i));
			return;
		}
		else
			container.push_back(in.substr(i, j-i));
		i = j + 1;
	}
}

// Gets the parameters from the evaluation result and sets them as conditionals
// Looks for activations of the form "Profiles,<Parameter>=<Value>" and sets the 
// corresponding conditional
// Returns true, if the system is waiting for user input (for DEMO purposes)
bool setConditionalsFromHobbitEvaluation(DecisionTool *dt, string &evaluationResult) {
	bool user_input = false;
	list<string> tokens;
	list<string> name_value;
	//tokenize command with "--" delimiter
	stringtokenizer(tokens, evaluationResult, "--");
	list<string>::const_iterator iter = tokens.begin();
	while(iter != tokens.end()) {
		string s = iter->data();
		if(!s.substr(0,9).compare("Profiles,")) {//command to change profile value
			//parse name/value pair
			stringtokenizer(name_value, s.substr(9,s.size()), "=");
			if(!name_value.back().compare("true")){//take care of boolean parameter
				dt->SetConditional("Initializer", name_value.front(), true);
			} else if(!name_value.back().compare("false")) {//take care of boolean parameter
				dt->SetConditional("Initializer", name_value.front(), false);
			} else {//string parameter
				dt->SetConditional("Initializer", name_value.front(), name_value.back());
			}
		} else if(!s.substr(0,12).compare("MMUI,Type=D_")) {//we have a dialog, user input required
			user_input = true;
		}
		iter++;
	}
	return user_input;
}


// Prints the evaluation result line by line
void printEvaluationTokens(string &evaluationResult) {
	list<string> tokens;
	stringtokenizer(tokens, evaluationResult, "--");
	list<string>::const_iterator iter = tokens.begin();
	cout << "--------------Evaluating Reaction--------------" << endl;
	while(iter != tokens.end()) {
		cout << iter->data() << endl;
		iter++;
	}
	cout << "-----------------------------------------------" << endl;
}

// Asks the user for input and stores the (y/n) result in 'response'
string getUserResponse(bool *response) {
	string input = "";

	cout << "\n\n======> HOBBIT is waiting for user input\ny=\"D_YES\", n=\"D_NO\", k=\"D_OK\", t=\"D_TIMEOUT\"\n[anything else taken literally (D_NAME, etc.)]: ";
	getline(cin, input);
	if (input.length() == 1) {
		if(input[0] == 'y') {
			*response = true;
			return "";
		} else if(input[0] == 'n') {
			*response = false;
			return "";
		}
	}
	return input;
}


int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "DecisionToolTester");

	// Initialize ROS node - useful for ROS_INFO etc.
	ros::NodeHandle n;

	// Get path to testfiles (path is for DecisionService node)
	std::string path = ros::package::getPath("decision_service").append("/testfiles/");

	list<string>	tokens;
	string			evaluationResult;
	string			userTextResponse = "";
	bool			userPositiveOrNegativeResponse = false;

	DecisionTool	*dta = new DecisionTool;

	if(dta->EngineExists("Initializer")) {
		dta->DisposeEngine("Initializer");
	}
	if(!dta->CreateEngine("Initializer", path+"InitializationPhase.txt")) {
		delete dta;
		return 1;
	}
	// Initialize Hobbit parameters
	dta->SetConditional("Initializer", "ROBOT.State", "Sleeping");
	dta->SetConditional("Initializer", "USER.HasConfiguredHobbit", false);
	dta->SetConditional("Initializer", "ROBOT.VolumeSet", false);
	dta->SetConditional("Initializer", "ROBOT.VolumeUpIntro",false);
	dta->SetConditional("Initializer", "ROBOT.VolumeDownIntro", false);
	dta->SetConditional("Initializer", "ROBOT.VoiceTypeSet", false);
	dta->SetConditional("Initializer", "ROBOT.VoiceTypeIntro", false);
	dta->SetConditional("Initializer", "ROBOT.VoiceSpeedSet", false);
	dta->SetConditional("Initializer", "ROBOT.VoiceSpeedDownIntro", false);
	dta->SetConditional("Initializer", "ROBOT.VoiceSpeedUpIntro", false);
	dta->SetConditional("Initializer", "ROBOT.HobbitNameSet", false);
	dta->SetConditional("Initializer", "ROBOT.HobbitNameQuestionAsked", false);
	dta->SetConditional("Initializer", "ROBOT.HobbitNameSetIntro", false);
	dta->SetConditional("Initializer", "ROBOT.HobbitRenamed", false);
	dta->SetConditional("Initializer", "ROBOT.HobbitRenameIntro", false);
	dta->SetConditional("Initializer", "ROBOT.MasterNameSet", false);
	dta->SetConditional("Initializer", "ROBOT.MasterNameQuestionAsked", false);
	dta->SetConditional("Initializer", "ROBOT.MasterNameSetIntro", false);
	dta->SetConditional("Initializer", "ROBOT.MasterRenamed", false);
	dta->SetConditional("Initializer", "ROBOT.MasterRenameIntro", false);
	dta->SetConditional("Initializer", "ROBOT.UserResponse", "NONE");
	dta->SetConditional("Initializer", "USER.Voice.Volume", 5);
	dta->SetConditional("Initializer", "USER.Voice.Speed", 3);

	cout << "Initializing HOBBIT..." << endl;

	bool configured = false;

	// Iterate over the initialization dialogue steps...
	while(!configured) {

		cout << "\n=================Current State=================" << endl;
		cout << "==  " << dta->GetProfileStringAttribute("Initializer", "ROBOT.State") << endl;
		cout << "===============================================\n" << endl;
		
		// DecisionTool selects the appropriate "Reaction" for the next step
		evaluationResult = dta->Evaluate("Initializer", "Reaction");
		if(evaluationResult.size() == 0) {
			cout << dta->GetError("Initializer") << endl;
			break;
		}
		printEvaluationTokens(evaluationResult);
		
		// Set any parameters in the response that start with "Profiles," and find out if user response needed
		bool user_input_needed = setConditionalsFromHobbitEvaluation(dta, evaluationResult);
		if(user_input_needed) {
			//get response from user
			userTextResponse = getUserResponse(&userPositiveOrNegativeResponse);
			if(userTextResponse.length() == 0) {// yes/no response
				dta->SetConditional("Initializer", "ROBOT.UserResponse", userPositiveOrNegativeResponse ? "D_YES" : "D_NO");
			} else {//other response
				dta->SetConditional("Initializer", "ROBOT.UserResponse", userTextResponse);
			}
		}		
		// When configuration is complete, HOBBIT enters the "idle" state and "waits" for commands (not implemented...)
		if(!dta->GetProfileStringAttribute("Initializer", "ROBOT.State").compare("Idle")) {
			configured = true;
			dta->SetConditional("Initializer", "USER.HasConfiguredHobbit", true);
		}
	}
	
	// When configuration completes, the updated values are stored in the profile
	// They can be loaded the next time, in order not to reinitialize HOBBIT (not implemented...)
	dta->SaveProfile("Initializer", path+"initialization.txt");
	cout << "\nHOBBIT Initialization ended..." << endl;

	// Initializer closes...
	dta->DisposeEngine("Initializer");
	delete dta;

	// Shut down ROS
	ros::shutdown();

	return 0;
}
