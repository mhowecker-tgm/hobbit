#include "ros/ros.h"
#include "DecisionTool.h"
#include "services/AppendRules.h"
#include "services/CreateEngine.h"
#include "services/EngineExists.h"
#include "services/DisposeEngine.h"
#include "services/Evaluate.h"
#include "services/SetStringConditional.h"
#include "services/SetNumberConditional.h"
#include "services/SetBoolConditional.h"
#include "services/SetGlobalStringConditional.h"
#include "services/SetGlobalNumberConditional.h"
#include "services/SetGlobalBoolConditional.h"
#include "services/GetError.h"
#include "services/GetProfileAttributesWithPrefix.h"
#include "services/GetProfileStringAttribute.h"
#include "services/GetProfileBoolAttribute.h"
#include "services/GetProfileNumberAttribute.h"
#include "services/GetGlobalProfileStringAttribute.h"
#include "services/GetGlobalProfileBoolAttribute.h"
#include "services/GetGlobalProfileNumberAttribute.h"
#include "services/LoadProfile.h"
#include "services/SaveProfile.h"
#include <iostream>
#include <iterator>

namespace dt {

bool DecisionTool::CreateEngine(const std::string& id, const std::string& file) {
	decision_service::CreateEngine srv;
	srv.request.id = id;
	srv.request.file = file;
	if (!ros::service::call("CreateEngine", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service CreateEngine.");
		return false;
	}
	return true;
}

bool DecisionTool::EngineExists(const std::string& id) const {
	decision_service::EngineExists srv;
	srv.request.id = id;
	if (!ros::service::call("EngineExists", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service EngineExists.");
		return false;
	}
	return srv.response.exists == 1;
}

bool DecisionTool::DisposeEngine(const std::string& id) {
	decision_service::DisposeEngine srv;
	srv.request.id = id;
	if (!ros::service::call("DisposeEngine", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service DisposeEngine.");
		return false;
	}
	return true;
}

const std::string DecisionTool::Evaluate(const std::string& id, const std::string& component) {
	decision_service::Evaluate srv;
	srv.request.id = id;
	srv.request.component = component;
	srv.response.evaluation = "error";
	if (ros::service::call("Evaluate", srv)) {
		return srv.response.evaluation;
	}
	return "";
}

bool DecisionTool::AppendRules(const std::string& id, const std::string& file) {
	decision_service::AppendRules srv;
	srv.request.id = id;
	srv.request.file = file;
	if (!ros::service::call("AppendRules", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service AppendRules.");
		return false;
	}
	return true;
}

const std::string DecisionTool::GetError(const std::string& id) const {
	decision_service::GetError srv;
	srv.request.id = id;
	if (ros::service::call("GetError", srv)) {
		return srv.response.error;
	} else {
		ROS_ERROR("DecisionTool: Failed to call service GetError.");
		return "";
	}
}

bool DecisionTool::SetConditional(const std::string& id, const std::string& attribute, std::string& value) {
	return SetConditional(id, attribute, value.c_str());
}

bool DecisionTool::SetConditional(const std::string& id, const std::string& attribute, const char* value) {
	decision_service::SetStringConditional srv;
	srv.request.id = id;
	srv.request.attribute = attribute;
	srv.request.value = value;
	if (!ros::service::call("SetStringConditional", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service SetStringConditional.");
		return false;
	}
	return true;
}

bool DecisionTool::SetConditional(const std::string& id, const std::string& attribute, double value) {
	decision_service::SetNumberConditional srv;
	srv.request.id = id;
	srv.request.attribute = attribute;
	srv.request.value = value;
	if (!ros::service::call("SetNumberConditional", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service SetNumberConditional.");
		return false;
	}
	return true;
}

bool DecisionTool::SetConditional(const std::string& id, const std::string& attribute, int value) {
	return SetConditional(id, attribute, (double)value);
}

bool DecisionTool::SetConditional(const std::string& id, const std::string& attribute, bool value) {
	decision_service::SetBoolConditional srv;
	srv.request.id = id;
	srv.request.attribute = attribute;
	srv.request.value = value == true?1:0;
	if (!ros::service::call("SetBoolConditional", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service SetBoolConditional.");
		return false;
	}
	return true;
}

bool DecisionTool::SetGlobalConditional(const std::string& attribute, std::string& value) {
	return SetGlobalConditional(attribute, value.c_str());
}

bool DecisionTool::SetGlobalConditional(const std::string& attribute, const char* value) {
	decision_service::SetGlobalStringConditional srv;
	srv.request.attribute = attribute;
	srv.request.value = value;
	if (!ros::service::call("SetGlobalStringConditional", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service SetGlobalStringConditional.");
		return false;
	}
	return true;
}

bool DecisionTool::SetGlobalConditional(const std::string& attribute, double value) {
	decision_service::SetGlobalNumberConditional srv;
	srv.request.attribute = attribute;
	srv.request.value = value;
	if (!ros::service::call("SetGlobalNumberConditional", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service SetGlobalNumberConditional.");
		return false;
	}
	return true;
}

bool DecisionTool::SetGlobalConditional(const std::string& attribute, int value) {
	return SetGlobalConditional(attribute, (double)value);
}

bool DecisionTool::SetGlobalConditional(const std::string& attribute, bool value) {
	decision_service::SetGlobalBoolConditional srv;
	srv.request.attribute = attribute;
	srv.request.value = value?1:0;
	if (!ros::service::call("SetGlobalBoolConditional", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service SetGlobalBoolConditional.");
		return false;
	}
	return true;
}

bool DecisionTool::LoadProfile(const std::string& id, const std::string& file) {
	decision_service::LoadProfile srv;
	srv.request.id = id;
	srv.request.file = file;
	if (!ros::service::call("LoadProfile", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service LoadProfile.");
		return false;
	}
	return true;
}

bool DecisionTool::SaveProfile(const std::string& id, const std::string& file) {
	decision_service::SaveProfile srv;
	srv.request.id = id;
	srv.request.file = file;
	if (!ros::service::call("SaveProfile", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service SaveProfile.");
		return false;
	}
	return true;

}

std::vector<std::string> DecisionTool::GetProfileAttributesWithPrefix(const std::string& id, const std::string& prefix, bool type) {
	decision_service::GetProfileAttributesWithPrefix srv;
	srv.request.id = id;
	srv.request.prefix = prefix;
	srv.request.type = type?1:0;
	std::vector<std::string> ret;
	if (!ros::service::call("GetProfileAttributesWithPrefix", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service GetProfileAttributesWithPrefix.");
		return ret;
	}
	ret = srv.response.names_values;
	return ret;
}

std::vector<std::string> DecisionTool::GetGlobalProfileAttributesWithPrefix(const std::string& prefix) {
	return GetProfileAttributesWithPrefix("", prefix, false);
}

const std::string DecisionTool::GetProfileStringAttribute(const std::string& id, const std::string& name) const {
	decision_service::GetProfileStringAttribute srv;
	srv.request.id = id;
	srv.request.name = name;
	if (!ros::service::call("GetProfileStringAttribute", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service GetProfileStringAttribute.");
		return "";
	}
	return srv.response.attribute;
}

double DecisionTool::GetProfileNumberAttribute(const std::string& id, const std::string& name) const {
	decision_service::GetProfileNumberAttribute srv;
	srv.request.id = id;
	srv.request.name = name;
	if (!ros::service::call("GetProfileNumberAttribute", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service GetProfileNumberAttribute.");
		return 0.0;
	}
	return srv.response.attribute;
}

bool DecisionTool::GetProfileBoolAttribute(const std::string& id, const std::string& name) const {
	decision_service::GetProfileBoolAttribute srv;
	srv.request.id = id;
	srv.request.name = name;
	if (!ros::service::call("GetProfileBoolAttribute", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service GetProfileBoolAttribute.");
		return false;
	}
	return srv.response.attribute == 1;
}

const std::string DecisionTool::GetGlobalProfileStringAttribute(const std::string& name) const {
	decision_service::GetGlobalProfileStringAttribute srv;
	srv.request.name = name;
	if (!ros::service::call("GetGlobalProfileStringAttribute", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service GetGlobalProfileStringAttribute.");
		return "";
	}
	return srv.response.attribute;
}

double DecisionTool::GetGlobalProfileNumberAttribute(const std::string& name) const {
	decision_service::GetGlobalProfileNumberAttribute srv;
	srv.request.name = name;
	if (!ros::service::call("GetGlobalProfileNumberAttribute", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service GetGlobalProfileNumberAttribute.");
		return 0.0;
	}
	return srv.response.attribute;
}

bool DecisionTool::GetGlobalProfileBoolAttribute(const std::string& name) const {
	decision_service::GetGlobalProfileBoolAttribute srv;
	srv.request.name = name;
	if (!ros::service::call("GetGlobalProfileBoolAttribute", srv)) {
		ROS_ERROR("DecisionTool: Failed to call service GetGlobalProfileBoolAttribute.");
		return false;
	}
	return srv.response.attribute == 1;
}

DecisionTool::DecisionTool() {}
DecisionTool::~DecisionTool() {}
}
