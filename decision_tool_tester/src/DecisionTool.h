#ifndef DECISION_TOOL_H
#define DECISION_TOOL_H

#include <string>

namespace dt {

	class DecisionTool {

	public:
		bool CreateEngine(const std::string& id, const std::string& file);
		bool EngineExists(const std::string& id) const;
		bool DisposeEngine(const std::string& id);
		const std::string Evaluate(const std::string& id, const std::string& component);
		bool AppendRules(const std::string& id, const std::string& file);
		
		bool SetConditional(const std::string& id, const std::string& attribute, std::string& value);
		bool SetConditional(const std::string& id, const std::string& attribute, const char* value);
		bool SetConditional(const std::string& id, const std::string& attribute, int value);
		bool SetConditional(const std::string& id, const std::string& attribute, double value);
		bool SetConditional(const std::string& id, const std::string& attribute, bool value);

		bool SetGlobalConditional(const std::string& attribute, std::string& value);
		bool SetGlobalConditional(const std::string& attribute, const char* value);
		bool SetGlobalConditional(const std::string& attribute, int value);
		bool SetGlobalConditional(const std::string& attribute, double value);
		bool SetGlobalConditional(const std::string& attribute, bool value);

		const std::string GetError(const std::string& id) const;

		std::vector<std::string> GetProfileAttributesWithPrefix(const std::string& id, const std::string& prefix, bool type);
		std::vector<std::string> GetGlobalProfileAttributesWithPrefix(const std::string& prefix);

		const std::string GetProfileStringAttribute(const std::string& id, const std::string& name) const;
		double GetProfileNumberAttribute(const std::string& id, const std::string& name) const;
		bool GetProfileBoolAttribute(const std::string& id, const std::string& name) const;

		const std::string GetGlobalProfileStringAttribute(const std::string& name) const;
		double GetGlobalProfileNumberAttribute(const std::string& name) const;
		bool GetGlobalProfileBoolAttribute(const std::string& name) const;

		bool	LoadProfile(const std::string& id, const std::string& file);
		bool	SaveProfile(const std::string& id, const std::string& file);

		DecisionTool();
		~DecisionTool();
	};
}

#endif

