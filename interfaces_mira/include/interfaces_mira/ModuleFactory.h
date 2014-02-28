/* ModuleFactory
* Author: chris burbridge
  Modified by Paloma de la Puente
*
* Creates instances of mira robot components (factory pattern)
*/

#ifndef MODULEFACTORY_H_
#define MODULEFACTORY_H_

#include <map>
#include <string>
#include <ros/ros.h>

#include <interfaces_mira/MiraRobotModule.h>
#include <interfaces_mira/MiraRobot.h>

class ModuleFactory {
private:
        ModuleFactory();
    ModuleFactory(const ModuleFactory &) { }
    ModuleFactory &operator=(const ModuleFactory &) { return *this; }
    std::map<std::string, ModuleCreator> modules_;
public:
    ~ModuleFactory() { modules_.clear(); }
    static ModuleFactory *Get()
    {
        static ModuleFactory instance;
        return &instance;
    }

    void Register(const std::string &name, ModuleCreator create);
    MiraRobotModule *CreateModule(std::string name, MiraRobot *robot);
    bool CheckForModule(const std::string &name);
};

#endif /* MODULEFACTORY_H_ */
