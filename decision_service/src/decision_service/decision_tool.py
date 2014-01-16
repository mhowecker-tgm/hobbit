#!/usr/bin/env python
import rospy
import ros

from decision_service.srv import *

class DecisionTool(object):
    def __init__(self, timeout=5):
        self._timeout = timeout

    def CreateEngine(self, id, file):
        try:
            rospy.wait_for_service('CreateEngine', self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy('CreateEngine', CreateEngine)
        try:
            servicecall(id, file)
        except rospy.ServiceException:
            raise
    
    def EngineExists(self, id):
        try:
            rospy.wait_for_service('EngineExists', self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy('EngineExists', EngineExists)
        try:
            result = servicecall(id)
        except rospy.ServiceException:
            raise
        return result.exists == 1


    def DisposeEngine(self, id):
        try:
            rospy.wait_for_service('DisposeEngine', self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy('DisposeEngine', DisposeEngine)
        try:
            servicecall(id)
        except rospy.ServiceException:
            raise
    
    def Evaluate(self, id, component):
        try:
            rospy.wait_for_service('Evaluate', self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy('Evaluate', Evaluate)
        try:
            result = servicecall(id, component)
        except rospy.ServiceException:
            raise
        return result.evaluation
    
    def AppendRules(self, id, file):
        try:
            rospy.wait_for_service('AppendRules', self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy('AppendRules', AppendRules)
        try:
            servicecall(id, file)
        except rospy.ServiceException:
            raise
    
    def GetError(self, id):
        try:
            rospy.wait_for_service('GetError', self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy('GetError', GetError)
        try:
            result = servicecall(id)
        except rospy.ServiceException:
            raise
        return result.error
    
    def SetConditional(self, id, attribute, value):
        if type(value) is str:
            service = 'SetStringConditional'
            struct = SetStringConditional
        elif type(value) is bool:
            service = 'SetBoolConditional'
            struct = SetBoolConditional
        else:
            service = 'SetNumberConditional'
            struct = SetNumberConditional
        try:
            rospy.wait_for_service(service, self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy(service, struct)
        try:
            servicecall(id, attribute, value)
        except rospy.ServiceException:
            raise
    
    def SetGlobalConditional(self, attribute, value):
        if type(value) is str:
            service = 'SetGlobalStringConditional'
            struct = SetGlobalStringConditional
        elif type(value) is bool:
            service = 'SetGlobalBoolConditional'
            struct = SetGlobalBoolConditional
        else:
            service = 'SetGlobalNumberConditional'
            struct = SetGlobalNumberConditional
        try:
            rospy.wait_for_service(service, self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy(service, struct)
        try:
            servicecall(attribute, value)
        except rospy.ServiceException:
            raise
        
    def SaveLoadProfile(self, id, file, service, struct):
        try:
            rospy.wait_for_service(service, self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy(service, struct)
        try:
            servicecall(id, file)
        except rospy.ServiceException:
            raise
    
    def LoadProfile(self, id, file):
        self.SaveLoadProfile(id, file, 'LoadProfile', LoadProfile)
    
    def SaveProfile(self, id, file):
        self.SaveLoadProfile(id, file, 'SaveProfile', SaveProfile)
        
    def GetProfileAttributesWithPrefix(self, id, prefix):
        try:
            rospy.wait_for_service('GetProfileAttributesWithPrefix', self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy('GetProfileAttributesWithPrefix', GetProfileAttributesWithPrefix)
        try:
            array = servicecall(id, prefix, False)
        except rospy.ServiceException:
            raise
        result = {}
        for i in range(len(array.names_values)):
            if i % 2 == 0:
                result[array.names_values[i]] = array.names_values[i+1]
        return result
    
    def GetGlobalProfileAttributesWithPrefix(self, prefix):
        return self.GetProfileAttributesWithPrefix('', prefix)
       
    def GetProfileAttribute(self, id, name, service, struct):
        try:
            rospy.wait_for_service(service, self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy(service, struct)
        try:
            result = servicecall(id, name)
        except rospy.ServiceException:
            raise
        return result.attribute

    def GetProfileStringAttribute(self, id, name):
        return self.GetProfileAttribute(id, name, 'GetProfileStringAttribute', GetProfileStringAttribute)
    
    def GetProfileNumberAttribute(self, id, name):
        return self.GetProfileAttribute(id, name, 'GetProfileNumberAttribute', GetProfileNumberAttribute)
    
    def GetProfileBoolAttribute(self, id, name):
        return self.GetProfileAttribute(id, name, 'GetProfileBoolAttribute', GetProfileBoolAttribute) == 1
    
    def GetGlobalProfileAttribute(self, name, service, struct):
        try:
            rospy.wait_for_service(service, self._timeout)
        except rospy.ROSException:
            raise
        servicecall = rospy.ServiceProxy(service, struct)
        try:
            result = servicecall(name)
        except rospy.ServiceException:
            raise
        return result.attribute

    def GetGlobalProfileStringAttribute(self, name):
        return self.GetGlobalProfileAttribute(name, 'GetGlobalProfileStringAttribute', GetGlobalProfileStringAttribute)
    
    def GetGlobalProfileNumberAttribute(self, name):
        return self.GetGlobalProfileAttribute(name, 'GetGlobalProfileNumberAttribute', GetGlobalProfileNumberAttribute)
    
    def GetGlobalProfileBoolAttribute(self, name):
        return self.GetGlobalProfileAttribute(name, 'GetGlobalProfileBoolAttribute', GetGlobalProfileBoolAttribute) == 1   
