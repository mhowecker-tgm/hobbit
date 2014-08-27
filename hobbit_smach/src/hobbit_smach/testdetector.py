#!/usr/bin/python

## walter wohlkinger
## 17.10.2012
## modified .... 


PKG = 'hobbit_smach'
#PKG = 'ActionSequencer'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import os

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Header
from hobbit_msgs.msg import Command, Status, Event, Parameter
from hobbit_msgs.srv import Request, RequestRequest
from hobbit_msgs.srv import SingleShotPC
from hobbit_msgs.srv import PointCloud2FeatureHistogram, ClustersOnPlane
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs.msg import PointField

#from RandomForestRecognition import RFR  

import subprocess
import struct
_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)



class TD:
    def __init__(self):
        self.pubM = rospy.Publisher("/wwclusters", PointCloud2)   
        
    def run(self):
	print "david1"
        r = rospy.Rate(10) # 10hz
        #pointcloud = self.getSingleShot()
        #groundplane = self.findGroundPlane(pointcloud)        
        while not rospy.is_shutdown():
            r.sleep()
	    print "david2"
            pointcloud = self.getSingleShot()
            #groundplane = self.findGroundPlane(pointcloud)        
            clusters = self.findObjectsOnFloor(pointcloud, [0,0,0,0])
            print len(clusters)
            cntr = 0
            #break
            
    
    
#    def saveDescr2File(self, descr, location, classname, filename):
#        f = open(location + '/' + classname +  '/' + filename +'.descr', 'w')
#        #print descr
#        for item in descr:
#            f.write(str(item) + ' ' )
#        f.close()
        
    def savePointCloud2File(self, pc, location, classname, filename):
        #extract points from PointCloud2 in python.... 
        fmt = self._get_struct_fmt(pc)
        narr = list()
        offset = 0
        for i in xrange(pc.width * pc.height):
            p = struct.unpack_from(fmt, pc.data, offset)
            offset += pc.point_step
            narr.append(p)
        #print "Number of points:", len(points)
        if len(narr) > 0:
            f = open(location + '/' + classname +  '/' + filename +'.pcd', 'w')
            f.write('# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH ' + str(len(narr))+ '\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ' + str(len(narr))+ '\nDATA ascii\n')
#            f.write('# .PCD v.5 - Point Cloud Data file format\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nWIDTH 640\nHEIGHT 480\nPOINTS ' + str(len(narr))+ '\nDATA ascii\n')
            for p in narr:
                #fs = "{0:.10f} {1:.10f} {2:.10f} \n".format(p[0],p[1],p[2])
                #f.write(fs)
                f.write(str(p[0]) + ' ' + str(p[1]) + ' ' + str(p[2]) + ' ' + str(p[3]) + '\n')
            f.close()

    def _get_struct_fmt(self, cloud, field_names=None):
        fmt = '>' if cloud.is_bigendian else '<'
        offset = 0
        for field in (f for f in sorted(cloud.fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in _DATATYPES:
                print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length
        return fmt    
    
    def getDescriptor4PointCloud(self, cloud):
        rospy.wait_for_service('/Feature_SF/do_feature_SF')
        get_func = rospy.ServiceProxy('/Feature_SF/do_feature_SF', PointCloud2FeatureHistogram)
        try:
            resp = get_func(cloud)    
            return resp.hist
        except rospy.ServiceException, e:
            print "ups", e
            return None
        
   
    def getSingleShot(self):
	print "testdetector.getSingleShot()"
        rospy.wait_for_service('/CloudSegmenthor/getSingleShot')
        servicecall = rospy.ServiceProxy('/CloudSegmenthor/getSingleShot', SingleShotPC)
        try:
            resp = servicecall("1")    # call the service            
            return resp.point_cloud
        except rospy.ServiceException, e:
          print "Service did not process request: %s"%str(e)
          return None

    def findTurnTable(self, pointcloud):
        rospy.wait_for_service('/CloudSegmenthor/findTurnTablePlane')
        servicecall = rospy.ServiceProxy('/CloudSegmenthor/findTurnTablePlane', PointCloud2FeatureHistogram)
        try:
            resp = servicecall(pointcloud)    # call the service            
            return resp.hist
        except rospy.ServiceException, e:
          print "Service did not process request: %s"%str(e)
          return None

    def findGroundPlane(self, pointcloud):
        rospy.wait_for_service('/CloudSegmenthor/findGroundPlane')
        servicecall = rospy.ServiceProxy('/CloudSegmenthor/findGroundPlane', PointCloud2FeatureHistogram)
        try:
            resp = servicecall(pointcloud)    # call the service            
            return resp.hist
        except rospy.ServiceException, e:
          print "Service did not process request: %s"%str(e)
          return None

    #df start
    def findTablePlane(self, pointcloud):
        rospy.wait_for_service('/CloudSegmenthor/findTablePlane')
        servicecall = rospy.ServiceProxy('/CloudSegmenthor/findTablePlane', PointCloud2FeatureHistogram)
        try:
            resp = servicecall(pointcloud)    # call the service            
            return resp.hist
        except rospy.ServiceException, e:
          print "Service did not process request: %s"%str(e)
          return None
    #df end

    def findObjectOnTurnTable(self, pointcloud, plane):
        rospy.wait_for_service('/CloudSegmenthor/findObjectOnTurnTable')
        servicecall = rospy.ServiceProxy('/CloudSegmenthor/findObjectOnTurnTable', ClustersOnPlane)
        try:
            resp = servicecall(pointcloud, plane)    # call the service            
            return resp.clusters
        except rospy.ServiceException, e:
          print "Service did not process request: %s"%str(e)
          return None

    def findObjectsOnFloor(self, pointcloud, groundplane):
	print "david3"
        rospy.wait_for_service('/CloudSegmenthor/findObjectsOnFloor')
	print "david3.5"
        servicecall = rospy.ServiceProxy('/CloudSegmenthor/findObjectsOnFloor', ClustersOnPlane)
	print "david4"
        try:
            resp = servicecall(pointcloud, groundplane)    # call the service            
            return resp.clusters
        except rospy.ServiceException, e:
          print "Service did not process request: %s"%str(e)
          return None

    #df start 31.01.2013
    def findObjectsOnTable(self, pointcloud, tableplane):
        rospy.wait_for_service('/CloudSegmenthor/findObjectsOnTable')
        servicecall = rospy.ServiceProxy('/CloudSegmenthor/findObjectsOnTable', ClustersOnPlane)
        try:
            resp = servicecall(pointcloud, tableplane)    # call the service            
            return resp.clusters
        except rospy.ServiceException, e:
          print "Service did not process request: %s"%str(e)
          return None
    #df end



def main(args):        
    rospy.init_node('ActionSequencerTD', anonymous=False)
    sbm = TD()
    sbm.run()
    
    rospy.spin()

if __name__ == "__main__":        
    main(sys.argv)
