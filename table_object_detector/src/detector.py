#!/usr/bin/python

## walter wohlkinger
## first version: 09.12.2010
## modified 13.03.2011
## integrated in final Framework: 27.04.2010
## modified 4 hobbit: Dec 4th 2012


PKG = 'TableObjectDetector'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import os
import time
import recognition_msgs
from std_msgs.msg import String
from recognition_msgs.msg import Candidate, DetectionRequest, Hypothesis, Categories, RankedFileList
from recognition_msgs.srv import GetCandidate, PointCloud2FeatureHistogram
from DB_Detector import ObjectCategory, DBDetector

import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField
import PointCloud2Helper
import Cats

class detector():
    def __init__(self):
        #self.DBPATH = rospy.get_param('~dbpath')
        self.DETECTOR_ID = 'SF' # assign a unique detector_id 
        self.detection_request_sub = rospy.Subscriber("/SM/detection_requests", DetectionRequest, self.detection_request_callback)    
        self.detection_categories_sub = rospy.Subscriber("/SM/detection_categories", Categories, self.detection_categories_callback)    
        self.pub = rospy.Publisher('/SM/detection_hypothesis', Hypothesis)        
        self.ranked_viz_pub = rospy.Publisher('/D2C/detection_rankedlist', RankedFileList)        
        self.queue = []    
        #self.Categories = []
        self.Categories = Cats.hard_categories
        
        self.shapeDetector = DBDetector(rospy.get_param("~dbpath"))
  
        if rospy.get_param("~generate_descriptor_on_db") == 1:
            print "Generating Descriptors for D2C Shape Distributions"
            self.shapeDetector.loadDatabase('categories_only',[])
            self.do_Feature_on_DB()
            #self.shapeDetector.clear_cache("d2cRs2hw")
        self.shapeDetector.loadDatabase('d2cRs2hw',self.Categories,filteredOnly = 0, hashing = 1) # load database with only this descriptors
        self.Prior = {}   # load_Prior()!!     
  
        
    def detection_request_callback(self, req):
        self.queue.append(req)    

    def detection_categories_callback(self, cats):
        self.Categories = cats.catlist

    # pop last request from queue, get pointcloud/image 
    def getData(self): 
        dr = self.queue.pop(0) # get on detectionReqeuest
        rospy.wait_for_service('/SM/get_candidate_service')   # wait till the service is available
        try:
            get_candidate_func = rospy.ServiceProxy('/SM/get_candidate_service', GetCandidate)
            request = recognition_msgs.srv.GetCandidateRequest() # provide the data to the request
            request.scene_id = dr.scene_id
            request.candidate_id = dr.candidate_id
            request.detector_id = self.DETECTOR_ID
            request.want_image = False
            request.want_point_cloud = True
            resp = get_candidate_func(request)    # call the service
            return resp.candidate  # there we have the pointcloud/image, now do some detection
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None
            
    def detect(self):
        while not rospy.is_shutdown():
            if len(self.queue) > 0:
                candidate = self.getData() 
                if not candidate == None:
                    if candidate.has_point_cloud == True:
                        rospy.loginfo(" new candidate arrived..... ")
                        # calculation of descriptor is done in C++, as a service call
                        try:
                            rospy.wait_for_service('/Feature_D2C/do_feature_D2C')
                            get_func = rospy.ServiceProxy('/Feature_D2C/do_feature_D2C', PointCloud2FeatureHistogram)
                            request = recognition_msgs.srv.PointCloud2FeatureHistogramRequest()
                            request.point_cloud = candidate.point_cloud
                            resp = get_func(request)    
                            
                            rankedlist = self.shapeDetector.compare_single(resp.hist, self.Categories, 'd2cRs2hw', '___L1') 
                            # DEBUG
                            rankedPath = []
                            for i,item in enumerate(rankedlist):
                                rankedPath.append(self.shapeDetector.directory + '/' + item[1] + '/views/' + item[2] + '.pcd ')
                                if i> 20:
                                    break
                                print item
                            for item in rankedPath:
                                print item
                            rl = RankedFileList()
                            rl.filelist = rankedPath
                            self.ranked_viz_pub.publish(rl)
#                            resultlist = self.shapeDetector.confidence_per_category(rankedlist)
                            resultlist = self.shapeDetector.bordacount(rankedlist, self.Categories)
                            
                            for idx, result in enumerate(resultlist):
                                hyp = Hypothesis()
                                hyp.scene_id = candidate.scene_id
                                hyp.candidate_id = candidate.candidate_id
                                hyp.detector_id = self.DETECTOR_ID    # here goes a id for each descriptor...!!!!!
                                hyp.category_str = result.category
                                
                                if result.category in self.Prior:
                                    hyp.confidence = result.confidence * self.Prior[result.category]
                                    print result.confidence
                                    print self.Prior[result.category]
                                else:
                                    hyp.confidence = result.confidence
                                
                                hyp.best_model = result.best_model
                                hyp.resultcount = len(resultlist)
                                self.pub.publish(hyp)
                                if idx < 10:
                                    rospy.loginfo("hyp: s_id:%d  c_id:%d  d_id:%s CAT = %s  conf = %f  model = %s", hyp.scene_id, 0, hyp.detector_id, hyp.category_str, hyp.confidence, hyp.best_model )


                        except rospy.ServiceException, e:
                            hyp = Hypothesis()
                            self.pub.publish(hyp)
                            print "Service call failed: %s"%e
                            rospy.loginfo("DUMMY detector hypothesis")


            else:
                time.sleep(0.1)                                        

    # a helper method, call this to calculate the descriptor/feature on the database (~dbpath)
    def do_Feature_on_DB(self):
        view_list = []
        #view_list = self.shapeDetector.provide_view_list(self.Categories)
        view_list.append('/home/walter/.ros/mug_clusters.pcd')
        view_list.append('/home/walter/.ros/mugb_clusters.pcd')
        for view in view_list:
            print view
            pts = PointCloud2Helper.read_pcd(view)
            if len(pts) < 50:
                continue
            cloud2 = PointCloud2Helper.create_cloud(pts)            
            rospy.wait_for_service('/Feature_D2C/do_feature_D2C')
            get_func = rospy.ServiceProxy('/Feature_D2C/do_feature_D2C', PointCloud2FeatureHistogram)
            request = recognition_msgs.srv.PointCloud2FeatureHistogramRequest()
            request.point_cloud = cloud2
            resp = get_func(request)    

            f = file(view + '.d2cRs2hw', "w")
            for v in resp.hist:
                f.write(str(v))
                f.write(" ")
            f.close()


def main(args):        
    rospy.init_node('detector_D2C', anonymous=False)
    dd = detector_D2C()
    dd.detect()



def test_do_Feature_on_DB():
    view_list = []
    view_list = open('/home/walter/3d_descriptor_stuff/V4RBrain/db.file').readlines()
    #print view_list
    
    for view in view_list:
        print view
        view = view.replace('\n','')
        pts = PointCloud2Helper.read_pcd(view)
        if len(pts) < 50:
            continue
        cloud2 = PointCloud2Helper.create_cloud(pts)            
        rospy.wait_for_service('/Feature_D2C/do_feature_D2C')
        get_func = rospy.ServiceProxy('/Feature_D2C/do_feature_D2C', PointCloud2FeatureHistogram)
        request = recognition_msgs.srv.PointCloud2FeatureHistogramRequest()
        request.point_cloud = cloud2
        resp = get_func(request)    

        f = file(view + '.d2cRs2hw', "w")
        for v in resp.hist:
            f.write(str(v))
            f.write(" ")
        f.close()

if __name__ == "__main__":     
    #test_do_Feature_on_DB()   
    main(sys.argv)

