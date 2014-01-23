#include <iostream>
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include "ros/param.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "distance_to_obstacle/distance_to_obstacle.h"

using namespace cv;
using namespace std;

int width;
int height;
Mat distances;
Mat occupancy;

double resolution;
double freeThresh;
double occupiedThresh;
Point2d origin;

bool d2o(distance_to_obstacle::distance_to_obstacle::Request &req,
         distance_to_obstacle::distance_to_obstacle::Response &res)
{
    Point2d query;    
    query.x = req.p.x;
    query.y = req.p.y;

    query -= origin;

    query.x /= resolution;
    query.y /= resolution;

    Point2i query_px;
    query_px.x = (int)round(query.x);
    query_px.y = distances.rows-(int)round(query.y);

    // check if coordinate is outside of map
    if(query_px.x < 0 || query_px.x >= width || query_px.y < 0 || query_px.y >= height)
    {
        res.d = -1;
        ROS_INFO("Target point outside of map!");
    }
    else
    {
        res.d = distances.at<float>(query_px.y, query_px.x);
        float occupancyValue = occupancy.at<float>(query_px.y, query_px.x);

        if(occupancyValue > freeThresh && occupancyValue < occupiedThresh)
        {
            res.d *= -1.0;
            ROS_INFO("Target point is in unexplored area.");
        }
    }

    return true;
}

void initializeDistances(Mat mapimage)
{
    distanceTransform(mapimage, distances, CV_DIST_L2, CV_DIST_MASK_PRECISE);

    distances *= resolution;

    width = distances.cols;
    height = distances.rows;

    Mat tmp;
    mapimage.convertTo(tmp, CV_32F);
    occupancy = (255.0f - tmp) / 255.0;
}

bool loadYamlFile(string yamlfile)
{
    ifstream ifs(yamlfile.c_str());

    if(!ifs)
    {
        return false;
    }

    string tmp;
    double x,y,z;
    bool res = false;
    bool ori = false;
    bool othresh = false;
    bool fthresh = false;

    while(ifs.good())
    {
       if(!(ifs >> tmp))
           break;

       if(tmp.compare("resolution:") == 0)
       {
           if(!(ifs >> setprecision(5) >> resolution))
               return false;
           res = true;
       }
       else if(tmp.compare("occupied_thresh:") == 0)
       {
           if(!(ifs >> setprecision(5) >> occupiedThresh))
               return false;
           othresh = true;
       }
       else if(tmp.compare("free_thresh:") == 0)
       {
           if(!(ifs >> setprecision(5) >> freeThresh))
               return false;
           fthresh = true;
       }
       else if(tmp.compare("origin:") == 0)
       {
           getline(ifs, tmp);
           int i = tmp.find_first_of('[');
           int j = tmp.find_last_of(']');
           string sub = tmp.substr(i+1,j-i-1);

           stringstream s(sub);
           if(!(s >> setprecision(6) >> x))
               return false;
           if(!(s >> tmp))
               return false;
           if(!(s >> y))
               return false;
           if(!(s >> tmp))
               return false;
           if(!(s >> z))
               return false;

           origin.x = x;
           origin.y = y;
           ori = true;
       }
    }
    ifs.close();

    return (res && ori && othresh && fthresh);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_to_obstacle");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    string pgmfile;

    if(!nh.getParam("pgmfile", pgmfile) || pgmfile.empty())
    {
        ROS_ERROR("Path to PGM file has not been set!");
        return 0;
    }

    Mat mapimage = imread(pgmfile, CV_LOAD_IMAGE_GRAYSCALE);

    if(!mapimage.data)
    {
        string tmp = "Map file " + pgmfile + " could not be loaded!";
        ROS_ERROR(tmp.c_str());
        return 0;
    }

    // cut off ".pgm"
    string yamlfile = pgmfile.substr(0, pgmfile.length()-4);
    yamlfile += ".yaml";

    if(!loadYamlFile(yamlfile))
    {
        string tmp = "Error reading map configuration file " + yamlfile;
        ROS_ERROR(tmp.c_str());
        return 0;
    }

    initializeDistances(mapimage);

    ros::ServiceServer service = n.advertiseService("distance_to_obstacle", d2o);

    ROS_INFO("Distance to obstacle service is ready.");
    ros::spin();

    return 0;
}
