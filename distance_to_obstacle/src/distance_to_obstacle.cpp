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

Mat distances;
double resolution;
Point2d origin;

bool d2o(distance_to_obstacle::distance_to_obstacle::Request &req,
         distance_to_obstacle::distance_to_obstacle::Response &res)
{
    Point2d query;
    query.x = req.p.x / resolution;
    query.y = req.p.y / resolution;
    query -= origin;

    cout << "Query point " << endl;
    Point2i query_px;
    query_px.x = (int)round(query.x);
    query_px.y = (int)round(query.y);

    res.d = distances.at<double>(query_px.y, query_px.x);
    return true;
}

void initializeDistances(Mat map)
{
    distanceTransform(map, distances, CV_DIST_L2, CV_DIST_MASK_PRECISE);

    distances *= resolution;

//    double m;
//    minMaxLoc(distances, NULL, &m, NULL, NULL);
//    cout << "MaxDistance = " << m << endl;

//    namedWindow("Image", WINDOW_AUTOSIZE);
//    namedWindow("Distances", WINDOW_AUTOSIZE);
//    imshow("Image", map);
//    imshow("Distances", distances/m);

//    waitKey(0);
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

    while(ifs.good())
    {
       ifs >> tmp;
       if(tmp.compare("resolution:") == 0)
       {
           if(!(ifs >> setprecision(5) >> resolution))
               return false;
           res = true;
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

       if(res && ori)
           break;
    }
    ifs.close();

    return (res && ori);
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

    Mat map = imread(pgmfile, CV_LOAD_IMAGE_GRAYSCALE);

    if(!map.data)
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

    initializeDistances(map);


    ros::ServiceServer service = n.advertiseService("distance_to_obstacle", d2o);

    ROS_INFO("Distance to obstacle service is ready.");
    ros::spin();

    return 0;
}
