/*  von david modifiziert 29.4.2011 (grundlage vtk_io.cpp)
 *
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: vtk_io.cpp 35701 2011-02-02 08:37:38Z rusu $
 *
 */

//#include <pcl/io/vtk_io.h>
#include "iv_io.h"
#include <fstream>
#include <iostream>
#include <pcl/io/io.h>
#include "sensor_msgs/PointCloud2.h"
#include <ros/ros.h>

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::io::saveIVFile (const std::string &file_name,
    const pcl::PolygonMesh &triangles, unsigned precision)
{
  if (triangles.cloud.data.empty ())
  {
    ROS_ERROR ("[pcl::io::saveIVFile] Input point cloud has no data!");
    return (-1);
  }

  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  int nr_points  = triangles.cloud.width * triangles.cloud.height;
  int point_size = triangles.cloud.data.size () / nr_points;

  // Write the header information
  //fs << "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS " << nr_points << " float" << std::endl;
  fs << "#Inventor V2.1 ascii" << std::endl;
  fs << "Separator {" << std::endl << std::endl;
  fs << "Coordinate3 {" << std::endl;
  fs << "point[" << std::endl;

  // Iterate through the points
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    for (size_t d = 0; d < triangles.cloud.fields.size (); ++d)
    {
      int count = triangles.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      if ((triangles.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
           triangles.cloud.fields[d].name == "x" ||
           triangles.cloud.fields[d].name == "y" ||
           triangles.cloud.fields[d].name == "z"))
      {
        float value;
        memcpy (&value, &triangles.cloud.data[i * point_size + triangles.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
        {
          fs << ",";
          break;
        }
      }
      fs << " ";
    }
    if (xyz != 3)
    {
      ROS_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no XYZ data!");
      return (-2);
    }
    fs << std::endl;
  }

  fs << "]" << std::endl; //point[
  fs << "}" << std::endl; //Coordinate{
  fs << "IndexedFaceSet {" << std::endl;
  fs << "coordIndex[" << std::endl;

  // Write polygons
  // note: Double check the second parameter!
  for (size_t i = 0; i < triangles.polygons.size (); ++i)
  {
    size_t j = 0;
    for (j = 0; j < triangles.polygons[i].vertices.size () - 1; ++j)
      fs << triangles.polygons[i].vertices[j] << ", ";
    fs << triangles.polygons[i].vertices[j] << ", -1," << std::endl;
  }

  // Write RGB values
  int field_index = getFieldIndex (triangles.cloud, "rgb");
  if (field_index != -1)
  {
    fs << "\nPOINT_DATA " << nr_points << "\nCOLOR_SCALARS scalars 3\n";
    for (int i = 0; i < nr_points; ++i)
    {
      int count = triangles.cloud.fields[field_index].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      if (triangles.cloud.fields[field_index].datatype == sensor_msgs::PointField::FLOAT32)
      {
        float value;
        memcpy (&value, &triangles.cloud.data[i * point_size + triangles.cloud.fields[field_index].offset + c * sizeof (float)], sizeof (float));
        int color = *reinterpret_cast<const int*>(&(value));
        int r = (0xff0000 & color) >> 16;
        int g = (0x00ff00 & color) >> 8;
        int b =  0x0000ff & color;
        fs << (float)r/255.0 << " " << (float)g/255.0 << " " << (float)b/255.0;
      }
      fs << std::endl;
    }
  }
  fs << "    ]" << std::endl; //coordIndex[
  fs << "}" << std::endl; //IndexedFaceSet{
  fs << "}" << std::endl; //PointSet{
  fs <<  std::endl;

  // Close file
  fs.close ();
  return (0);
}

