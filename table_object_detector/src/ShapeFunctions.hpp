
#include <pcl/common/common.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/features/feature.h>
//#include <pcl/common/transform.h>
#include "pcl/features/moment_invariants.h"
#include <pcl/registration/transforms.h>
#include "pcl/filters/passthrough.h"
#include "pcl/features/normal_3d.h"
//#include "pcl/features/normal_3d_tbb.h"
#include "pcl/kdtree/kdtree.h"
//#include "pcl/kdtree/kdtree_ann.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
//#include "pcl_ros/subscriber.h"
#include "pcl_ros/publisher.h"
#include <pcl/features/vfh.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/pca.h>
#include "hobbit_msgs/PointCloud2FeatureHistogram.h"

#define _USE_MATH_DEFINES

#define GRIDSIZE 64
#define GRIDSIZE_H GRIDSIZE/2

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <opencv/cv.h>
#include <fstream>
#include <string>
#include <vector>
using namespace std;
#include <ros/ros.h>

class SF
{
public:
	ros::NodeHandle nh_;
	vector<vector<vector<bool> > > lut;
    ros::ServiceServer service;
    pcl::PointCloud<pcl::PointXYZ> local_cloud_;
    SF(): nh_ ("~")
	{
		lut.resize(GRIDSIZE);
		for (int i = 0; i < GRIDSIZE; ++i)
		{
			lut[i].resize(GRIDSIZE);
			for (int j = 0; j < GRIDSIZE; ++j)
				lut[i][j].resize(GRIDSIZE);
		}

    	this->service = nh_.advertiseService("do_feature_SF", &SF::do_SF, this);
	}

	bool do_SF(HobbitMsgs::PointCloud2FeatureHistogram::Request& req, HobbitMsgs::PointCloud2FeatureHistogram::Response& res)
	{

		ros::Time t1 = ros::Time::now ();
	    pcl::PointCloud<pcl::PointXYZ> cloud;
	    pcl::fromROSMsg (req.point_cloud, cloud);
	    Eigen::Vector4f xyz_centroid;
	    float scale = this->scale_points_unit_sphere (cloud, static_cast<float>(GRIDSIZE_H), xyz_centroid);
	    this->voxelize9 (local_cloud_);
	    this->computeSF (local_cloud_, res.hist);
	    this->cleanup9 (local_cloud_);
	    res.hist.push_back(scale);
	    ROS_WARN ("SERVICE_CALL done :: Spent %f seconds in 'do_SF'.", (ros::Time::now () - t1).toSec ());
	    return true;
	}


	void computeSF(pcl::PointCloud<pcl::PointXYZ> &pc, vector<float> &hist)
	{
	  const int binsize = 64;
	  unsigned int sample_size = 20000;
	  srand (static_cast<unsigned int> (time (0)));
	  int maxindex = static_cast<int> (pc.points.size ());

	  int index1, index2, index3;
	  std::vector<float> d2v, d1v, d3v, wt_d3;
	  std::vector<int> wt_d2;
	  d1v.reserve (sample_size);
	  d2v.reserve (sample_size * 3);
	  d3v.reserve (sample_size);
	  wt_d2.reserve (sample_size * 3);
	  wt_d3.reserve (sample_size);

	  float h_in[binsize] = {0};
	  float h_out[binsize] = {0};
	  float h_mix[binsize] = {0};
	  float h_mix_ratio[binsize] = {0};

	  float h_a3_in[binsize] = {0};
	  float h_a3_out[binsize] = {0};
	  float h_a3_mix[binsize] = {0};
	  float h_d1[binsize] = {0};

	  float h_d3_in[binsize] = {0};
	  float h_d3_out[binsize] = {0};
	  float h_d3_mix[binsize] = {0};

	  float ratio=0.0;
	  float pih = static_cast<float>(M_PI) / 2.0f;
	  float a,b,c,s;
	  int th1,th2,th3;
	  int vxlcnt = 0;
	  int pcnt1,pcnt2,pcnt3;
	  for (size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx)
	  {
	    // get a new random point
	    index1 = rand()%maxindex;
	    index2 = rand()%maxindex;
	    index3 = rand()%maxindex;

	    if (index1==index2 || index1 == index3 || index2 == index3)
	    {
	      nn_idx--;
	      continue;
	    }

	    Eigen::Vector4f p1 = pc.points[index1].getVector4fMap ();
	    Eigen::Vector4f p2 = pc.points[index2].getVector4fMap ();
	    Eigen::Vector4f p3 = pc.points[index3].getVector4fMap ();

	    // A3
	    Eigen::Vector4f v21 (p2 - p1);
	    Eigen::Vector4f v31 (p3 - p1);
	    Eigen::Vector4f v23 (p2 - p3);
	    a = v21.norm (); b = v31.norm (); c = v23.norm (); s = (a+b+c) * 0.5f;
	    if (s * (s-a) * (s-b) * (s-c) <= 0.001f)
	      continue;

	    v21.normalize ();
	    v31.normalize ();
	    v23.normalize ();

	    //TODO: .dot gives nan's
	    th1 = static_cast<int> (pcl_round (acos (fabs (v21.dot (v31))) / pih * (binsize-1)));
	    th2 = static_cast<int> (pcl_round (acos (fabs (v23.dot (v31))) / pih * (binsize-1)));
	    th3 = static_cast<int> (pcl_round (acos (fabs (v23.dot (v21))) / pih * (binsize-1)));
	    if (th1 < 0 || th1 >= binsize)
	    {
	      nn_idx--;
	      continue;
	    }
	    if (th2 < 0 || th2 >= binsize)
	    {
	      nn_idx--;
	      continue;
	    }
	    if (th3 < 0 || th3 >= binsize)
	    {
	      nn_idx--;
	      continue;
	    }

	    //pcl::PointXYZ cog(((rand()%100)-50.0f) / 100.0f,((rand()%100)-50.0f) / 100.0f,((rand()%100)-50.0f) / 100.0f);
	    // D1
	    //                      d1v.push_back( pcl::euclideanDistance(cog, pc.points[index1]) );

	    // D2
	    d2v.push_back (pcl::euclideanDistance (pc.points[index1], pc.points[index2]));
	    d2v.push_back (pcl::euclideanDistance (pc.points[index1], pc.points[index3]));
	    d2v.push_back (pcl::euclideanDistance (pc.points[index2], pc.points[index3]));

	    int vxlcnt_sum = 0;
	    int p_cnt = 0;
	    // IN, OUT, MIXED, Ratio line tracing, index1->index2
	    {
	      const int xs = p1[0] < 0.0? static_cast<int>(floor(p1[0])+GRIDSIZE_H): static_cast<int>(ceil(p1[0])+GRIDSIZE_H-1);
	      const int ys = p1[1] < 0.0? static_cast<int>(floor(p1[1])+GRIDSIZE_H): static_cast<int>(ceil(p1[1])+GRIDSIZE_H-1);
	      const int zs = p1[2] < 0.0? static_cast<int>(floor(p1[2])+GRIDSIZE_H): static_cast<int>(ceil(p1[2])+GRIDSIZE_H-1);
	      const int xt = p2[0] < 0.0? static_cast<int>(floor(p2[0])+GRIDSIZE_H): static_cast<int>(ceil(p2[0])+GRIDSIZE_H-1);
	      const int yt = p2[1] < 0.0? static_cast<int>(floor(p2[1])+GRIDSIZE_H): static_cast<int>(ceil(p2[1])+GRIDSIZE_H-1);
	      const int zt = p2[2] < 0.0? static_cast<int>(floor(p2[2])+GRIDSIZE_H): static_cast<int>(ceil(p2[2])+GRIDSIZE_H-1);
	      wt_d2.push_back (this->lci (xs, ys, zs, xt, yt, zt, ratio, vxlcnt, pcnt1));
	      if (wt_d2.back () == 2)
	        h_mix_ratio[static_cast<int> (pcl_round (ratio * (binsize-1)))]++;
	      vxlcnt_sum += vxlcnt;
	      p_cnt += pcnt1;
	    }
	    // IN, OUT, MIXED, Ratio line tracing, index1->index3
	    {
	      const int xs = p1[0] < 0.0? static_cast<int>(floor(p1[0])+GRIDSIZE_H): static_cast<int>(ceil(p1[0])+GRIDSIZE_H-1);
	      const int ys = p1[1] < 0.0? static_cast<int>(floor(p1[1])+GRIDSIZE_H): static_cast<int>(ceil(p1[1])+GRIDSIZE_H-1);
	      const int zs = p1[2] < 0.0? static_cast<int>(floor(p1[2])+GRIDSIZE_H): static_cast<int>(ceil(p1[2])+GRIDSIZE_H-1);
	      const int xt = p3[0] < 0.0? static_cast<int>(floor(p3[0])+GRIDSIZE_H): static_cast<int>(ceil(p3[0])+GRIDSIZE_H-1);
	      const int yt = p3[1] < 0.0? static_cast<int>(floor(p3[1])+GRIDSIZE_H): static_cast<int>(ceil(p3[1])+GRIDSIZE_H-1);
	      const int zt = p3[2] < 0.0? static_cast<int>(floor(p3[2])+GRIDSIZE_H): static_cast<int>(ceil(p3[2])+GRIDSIZE_H-1);
	      wt_d2.push_back (this->lci (xs, ys, zs, xt, yt, zt, ratio, vxlcnt, pcnt2));
	      if (wt_d2.back () == 2)
	        h_mix_ratio[static_cast<int>(pcl_round (ratio * (binsize-1)))]++;
	      vxlcnt_sum += vxlcnt;
	      p_cnt += pcnt2;
	    }
	    // IN, OUT, MIXED, Ratio line tracing, index2->index3
	    {
	      const int xs = p2[0] < 0.0? static_cast<int>(floor(p2[0])+GRIDSIZE_H): static_cast<int>(ceil(p2[0])+GRIDSIZE_H-1);
	      const int ys = p2[1] < 0.0? static_cast<int>(floor(p2[1])+GRIDSIZE_H): static_cast<int>(ceil(p2[1])+GRIDSIZE_H-1);
	      const int zs = p2[2] < 0.0? static_cast<int>(floor(p2[2])+GRIDSIZE_H): static_cast<int>(ceil(p2[2])+GRIDSIZE_H-1);
	      const int xt = p3[0] < 0.0? static_cast<int>(floor(p3[0])+GRIDSIZE_H): static_cast<int>(ceil(p3[0])+GRIDSIZE_H-1);
	      const int yt = p3[1] < 0.0? static_cast<int>(floor(p3[1])+GRIDSIZE_H): static_cast<int>(ceil(p3[1])+GRIDSIZE_H-1);
	      const int zt = p3[2] < 0.0? static_cast<int>(floor(p3[2])+GRIDSIZE_H): static_cast<int>(ceil(p3[2])+GRIDSIZE_H-1);
	      wt_d2.push_back (this->lci (xs,ys,zs,xt,yt,zt,ratio,vxlcnt,pcnt3));
	      if (wt_d2.back () == 2)
	        h_mix_ratio[static_cast<int>(pcl_round(ratio * (binsize-1)))]++;
	      vxlcnt_sum += vxlcnt;
	      p_cnt += pcnt3;
	    }

	    // D3 ( herons formula )
	    d3v.push_back (sqrt (sqrt (s * (s-a) * (s-b) * (s-c))));
	    if (vxlcnt_sum <= 21)
	    {
	      wt_d3.push_back (0);
	      h_a3_out[th1] += static_cast<float> (pcnt3) / 32.0f;
	      h_a3_out[th2] += static_cast<float> (pcnt1) / 32.0f;
	      h_a3_out[th3] += static_cast<float> (pcnt2) / 32.0f;
	    }
	    else
	      if (p_cnt - vxlcnt_sum < 4)
	      {
	        h_a3_in[th1] += static_cast<float> (pcnt3) / 32.0f;
	        h_a3_in[th2] += static_cast<float> (pcnt1) / 32.0f;
	        h_a3_in[th3] += static_cast<float> (pcnt2) / 32.0f;
	        wt_d3.push_back (1);
	      }
	      else
	      {
	        h_a3_mix[th1] += static_cast<float> (pcnt3) / 32.0f;
	        h_a3_mix[th2] += static_cast<float> (pcnt1) / 32.0f;
	        h_a3_mix[th3] += static_cast<float> (pcnt2) / 32.0f;
	        wt_d3.push_back (static_cast<float> (vxlcnt_sum) / static_cast<float> (p_cnt));
	      }
	  }
	  // Normalizing, get max
	  float maxd1 = 0;
	  float maxd2 = 0;
	  float maxd3 = 0;

	  for (size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx)
	  {
	    // get max of Dx
	    if (d1v[nn_idx] > maxd1)
	      maxd1 = d1v[nn_idx];
	    if (d2v[nn_idx] > maxd2)
	      maxd2 = d2v[nn_idx];
	    if (d2v[sample_size + nn_idx] > maxd2)
	      maxd2 = d2v[sample_size + nn_idx];
	    if (d2v[sample_size*2 +nn_idx] > maxd2)
	      maxd2 = d2v[sample_size*2 +nn_idx];
	    if (d3v[nn_idx] > maxd3)
	      maxd3 = d3v[nn_idx];
	  }

	  // Normalize and create histogram
	  int index;
	  for (size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx)
	  {
	    h_d1[static_cast<int>(pcl_round (d1v[nn_idx] / maxd1 * (binsize-1)))]++ ;

	    if (wt_d3[nn_idx] >= 0.999) // IN
	    {
	      index = static_cast<int>(pcl_round (d3v[nn_idx] / maxd3 * (binsize-1)));
	      if (index >= 0 && index < binsize)
	        h_d3_in[index]++;
	    }
	    else
	    {
	      if (wt_d3[nn_idx] <= 0.001) // OUT
	      {
	        index = static_cast<int>(pcl_round (d3v[nn_idx] / maxd3 * (binsize-1)));
	        if (index >= 0 && index < binsize)
	          h_d3_out[index]++ ;
	      }
	      else
	      {
	        index = static_cast<int>(pcl_round (d3v[nn_idx] / maxd3 * (binsize-1)));
	        if (index >= 0 && index < binsize)
	          h_d3_mix[index]++;
	      }
	    }
	  }
	  //normalize and create histogram
	  for (size_t nn_idx = 0; nn_idx < d2v.size(); ++nn_idx )
	  {
	    if (wt_d2[nn_idx] == 0)
	      h_in[static_cast<int>(pcl_round (d2v[nn_idx] / maxd2 * (binsize-1)))]++ ;
	    if (wt_d2[nn_idx] == 1)
	      h_out[static_cast<int>(pcl_round (d2v[nn_idx] / maxd2 * (binsize-1)))]++;
	    if (wt_d2[nn_idx] == 2)
	      h_mix[static_cast<int>(pcl_round (d2v[nn_idx] / maxd2 * (binsize-1)))]++ ;
	  }

	  //float weights[10] = {1,  1,  1,  1,  1,  1,  1,  1 , 1 ,  1};
	  float weights[10] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 1.0f,  1.0f, 2.0f, 2.0f, 2.0f};

	  hist.reserve (binsize * 10);
	  for (int i = 0; i < binsize; i++)
	    hist.push_back (h_a3_in[i] * weights[0]);
	  for (int i = 0; i < binsize; i++)
	    hist.push_back (h_a3_out[i] * weights[1]);
	  for (int i = 0; i < binsize; i++)
	    hist.push_back (h_a3_mix[i] * weights[2]);

	  for (int i = 0; i < binsize; i++)
	    hist.push_back (h_d3_in[i] * weights[3]);
	  for (int i = 0; i < binsize; i++)
	    hist.push_back (h_d3_out[i] * weights[4]);
	  for (int i = 0; i < binsize; i++)
	    hist.push_back (h_d3_mix[i] * weights[5]);

	  for (int i = 0; i < binsize; i++)
	    hist.push_back (h_in[i]*0.5f * weights[6]);
	  for (int i = 0; i < binsize; i++)
	    hist.push_back (h_out[i] * weights[7]);
	  for (int i = 0; i < binsize; i++)
	    hist.push_back (h_mix[i] * weights[8]);
	  for (int i = 0; i < binsize; i++)
	    hist.push_back (h_mix_ratio[i]*0.5f * weights[9]);

	  float sm = 0;
	  for (size_t i = 0; i < hist.size (); i++)
	    sm += hist[i];

	  for (size_t i = 0; i < hist.size (); i++)
	    hist[i] /= sm;






	}


	int lci (
		    const int x1, const int y1, const int z1,
		    const int x2, const int y2, const int z2,
		    float &ratio, int &incnt, int &pointcount)
		{
		  int voxelcount = 0;
		  int voxel_in = 0;
		  int act_voxel[3];
		  act_voxel[0] = x1;
		  act_voxel[1] = y1;
		  act_voxel[2] = z1;
		  int x_inc, y_inc, z_inc;
		  int dx = x2 - x1;
		  int dy = y2 - y1;
		  int dz = z2 - z1;
		  if (dx < 0)
		    x_inc = -1;
		  else
		    x_inc = 1;
		  int l = abs (dx);
		  if (dy < 0)
		    y_inc = -1 ;
		  else
		    y_inc = 1;
		  int m = abs (dy);
		  if (dz < 0)
		    z_inc = -1 ;
		  else
		    z_inc = 1;
		  int n = abs (dz);
		  int dx2 = 2 * l;
		  int dy2 = 2 * m;
		  int dz2 = 2 * n;
		  if ((l >= m) & (l >= n))
		  {
		    int err_1 = dy2 - l;
		    int err_2 = dz2 - l;
		    for (int i = 1; i<l; i++)
		    {
		      voxelcount++;;
		      voxel_in +=  static_cast<int>(lut[act_voxel[0]][act_voxel[1]][act_voxel[2]] == 1);
		      if (err_1 > 0)
		      {
		        act_voxel[1] += y_inc;
		        err_1 -=  dx2;
		      }
		      if (err_2 > 0)
		      {
		        act_voxel[2] += z_inc;
		        err_2 -= dx2;
		      }
		      err_1 += dy2;
		      err_2 += dz2;
		      act_voxel[0] += x_inc;
		    }
		  }
		  else if ((m >= l) & (m >= n))
		  {
		    int err_1 = dx2 - m;
		    int err_2 = dz2 - m;
		    for (int i=1; i<m; i++)
		    {
		      voxelcount++;
		      voxel_in +=  static_cast<int>(lut[act_voxel[0]][act_voxel[1]][act_voxel[2]] == 1);
		      if (err_1 > 0)
		      {
		        act_voxel[0] +=  x_inc;
		        err_1 -= dy2;
		      }
		      if (err_2 > 0)
		      {
		        act_voxel[2] += z_inc;
		        err_2 -= dy2;
		      }
		      err_1 += dx2;
		      err_2 += dz2;
		      act_voxel[1] += y_inc;
		    }
		  }
		  else
		  {
		    int err_1 = dy2 - n;
		    int err_2 = dx2 - n;
		    for (int i=1; i<n; i++)
		    {
		      voxelcount++;
		      voxel_in +=  static_cast<int>(lut[act_voxel[0]][act_voxel[1]][act_voxel[2]] == 1);
		      if (err_1 > 0)
		      {
		        act_voxel[1] += y_inc;
		        err_1 -= dz2;
		      }
		      if (err_2 > 0)
		      {
		        act_voxel[0] += x_inc;
		        err_2 -= dz2;
		      }
		      err_1 += dy2;
		      err_2 += dx2;
		      act_voxel[2] += z_inc;
		    }
		  }
		  voxelcount++;
		  voxel_in +=  static_cast<int>(lut[act_voxel[0]][act_voxel[1]][act_voxel[2]] == 1);
		  incnt = voxel_in;
		  pointcount = voxelcount;

		  if (voxel_in >=  voxelcount-1)
		    return (0);

		  if (voxel_in <= 7)
		    return (1);

		  ratio = static_cast<float>(voxel_in) / static_cast<float>(voxelcount);
		  return (2);
		}


    void voxelize9(pcl::PointCloud<pcl::PointXYZ> &cluster)
    {
      int xi,yi,zi,xx,yy,zz;
      for (size_t i = 0; i < cluster.points.size (); ++i)
      {
        xx = cluster.points[i].x<0.0? static_cast<int>(floor(cluster.points[i].x)+GRIDSIZE_H) : static_cast<int>(ceil(cluster.points[i].x)+GRIDSIZE_H-1);
        yy = cluster.points[i].y<0.0? static_cast<int>(floor(cluster.points[i].y)+GRIDSIZE_H) : static_cast<int>(ceil(cluster.points[i].y)+GRIDSIZE_H-1);
        zz = cluster.points[i].z<0.0? static_cast<int>(floor(cluster.points[i].z)+GRIDSIZE_H) : static_cast<int>(ceil(cluster.points[i].z)+GRIDSIZE_H-1);

        for (int x = -1; x < 2; x++)
          for (int y = -1; y < 2; y++)
            for (int z = -1; z < 2; z++)
            {
              xi = xx + x;
              yi = yy + y;
              zi = zz + z;

              if (yi >= GRIDSIZE || xi >= GRIDSIZE || zi>=GRIDSIZE || yi < 0 || xi < 0 || zi < 0)
              {
                ;
              }
              else
                this->lut[xi][yi][zi] = 1;
            }
      }
    }


    void cleanup9(pcl::PointCloud<pcl::PointXYZ> &cluster)
    {
    	  int xi,yi,zi,xx,yy,zz;
    	  for (size_t i = 0; i < cluster.points.size (); ++i)
    	  {
    	    xx = cluster.points[i].x<0.0? static_cast<int>(floor(cluster.points[i].x)+GRIDSIZE_H) : static_cast<int>(ceil(cluster.points[i].x)+GRIDSIZE_H-1);
    	    yy = cluster.points[i].y<0.0? static_cast<int>(floor(cluster.points[i].y)+GRIDSIZE_H) : static_cast<int>(ceil(cluster.points[i].y)+GRIDSIZE_H-1);
    	    zz = cluster.points[i].z<0.0? static_cast<int>(floor(cluster.points[i].z)+GRIDSIZE_H) : static_cast<int>(ceil(cluster.points[i].z)+GRIDSIZE_H-1);

    	    for (int x = -1; x < 2; x++)
    	      for (int y = -1; y < 2; y++)
    	        for (int z = -1; z < 2; z++)
    	        {
    	          xi = xx + x;
    	          yi = yy + y;
    	          zi = zz + z;

    	          if (yi >= GRIDSIZE || xi >= GRIDSIZE || zi>=GRIDSIZE || yi < 0 || xi < 0 || zi < 0)
    	          {
    	            ;
    	          }
    	          else
    	            this->lut[xi][yi][zi] = 0;
    	        }
    	  }
    }


    float scale_points_unit_sphere (
    	    const pcl::PointCloud<pcl::PointXYZ> &pc, float scalefactor, Eigen::Vector4f& centroid)
    	{

    	  pcl::compute3DCentroid (pc, centroid);
    	  pcl::demeanPointCloud (pc, centroid, local_cloud_);

    	  float max_distance = 0, d;
    	  pcl::PointXYZ cog (0, 0, 0);

    	  for (size_t i = 0; i < local_cloud_.points.size (); ++i)
    	  {
    	    d = pcl::euclideanDistance(cog,local_cloud_.points[i]);
    	    if (d > max_distance)
    	      max_distance = d;
    	  }

    	  float scale_factor = 1.0f / max_distance * scalefactor;

    	  Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
    	  matrix.scale (scale_factor);
    	  pcl::transformPointCloud (local_cloud_, local_cloud_, matrix);

    	  return max_distance;
    	}

};
