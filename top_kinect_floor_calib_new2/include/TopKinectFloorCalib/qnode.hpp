//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 30.3.2014
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef TopKinectFloorCalib_QNODE_HPP_
#define TopKinectFloorCalib_QNODE_HPP_
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <stdint.h>
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <QThread>
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define HIST_SIZE 100
#define MAX_ANG 45
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace TopKinectFloorCalib {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class QNode : public QThread
{
  Q_OBJECT

  private:
    int init_argc;
    char **init_argv;
    bool bRun;

    unsigned int ResultWidth, ResultHeight;
    float *pResult;
    float *MaxHistDisp;
    uint16_t *pHistAccu;
    uint16_t *MaxHistCnt;
    uint8_t *pUseMap;
    float VDCosLUT[2 * MAX_ANG];
    float VDSinLUT[2 * MAX_ANG];
    uint16_t pVDHoughAccu[4 * HIST_SIZE * MAX_ANG];

    void MakeVDLUTs(void);
    void MakeReducedResolutionDisparityImage(float *pDisp);
    void Gradient(float Min, float Max);
    void MakeReducedVDisparity(float *pDisp);
    void AnalyseHistogram(void);
    void DoVDHoughTransform(float &k, float &d);
    void Prune(float k, float d);
    void FitLine(void);
    void GetMaxDisparity(float *pDisp);

    void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& msg);

  public:
    uint8_t *pBuffer;
    unsigned int ImgWidth, ImgHeight;
    unsigned int ImgBytesPerRow;
    unsigned int DispStep;

    float k_disp, d_disp, nx_plane, ny_plane, nz_plane, d_plane;
    float maxDisparity;
    bool bShowLine, bShowMax;

    QNode(int argc, char **argv);
    virtual ~QNode();
    bool init();
    void run();

    void FLOAT32_to_RGB888(float *pIn, uint8_t *pOut);
    void VDisp_to_RGB888(uint16_t *pIn, uint8_t *pOut);
    void VDHoughAccu_to_RGB888(uint8_t *pOut);
    void Map_to_RGB888(uint8_t *pOut);

  Q_SIGNALS:
    void imageUpdated();
    void maxUpdated(double Max);
    void rosShutdown();
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

