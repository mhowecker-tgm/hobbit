#ifndef HOBBIT_CALIBRATION_DATA_STRUCTURES_H_INCLUDED
#define HOBBIT_CALIBRATION_DATA_STRUCTURES_H_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

 
enum calibIntrinsicsHobbit
{
  HUBT_CALIB_INTR_FX = 0 ,
  HUBT_CALIB_INTR_FY = 4 ,
  HUBT_CALIB_INTR_CX = 2 ,
  HUBT_CALIB_INTR_CY = 5
};

struct calibrationHobbit
{
  // CAMERA INTRINSIC PARAMETERS
  char intrinsicParametersSet;
  double intrinsic[9];
  double k1,k2,p1,p2,k3;

  // CAMERA EXTRINSIC PARAMETERS
  char extrinsicParametersSet;
  double extrinsicRotationRodriguez[3];
  double extrinsicTranslation[3];
  double extrinsic[16];

  //CAMERA DIMENSIONS ( WHEN RENDERING )
  double nearPlane,farPlane;
  unsigned int width;
  unsigned int height;

  double depthUnit;

  //CONFIGURATION
  int imagesUsed;
  int boardWidth;
  int boardHeight;
  double squareSize;

  //---Make Sure we don't change from version to version
  unsigned long guardByte;
};



#ifdef __cplusplus
}
#endif


#endif
