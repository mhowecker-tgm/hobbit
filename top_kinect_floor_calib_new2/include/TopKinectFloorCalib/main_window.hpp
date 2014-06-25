//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Peter Einramhof
// Last update: 20.3.2013
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef TopKinectFloorCalib_MAIN_WINDOW_H
#define TopKinectFloorCalib_MAIN_WINDOW_H
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <new>
#include <stdint.h>
#include <QObject>
#include <QWidget>
#include <QPushButton>
#include <QImage>
#include <QCheckBox>
#include <QLabel>
#include "qnode.hpp"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
namespace TopKinectFloorCalib {
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class MainWindow : public QWidget
{
  Q_OBJECT

  public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

  public Q_SLOTS:
    void updateQImage();
    void updateMax(double Max);

  private Q_SLOTS:
    void connectRequest(void);
    void nominalRequest(void);
    void maximumRequest(void);
    void minimumRequest(void);
    void saveRequest(void);
    void lineRequest(void);
    void maxRequest(void);

  private:
    QNode qnode;

    bool bConnected;
    uint8_t *ImageBuffer;
    QImage *pImage;
    QPushButton *pConnect;
    QPushButton *pNominal;
    QPushButton *pMaximum;
    QPushButton *pMinimum;
    QPushButton *pSave;
    QCheckBox *pLine;
    QLabel *pLabel;
    QCheckBox *pMax;
    QLabel *pLabel2;

    float k_nom, d_nom, k_min, d_min, k_max, d_max;
    float nx, ny, nz, d;

    void paintEvent(QPaintEvent *event);
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

