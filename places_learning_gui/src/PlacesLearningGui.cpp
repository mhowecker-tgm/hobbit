//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Paloma de la Puente
// Last update: 15.4.2014
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "ros/package.h"
#include <QtGui>
#include <QApplication>
#include "../include/PlacesLearningGui/mainwindow.h"
using namespace std;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    MainWindow w(argc,argv);
    w.show();
    
    return a.exec();
  
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


