
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef MYQLABEL_H
#define MYQLABEL_H
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <QLabel>
#include "hobbit_msgs/RoomsVector.h"

class MyQLabel : public QLabel
{

public:
    MyQLabel (QWidget * parent = 0, Qt::WindowFlags f = 0):QLabel(parent,f){map_opened = false;}
    MyQLabel (const QString & text, QWidget * parent = 0, Qt::WindowFlags f = 0):QLabel(text,parent,f){;}
    virtual ~MyQLabel(){}   

    virtual void paintEvent(QPaintEvent * e);

    hobbit_msgs::RoomsVector* rooms;

    bool map_opened;

    float* current_x;
    float* current_y;
    float* current_theta;

    void drawArrow(QPoint p1, float th, QColor color);

    float map_resolution;
    float map_size_relation;
    float input_map_height;
    float map_origin_x;
    float map_origin_y;


};

#endif // MYQLABEL_H
