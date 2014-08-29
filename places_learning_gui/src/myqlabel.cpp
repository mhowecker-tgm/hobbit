#include "../include/PlacesLearningGui/myqlabel.h"
#include <QPainter>
#include <QMouseEvent>
#include "hobbit_msgs/Point2D.h"

#include <math.h>

void MyQLabel ::drawArrow(QPoint p1, float th, QColor color)
{
    QPoint v_target(p1.x()+15*cos(th), p1.y()-15*sin(th));

    QPoint v_init(v_target.x()+3*cos(th+0.75*M_PI),v_target.y()-3*sin(th+0.75*M_PI)); 
    QPoint v_end(v_target.x()+3*cos(th-0.75*M_PI),v_target.y()-3*sin(th-0.75*M_PI));

    QPoint* points = new QPoint[3];
    points[0] = v_target;
    points[1] = v_init;
    points[2] = v_end;

    QPainter painter1(this);
    QPen paintpen1(color);
    paintpen1.setWidth(1);
    painter1.setPen(paintpen1);
    painter1.drawPolygon(points,3);

    painter1.drawLine(p1,v_target);
 

}


void MyQLabel :: paintEvent(QPaintEvent * e)
{

    QLabel::paintEvent(e);

    if(map_opened)
    {

	// draw current robot pose
	QPainter painter(this);
	QPen paintpen(Qt::red);
	paintpen.setWidth(5);
	    
	QPoint p1;

	float x_converted = ((*current_x) - map_origin_x)/(map_width_relation*map_resolution);
        float y_converted = (map_origin_y + input_map_height*map_resolution - (*current_y))/(map_height_relation*map_resolution);

	p1.setX(x_converted);
	p1.setY(y_converted);
	painter.setPen(paintpen);
	drawArrow(p1, (*current_theta), Qt::red);


	// draw rooms and places
        for(unsigned int i=0;i<rooms->rooms_vector.size();i++)
        {
         
               unsigned int n = rooms->rooms_vector[i].vertices_vector.size();
               QPoint* points = new QPoint[n];

               float x_sum=0;
               float y_sum=0;
               for (unsigned int j=0; j<n;j++)
               {
                   hobbit_msgs::Point2D corner = rooms->rooms_vector[i].vertices_vector[j];

                   QPoint vertex;
		   float vertex_x_converted = (corner.x - map_origin_x)/(map_width_relation*map_resolution);
		   float vertex_y_converted = (map_origin_y + input_map_height*map_resolution - corner.y)/(map_height_relation*map_resolution);

                   vertex.setX(vertex_x_converted);
                   vertex.setY(vertex_y_converted);

                   points[j] = vertex;

                   x_sum+=corner.x;
                   y_sum+=corner.y;

               }


		QPainter painter1(this);
                QPen paintpen1(Qt::blue);
                paintpen1.setWidth(2);
                painter1.setPen(paintpen1);
                painter1.drawPolygon(points,n);

		QPainter painter2(this);
                QPen paintpen2(Qt::blue);
                paintpen2.setWidth(2);
                painter2.setPen(paintpen2);

                /*QPoint cg(x_sum/n,y_sum/n);
                QString room_name(rooms->rooms_vector[i].room_name.c_str());

                painter.setFont(QFont("Arial",10));
                painter.drawText(cg,room_name);*/

		unsigned int n_places = rooms->rooms_vector[i].places_vector.size();
		QPoint* places = new QPoint[n_places];

		for (unsigned int j=0; j<n_places;j++)
                {
                   hobbit_msgs::Place place = rooms->rooms_vector[i].places_vector[j];

                   QPoint place_point;

		   float place_x_converted = (place.x - map_origin_x)/(map_width_relation*map_resolution);
 		   float place_y_converted = (map_origin_y + input_map_height*map_resolution - place.y)/(map_height_relation*map_resolution);

                   place_point.setX(place_x_converted);
                   place_point.setY(place_y_converted);

                   places[j] = place_point;
		   drawArrow(place_point, place.theta, Qt::cyan);

                }




        }




    }

}
