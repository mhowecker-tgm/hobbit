#ifndef MYPANEL_H
#define MYPANEL_H

#include <QtOpenGL/QGLWidget>

class MyPanel : public QGLWidget
{
    Q_OBJECT
public:
    explicit MyPanel(QWidget *parent = 0);
    
Q_SIGNALS:
    
public Q_SLOTS:


protected:
    void initializeGL();
    void resizeGL(int x, int h);
    void paintGL();

private:

    
};

#endif // MYPANEL_H
