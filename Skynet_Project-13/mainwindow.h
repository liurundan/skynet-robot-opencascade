#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QString>

#include <gui_positions.h>
#include <gui_controls.h>

#include <kinematic.h>
#include <scurve.h>
#include <iostream>
#include <string.h>

//! Make conversion's easy:
#define toRadians M_PI/180.0
#define toDegrees (180.0/M_PI)

#ifdef Success
#undef Success
#endif

// libocc
#include <opencascade.h>
using namespace occ;

// libspline
#include <cubic_spline.h>

// libdxfrw
#include <libdxfrw_functions.h>

// libmotion
#include <create_program.h>
#include <create_stream.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    bool loadmodel();

private slots:

    void open_dxf();

    void stop();
    void reset();

    void on_toolButton_delete_item_pressed();

    void add_startpoint();
    void add_waypoint();
    void add_endpoint();
    void add_io();
    void set_stackedwidget_index(int i);

    void make_program();
    void goto_startposition_program();
    void play_program();

private:
    Ui::MainWindow *ui;

    Opencascade *OpencascadeWidget;

    gui_positions* positions;
    gui_controls* controls;

    void mainloop();
    void add_point(std::string type); // Startpoint, waypoint or endpoint.

    std::vector<gp_Pnt> pointvec;
    gp_Trsf euler;
    std::vector<gp_Euler> eulervec;
    std::vector<Opencascade::io> iovec;

    bool programplay=0;
    bool programstartpoint=0;
    bool okstart=0;

    unsigned int file=0;

};
#endif // MAINWINDOW_H





















