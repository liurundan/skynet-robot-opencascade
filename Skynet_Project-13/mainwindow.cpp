#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <kinematic.h>
#include <halio.h>
#include <jog.h>
#include <iomanip>
#include <fstream>
#include <dirent.h>
#include <variable.h>

//! Make conversion's easy:
#define toRadians M_PI/180.0
#define toDegrees (180.0/M_PI)

using namespace Eigen;
using namespace occ;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QObject::connect(ui->toolButton_emergency, SIGNAL(pressed()),SLOT(stop()));
    QObject::connect(ui->toolButton_reset, SIGNAL(pressed()),SLOT(reset()));

    QObject::connect(ui->toolButton_point_startpoint, SIGNAL(pressed()),SLOT(add_startpoint()));
    QObject::connect(ui->toolButton_line_startpoint, SIGNAL(pressed()),SLOT(add_startpoint()));
    QObject::connect(ui->toolButton_wire_startpoint, SIGNAL(pressed()),SLOT(add_startpoint()));
    QObject::connect(ui->toolButton_arc_startpoint, SIGNAL(pressed()),SLOT(add_startpoint()));
    QObject::connect(ui->toolButton_spline_startpoint, SIGNAL(pressed()),SLOT(add_startpoint()));
    QObject::connect(ui->toolButton_circle_startpoint, SIGNAL(pressed()),SLOT(add_startpoint()));

    QObject::connect(ui->toolButton_arc_waypoint, SIGNAL(pressed()),SLOT(add_waypoint()));
    QObject::connect(ui->toolButton_wire_waypoint, SIGNAL(pressed()),SLOT(add_waypoint()));
    QObject::connect(ui->toolButton_spline_waypoint, SIGNAL(pressed()),SLOT(add_waypoint()));
    QObject::connect(ui->toolButton_circle_waypoint, SIGNAL(pressed()),SLOT(add_waypoint()));

    QObject::connect(ui->toolButton_line_endpoint, SIGNAL(pressed()),SLOT(add_endpoint()));
    QObject::connect(ui->toolButton_wire_endpoint, SIGNAL(pressed()),SLOT(add_endpoint()));
    QObject::connect(ui->toolButton_arc_endpoint, SIGNAL(pressed()),SLOT(add_endpoint()));
    QObject::connect(ui->toolButton_circle_endpoint, SIGNAL(pressed()),SLOT(add_endpoint()));
    QObject::connect(ui->toolButton_spline_endpoint, SIGNAL(pressed()),SLOT(add_endpoint()));

    QObject::connect(ui->toolButton_add_io, SIGNAL(pressed()),SLOT(add_io()));

    QObject::connect(ui->comboBox_primitivetype, SIGNAL(currentIndexChanged(int)),SLOT(set_stackedwidget_index(int)));

    QObject::connect(ui->pushButton_open_dxf, SIGNAL(pressed()),SLOT(open_dxf()));

    QObject::connect(ui->toolButton_create_program, SIGNAL(pressed()),SLOT(make_program()));
    QObject::connect(ui->toolButton_play_program, SIGNAL(pressed()),SLOT(play_program()));
    QObject::connect(ui->toolButton_goto_startposition, SIGNAL(pressed()),SLOT(goto_startposition_program()));

    system("/opt/linuxcnc/scripts/./halrun -U");
    halio().Init();
    kinematic().Init();

    OpencascadeWidget = new Opencascade(this);
    ui->gridLayout_Opencascade->addWidget(OpencascadeWidget);

    positions = new gui_positions(this);
    ui->gridLayout_positions->addWidget(positions);
    controls = new gui_controls(this);
    ui->gridLayout_controls->addWidget(controls);

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::mainloop);
    timer->start(200); //1000 milliseond.
}

MainWindow::~MainWindow()
{
    hal_exit(comp_id);      // Unload hal component.
    system("/opt/linuxcnc/scripts/./halrun -U");
    delete ui;
}

void MainWindow::make_program(){
    // Remove previous performed files from the ./stream dir.
    system("rm -rf ./programstream/*.txt");
    system("rm -rf ./stream/*.txt");

    bool ok=create_program().point_every_ms(OpencascadeWidget->bucketvec);
    if(!ok){
        std::cout<<"error from mainwindow, function make_program"<<std::endl;
    }
}



void MainWindow::play_program(){
    programplay=1;
}


void MainWindow::goto_startposition_program(){
    std::string meat="/opt/linuxcnc/bin/./halstreamer -c 0 ./programstream/stream";
    std::string str1= std::to_string(0); // startpoint file.
    std::string givemeat=meat+str1+".txt";
    system(givemeat.c_str());
}

void MainWindow::open_dxf(){
    std::string filename=ui->lineEdit_dxf_file->text().toStdString();
    std::vector<Handle(AIS_Shape)> shapes=libdxfrw_functions().open_dxf_file(filename);

    for(unsigned int i=0; i<shapes.size(); i++){
        shapes.at(i)=draw_primitives().colorize(shapes.at(i),Quantity_NOC_BLUE1,0);

        gp_Trsf trsf;
        trsf.SetTranslation({0,0,0},{500,0,500});

        shapes.at(i)->SetLocalTransformation(trsf);
        OpencascadeWidget->show_shape(shapes.at(i));
    }
}

void MainWindow::add_startpoint(){
    add_point("startpoint");
}

void MainWindow::add_waypoint(){
    add_point("waypoint");
}

void MainWindow::add_endpoint(){
    add_point("endpoint");
}

void MainWindow::add_io(){
    add_point("io");
}

void MainWindow::set_stackedwidget_index(int i){ // Parallel to combobox index.
    //std::cout<<"str:"<<arg1.toStdString()<<std::endl;
    ui->stackedWidget->setCurrentIndex(i);
}

void MainWindow::add_point(std::string type){

    std::string primitivetype=ui->comboBox_primitivetype->currentText().toStdString();

    if(type=="startpoint"){
        pointvec.clear();
        eulervec.clear();
        iovec.clear();

        gp_Pnt p;
        p.SetX(cart.p.x());
        p.SetY(cart.p.y());
        p.SetZ(cart.p.z());
        pointvec.push_back(p);

        gp_Pnt e;
        double x,y,z;
        cart.M.GetEulerZYX(z,y,x);
        e.SetX(x);
        e.SetY(y);
        e.SetZ(z);
        eulervec.push_back(e);

        Opencascade::io io;
        io.halcommand="";
        iovec.push_back(io);

        if(primitivetype=="point"){ // Create primitive
            // Perform:
            Opencascade::bucket b;
            b.primitivetype="io";
            b.pointvec=pointvec;
            b.eulervec=eulervec;
            b.info="** io **";
            b.iovec=iovec;

            // Extra info
            b.vo=ui->lineEdit_teach_vel_start->text().toDouble();
            b.ve=ui->lineEdit_teach_vel_end->text().toDouble();
            b.velmax=ui->lineEdit_teach_vel_max->text().toDouble();
            b.accmax=ui->lineEdit_teach_acc_max->text().toDouble();

            // Print line id
            std::string text=std::to_string(OpencascadeWidget->bucketvec.size());
            b.Ais_shape=draw_primitives().draw_3d_point_origin_cone_text(b.pointvec.back(), b.eulervec.back(), text, 15, 0);
            OpencascadeWidget->bucketvec.push_back(b);
            OpencascadeWidget->show_shape(OpencascadeWidget->bucketvec.back().Ais_shape);
        }

        OpencascadeWidget->draw_preview_cone("startpoint",level0x1x2x3x4x5x6);
    }

    if(type=="io"){
        pointvec.clear();
        eulervec.clear();
        iovec.clear();

        gp_Pnt p;
        p.SetX(cart.p.x());
        p.SetY(cart.p.y());
        p.SetZ(cart.p.z());
        pointvec.push_back(p);

        gp_Pnt e;
        double x,y,z;
        cart.M.GetEulerZYX(z,y,x);
        e.SetX(x);
        e.SetY(y);
        e.SetZ(z);
        eulervec.push_back(e);

        Opencascade::io io;
        io.halcommand=ui->lineEdit_io_halcommand->text().toStdString();
        iovec.push_back(io);

        OpencascadeWidget->draw_preview_cone("iostatus",level0x1x2x3x4x5x6);

        // Perform:
        Opencascade::bucket b;
        b.primitivetype="io";
        b.pointvec=pointvec;
        b.eulervec=eulervec;
        b.info="** io **";
        b.iovec=iovec;
        b.Ais_shape=draw_primitives().draw_3d_point_origin_cone_text(b.pointvec.back(), b.eulervec.back(),b.iovec.back().halcommand,20,0);
        OpencascadeWidget->bucketvec.push_back(b);
        OpencascadeWidget->show_shape(OpencascadeWidget->bucketvec.back().Ais_shape);
    }

    if(type=="waypoint"){
        gp_Pnt p;
        p.SetX(cart.p.x());
        p.SetY(cart.p.y());
        p.SetZ(cart.p.z());
        pointvec.push_back(p);

        gp_Pnt e;
        double x,y,z;
        cart.M.GetEulerZYX(z,y,x);
        e.SetX(x);
        e.SetY(y);
        e.SetZ(z);
        eulervec.push_back(e);

        Opencascade::io io;
        io.halcommand="";
        iovec.push_back(io);

        OpencascadeWidget->draw_preview_cone("waypoint",level0x1x2x3x4x5x6);
    }
    if(type=="endpoint"){
        gp_Pnt p;
        p.SetX(cart.p.x());
        p.SetY(cart.p.y());
        p.SetZ(cart.p.z());
        pointvec.push_back(p);

        gp_Pnt e;
        double x,y,z;
        cart.M.GetEulerZYX(z,y,x);
        e.SetX(x);
        e.SetY(y);
        e.SetZ(z);
        eulervec.push_back(e);

        OpencascadeWidget->empty_preview_bucket();

        // Create primitives
        if(primitivetype=="line" && pointvec.size()>1){

            Opencascade::bucket b;
            b.primitivetype="line";
            b.pointvec=pointvec;
            b.eulervec=eulervec;
            b.info="** line **";
            b.iovec=iovec;

            // Extra info
            b.vo=ui->lineEdit_teach_vel_start->text().toDouble();
            b.ve=ui->lineEdit_teach_vel_end->text().toDouble();
            b.velmax=ui->lineEdit_teach_vel_max->text().toDouble();
            b.accmax=ui->lineEdit_teach_acc_max->text().toDouble();

            // Check if line has a lenght > 0
            double l=sqrt(pow(pointvec.back().X()-pointvec.front().X(),2)+pow(pointvec.back().Y()-pointvec.front().Y(),2)+pow(pointvec.back().Z()-pointvec.front().Z(),2));
            if(l<0.1){
                pointvec.back().SetX(pointvec.back().X()+0.1); // Add a little value to avoid error.
            }

            // Print line id
            std::string text=std::to_string(OpencascadeWidget->bucketvec.size());
            b.Ais_shape=draw_primitives().draw_3d_line_origin_cone_text(pointvec.front(), pointvec.back(), eulervec.front(), eulervec.back(), text, 15);

            OpencascadeWidget->bucketvec.push_back(b);
            OpencascadeWidget->show_shape(OpencascadeWidget->bucketvec.back().Ais_shape);

            pointvec.erase(pointvec.begin()); // Enables only clicking endpoints.
            eulervec.erase(eulervec.begin());
        }
        if(primitivetype=="wire" && pointvec.size()>0){
            Opencascade::bucket b;
            b.primitivetype="wire";
            b.pointvec=pointvec;
            b.eulervec=eulervec;
            b.info="** wire **";
            b.iovec=iovec;

            // Extra info
            b.vo=ui->lineEdit_teach_vel_start->text().toDouble();
            b.ve=ui->lineEdit_teach_vel_end->text().toDouble();
            b.velmax=ui->lineEdit_teach_vel_max->text().toDouble();
            b.accmax=ui->lineEdit_teach_acc_max->text().toDouble();

            // Print line id
            std::string text=std::to_string(OpencascadeWidget->bucketvec.size());

            b.Ais_shape=draw_primitives().draw_3d_wire_origin_cone_text(pointvec, eulervec, text, 15);
            OpencascadeWidget->bucketvec.push_back(b);
            OpencascadeWidget->show_shape(OpencascadeWidget->bucketvec.back().Ais_shape);

            std::vector<gp_Pnt> point_temp; // Now only use the wire endpoint for next primitive.
            std::vector<gp_Pnt> euler_temp;
            point_temp=pointvec;
            euler_temp=eulervec;
            pointvec.clear();
            eulervec.clear();
            pointvec.push_back(point_temp.back());
            eulervec.push_back(euler_temp.back());
            point_temp.clear();
            euler_temp.clear();
        }
        if(primitivetype=="arc" && pointvec.size()>2){
            Opencascade::bucket b;
            b.primitivetype="arc";
            b.pointvec=pointvec;
            b.eulervec=eulervec;
            b.info="** arc **";
            b.iovec=iovec;

            // Extra info
            b.vo=ui->lineEdit_teach_vel_start->text().toDouble();
            b.ve=ui->lineEdit_teach_vel_end->text().toDouble();
            b.velmax=ui->lineEdit_teach_vel_max->text().toDouble();
            b.accmax=ui->lineEdit_teach_acc_max->text().toDouble();

            // Print line id
            std::string text=std::to_string(OpencascadeWidget->bucketvec.size());

            b.Ais_shape=draw_primitives().draw_3d_arc_origin_cone_text(pointvec, eulervec, text, 15);
            OpencascadeWidget->bucketvec.push_back(b);
            OpencascadeWidget->show_shape(OpencascadeWidget->bucketvec.back().Ais_shape);

            pointvec.erase(pointvec.begin());
            pointvec.erase(pointvec.begin());
            eulervec.erase(eulervec.begin());
            eulervec.erase(eulervec.begin());
        }
        if(primitivetype=="circle" && pointvec.size()>2){
            Opencascade::bucket b;
            b.primitivetype="circle";
            b.pointvec=pointvec;
            b.eulervec=eulervec;
            b.info="** circle **";
            b.iovec=iovec;

            // Extra info
            b.vo=ui->lineEdit_teach_vel_start->text().toDouble();
            b.ve=ui->lineEdit_teach_vel_end->text().toDouble();
            b.velmax=ui->lineEdit_teach_vel_max->text().toDouble();
            b.accmax=ui->lineEdit_teach_acc_max->text().toDouble();

            // Print line id
            std::string text=std::to_string(OpencascadeWidget->bucketvec.size());

            b.Ais_shape=draw_primitives().draw_3d_circle_origin_cone_text(pointvec, eulervec, text, 15);
            OpencascadeWidget->bucketvec.push_back(b);
            OpencascadeWidget->show_shape(OpencascadeWidget->bucketvec.back().Ais_shape);

            pointvec.erase(pointvec.begin());
            pointvec.erase(pointvec.begin());
            eulervec.erase(eulervec.begin());
            eulervec.erase(eulervec.begin());
        }
        if(primitivetype=="spline" && pointvec.size()>2){
            Opencascade::bucket b;
            b.primitivetype="spline";
            b.pointvec=pointvec;
            b.eulervec=eulervec;
            b.info="** spline **";
            b.iovec=iovec;

            // Extra info
            b.vo=ui->lineEdit_teach_vel_start->text().toDouble();
            b.ve=ui->lineEdit_teach_vel_end->text().toDouble();
            b.velmax=ui->lineEdit_teach_vel_max->text().toDouble();
            b.accmax=ui->lineEdit_teach_acc_max->text().toDouble();

            // Print line id
            std::string text=std::to_string(OpencascadeWidget->bucketvec.size());

            b.Ais_shape=draw_primitives().draw_3d_spline_origin_cone_text(pointvec, eulervec,5, text, 15);
            OpencascadeWidget->bucketvec.push_back(b);
            OpencascadeWidget->show_shape(OpencascadeWidget->bucketvec.back().Ais_shape);

            std::vector<gp_Pnt> point_temp; // Now only use the wire endpoint for next primitive.
            std::vector<gp_Pnt> euler_temp;
            point_temp=pointvec;
            euler_temp=eulervec;
            pointvec.clear();
            eulervec.clear();
            pointvec.push_back(point_temp.back());
            eulervec.push_back(euler_temp.back());
            point_temp.clear();
            euler_temp.clear();
        }
        if(primitivetype=="io"){
            // Nothing here.
        }
    }
}

void MainWindow::stop(){
    system("/opt/linuxcnc/bin/./halcmd setp streamer.0.enable false");
}

void MainWindow::reset(){
    system("/opt/linuxcnc/bin/./halcmd setp streamer.0.enable true");
}

bool MainWindow::loadmodel()
{
    bool ok=OpencascadeWidget->Readstepfile("config/robot/kuka_base.step");
    if(!ok){return 0;}
    ok=OpencascadeWidget->Readstepfile("config/robot/kuka_joint_1.step");
    if(!ok){return 0;}
    ok=OpencascadeWidget->Readstepfile("config/robot/kuka_joint_2.step");
    if(!ok){return 0;}
    ok=OpencascadeWidget->Readstepfile("config/robot/kuka_joint_3.step");
    if(!ok){return 0;}
    ok=OpencascadeWidget->Readstepfile("config/robot/kuka_joint_4.step");
    if(!ok){return 0;}
    ok=OpencascadeWidget->Readstepfile("config/robot/kuka_joint_5.step");
    if(!ok){return 0;}
    ok=OpencascadeWidget->Readstepfile("config/robot/kuka_joint_6.step");
    if(!ok){return 0;}

    OpencascadeWidget->setup_tcp_origin();

    return ok;
}

void MainWindow::mainloop(){

    if(!ready){
        bool ok=loadmodel();
        if(ok){
            ready=1;
        }
    }

    if(ready && *streamermeat->Pin==0){ // Streamer is active, lock several gui buttons, otherwise we get a overflow.
        //controls->setEnabled(true);
        controls->set_enabled(true);
    } else {
        //controls->setEnabled(false);
        controls->set_enabled(false);
    }

    if(ready && okmove){ // Manual, jog, mdi
        if(*streamermeat->Pin<6000){

            std::string meat="/opt/linuxcnc/bin/./halstreamer -c 0 ./stream/stream";
            std::string str1= std::to_string(file);
            std::string givemeat=meat+str1+".txt";
            system(givemeat.c_str());
            //std::cout<<"filenr loaded:"<<file<<std::endl;
            file++;

            if(file==ammount){
                okmove=0;
                file=0;
            }
        }
    }

    if(ready && programplay){ // Program play
        if(*streamermeat->Pin<6000){

            std::string meat="/opt/linuxcnc/bin/./halstreamer -c 0 ./programstream/stream";
            std::string str1= std::to_string(file);
            std::string givemeat=meat+str1+".txt";
            system(givemeat.c_str());
            //std::cout<<"filenr loaded:"<<file<<std::endl;
            file++;

            if(file==programammount){
                programplay=0;
                file=0;

                // Set current robot position.
                kinematic().Fk();
                kinematic().Ik();

                std::cout<<"machine program completed"<<std::endl;
            }
        }
    }

    if(ready){
        if(mode_feedback){
            OpencascadeWidget->update_jointpos(*J0_Fb->Pin*toRadians,
                                               *J1_Fb->Pin*toRadians,
                                               *J2_Fb->Pin*toRadians,
                                               *J3_Fb->Pin*toRadians,
                                               *J4_Fb->Pin*toRadians,
                                               *J5_Fb->Pin*toRadians);
        }
        if(!mode_feedback){
            OpencascadeWidget->update_jointpos(KDLJointCur(0),
                                               KDLJointCur(1),
                                               KDLJointCur(2),
                                               KDLJointCur(3),
                                               KDLJointCur(4),
                                               KDLJointCur(5));
        }
    }
}

void MainWindow::on_toolButton_delete_item_pressed()
{
    OpencascadeWidget->delete_selections();
}





