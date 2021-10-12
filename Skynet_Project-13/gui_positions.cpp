#include "gui_positions.h"
#include "ui_gui_positions.h"
#include "halio.h"
#include "kinematic.h"

//! Make conversion's easy:
#define toRadians M_PI/180.0
#define toDegrees (180.0/M_PI)

gui_positions::gui_positions(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::gui_positions)
{
    ui->setupUi(this);

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &gui_positions::mainloop);
    timer->start(200);
}

gui_positions::~gui_positions()
{
    delete ui;
}

void gui_positions::mainloop(){

    //std::cout<<"mainloop gui positions active"<<std::endl;

    if(mode_feedback){
        ui->lineEdit_j0->setText(QString::number(*J0_Fb->Pin,'f',3));
        ui->lineEdit_j1->setText(QString::number(*J1_Fb->Pin,'f',3));
        ui->lineEdit_j2->setText(QString::number(*J2_Fb->Pin,'f',3));
        ui->lineEdit_j3->setText(QString::number(*J3_Fb->Pin,'f',3));
        ui->lineEdit_j4->setText(QString::number(*J4_Fb->Pin,'f',3));
        ui->lineEdit_j5->setText(QString::number(*J5_Fb->Pin,'f',3));

        ui->lineEdit_cartx->setText(QString::number(*CartX_Fb->Pin,'f',3));
        ui->lineEdit_carty->setText(QString::number(*CartY_Fb->Pin,'f',3));
        ui->lineEdit_cartz->setText(QString::number(*CartZ_Fb->Pin,'f',3));
        ui->lineEdit_eulerx->setText(QString::number(*EulerX_Fb->Pin,'f',3));
        ui->lineEdit_eulery->setText(QString::number(*EulerY_Fb->Pin,'f',3));
        ui->lineEdit_eulerz->setText(QString::number(*EulerZ_Fb->Pin,'f',3));

        ui->lineEdit_stepsize_fb->setText(QString::number(cart_stepsize->Pin,'f',3));
        ui->lineEdit_velmax_fb->setText(QString::number(velmax->Pin,'f',3));
        ui->lineEdit_accmax_fb->setText(QString::number(accmax->Pin,'f',3));
    }
    if(!mode_feedback){
        ui->lineEdit_j0->setText(QString::number(KDLJointCur(0)*toDegrees,'f',3));
        ui->lineEdit_j1->setText(QString::number(KDLJointCur(1)*toDegrees,'f',3));
        ui->lineEdit_j2->setText(QString::number(KDLJointCur(2)*toDegrees,'f',3));
        ui->lineEdit_j3->setText(QString::number(KDLJointCur(3)*toDegrees,'f',3));
        ui->lineEdit_j4->setText(QString::number(KDLJointCur(4)*toDegrees,'f',3));
        ui->lineEdit_j5->setText(QString::number(KDLJointCur(5)*toDegrees,'f',3));

        ui->lineEdit_cartx->setText(QString::number(cart.p.x(),'f',3));
        ui->lineEdit_carty->setText(QString::number(cart.p.y(),'f',3));
        ui->lineEdit_cartz->setText(QString::number(cart.p.z(),'f',3));
        ui->lineEdit_eulerx->setText(QString::number(cart.M.GetRot().x()*toDegrees,'f',3));
        ui->lineEdit_eulery->setText(QString::number(cart.M.GetRot().y()*toDegrees,'f',3));
        ui->lineEdit_eulerz->setText(QString::number(cart.M.GetRot().z()*toDegrees,'f',3));

        ui->lineEdit_stepsize_fb->setText(QString::number(cart_stepsize->Pin,'f',3));
        ui->lineEdit_velmax_fb->setText(QString::number(velmax->Pin,'f',3));
        ui->lineEdit_accmax_fb->setText(QString::number(accmax->Pin,'f',3));
    }

    // Tool
    if(*tool0->Pin==1){
        ui->pushButton_tool0->setStyleSheet(green);

    } else {
        ui->pushButton_tool0->setStyleSheet(red);
    }

    if(*tool1->Pin==1){
        ui->pushButton_tool1->setStyleSheet(green);
    } else {
        ui->pushButton_tool1->setStyleSheet(red);
    }

    if(*tool2->Pin==1){
        ui->pushButton_tool2->setStyleSheet(green);
    } else {
        ui->pushButton_tool2->setStyleSheet(red);
    }
}
























