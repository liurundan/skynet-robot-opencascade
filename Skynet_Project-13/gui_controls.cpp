#include "gui_controls.h"
#include "ui_gui_controls.h"

//! Make conversion's easy:
#define toRadians M_PI/180.0
#define toDegrees (180.0/M_PI)

gui_controls::gui_controls(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::gui_controls)
{
    ui->setupUi(this);

    ui->doubleSpinBox_cart_stepsize->setValue(cart_stepsize->Pin);
    ui->doubleSpinBox_euler_stepsize->setValue(euler_stepsize->Pin);
    ui->doubleSpinBox_joint_stepsize->setValue(joint_stepsize->Pin);
    ui->doubleSpinBox_velocity->setValue(velmax->Pin);
    ui->doubleSpinBox_acceleration->setValue(accmax->Pin);
    ui->doubleSpinBox_euler_max_deg_sec->setValue(euler_maxdegsec->Pin);
    ui->doubleSpinBox_joint_max_deg_sec->setValue(joint_maxdegsec->Pin);
    ui->doubleSpinBox_tooldir_stepsize->setValue(tooldir_stepsize->Pin);

    // Cart
    QObject::connect(ui->doubleSpinBox_cart_stepsize, SIGNAL(valueChanged(double)),SLOT(update_cart_stepsize(double)));

    QObject::connect(ui->doubleSpinBox_acceleration, SIGNAL(valueChanged(double)),SLOT(update_accmax(double)));
    QObject::connect(ui->doubleSpinBox_velocity, SIGNAL(valueChanged(double)),SLOT(update_velmax(double)));

    QObject::connect(ui->pushButton_cart_step_plus, SIGNAL(pressed()),SLOT(cart_step_plus()));
    QObject::connect(ui->pushButton_cart_step_min, SIGNAL(pressed()),SLOT(cart_step_min()));

    QObject::connect(ui->pushButton_acc_plus, SIGNAL(pressed()),SLOT(acc_plus()));
    QObject::connect(ui->pushButton_acc_min, SIGNAL(pressed()),SLOT(acc_min()));

    QObject::connect(ui->pushButton_vel_plus, SIGNAL(pressed()),SLOT(vel_plus()));
    QObject::connect(ui->pushButton_vel_min, SIGNAL(pressed()),SLOT(vel_min()));

    QObject::connect(ui->pushButton_cart_x_plus, SIGNAL(pressed()),SLOT(cartx_plus()));
    QObject::connect(ui->pushButton_cart_x_min, SIGNAL(pressed()),SLOT(cartx_min()));
    QObject::connect(ui->pushButton_cart_y_plus, SIGNAL(pressed()),SLOT(carty_plus()));
    QObject::connect(ui->pushButton_cart_y_min, SIGNAL(pressed()),SLOT(carty_min()));
    QObject::connect(ui->pushButton_cart_z_plus, SIGNAL(pressed()),SLOT(cartz_plus()));
    QObject::connect(ui->pushButton_cart_z_min, SIGNAL(pressed()),SLOT(cartz_min()));

    // EUler
    QObject::connect(ui->doubleSpinBox_euler_stepsize, SIGNAL(valueChanged(double)),SLOT(update_euler_stepsize(double)));
    QObject::connect(ui->doubleSpinBox_euler_max_deg_sec, SIGNAL(valueChanged(double)),SLOT(update_euler_maxdegsec(double)));

    QObject::connect(ui->pushButton_euler_step_plus, SIGNAL(pressed()),SLOT(euler_step_plus()));
    QObject::connect(ui->pushButton_euler_step_min, SIGNAL(pressed()),SLOT(euler_step_min()));
    QObject::connect(ui->pushButton_euler_deg_sec_plus, SIGNAL(pressed()),SLOT(euler_deg_sec_plus()));
    QObject::connect(ui->pushButton_euler_deg_sec_min, SIGNAL(pressed()),SLOT(euler_deg_sec_min()));

    QObject::connect(ui->pushButton_euler_x_plus, SIGNAL(pressed()),SLOT(eulerx_plus()));
    QObject::connect(ui->pushButton_euler_x_min, SIGNAL(pressed()),SLOT(eulerx_min()));
    QObject::connect(ui->pushButton_euler_y_plus, SIGNAL(pressed()),SLOT(eulery_plus()));
    QObject::connect(ui->pushButton_euler_y_min, SIGNAL(pressed()),SLOT(eulery_min()));
    QObject::connect(ui->pushButton_euler_z_plus, SIGNAL(pressed()),SLOT(eulerz_plus()));
    QObject::connect(ui->pushButton_euler_z_min, SIGNAL(pressed()),SLOT(eulerz_min()));

    // Joints
    QObject::connect(ui->doubleSpinBox_joint_stepsize, SIGNAL(valueChanged(double)),SLOT(update_joint_stepsize(double)));
    QObject::connect(ui->doubleSpinBox_joint_max_deg_sec, SIGNAL(valueChanged(double)),SLOT(update_joint_maxdegsec(double)));

    QObject::connect(ui->pushButton_joint_deg_sec_plus, SIGNAL(pressed()),SLOT(joint_deg_sec_plus()));
    QObject::connect(ui->pushButton_joint_deg_sec_min, SIGNAL(pressed()),SLOT(joint_deg_sec_min()));
    QObject::connect(ui->pushButton_joint_step_plus, SIGNAL(pressed()),SLOT(joint_step_plus()));
    QObject::connect(ui->pushButton_joint_step_min, SIGNAL(pressed()),SLOT(joint_step_min()));

    QObject::connect(ui->pushButton_j0_plus, SIGNAL(pressed()),SLOT(j0_plus()));
    QObject::connect(ui->pushButton_j1_plus, SIGNAL(pressed()),SLOT(j1_plus()));
    QObject::connect(ui->pushButton_j2_plus, SIGNAL(pressed()),SLOT(j2_plus()));
    QObject::connect(ui->pushButton_j3_plus, SIGNAL(pressed()),SLOT(j3_plus()));
    QObject::connect(ui->pushButton_j4_plus, SIGNAL(pressed()),SLOT(j4_plus()));
    QObject::connect(ui->pushButton_j5_plus, SIGNAL(pressed()),SLOT(j5_plus()));

    QObject::connect(ui->pushButton_j0_min, SIGNAL(pressed()),SLOT(j0_min()));
    QObject::connect(ui->pushButton_j1_min, SIGNAL(pressed()),SLOT(j1_min()));
    QObject::connect(ui->pushButton_j2_min, SIGNAL(pressed()),SLOT(j2_min()));
    QObject::connect(ui->pushButton_j3_min, SIGNAL(pressed()),SLOT(j3_min()));
    QObject::connect(ui->pushButton_j4_min, SIGNAL(pressed()),SLOT(j4_min()));
    QObject::connect(ui->pushButton_j5_min, SIGNAL(pressed()),SLOT(j5_min()));

    // Tools
    QObject::connect(ui->pushButton_tool0_on, SIGNAL(pressed()),SLOT(tool0_on()));
    QObject::connect(ui->pushButton_tool0_off, SIGNAL(pressed()),SLOT(tool0_off()));
    QObject::connect(ui->pushButton_tool1_on, SIGNAL(pressed()),SLOT(tool1_on()));
    QObject::connect(ui->pushButton_tool1_off, SIGNAL(pressed()),SLOT(tool1_off()));
    QObject::connect(ui->pushButton_tool2_on, SIGNAL(pressed()),SLOT(tool2_on()));
    QObject::connect(ui->pushButton_tool2_off, SIGNAL(pressed()),SLOT(tool2_off()));

    // Mdi joints
    QObject::connect(ui->toolButton_mdi_joint_go, SIGNAL(pressed()),SLOT(mdi_joint_go()));
    QObject::connect(ui->toolButton_mdi_copy_cur_joint_pos, SIGNAL(pressed()),SLOT(mdi_copy_cur_joint_pos()));
    QObject::connect(ui->toolButton_mdi_go_home_pos, SIGNAL(pressed()),SLOT(mdi_go_home_pos()));

    // Mdi cartesian
    QObject::connect(ui->toolButton_mdi_cart_go, SIGNAL(pressed()),SLOT(mdi_cart_go()));
    QObject::connect(ui->toolButton_mdi_copy_cur_cart_pos, SIGNAL(pressed()),SLOT(mdi_copy_cur_cart_pos()));

    // Stream controls
    QObject::connect(ui->toolButton_pause_stream, SIGNAL(pressed()),SLOT(pause()));
    QObject::connect(ui->toolButton_resume_stream, SIGNAL(pressed()),SLOT(resume()));

    // Tooldir controls
    QObject::connect(ui->pushButton_tooldir_x_min, SIGNAL(pressed()),SLOT(tooldir_x_min()));
    QObject::connect(ui->pushButton_tooldir_x_plus, SIGNAL(pressed()),SLOT(tooldir_x_plus()));
    QObject::connect(ui->pushButton_tooldir_y_min, SIGNAL(pressed()),SLOT(tooldir_y_min()));
    QObject::connect(ui->pushButton_tooldir_y_plus, SIGNAL(pressed()),SLOT(tooldir_y_plus()));
    QObject::connect(ui->pushButton_tooldir_z_min, SIGNAL(pressed()),SLOT(tooldir_z_min()));
    QObject::connect(ui->pushButton_tooldir_z_plus, SIGNAL(pressed()),SLOT(tooldir_z_plus()));

    // Mdi tooldir
    QObject::connect(ui->toolButton_mdi_tool_go, SIGNAL(pressed()),SLOT(mdi_tool_go()));
}

gui_controls::~gui_controls()
{
    delete ui;
}

void gui_controls::set_enabled(bool ok){
    ui->pushButton_j0_min->setEnabled(ok);
    ui->pushButton_j1_min->setEnabled(ok);
    ui->pushButton_j2_min->setEnabled(ok);
    ui->pushButton_j3_min->setEnabled(ok);
    ui->pushButton_j4_min->setEnabled(ok);
    ui->pushButton_j5_min->setEnabled(ok);

    ui->pushButton_j0_plus->setEnabled(ok);
    ui->pushButton_j1_plus->setEnabled(ok);
    ui->pushButton_j2_plus->setEnabled(ok);
    ui->pushButton_j3_plus->setEnabled(ok);
    ui->pushButton_j4_plus->setEnabled(ok);
    ui->pushButton_j5_plus->setEnabled(ok);

    ui->pushButton_euler_x_min->setEnabled(ok);
    ui->pushButton_euler_x_plus->setEnabled(ok);
    ui->pushButton_euler_y_min->setEnabled(ok);
    ui->pushButton_euler_y_plus->setEnabled(ok);
    ui->pushButton_euler_z_min->setEnabled(ok);
    ui->pushButton_euler_z_plus->setEnabled(ok);

    ui->pushButton_cart_x_min->setEnabled(ok);
    ui->pushButton_cart_x_plus->setEnabled(ok);
    ui->pushButton_cart_y_min->setEnabled(ok);
    ui->pushButton_cart_y_plus->setEnabled(ok);
    ui->pushButton_cart_z_min->setEnabled(ok);
    ui->pushButton_cart_z_plus->setEnabled(ok);

    ui->toolButton_mdi_joint_go->setEnabled(ok);
    ui->toolButton_mdi_cart_go->setEnabled(ok);
    ui->toolButton_mdi_go_home_pos->setEnabled(ok);
}

void gui_controls::pause(){
    system("/opt/linuxcnc/bin/./halcmd setp streamer.0.enable false");
    ui->toolButton_resume_stream->setStyleSheet(orange);
}

void gui_controls::resume(){
    system("/opt/linuxcnc/bin/./halcmd setp streamer.0.enable true");
    ui->toolButton_resume_stream->setStyleSheet(grey);
}

void gui_controls::mdi_go_home_pos(){
    mdi_reset_cur_joint_pos();
    mdi_joint_go();
}

void gui_controls::mdi_copy_cur_cart_pos(){
    if(ui->checkBox_mode_fb->isChecked()){
        ui->lineEdit_mdi_cartx->setText(QString::number(*CartX_Fb->Pin,'f',3));
        ui->lineEdit_mdi_carty->setText(QString::number(*CartY_Fb->Pin,'f',3));
        ui->lineEdit_mdi_cartz->setText(QString::number(*CartZ_Fb->Pin,'f',3));
        ui->lineEdit_mdi_eulx->setText(QString::number(*EulerX_Fb->Pin,'f',3));
        ui->lineEdit_mdi_euly->setText(QString::number(*EulerY_Fb->Pin,'f',3));
        ui->lineEdit_mdi_eulz->setText(QString::number(*EulerZ_Fb->Pin,'f',3));
    }
    if(!ui->checkBox_mode_fb->isChecked()){
        ui->lineEdit_mdi_cartx->setText(QString::number(cart.p.x(),'f',3));
        ui->lineEdit_mdi_carty->setText(QString::number(cart.p.y(),'f',3));
        ui->lineEdit_mdi_cartz->setText(QString::number(cart.p.z(),'f',3));
        ui->lineEdit_mdi_eulx->setText(QString::number(cart.M.GetRot().x()* toDegrees,'f',3));
        ui->lineEdit_mdi_euly->setText(QString::number(cart.M.GetRot().y()* toDegrees,'f',3));
        ui->lineEdit_mdi_eulz->setText(QString::number(cart.M.GetRot().z()* toDegrees,'f',3));
    }
}

void gui_controls::mdi_cart_go(){
    mdi_cart(ui->lineEdit_mdi_cartx->text().toDouble(),
             ui->lineEdit_mdi_carty->text().toDouble(),
             ui->lineEdit_mdi_cartz->text().toDouble(),false); // Change euler angles ok.
}

void gui_controls::mdi_tool_go(){
    double x=ui->lineEdit_mdi_toolx->text().toDouble();
    double y=ui->lineEdit_mdi_tooly->text().toDouble();
    double z=ui->lineEdit_mdi_toolz->text().toDouble();

    double x_out,y_out,z_out;
    int ok=kinematic().Fk_tooldir(x,y,z,x_out,y_out,z_out); // Relative move
    if(ok){
        mdi_cart(x_out,y_out,z_out,1); // If point on axis if found, perform a move.
    } else { std::cout<<"Fk tooldir error"<<std::endl; }
}

void gui_controls::tooldir_x_min(){
    // Move in tool direction, axis x. Find point on axis x with the help of the kinematic model.
    double x_out,y_out,z_out;
    int ok=kinematic().Fk_tooldir(ui->doubleSpinBox_tooldir_stepsize->value()*-1,0,0,x_out,y_out,z_out);
    if(ok){
        mdi_cart(x_out,y_out,z_out,1); // If point on axis if found, perform a move.
    } else { std::cout<<"Fk tooldir error"<<std::endl; }
}

void gui_controls::tooldir_x_plus(){
    double x_out,y_out,z_out;
    int ok=kinematic().Fk_tooldir(ui->doubleSpinBox_tooldir_stepsize->value(),0,0,x_out,y_out,z_out);
    if(ok){
        mdi_cart(x_out,y_out,z_out,1);
    } else { std::cout<<"Fk tooldir error"<<std::endl; }
}

void gui_controls::tooldir_y_plus(){
    double x_out,y_out,z_out;
    int ok=kinematic().Fk_tooldir(0,ui->doubleSpinBox_tooldir_stepsize->value(),0,x_out,y_out,z_out);
    if(ok){
        mdi_cart(x_out,y_out,z_out,1);
    } else { std::cout<<"Fk tooldir error"<<std::endl; }
}

void gui_controls::tooldir_y_min(){
    double x_out,y_out,z_out;
    int ok=kinematic().Fk_tooldir(0,ui->doubleSpinBox_tooldir_stepsize->value()*-1,0,x_out,y_out,z_out);
    if(ok){
        mdi_cart(x_out,y_out,z_out,1);
    } else { std::cout<<"Fk tooldir error"<<std::endl; }
}

void gui_controls::tooldir_z_plus(){
    double x_out,y_out,z_out;
    int ok=kinematic().Fk_tooldir(0,0,ui->doubleSpinBox_tooldir_stepsize->value(),x_out,y_out,z_out);
    if(ok){
        mdi_cart(x_out,y_out,z_out,1);
    } else { std::cout<<"Fk tooldir error"<<std::endl; }
}

void gui_controls::tooldir_z_min(){
    double x_out,y_out,z_out;
    int ok=kinematic().Fk_tooldir(0,0,ui->doubleSpinBox_tooldir_stepsize->value()*-1,x_out,y_out,z_out);
    if(ok){
        mdi_cart(x_out,y_out,z_out,1);
    } else { std::cout<<"Fk tooldir error"<<std::endl; }
}

void gui_controls::mdi_cart(double x, double y, double z, bool tooldir){

    KDL::Frame temp=cart;
    double eulx_rad=ui->lineEdit_mdi_eulx->text().toDouble()*toRadians;
    double euly_rad=ui->lineEdit_mdi_euly->text().toDouble()*toRadians;
    double eulz_rad=ui->lineEdit_mdi_eulz->text().toDouble()*toRadians;
    double eulx_deg=ui->lineEdit_mdi_eulx->text().toDouble();
    double euly_deg=ui->lineEdit_mdi_euly->text().toDouble();
    double eulz_deg=ui->lineEdit_mdi_eulz->text().toDouble();

    if(ui->checkBox_mode_fb->isChecked()){

        // First do a test to the endpoint.
        cart.p.x(x);
        cart.p.y(y);
        cart.p.z(z);
        if(!tooldir){ // When in tooldir, don't change euler angles.
            cart.M.DoRotX(eulx_rad-temp.M.GetRot().x());
            cart.M.DoRotY(euly_rad-temp.M.GetRot().y());
            cart.M.DoRotZ(eulz_rad-temp.M.GetRot().z());
        }
        int ok=kinematic().Ik();
        if(!ok){
            cart=temp;      // Ik error, restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart=temp;      // Restor value, from this beginpoint create the stream.
            int ok=0;
            if(!tooldir){   // Move cartesian
                ok=jog().jog_cart_euler(x-(*CartX_Fb->Pin),             // Dist is input.
                                        y-(*CartY_Fb->Pin),
                                        z-(*CartZ_Fb->Pin),
                                        eulx_deg-(*EulerX_Fb->Pin),     // Angle input in degrees.
                                        euly_deg-(*EulerY_Fb->Pin),
                                        eulz_deg-(*EulerZ_Fb->Pin));
            } else {        // Move tooldir.
                ok=jog().jog_cart_euler(x-(*CartX_Fb->Pin),             // Dist is input.
                                        y-(*CartY_Fb->Pin),
                                        z-(*CartZ_Fb->Pin),
                                        0,                              // Value 0 for don't change Euler angles.
                                        0,
                                        0);
            }
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        }
    } else {
        cart.p.x(x);
        cart.p.y(y);
        cart.p.z(z);
        if(!tooldir){ // When in tooldir, don't change euler angles
            cart.M.DoRotX(eulx_rad-temp.M.GetRot().x());
            cart.M.DoRotY(euly_rad-temp.M.GetRot().y());
            cart.M.DoRotZ(eulz_rad-temp.M.GetRot().z());
        }

        int ok=kinematic().Ik();
        if(!ok){
            cart=temp; //restore failure
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::mdi_joint_go(){
    double j0=ui->lineEdit_mdi_j0->text().toDouble();
    double j1=ui->lineEdit_mdi_j1->text().toDouble();
    double j2=ui->lineEdit_mdi_j2->text().toDouble();
    double j3=ui->lineEdit_mdi_j3->text().toDouble();
    double j4=ui->lineEdit_mdi_j4->text().toDouble();
    double j5=ui->lineEdit_mdi_j5->text().toDouble();

    double j0_rad=ui->lineEdit_mdi_j0->text().toDouble()*toRadians;
    double j1_rad=ui->lineEdit_mdi_j1->text().toDouble()*toRadians;
    double j2_rad=ui->lineEdit_mdi_j2->text().toDouble()*toRadians;
    double j3_rad=ui->lineEdit_mdi_j3->text().toDouble()*toRadians;
    double j4_rad=ui->lineEdit_mdi_j4->text().toDouble()*toRadians;
    double j5_rad=ui->lineEdit_mdi_j5->text().toDouble()*toRadians;

    KDL::Frame temp=cart; // Copy current config for safety.

    // Check for joint limits
    if(j0_rad>KDLJointMin(0) && j0_rad<KDLJointMax(0) &&
            j1_rad>KDLJointMin(1) && j1_rad<KDLJointMax(1) &&
            j2_rad>KDLJointMin(2) && j2_rad<KDLJointMax(2) &&
            j3_rad>KDLJointMin(3) && j3_rad<KDLJointMax(3) &&
            j4_rad>KDLJointMin(4) && j4_rad<KDLJointMax(4) &&
            j5_rad>KDLJointMin(5) && j5_rad<KDLJointMax(5) ){
        // If move is within the joint limits, go on.

        if(ui->checkBox_mode_fb->isChecked()){ // Live mode
            int ok=jog().jog_mdi_joint(j0,j1,j2,j3,j4,j5,ui->doubleSpinBox_mdi_joint_max_deg_sec->value());    // Create stream
            if(ok){
                okmove=true;    // Stream ok, move
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;      // Stream error, restore frame
                ui->toolButton_status->setStyleSheet(red);
            }

        } else { // No live mode
            KDLJointCur(0)=j0*toRadians;
            KDLJointCur(1)=j1*toRadians;
            KDLJointCur(2)=j2*toRadians;
            KDLJointCur(3)=j3*toRadians;
            KDLJointCur(4)=j4*toRadians;
            KDLJointCur(5)=j5*toRadians;
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        }
    } else {
        // Restore values.
        cart=temp;
        ui->toolButton_status->setStyleSheet(red);
    }
}

void gui_controls::mdi_reset_cur_joint_pos(){
    ui->lineEdit_mdi_j0->setText(QString::number(0,'f',3));
    ui->lineEdit_mdi_j1->setText(QString::number(0,'f',3));
    ui->lineEdit_mdi_j2->setText(QString::number(0,'f',3));
    ui->lineEdit_mdi_j3->setText(QString::number(0,'f',3));
    ui->lineEdit_mdi_j4->setText(QString::number(0,'f',3));
    ui->lineEdit_mdi_j5->setText(QString::number(0,'f',3));
}

void gui_controls::mdi_copy_cur_joint_pos(){
    if(ui->checkBox_mode_fb->isChecked()){
        ui->lineEdit_mdi_j0->setText(QString::number(*J0_Fb->Pin,'f',3));
        ui->lineEdit_mdi_j1->setText(QString::number(*J1_Fb->Pin,'f',3));
        ui->lineEdit_mdi_j2->setText(QString::number(*J2_Fb->Pin,'f',3));
        ui->lineEdit_mdi_j3->setText(QString::number(*J3_Fb->Pin,'f',3));
        ui->lineEdit_mdi_j4->setText(QString::number(*J4_Fb->Pin,'f',3));
        ui->lineEdit_mdi_j5->setText(QString::number(*J5_Fb->Pin,'f',3));
    }
    if(!ui->checkBox_mode_fb->isChecked()){
        ui->lineEdit_mdi_j0->setText(QString::number(KDLJointCur(0)* toDegrees,'f',3));
        ui->lineEdit_mdi_j1->setText(QString::number(KDLJointCur(1)* toDegrees,'f',3));
        ui->lineEdit_mdi_j2->setText(QString::number(KDLJointCur(2)* toDegrees,'f',3));
        ui->lineEdit_mdi_j3->setText(QString::number(KDLJointCur(3)* toDegrees,'f',3));
        ui->lineEdit_mdi_j4->setText(QString::number(KDLJointCur(4)* toDegrees,'f',3));
        ui->lineEdit_mdi_j5->setText(QString::number(KDLJointCur(5)* toDegrees,'f',3));
    }
}

void gui_controls::j0_plus(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(0)+=joint_stepsize->Pin*toRadians;                          // Add joint value
        if(KDLJointCur(0)>KDLJointMin(0) && KDLJointCur(0)<KDLJointMax(0)){     // Check joint value
            KDLJointCur(0)-=joint_stepsize->Pin*toRadians;                      // Substract joint value
            int ok=jog().jog_joint("j0",joint_stepsize->Pin);                   // Create stream
            if(ok){
                okmove=true;                                                    // Stream ok. move
                ui->toolButton_status->setStyleSheet(green);

            } else {
                cart=temp;                                                      // Stream error, restore frame
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(0)-=joint_stepsize->Pin*toRadians;                      // Joint out of limits, restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    } else { // Mode without feedback
        KDLJointCur(0)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(0)>KDLJointMin(0) && KDLJointCur(0)<KDLJointMax(0)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(0)-=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j0_min(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(0)-=joint_stepsize->Pin*toRadians;                          // Add joint value
        if(KDLJointCur(0)>KDLJointMin(0) && KDLJointCur(0)<KDLJointMax(0)){     // Check joint value
            KDLJointCur(0)+=joint_stepsize->Pin*toRadians;                      // Substract joint value
            int ok=jog().jog_joint("j0",joint_stepsize->Pin*-1);                // Create stream
            if(ok){
                okmove=true;                                                    // Stream ok. move
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;                                                      // Stream error, restore frame
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(0)+=joint_stepsize->Pin*toRadians;                      // Joint out of limits, restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    } else {
        KDLJointCur(0)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(0)>KDLJointMin(0) && KDLJointCur(0)<KDLJointMax(0)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(0)+=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j1_plus(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(1)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(1)>KDLJointMin(1) && KDLJointCur(1)<KDLJointMax(1)){
            KDLJointCur(1)-=joint_stepsize->Pin*toRadians;
            int ok=jog().jog_joint("j1",joint_stepsize->Pin);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(1)-=joint_stepsize->Pin*toRadians;
            ui->toolButton_status->setStyleSheet(red);
        }
    } else{
        KDLJointCur(1)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(1)>KDLJointMin(1) && KDLJointCur(1)<KDLJointMax(1)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(1)-=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j1_min(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(1)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(1)>KDLJointMin(1) && KDLJointCur(1)<KDLJointMax(1)){
            KDLJointCur(1)+=joint_stepsize->Pin*toRadians;
            int ok=jog().jog_joint("j1",joint_stepsize->Pin*-1);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(1)+=joint_stepsize->Pin*toRadians;
            ui->toolButton_status->setStyleSheet(red);
        }
    } else {
        KDLJointCur(1)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(1)>KDLJointMin(1) && KDLJointCur(1)<KDLJointMax(1)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(1)+=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j2_plus(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(2)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(2)>KDLJointMin(2) && KDLJointCur(2)<KDLJointMax(2)){
            KDLJointCur(2)-=joint_stepsize->Pin*toRadians;
            int ok=jog().jog_joint("j2",joint_stepsize->Pin);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(2)-=joint_stepsize->Pin*toRadians;
            ui->toolButton_status->setStyleSheet(red);
        }
    } else {
        KDLJointCur(2)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(2)>KDLJointMin(2) && KDLJointCur(2)<KDLJointMax(2)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(2)-=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j2_min(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(2)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(2)>KDLJointMin(2) && KDLJointCur(2)<KDLJointMax(2)){
            KDLJointCur(2)+=joint_stepsize->Pin*toRadians;
            int ok=jog().jog_joint("j2",joint_stepsize->Pin*-1);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(2)+=joint_stepsize->Pin*toRadians;
            ui->toolButton_status->setStyleSheet(red);
        }
    } else {
        KDLJointCur(2)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(2)>KDLJointMin(2) && KDLJointCur(2)<KDLJointMax(2)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(2)+=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j3_plus(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(3)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(3)>KDLJointMin(3) && KDLJointCur(3)<KDLJointMax(3)){
            KDLJointCur(3)-=joint_stepsize->Pin*toRadians;
            int ok=jog().jog_joint("j3",joint_stepsize->Pin);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(3)-=joint_stepsize->Pin*toRadians;
            ui->toolButton_status->setStyleSheet(red);
        }
    } else {
        KDLJointCur(3)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(3)>KDLJointMin(3) && KDLJointCur(3)<KDLJointMax(3)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(3)-=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j3_min(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(3)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(3)>KDLJointMin(3) && KDLJointCur(3)<KDLJointMax(3)){
            KDLJointCur(3)+=joint_stepsize->Pin*toRadians;
            int ok=jog().jog_joint("j3",joint_stepsize->Pin*-1);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(3)+=joint_stepsize->Pin*toRadians;
            ui->toolButton_status->setStyleSheet(red);
        }
    } else {
        KDLJointCur(3)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(3)>KDLJointMin(3) && KDLJointCur(3)<KDLJointMax(3)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(3)+=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j4_plus(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(4)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(4)>KDLJointMin(4) && KDLJointCur(4)<KDLJointMax(4)){
            KDLJointCur(4)-=joint_stepsize->Pin*toRadians;
            int ok=jog().jog_joint("j4",joint_stepsize->Pin);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(4)-=joint_stepsize->Pin*toRadians;
            ui->toolButton_status->setStyleSheet(red);
        }
    } else {
        KDLJointCur(4)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(4)>KDLJointMin(4) && KDLJointCur(4)<KDLJointMax(4)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(4)-=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j4_min(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(4)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(4)>KDLJointMin(4) && KDLJointCur(4)<KDLJointMax(4)){
            KDLJointCur(4)+=joint_stepsize->Pin*toRadians;
            int ok=jog().jog_joint("j4",joint_stepsize->Pin*-1);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(4)+=joint_stepsize->Pin*toRadians;
            ui->toolButton_status->setStyleSheet(red);
        }
    } else {
        KDLJointCur(4)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(4)>KDLJointMin(4) && KDLJointCur(4)<KDLJointMax(4)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(4)+=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j5_plus(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(5)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(5)>KDLJointMin(5) && KDLJointCur(5)<KDLJointMax(5)){
            KDLJointCur(5)-=joint_stepsize->Pin*toRadians;
            int ok=jog().jog_joint("j5",joint_stepsize->Pin);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(5)-=joint_stepsize->Pin*toRadians;
            ui->toolButton_status->setStyleSheet(red);
        }
    } else {
        KDLJointCur(5)+=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(5)>KDLJointMin(5) && KDLJointCur(5)<KDLJointMax(5)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(5)-=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::j5_min(){
    if(ui->checkBox_mode_fb->isChecked()){
        KDL::Frame temp=cart;
        KDLJointCur(5)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(5)>KDLJointMin(5) && KDLJointCur(5)<KDLJointMax(5)){
            KDLJointCur(5)+=joint_stepsize->Pin*toRadians;
            int ok=jog().jog_joint("j5",joint_stepsize->Pin*-1);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        } else {
            KDLJointCur(5)+=joint_stepsize->Pin*toRadians;
            ui->toolButton_status->setStyleSheet(red);
        }
    } else {
        KDLJointCur(5)-=joint_stepsize->Pin*toRadians;
        if(KDLJointCur(5)>KDLJointMin(5) && KDLJointCur(5)<KDLJointMax(5)){
            kinematic().Fk();
            ui->toolButton_status->setStyleSheet(green);
        } else {
            KDLJointCur(5)+=joint_stepsize->Pin*toRadians; // Restore value
            ui->toolButton_status->setStyleSheet(red);
        }
    }
}

void gui_controls::acc_plus(){
    ui->doubleSpinBox_acceleration->stepUp();
}

void gui_controls::acc_min(){
    ui->doubleSpinBox_acceleration->stepDown();
}

void gui_controls::cartx_plus(){

    if(ui->checkBox_mode_fb->isChecked()){

        KDL::Frame temp=cart;
        cart.p.x(cart.p.x()+cart_stepsize->Pin);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.x(cart.p.x()-cart_stepsize->Pin);    // Ik error, restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart.p.x(cart.p.x()-cart_stepsize->Pin);    // Ik ok, substrackt test value.

            // Create stream.
            int ok=jog().jog_cart_euler(cart_stepsize->Pin,0,0,0,0,0);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        }
    } else {
        cart.p.x(cart.p.x()+cart_stepsize->Pin);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.x(cart.p.x()-cart_stepsize->Pin); //restore failure
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::cartx_min(){
    if(ui->checkBox_mode_fb->isChecked()){

        KDL::Frame temp=cart;
        cart.p.x(cart.p.x()-cart_stepsize->Pin);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.x(cart.p.x()+cart_stepsize->Pin);    // Ik error, restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart.p.x(cart.p.x()+cart_stepsize->Pin);    // Ik ok, substrackt test value.

            // Create stream.
            int ok=jog().jog_cart_euler(cart_stepsize->Pin*-1,0,0,0,0,0);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        }
    } else {
        cart.p.x(cart.p.x()+cart_stepsize->Pin*-1);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.x(cart.p.x()-cart_stepsize->Pin*-1); //restore failure
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::carty_plus(){
    if(ui->checkBox_mode_fb->isChecked()){

        KDL::Frame temp=cart;
        cart.p.y(cart.p.y()+cart_stepsize->Pin);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.y(cart.p.y()-cart_stepsize->Pin);    // Ik error, restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart.p.y(cart.p.y()-cart_stepsize->Pin);    // Ik ok, substrackt test value.

            // Create stream.
            int ok=jog().jog_cart_euler(0,cart_stepsize->Pin,0,0,0,0);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        }

    } else {
        cart.p.y(cart.p.y()+cart_stepsize->Pin);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.y(cart.p.y()-cart_stepsize->Pin); //restore failure
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::carty_min(){
    if(ui->checkBox_mode_fb->isChecked()){

        KDL::Frame temp=cart;
        cart.p.y(cart.p.y()-cart_stepsize->Pin);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.y(cart.p.y()+cart_stepsize->Pin);    // Ik error, restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart.p.y(cart.p.y()+cart_stepsize->Pin);    // Ik ok, substrackt test value.

            // Create stream.
            int ok=jog().jog_cart_euler(0,cart_stepsize->Pin*-1,0,0,0,0);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        }

    } else {
        cart.p.y(cart.p.y()+cart_stepsize->Pin*-1);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.y(cart.p.y()-cart_stepsize->Pin*-1); //restore failure
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::cartz_plus(){
    if(ui->checkBox_mode_fb->isChecked()){

        KDL::Frame temp=cart;
        cart.p.z(cart.p.z()+cart_stepsize->Pin);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.z(cart.p.z()-cart_stepsize->Pin);    // Ik error, restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart.p.z(cart.p.z()-cart_stepsize->Pin);    // Ik ok, substrackt test value.
            // Create stream.
            int ok=jog().jog_cart_euler(0,0,cart_stepsize->Pin,0,0,0);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        }

    } else {
        cart.p.z(cart.p.z()+cart_stepsize->Pin);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.z(cart.p.z()-cart_stepsize->Pin); //restore failure
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::cartz_min(){
    if(ui->checkBox_mode_fb->isChecked()){

        KDL::Frame temp=cart;
        cart.p.z(cart.p.z()-cart_stepsize->Pin);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.z(cart.p.z()+cart_stepsize->Pin);    // Ik error, restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart.p.z(cart.p.z()+cart_stepsize->Pin);    // Ik ok, substrackt test value.

            // Create stream.
            int ok=jog().jog_cart_euler(0,0,cart_stepsize->Pin*-1,0,0,0);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                cart=temp;
                ui->toolButton_status->setStyleSheet(red);
            }
        }

    } else {
        cart.p.z(cart.p.z()+cart_stepsize->Pin*-1);
        int ok=kinematic().Ik();
        if(!ok){
            cart.p.z(cart.p.z()-cart_stepsize->Pin*-1); //restore failure
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::eulerx_plus(){
    if(ui->checkBox_mode_fb->isChecked()){
        cart.M.DoRotX(euler_stepsize->Pin*toRadians);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotX(euler_stepsize->Pin*toRadians*-1); // Restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {                                             // Test is ok.
            cart.M.DoRotX(euler_stepsize->Pin*toRadians*-1); // Substract value to original value

            int ok=jog().jog_cart_euler(0,0,0,euler_stepsize->Pin,0,0);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                ui->toolButton_status->setStyleSheet(red);
            }
        }

    } else {
        cart.M.DoRotX(euler_stepsize->Pin*toRadians);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotX(euler_stepsize->Pin*toRadians*-1); // Restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::eulerx_min(){
    if(ui->checkBox_mode_fb->isChecked()){
        cart.M.DoRotX(euler_stepsize->Pin*toRadians*-1);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotX(euler_stepsize->Pin*toRadians);
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart.M.DoRotX(euler_stepsize->Pin*toRadians);

            int ok=jog().jog_cart_euler(0,0,0,euler_stepsize->Pin*-1,0,0);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                ui->toolButton_status->setStyleSheet(red);
            }
        }
    } else {
        cart.M.DoRotX(euler_stepsize->Pin*toRadians*-1);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotX(euler_stepsize->Pin*toRadians); // Restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::eulery_plus(){
    if(ui->checkBox_mode_fb->isChecked()){
        cart.M.DoRotY(euler_stepsize->Pin*toRadians);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotY(euler_stepsize->Pin*toRadians*-1); // Restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {                                             // Test is ok.
            cart.M.DoRotY(euler_stepsize->Pin*toRadians*-1); // Substract value to original value

            int ok=jog().jog_cart_euler(0,0,0,0,euler_stepsize->Pin,0);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                ui->toolButton_status->setStyleSheet(red);
            }
        }
    } else {
        cart.M.DoRotY(euler_stepsize->Pin*toRadians);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotY(euler_stepsize->Pin*toRadians*-1); // Restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::eulery_min(){
    if(ui->checkBox_mode_fb->isChecked()){
        cart.M.DoRotY(euler_stepsize->Pin*toRadians*-1);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotY(euler_stepsize->Pin*toRadians);
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart.M.DoRotY(euler_stepsize->Pin*toRadians);

            int ok=jog().jog_cart_euler(0,0,0,0,euler_stepsize->Pin*-1,0);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                ui->toolButton_status->setStyleSheet(red);
            }
        }
    } else {
        cart.M.DoRotY(euler_stepsize->Pin*toRadians*-1);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotY(euler_stepsize->Pin*toRadians); // Restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::eulerz_plus(){
    if(ui->checkBox_mode_fb->isChecked()){
        cart.M.DoRotZ(euler_stepsize->Pin*toRadians);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotZ(euler_stepsize->Pin*toRadians*-1);
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart.M.DoRotZ(euler_stepsize->Pin*toRadians*-1);

            int ok=jog().jog_cart_euler(0,0,0,0,0,euler_stepsize->Pin);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                ui->toolButton_status->setStyleSheet(red);
            }
        }
    } else {
        cart.M.DoRotZ(euler_stepsize->Pin*toRadians);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotZ(euler_stepsize->Pin*toRadians*-1); // Restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::eulerz_min(){
    if(ui->checkBox_mode_fb->isChecked()){
        cart.M.DoRotZ(euler_stepsize->Pin*toRadians*-1);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotZ(euler_stepsize->Pin*toRadians);
            ui->toolButton_status->setStyleSheet(red);
        } else {
            cart.M.DoRotZ(euler_stepsize->Pin*toRadians);

            int ok=jog().jog_cart_euler(0,0,0,0,0,euler_stepsize->Pin*-1);
            if(ok){
                okmove=true;
                ui->toolButton_status->setStyleSheet(green);
            } else {
                ui->toolButton_status->setStyleSheet(red);
            }
        }
    } else {
        cart.M.DoRotZ(euler_stepsize->Pin*toRadians*-1);
        int ok=kinematic().Ik();
        if(!ok){
            cart.M.DoRotZ(euler_stepsize->Pin*toRadians); // Restore value
            ui->toolButton_status->setStyleSheet(red);
        } else {
            ui->toolButton_status->setStyleSheet(green);
        }
    }
}

void gui_controls::update_cart_stepsize(double val){
    cart_stepsize->Pin=float(val);
}

void gui_controls::update_euler_stepsize(double val){
    euler_stepsize->Pin=float(val);
}

void gui_controls::update_euler_maxdegsec(double val){
    euler_maxdegsec->Pin=val;
}

void gui_controls::update_joint_stepsize(double val){
    joint_stepsize->Pin=float(val);
}

void gui_controls::update_joint_maxdegsec(double val){
    joint_maxdegsec->Pin=val;
}

void gui_controls::update_accmax(double val){
    accmax->Pin=float(val);
}

void gui_controls::update_velmax(double val){
    velmax->Pin=float(val);
}

void gui_controls::vel_plus(){
    ui->doubleSpinBox_velocity->stepUp();
}

void gui_controls::vel_min(){
    ui->doubleSpinBox_velocity->stepDown();
}

void gui_controls::euler_deg_sec_plus(){
    ui->doubleSpinBox_euler_max_deg_sec->stepUp();
}

void gui_controls::euler_deg_sec_min(){
    ui->doubleSpinBox_euler_max_deg_sec->stepDown();
}

void gui_controls::joint_deg_sec_plus(){
    ui->doubleSpinBox_joint_max_deg_sec->stepUp();
}

void gui_controls::joint_deg_sec_min(){
    ui->doubleSpinBox_joint_max_deg_sec->stepDown();
}

void gui_controls::euler_step_plus(){
    ui->doubleSpinBox_euler_stepsize->stepUp();
}

void gui_controls::euler_step_min(){
    ui->doubleSpinBox_euler_stepsize->stepDown();
}

void gui_controls::joint_step_plus(){
    ui->doubleSpinBox_joint_stepsize->stepUp();
}

void gui_controls::joint_step_min(){
    ui->doubleSpinBox_joint_stepsize->stepDown();
}

void gui_controls::cart_step_plus(){
    ui->doubleSpinBox_cart_stepsize->stepUp();
}

void gui_controls::cart_step_min(){
    ui->doubleSpinBox_cart_stepsize->stepDown();
}

void gui_controls::on_checkBox_mode_fb_toggled(bool checked)
{
    if(checked){
        mode_feedback=1;
        int ok=jog().jog_cart_euler(0,0,0,0,0,0);
        if(ok){
            okmove=true;
        }
    }
    if(!checked){
        mode_feedback=0;
    }
}

void gui_controls::on_checkBox_mode_ik_from_init_toggled(bool checked)
{
    if(checked){
        mode_ikfrominit=1;
    }
    if(!checked){
        mode_ikfrominit=0;
    }
}

void gui_controls::tool0_on(){
    *tool0->Pin=true;
}

void gui_controls::tool0_off(){
    *tool0->Pin=false;
}

void gui_controls::tool1_on(){
    *tool1->Pin=true;
}

void gui_controls::tool1_off(){
    *tool1->Pin=false;
}

void gui_controls::tool2_on(){
    *tool2->Pin=true;
}

void gui_controls::tool2_off(){
    *tool2->Pin=false;
}
