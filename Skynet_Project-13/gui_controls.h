#ifndef GUI_CONTROLS_H
#define GUI_CONTROLS_H

#include <halio.h>
#include <jog.h>
#include <variable.h>

#include <QDialog>

namespace Ui {
class gui_controls;
}

class gui_controls : public QDialog
{
    Q_OBJECT

public:
    explicit gui_controls(QWidget *parent = nullptr);
    ~gui_controls();

    void set_enabled(bool ok);

private slots:

    void cartx_plus();
    void cartx_min();
    void carty_plus();
    void carty_min();
    void cartz_plus();
    void cartz_min();
    void update_cart_stepsize(double val);
    void update_euler_stepsize(double val);
    void update_euler_maxdegsec(double val);
    void update_joint_stepsize(double val);
    void update_joint_maxdegsec(double val);
    void update_accmax(double val);
    void update_velmax(double val);

    void eulerx_plus();
    void eulerx_min();
    void eulery_plus();
    void eulery_min();
    void eulerz_plus();
    void eulerz_min();

    void on_checkBox_mode_fb_toggled(bool checked);
    void on_checkBox_mode_ik_from_init_toggled(bool checked);

    void euler_step_plus();
    void euler_step_min();
    void joint_step_plus();
    void joint_step_min();
    void cart_step_plus();
    void cart_step_min();
    void acc_plus();
    void acc_min();
    void vel_plus();
    void vel_min();
    void euler_deg_sec_plus();
    void euler_deg_sec_min();
    void joint_deg_sec_plus();
    void joint_deg_sec_min();
    void j0_plus();
    void j0_min();
    void j1_plus();
    void j1_min();
    void j2_plus();
    void j2_min();
    void j3_plus();
    void j3_min();
    void j4_plus();
    void j4_min();
    void j5_plus();
    void j5_min();

    void tool0_on();
    void tool0_off();
    void tool1_on();
    void tool1_off();
    void tool2_on();
    void tool2_off();

    void mdi_joint_go();
    void mdi_tool_go();
    void mdi_copy_cur_joint_pos();
    void mdi_reset_cur_joint_pos();
    void mdi_go_home_pos();

    void mdi_cart(double x, double y, double z, bool tooldir);
    void mdi_cart_go();
    void mdi_copy_cur_cart_pos();

    void pause();
    void resume();

    void tooldir_x_min();
    void tooldir_x_plus();
    void tooldir_y_min();
    void tooldir_y_plus();
    void tooldir_z_min();
    void tooldir_z_plus();

private:
    Ui::gui_controls *ui;

    QString darkgrey="background-color: rgba(65, 65, 65, 0);\ncolor: rgb(255, 255, 255);";
    QString green="background-color: rgba(170, 255, 0, 0);\ncolor: rgb(0, 0, 0);";
    QString red="background-color: rgba(242, 0, 0, 0);\ncolor: rgb(0, 0, 0);";
    QString orange="background-color: rgba(255, 170, 0, 0);\ncolor: rgb(0, 0, 0);";
    QString grey="background-color: rgba(81, 81, 81, 0);\ncolor: rgb(255, 255, 255);";

};

#endif // GUI_CONTROLS_H
































