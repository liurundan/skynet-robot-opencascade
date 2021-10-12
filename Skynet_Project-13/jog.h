#ifndef JOG_H
#define JOG_H

#include <variable.h>
#include <scurve.h>
#include <kinematic.h>
#include <halio.h>

#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <dirent.h>

class jog
{
public:
    jog();
    int jog_cart_euler(double distx, double disty, double distz,
                       double degx, double degy, double degz);
    int jog_joint(std::string joint, double deg);
    int jog_mdi_joint(double j0_end_deg, double j1_end_deg, double j2_end_deg, double j3_end_deg, double j4_end_deg, double j5_end_deg, double maxdegsec);

private:

};

#endif // JOG_H
