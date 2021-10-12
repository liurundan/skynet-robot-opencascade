#ifndef SCURVE_H
#define SCURVE_H

#include <iostream>
#include <vector>
#include <math.h>
#include <string>

#include <QString>
#include <gp_Pnt.hxx>

class scurve
{
public:
    scurve();

    struct point {
        double x=0,y=0,z=0;
    };
    std::vector<point> pointvec; // holds the calculated traject points for every 1ms
    double linelenght=0;

    std::vector<point> create_point_for_every_ms_path(double velmax, double accmax, double vo, double ve, std::vector<point> pathvec);
    //std::vector<gp_Pnt> pointvecms;

    std::vector<point> scurve_create_point_for_every_ms(double velmax, double accmax, double vo, double ve, double xs, double ys, double zs, double xe, double ye, double ze);

    struct lines {
        double xs=0,ys=0,zs=0;
        double xe=0,ye=0,ze=0;
    };

    struct waypoint {
        std::string type;       //line
        double xs=0,ys=0,zs=0;  //startpos
        double xe=0,ye=0,ze=0;  //endpos
        double lenght=0;        //lenght of primitive.
    };

    struct traject {

        double Vo=0;            //start velocity
        double Ve=0;            //end velocity
        double Vm=0;            //max velocity if atspeed can not be reached
        double Vel=0;           //feedrate
        double Acc_lineair=0;   //lineair acceleration, mm/sec^2
        double Acc_inflation=0; //max acceleration at inflection point, acc_lin*2, mm/sec^2

        double T1=0;            //total Acc time.
        double T2=0;            //total atspeed time.
        double T3=0;            //total Dcc time.
        double Ttot=0;          //total traject time, T1+T2+T3
        double T1h=0;           //total half Acc time, time to inflection point.
        double T3h=0;           //total half Dcc time, time to inflection point.

        double L1=0;            //acceleration lenght
        double L2=0;            //atspeed lenght.
        double L3=0;            //deacceleration lenght
        double Ltot=0;          //total traject lenght, L1+L2+L3
        double Li=0;            //line-line intersection x-value

        double Jm=0;            //max jerk for profile, x*acc_infl/T1 or T3

        std::vector<waypoint> Waypoints;
    };

    traject TrajectCalculator(double Vel, double Acc, double Vo, double Ve, std::vector<waypoint> waypoints);
    traject ScurveUp(traject tr);
    traject ScurveSteady(traject tr);
    traject ScurveDown(traject tr);
    waypoint LineLenght(waypoint p);
    double ArcLenght(waypoint p);
    double TrajectLenght(std::vector<waypoint> pv);
    point LineLineIntersect(lines a, lines b);

private:


};

#endif // SCURVE_H
