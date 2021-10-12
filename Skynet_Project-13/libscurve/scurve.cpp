#include "scurve.h"

scurve::scurve()
{

}

std::vector<scurve::point> scurve::create_point_for_every_ms_path(double velmax, double accmax, double vo, double ve, std::vector<scurve::point> pathvec){

    // Current problem, the scurve does only calculate one line at the moment.

//    // Empty previous calculated tasks.
//    pointvec.clear();

//    //linelenght=sqrt(pow(xe-xs,2)+pow(ye-ys,2)+pow(ze-zs,2));

//    // This command creates one motion from start to stop.

//    // Storage container, bucket
//    std::vector<waypoint> wpvec;
//    waypoint wp;

//    for(unsigned int i=0; i<pathvec.size()-1; i++){
//        wp.type="line";
//        wp.xs=pathvec.at(i).x; wp.ys=pathvec.at(i).y; wp.zs=pathvec.at(i).z;
//        wp.xe=pathvec.at(i+1).x; wp.ye=pathvec.at(i+1).y; wp.ze=pathvec.at(i+1).z;
//        wp=LineLenght(wp);
//        wpvec.push_back(wp);

//        std::cout<<"pathvec start x:"<<wp.xs<<" y:"<<wp.ys<<" z:"<<wp.zs<<std::endl;
//        std::cout<<"pathvec end   x:"<<wp.xe<<" y:"<<wp.ye<<" z:"<<wp.ze<<std::endl;
//    }

//    traject traject;

//    // Add traject properties and calculate traject interval
//    traject = TrajectCalculator(velmax,
//                                accmax,
//                                vo,
//                                ve,
//                                wpvec);

//    // Calculate Scurve
//    ScurveUp(traject);
//    ScurveSteady(traject);
//    ScurveDown(traject);

//    //std::cout<<"pointvec.size:"<<pointvec.size()<<std::endl;

//    for(unsigned int i=0; i<pointvec.size(); i++){
//        std::cout<<"scurve function pointvec at i:"<<i<<" x:"<<pointvec.at(i).x<<" y:"<<pointvec.at(i).y<<" z:"<<pointvec.at(i).z<<std::endl;
//    }

    return pointvec; // Every point is 1 ms.
}

std::vector<scurve::point> scurve::scurve_create_point_for_every_ms(double velmax, double accmax, double vo, double ve,
                                                                    double xs, double ys, double zs,
                                                                    double xe, double ye, double ze){
    // Empty previous calculated tasks.
    pointvec.clear();

    linelenght=sqrt(pow(xe-xs,2)+pow(ye-ys,2)+pow(ze-zs,2));

    // This command creates one motion from start to stop.

    // Storage container, bucket
    std::vector<waypoint> wpvec;
    waypoint wp;

    wp.type="line";
    wp.xs=xs; wp.ys=ys; wp.zs=zs;
    wp.xe=xe; wp.ye=ye; wp.ze=ze;
    wp=LineLenght(wp);
    wpvec.push_back(wp);

    traject traject;

    // Add traject properties and calculate traject interval
    traject = TrajectCalculator(velmax,
                                accmax,
                                vo,
                                ve,
                                wpvec);

    // Calculate Scurve
    ScurveUp(traject);
    ScurveSteady(traject);
    ScurveDown(traject);

    //std::cout<<"pointvec.size:"<<pointvec.size()<<std::endl;
    return pointvec; // Every point is 1 ms.
}

scurve::traject scurve::ScurveUp(traject tr){

    double A=0;                                     // Current acceleration.
    double V=0;                                     // Current velocity.
    double t=0;                                     // Time.
    double Jm=0;                                    // Max Jerk for profile, 2*Ainfl/T <= Jm.
    double Sa=0;                                    // Concave distance.
    double Sb=0;                                    // Convex distance.

    for(double T=0; T<=tr.T1; T+=0.001){

        Jm = 2*tr.Acc_inflation/tr.T1;

        if(T<(tr.T1/2)){

            t=T;

            V = Jm*(t*t)/2;
            V+=tr.Vo;                               // Add initial velocity.
            A = Jm*t;
            Sa = 0*t + Jm*(t*t*t)/6;

            point p;
            p.x=tr.Waypoints.at(0).xs+((tr.Waypoints.at(0).xe-tr.Waypoints.at(0).xs)*(Sa/linelenght));
            p.y=tr.Waypoints.at(0).ys+((tr.Waypoints.at(0).ye-tr.Waypoints.at(0).ys)*(Sa/linelenght));
            p.z=tr.Waypoints.at(0).zs+((tr.Waypoints.at(0).ze-tr.Waypoints.at(0).zs)*(Sa/linelenght));
            pointvec.push_back(p);         // Store the path point every 1ms, t=0.001

            //std::cout << "Concave V:"<<V<< " A:"<<A<<" S:"<<Sa<<std::endl;
            //            ellipse = scene->addEllipse(T, V*-1, 0.1, 0.1, CyanPen); // Be aware graphicsview is inverted on y-axis.
            //            ellipse = scene->addEllipse(T, A*-1, 0.1, 0.1, BlackPen);
            //            ellipse = scene->addEllipse(T, scale*(Sa*-1), 0.1, 0.1, GreyPen);
        }

        if(T>(tr.T1/2) && T<=(tr.T1)){

            t=tr.T1h-(T-(0.5*tr.T1));               // T inverted

            V = Jm*(t*t)/2;
            V = tr.Vel-V;                           // Mirror
            A = Jm*t;

            t=T-(0.5*tr.T1);                        // T non inverted
            Sb = Jm*(tr.T1h*tr.T1h)/2*t + tr.Acc_inflation*(t*t)/2 -Jm*(t*t*t)/6;

            point p;
            p.x=tr.Waypoints.at(0).xs+((tr.Waypoints.at(0).xe-tr.Waypoints.at(0).xs)*((Sa+Sb)/linelenght));
            p.y=tr.Waypoints.at(0).ys+((tr.Waypoints.at(0).ye-tr.Waypoints.at(0).ys)*((Sa+Sb)/linelenght));
            p.z=tr.Waypoints.at(0).zs+((tr.Waypoints.at(0).ze-tr.Waypoints.at(0).zs)*((Sa+Sb)/linelenght));
            pointvec.push_back(p);         // Store the path point every 1ms, t=0.001

            //std::cout << "Convex V:"<<V<< " A:"<<A<<" S:"<<Sa+Sb<<std::endl;
            //            ellipse = scene->addEllipse(T, V*-1, 0.1, 0.1, RedPen);
            //            ellipse = scene->addEllipse(T, A*-1, 0.1, 0.1, BlackPen);
            //            ellipse = scene->addEllipse(T, scale*((Sa+Sb)*-1), 0.1, 0.1, GreyPen);
        }
    }
    //std::cout << "Convex V:"<<tr.Vel<< " A:"<<0<<" S:"<<tr.L1<<std::endl;
    point p;
    p.x=tr.Waypoints.at(0).xs+((tr.Waypoints.at(0).xe-tr.Waypoints.at(0).xs)*((Sa+Sb)/linelenght));
    p.y=0;
    p.z=0;
    pointvec.push_back(p);         // Store the path point every 1ms, t=0.001

    return tr;
}

scurve::traject scurve::ScurveSteady(traject tr){

    double A=0;                                     // Current acceleration.
    double V=0;                                     // Current velocity.
    double S=0;                                     // S(t) distance.

    for(double T=tr.T1; T<=tr.T1+tr.T2; T+=0.001){

        V=tr.Vel;
        A=0;
        //S=(tr.Vel*T)-tr.L1;

        S=tr.L1+((T-tr.T1)*tr.Vel);
        // Todo=Sa/tr.Ltot;                     // Point on path ratio.

        point p;
        p.x=tr.Waypoints.at(0).xs+((tr.Waypoints.at(0).xe-tr.Waypoints.at(0).xs)*(S/linelenght));
        p.y=tr.Waypoints.at(0).ys+((tr.Waypoints.at(0).ye-tr.Waypoints.at(0).ys)*(S/linelenght));
        p.z=tr.Waypoints.at(0).zs+((tr.Waypoints.at(0).ze-tr.Waypoints.at(0).zs)*(S/linelenght));
        pointvec.push_back(p);         // Store the path point every 1ms, t=0.001

        //std::cout << "Steady V:"<<V<< " A:"<<A<<" S:"<<S<<std::endl;
        //        ellipse = scene->addEllipse(T, V*-1, 0.1, 0.1, CyanPen); // Be aware graphicsview is inverted on y-axis.
        //        ellipse = scene->addEllipse(T, A*-1, 0.1, 0.1, BlackPen);
        //        ellipse = scene->addEllipse(T, scale*(S*-1), 0.1, 0.1, GreyPen);
    }
    return tr;
}

scurve::traject scurve::ScurveDown(traject tr){

    double A=0;                                     // Current acceleration.
    double V=0;                                     // Current velocity.
    double t=0;                                     // Time.
    double Jm=0;                                    // Max Jerk for profile, 2*Ainfl/T <= Jm.
    double Sa=0;                                    // Concave distance.
    double Sb=0;                                    // Convex distance.

    for(double T=tr.T1+tr.T2; T<=tr.Ttot; T+=0.001){

        Jm = 2*tr.Acc_inflation/tr.T3;

        if(T<(tr.T1+tr.T2+tr.T3h)){

            t=T-tr.T1-tr.T2;

            V = Jm*(t*t)/2;
            V = tr.Vel-V;                           // Mirror
            A = -abs(Jm*t);

            t=tr.T3h;
            double Sbmax=Jm*(tr.T3h*tr.T3h)/2*t + tr.Acc_inflation*(t*t)/2 -Jm*(t*t*t)/6;

            t=T-tr.T1-tr.T2;
            t=tr.T3h-t;

            Sb = Sbmax-( Jm*(tr.T3h*tr.T3h)/2*t + tr.Acc_inflation*(t*t)/2 -Jm*(t*t*t)/6 );
            Sb+= tr.L1+tr.L2;

            point p;
            p.x=tr.Waypoints.at(0).xs+((tr.Waypoints.at(0).xe-tr.Waypoints.at(0).xs)*(Sb/linelenght));
            p.y=tr.Waypoints.at(0).ys+((tr.Waypoints.at(0).ye-tr.Waypoints.at(0).ys)*(Sb/linelenght));
            p.z=tr.Waypoints.at(0).zs+((tr.Waypoints.at(0).ze-tr.Waypoints.at(0).zs)*(Sb/linelenght));
            pointvec.push_back(p);         // Store the path point every 1ms, t=0.001

            //std::cout << "Convex V:"<<V<< " A:"<<A<<" S:"<<Sb<<std::endl;
            //            ellipse = scene->addEllipse(T, V*-1, 0.1, 0.1, RedPen);
            //            ellipse = scene->addEllipse(T, A*-1, 0.1, 0.1, BlackPen);
            //            ellipse = scene->addEllipse(T, scale*(Sb*-1), 0.1, 0.1, GreyPen);
        }

        if(T>tr.T1+tr.T2+tr.T3h){

            t=T-tr.T1-tr.T2-tr.T3h;
            t=tr.T3h-t;

            V = Jm*(t*t)/2;
            V+=tr.Ve;
            A = -abs(Jm*t);

            t=tr.T3h;
            double SaMax =  0*t + Jm*(t*t*t)/6;

            t=T-tr.T1-tr.T2-tr.T3h;
            t=tr.T3h-t; // Mirror
            Sa = SaMax-( 0*t + Jm*(t*t*t)/6);

            point p;
            p.x=tr.Waypoints.at(0).xs+((tr.Waypoints.at(0).xe-tr.Waypoints.at(0).xs)*((Sb+Sa)/linelenght));
            p.y=tr.Waypoints.at(0).ys+((tr.Waypoints.at(0).ye-tr.Waypoints.at(0).ys)*((Sb+Sa)/linelenght));
            p.z=tr.Waypoints.at(0).zs+((tr.Waypoints.at(0).ze-tr.Waypoints.at(0).zs)*((Sb+Sa)/linelenght));
            pointvec.push_back(p);         // Store the path point every 1ms, t=0.001

            //std::cout << "Concave V:"<<V<< " A:"<<A<<" S:"<<Sb+Sa<<std::endl;
            //            ellipse = scene->addEllipse(T, V*-1, 0.1, 0.1, CyanPen); // Be aware graphicsview is inverted on y-axis.
            //            ellipse = scene->addEllipse(T, A*-1, 0.1, 0.1, BlackPen);
            //            ellipse = scene->addEllipse(T, scale*((Sb+Sa)*-1), 0.1, 0.1, GreyPen);
        }

    }
    //std::cout << "Convex V:"<<tr.Vel<< " A:"<<0<<" S:"<<Sa+Sb<<std::endl;
    //std::cout<<"End of path:"<<Sa+Sb<<std::endl;

    return tr;
}

double scurve::TrajectLenght(std::vector<waypoint> pv){
    double ltot=0;
    for(unsigned int i=0; i<pv.size(); i++){
        ltot+= pv.at(i).lenght;
    }
    return ltot;
}

scurve::waypoint scurve::LineLenght(waypoint p){
    if(p.type=="line"){
        p.lenght=sqrt(pow(p.xe-p.xs,2)+pow(p.ye-p.ys,2)+pow(p.ze-p.zs,2));
    }
    return p;
}

scurve::point scurve::LineLineIntersect(lines a,lines b){

    point p={0,0,0};

    //line a
    double delta_y0 = a.ye-a.ys;
    double delta_x0 = a.xs-a.xe;
    double c0 = delta_y0 * a.xs + delta_x0 * a.ys;

    //line b
    double delta_y1 = b.ye-b.ys;
    double delta_x1 = b.xs-b.xe;
    double c1 = delta_y1 * b.xs + delta_x1 * b.ys;

    double determinant = delta_y0*delta_x1 - delta_y1*delta_x0;

    if (determinant == 0){
        //std::cout<< "the lines are parallel"<<std::endl;
    } else {
        p.x = (delta_x1*c0 - delta_x0*c1)/determinant;
        p.y = (delta_y0*c1 - delta_y1*c0)/determinant;
    }

    //std::cout<<"x:"<<p.x << " y:"<<p.y<<std::endl;
    return p;
}

scurve::traject scurve::TrajectCalculator(double Vel, double Acc, double Vo, double Ve, std::vector<waypoint> waypoints){

    traject tr;
    // Data.
    tr.Vel=Vel;
    tr.Vo=Vo;
    tr.Ve=Ve;
    tr.Acc_lineair=Acc;

    // Formula.
    tr.Acc_inflation=tr.Acc_lineair*2;                      // Acceleration at s-curve inflation point = normal acceleration*2.
    tr.Waypoints=waypoints;
    tr.Ltot=TrajectLenght(tr.Waypoints);                    // Sum of all primitive lenghts = trajectlenght Ltot.

    // Acc path.
    tr.T1=(tr.Vel-tr.Vo)/tr.Acc_lineair;
    tr.L1=0.5*tr.Acc_lineair*pow(tr.T1,2);

    // Dcc path.
    tr.T3=(tr.Vel-tr.Ve)/tr.Acc_lineair;                    // We could use a seperate dcc value.
    tr.L3=0.5*tr.Acc_lineair*pow(tr.T3,2);

    // At speed path.
    tr.L2=tr.Ltot-tr.L1-tr.L3;
    tr.T2=tr.L2/tr.Vel;


    // Motion can not reach full speed, find acc dcc intersection.
    if(tr.L2<0){
        // Decrease Velocity until L2 is positive, if Acceleration stays the same value.
        double TempVel=tr.Vel;
        for(double i=TempVel; i>0; i-=0.1){

            tr.Vel=i;
            //std::cout<<"Vel:"<<tr.Vel<<std::endl;

            // Acc path.
            tr.T1=(tr.Vel-tr.Vo)/tr.Acc_lineair;
            tr.L1=0.5*tr.Acc_lineair*pow(tr.T1,2);

            // Dcc path.
            tr.T3=(tr.Vel-tr.Ve)/tr.Acc_lineair;            // We could use a seperate dcc value.
            tr.L3=0.5*tr.Acc_lineair*pow(tr.T3,2);

            // At speed path.
            tr.L2=tr.Ltot-tr.L1-tr.L3;
            tr.T2=tr.L2/tr.Vel;

            if(tr.L2>=0){

                //ui->doubleSpinBox_velocity_max->setValue(i);
                break;
            }
        }
    }

    tr.Ttot=tr.T1+tr.T2+tr.T3;
    tr.T1h=tr.T1/2;
    tr.T3h=tr.T3/2;

    /*
    std::cout << ""<< std::endl;
    std::cout << "TRAJECT CALCULATOR"<< std::endl;
    std::cout << "L1  : " << tr.L1 << std::endl;
    std::cout << "L2  : " << tr.L2 << std::endl;
    std::cout << "L3  : " << tr.L3 << std::endl;
    std::cout << "Ltot: " << tr.Ltot << std::endl;
    std::cout << ""<< std::endl;
    std::cout << "T1   : " << tr.T1 << std::endl;
    std::cout << "T2   : " << tr.T2 << std::endl;
    std::cout << "T3   : " << tr.T3 << std::endl;
    std::cout << "Ttot : " << tr.Ttot << std::endl;
    std::cout << ""<< std::endl;
    std::cout << "Vo   : " << tr.Vo << std::endl;
    std::cout << "Ve   : " << tr.Ve << std::endl;
    std::cout << "Acc  : " << tr.Acc_lineair << std::endl;
    std::cout << ""<< std::endl;
    */

    return tr;
}
