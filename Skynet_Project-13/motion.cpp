#include "motion.h"
#include <math.h>
#include <chrono>
#include "kinematic.h"
#include "halio.h"

extern bool start;
bool pause=false;
bool resume=false;
bool stop=false;

bool initmotion=false;
double adaptive_feed=1; //default 100% forward move
double variotime=0;
double dtg=0;
double timetogo=0;

double plottime=0;
bool plot=0;

std::vector<MOTION> MotionVec;

void motion::Init(){
    Thread = new std::thread(&motion::Program,this);
    Thread->detach(); // Execute the thread independent from other stuff.
    std::cout<<"Motion init ok"<<std::endl;
}

void motion::Program(){

    while(1){

        if(!initmotion){

            // This is a motion example :

            MOTION M;

            // A motion;
            M.PaX=625;
            M.PaY=0;
            M.PaZ=890;
            M.PbX=625;
            M.PbY=100;
            M.PbZ=990;
            M.Vo=0;
            M.Vel=200;
            M.Al=100;
            M.CurveType=0;
            MotionVec.push_back(M);



            initmotion=1;
        }

        while(start){

            using clock_type = std::chrono::high_resolution_clock;

            // Preprocess, we have to know the motion total time Ttot.
            for(unsigned int i=0; i<MotionVec.size(); i++){

                MotionVec.at(i)=MotionScurveCmd(MotionVec.at(i)); // Singleshot, get the total path time Ttot.

                auto starttime = clock_type::now();

                bool finished=0;
                while (!finished){

                    auto endtime = clock_type::now();
                    auto total = endtime - starttime;
                    variotime = std::chrono::duration_cast<std::chrono::nanoseconds>(total).count()*0.000000001;

                    if(variotime<=MotionVec.at(i).Ttot){

                        MotionVec.at(i).t = variotime;
                        MotionVec.at(i)=MotionScurveCmd(MotionVec.at(i));

                        // Debug
                        //Print("Px",MotionVec.at(i).Px);
                        //Print("Py",MotionVec.at(i).Py);
                        //Print("Pz",MotionVec.at(i).Pz);

                        // Perform kinematic
                        XyzList.at(0) = MotionVec.at(i).Px;
                        XyzList.at(1) = MotionVec.at(i).Py;
                        XyzList.at(2) = MotionVec.at(i).Pz;

                        EulerList.at(0) = 0;
                        EulerList.at(1) = 0;
                        EulerList.at(2) = 0;


                        int ok=kinematic().Ik(0);
                        if(!ok){std::cout<<"Ik error"<<std::endl;} else {
                            //std::cout<<"Ik ok"<<std::endl;
                        }

                        // Update hal pins.
                        //ok=halio().Update();
                        //if(!ok){std::cout<<"Hal update error"<<std::endl;} else {
                            //std::cout<<"Hal update ok"<<std::endl;
                        //}

                    } else {
                        finished=1;
                        variotime=0;
                    }
                }

            }
            start=false; // Program finished.
            MotionVec.clear();
            initmotion=0;

        }

    }
}

//! Return point at current motioncommand.
//! Coded in C-style to ensure halcompile --compile compatible
MOTION motion::MotionScurveCmd(MOTION M){

    //! -- --- -- Input -- --- --

    double t=M.t;                                   // Return xyz at time point t.
    double r=0;                                     // Path ratio r at time point t.
    double PaX=M.PaX;                               // Path start point.
    double PaY=M.PaY;
    double PaZ=M.PaZ;
    double PbX=M.PbX;                               // Path end point.
    double PbY=M.PbY;
    double PbZ=M.PbZ;
    double Ltot = sqrt(pow(PaX-PbX,2)+pow(PaY-PbY,2)+pow(PaZ-PbZ,2));

    double Vel = M.Vel;                             // mm/sec
    double Al = M.Al;                               // Lineair acceleration, mm/sec^2
    double Vo = M.Vo;                               // Start velocity (starting at concave period).

    int CurveType = M.CurveType;                    // CurveType = 0 => S-curve at both sides.
    // CurveType = 1 => No S-curve at left side.
    // CurveType = 2 => No S-curve at right side.
    // CurveType = 3 => No S-curve at both sides.
    //! -- --- -- Function -- --- --
    //!

    double As = Al*2;                               // Maximum acceleration encountered at the S-curve inflection point, Note at page 3 of document, mm/sec2
    double T  = Vel/Al;                             // Total Acc, dcc time.

    //double A  = 0;                                // Current acceleration.
    double Jm = 2*As/T;                             // Max Jerk for profile, 2*As/T <= Jm.
    //double V  = 0;                                // Current velocity.

    double Vh = 0;                                  // Vel at start of convex period.
    double Sa = 0;                                  // S(t) Distance at concave period, changed from S to Sa.
    double Sb = 0;                                  // S(t) Distance at convex period, changed from S to Sb.
    double Sc = 0;                                  // Distance of atspeed period.

    //! Acc, Dcc, AtSpeed lenghts can be calculated as normal linear curve.

    double L1 = Vo*T + 0.5*Al*(T*T);                // Orginal formula : S=Vo*t + 0.5*Acc*t^2
    double L2 = Ltot-(2*L1);
    double L3 = L1;


    double T1 = T;                                  // acc time.
    double T2 = L2/Vel;                             // Time at full speed.
    double T3 = T;                                  // dcc time.
    double Ttot = T1+T2+T3;                         // Total traject time, if acc and dcc is active.

    // Safety function, you can turn this off to see difference in graph.
    if(L2<0){ // If we have a motion that can not reach a full speed, we have to act.
        L1-=Ltot/2;
        L2=0;
        L3-=Ltot/2;
        Ltot=L1+L2+L3;

        T1=Ttot/2;
        T2=0;
        T3=Ttot/2;
        Ttot=T1+T2+T3;

        Jm = 2*As/T1; //update jerk.

        //in this case
        CurveType=0;
    }

    // Set the no Scurve left side bit.
    if(CurveType==1){
        L2+=L1;
        L1=0;
        Ltot=L1+L2+L3;

        T2=L2/Vel; //repaired.
        T1=0;
        Ttot=T1+T2+T3;
    }

    // Set the no Scurve right side bit.
    if(CurveType==2){
        L2+=L3;
        L3=0;
        Ltot=L1+L2+L3;

        T2=L2/Vel; //repaired.
        T3=0;
        Ttot=T1+T2+T3;
    }

    // Set the no Scurve bit on both sides.
    if(CurveType==3){
        L1=0;
        L2=Ltot;
        L3=0;
        Ltot=L1+L2+L3;

        Ttot=Ltot/Vel;
        T1=0;
        T2=Ttot;
        T3=0;
        Ttot=T1+T2+T3;
    }

    M.Ttot=Ttot; // At this stage we now total traject time of this line.

    // Debug :
    //        std::cout << "L1  : " << L1 << std::endl;
    //        std::cout << "L2  : " << L2 << std::endl;
    //        std::cout << "L3  : " << L3 << std::endl;
    //        std::cout << "Ltot: " << Ltot << std::endl;

    //        std::cout << "T1   : " << T1 << std::endl;
    //        std::cout << "T2   : " << T2 << std::endl;
    //        std::cout << "T3   : " << T3 << std::endl;
    //        std::cout << "Ttot : " << Ttot << std::endl;

    //    for(double t=0; t<=Ttot; t+=0.005){ //for testing.

    if(t<T1){                                               //! Acc period.
        if(t<=T1/2){                                        // Acc concave period.
            Sa = Vo*t + Jm*(t*t*t)/6;
            M.V = Vo + Jm*(t*t)/2;
            M.A = Jm*t;

            r=Sa/Ltot;                                      // Path ratio
            M.Px= PaX+( r*(PbX-PaX));                         // Interpolate path point.
            M.Py= PaY+( r*(PbY-PaY));
            M.Pz= PaZ+( r*(PbZ-PaZ));

            return M;
        }
        if(t>T1/2){                                         //! Acc convex period.
            double th=T1/2;                                 // th=half acc,dcc period.
            Sa = Vo*th + Jm*(th*th*th)/6;
            Vh = Vo + Jm*(th*th)/2;                         // Velocity at end of concave period

            double tt=t-(T1/2);                             // Convex starttime period=0, so tt=t-convave period
            Sb = Vh*tt + As*(tt*tt)/2 -Jm*(tt*tt*tt)/6;
            M.V = Vh + As*tt -Jm*(tt*tt)/2;
            M.A = As - (Jm*tt);

            r=(Sa+Sb)/Ltot;                                 // Concave period + part of convex period
            M.Px= PaX+( r*(PbX-PaX));
            M.Py= PaY+( r*(PbY-PaY));
            M.Pz= PaZ+( r*(PbZ-PaZ));

            return M;
        }
    }

    if(t>=T1 && t<=T1+T2){                                  //! Atspeed period.
        Sc = (t-T1)*Vel;                                 // Time=Dist/Vel
        M.V = Vo+Vel; //repaired.
        M.A = 0;

        r= (L1+Sc)/Ltot;
        M.Px= PaX+( r*(PbX-PaX));
        M.Py= PaY+( r*(PbY-PaY));
        M.Pz= PaZ+( r*(PbZ-PaZ));

        return M;
    }

    if((t>T1+T2)){                         //! Dcc period
        if(t<=(T1+T2+(T3/2))){                              // Dcc concave period.
            double th=T3/2;                                 // th=half acc,dcc period.
            Sa = Vo*th + Jm*(th*th*th)/6;
            Vh = Vo + Jm*(th*th)/2;                         // Velocity at end of concave period

            double tempSb = Vh*th + As*(th*th)/2 -Jm*(th*th*th)/6;  // Convex path lenght of acc period.

            double tt= th - (t-(T1+T2));
            Sb = tempSb - ( Vh*tt + As*(tt*tt)/2 -Jm*(tt*tt*tt)/6 );

            M.V = Vh + As*tt -Jm*(tt*tt)/2;
            M.A = -abs(As - (Jm*tt));

            r= (L1+L2+Sb)/Ltot;
            M.Px= PaX+( r*(PbX-PaX));
            M.Py= PaY+( r*(PbY-PaY));
            M.Pz= PaZ+( r*(PbZ-PaZ));

            return M;
        }
        if(t>(T1+T2+(T3/2))){                               // Dcc convex period.

            double th=T3/2;                                 // Th=half acc,dcc period.
            double tempSa = Vo*th + Jm*(th*th*th)/6;

            Vh = Vo + Jm*(th*th)/2;
            Sb = Vh*th + As*(th*th)/2 -Jm*(th*th*th)/6;

            double tt= th- (t-(T1+T2+th));

            Sa = tempSa - ( Vo*tt + Jm*(tt*tt*tt)/6 );
            M.V = Vo + Jm*(tt*tt)/2;
            M.A = -abs(Jm*tt);

            r= (L1+L2+Sb+Sa)/Ltot;
            M.Px= PaX+( r*(PbX-PaX));
            M.Py= PaY+( r*(PbY-PaY));
            M.Pz= PaZ+( r*(PbZ-PaZ));

            return M;
        }
    }
    //} // For testing.
    return M;
}

void motion::Print(std::string string, double value){
    string.append(": ");
    std::cout<<string<<value<<std::endl;
}
