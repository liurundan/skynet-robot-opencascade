#include "jog.h"

//! Make conversion's easy:
#define toRadians M_PI/180.0
#define toDegrees (180.0/M_PI)

jog::jog()
{

}

int jog::jog_mdi_joint(double j0_end_deg, double j1_end_deg, double j2_end_deg, double j3_end_deg, double j4_end_deg, double j5_end_deg, double maxdegsec){

    KDL::Frame temp=cart;
    // Remove previous performed files from the ./stream dir.
    system("rm -rf ./stream/*.txt");

    // First look wich joint has to move the most in degrees. This one will be the 100% joint.
    double j0_dis=abs(j0_end_deg-(*J0_Fb->Pin));
    double j1_dis=abs(j1_end_deg-(*J1_Fb->Pin));
    double j2_dis=abs(j2_end_deg-(*J2_Fb->Pin));
    double j3_dis=abs(j3_end_deg-(*J3_Fb->Pin));
    double j4_dis=abs(j4_end_deg-(*J4_Fb->Pin));
    double j5_dis=abs(j5_end_deg-(*J5_Fb->Pin));

    // Get the highest value
    double deg=0;
    if(deg<j0_dis){
        deg=j0_dis;
    }
    if(deg<j1_dis){
        deg=j1_dis;
    }
    if(deg<j2_dis){
        deg=j2_dis;
    }
    if(deg<j3_dis){
        deg=j3_dis;
    }
    if(deg<j4_dis){
        deg=j4_dis;
    }
    if(deg<j5_dis){
        deg=j5_dis;
    }
    //std::cout<<"deg:"<<deg<<std::endl;

    std::fstream newfile;
    std::string filename="./stream/stream";
    unsigned int j=0, filenr=0;

    double sec=abs(deg)/maxdegsec;
    double ms=sec*1000;

    // Start position
    double j0_start_deg=*J0_Fb->Pin;
    double j1_start_deg=*J1_Fb->Pin;
    double j2_start_deg=*J2_Fb->Pin;
    double j3_start_deg=*J3_Fb->Pin;
    double j4_start_deg=*J4_Fb->Pin;
    double j5_start_deg=*J5_Fb->Pin;

    for(unsigned int i=0; i<ms; i++){

        if(j==0){ // Up to 1000 lines.
            std::string tot=filename+std::to_string(filenr)+".txt";
            newfile.open(tot,std::ios_base::out); // out = new clean file, app = append to file
        }
        if(newfile.is_open()){

            KDLJointCur(0)+=(1/ms)*((j0_end_deg-j0_start_deg)*toRadians);
            KDLJointCur(1)+=(1/ms)*((j1_end_deg-j1_start_deg)*toRadians);
            KDLJointCur(2)+=(1/ms)*((j2_end_deg-j2_start_deg)*toRadians);
            KDLJointCur(3)+=(1/ms)*((j3_end_deg-j3_start_deg)*toRadians);
            KDLJointCur(4)+=(1/ms)*((j4_end_deg-j4_start_deg)*toRadians);
            KDLJointCur(5)+=(1/ms)*((j5_end_deg-j5_start_deg)*toRadians);
            kinematic().Fk();

            int ok=kinematic().Ik();
            if(ok){
                newfile<<std::fixed<<std::setprecision(3)<<
                         KDLJointCur(0)*toDegrees   <<" "<<
                         KDLJointCur(1)*toDegrees   <<" "<<
                         KDLJointCur(2)*toDegrees   <<" "<<
                         KDLJointCur(3)*toDegrees   <<" "<<
                         KDLJointCur(4)*toDegrees   <<" "<<
                         KDLJointCur(5)*toDegrees   <<" "<< // Radians
                         cart.p.x()                 <<" "<< // Hal feedbak
                         cart.p.y()                 <<" "<<
                         cart.p.z()                 <<" "<<
                         cart.M.GetRot().x()*toDegrees        <<" "<< // Hal feedback
                         cart.M.GetRot().y()*toDegrees        <<" "<<
                         cart.M.GetRot().z()*toDegrees        <<" "<<
                         std::endl;
            }
            if(!ok){
                std::cout<<"ik error"<<std::endl;
                newfile.close();
                system("rm -rf ./stream/*.txt");
                cart=temp;
                kinematic().Fk();
                kinematic().Ik();
                return 0;
            }

            j++;

            if(j==1000){
                newfile.close();
                filenr++;
                j=0;
            }
        }
    }
    //std::cout<<"ready"<<std::endl;

    // Get ammount of files.
    newfile.close();
    ammount=0;
    struct dirent *d;
    DIR *dr;
    dr = opendir("./stream/");
    if(dr!=NULL)
    {
        //std::cout<<"List of Files & Folders:-\n";
        for(d=readdir(dr); d!=NULL; d=readdir(dr))
        {
            //std::cout<<d->d_name<<std::endl;
            ammount++;
        }
        closedir(dr);
    }
    else {
        std::cout<<"\nError Occurred!";
        return 0;
    }

    // Activate the halstream by giving it data
    return 1;
}

int jog::jog_joint(std::string joint, double deg){

    KDL::Frame temp=cart;
    // Remove previous performed files from the ./stream dir.
    system("rm -rf ./stream/*.txt");

    std::fstream newfile;
    std::string filename="./stream/stream";
    unsigned int j=0, filenr=0;

    double sec=abs(deg)/joint_maxdegsec->Pin;
    double ms=sec*1000;
    double rot=0;

    for(unsigned int i=0; i<ms; i++){

        if(j==0){ // Up to 1000 lines.
            std::string tot=filename+std::to_string(filenr)+".txt";
            newfile.open(tot,std::ios_base::out); // out = new clean file, app = append to file
        }
        if(newfile.is_open()){

            //double procentual=((double(i)+1)/(double(pointvec.size())))*100;
            //std::cout<<"procentual:"<<procentual<<std::endl;

            rot=(1/ms)*(deg*toRadians);

            if(joint=="j0"){
                KDLJointCur(0)+=rot;
                kinematic().Fk();
            }
            if(joint=="j1"){
                KDLJointCur(1)+=rot;
                kinematic().Fk();
            }
            if(joint=="j2"){
                KDLJointCur(2)+=rot;
                kinematic().Fk();
            }
            if(joint=="j3"){
                KDLJointCur(3)+=rot;
                kinematic().Fk();
            }
            if(joint=="j4"){
                KDLJointCur(4)+=rot;
                kinematic().Fk();
            }
            if(joint=="j5"){
                KDLJointCur(5)+=rot;
                kinematic().Fk();
            }

            int ok=kinematic().Ik();
            if(ok){
                newfile<<std::fixed<<std::setprecision(3)<<
                         KDLJointCur(0)*toDegrees   <<" "<<
                         KDLJointCur(1)*toDegrees   <<" "<<
                         KDLJointCur(2)*toDegrees   <<" "<<
                         KDLJointCur(3)*toDegrees   <<" "<<
                         KDLJointCur(4)*toDegrees   <<" "<<
                         KDLJointCur(5)*toDegrees   <<" "<< // Radians
                         cart.p.x()                 <<" "<< // Hal feedbak
                         cart.p.y()                 <<" "<<
                         cart.p.z()                 <<" "<<
                         cart.M.GetRot().x()*toDegrees        <<" "<< // Hal feedback
                         cart.M.GetRot().y()*toDegrees        <<" "<<
                         cart.M.GetRot().z()*toDegrees        <<" "<<
                         std::endl;
            }
            if(!ok){
                std::cout<<"ik error"<<std::endl;
                newfile.close();
                system("rm -rf ./stream/*.txt");
                cart=temp;
                kinematic().Fk();
                kinematic().Ik();
                return 0;
            }

            j++;

            if(j==1000){
                newfile.close();
                filenr++;
                j=0;
            }
        }
    }
    //std::cout<<"ready"<<std::endl;

    // Get ammount of files.
    newfile.close();
    ammount=0;
    struct dirent *d;
    DIR *dr;
    dr = opendir("./stream/");
    if(dr!=NULL)
    {
        //std::cout<<"List of Files & Folders:-\n";
        for(d=readdir(dr); d!=NULL; d=readdir(dr))
        {
            //std::cout<<d->d_name<<std::endl;
            ammount++;
        }
        closedir(dr);
    }
    else {
        std::cout<<"\nError Occurred!";
        return 0;
    }

    // Activate the halstream by giving it data
    return 1;
}

int jog::jog_cart_euler(double distx, double disty, double distz,
                        double degx, double degy, double degz){

    KDL::Frame temp=cart;

    // Remove previous performed files from the ./stream dir.
    system("rm -rf ./stream/*.txt");

    // S-curve calculated points for the given stepsize path lenght.
    std::vector<scurve::point> pointvec;
    pointvec=scurve().scurve_create_point_for_every_ms(velmax->Pin,
                                                       accmax->Pin,
                                                       0,                       //vo, vel initial
                                                       0,                       //ve, vel end
                                                       cart.p.x(),        //x current cart pos
                                                       cart.p.y(),        //y
                                                       cart.p.z(),        //z
                                                       cart.p.x()+distx,  //x final cart pos
                                                       cart.p.y()+disty,
                                                       cart.p.z()+distz);

    std::fstream newfile;
    std::string filename="./stream/stream";
    unsigned int j=0, filenr=0;

    // Problem with s-curve when distance=0mm, the value is nan.
    if(distx==0 && disty==0 && distz==0){
        pointvec.resize(1);
        pointvec.back().x=cart.p.x();
        pointvec.back().y=cart.p.y();
        pointvec.back().z=cart.p.z();


        // When performing euler angles without cart move, we want the tcp stay's concentrated at tcp point during move.
        // We add extra waypoints in ms, with a maxdegsec parameter to limit the euler speed.
        double sec=0;
        double secx=abs(degx)/euler_maxdegsec->Pin;
        double secy=abs(degy)/euler_maxdegsec->Pin;
        double secz=abs(degz)/euler_maxdegsec->Pin;

        // Get the max sec value
        if(sec<secx){
            sec=secx;
        }
        if(sec<secy){
            sec=secy;
        }
        if(sec<secz){
            sec=secz;
        }
        double mssec=sec*1000;

        // Resize pointvec to the needed ms value.
        scurve::point p; // Current tcp cart calue.
        p.x=cart.p.x();
        p.y=cart.p.y();
        p.z=cart.p.z();
        for(unsigned int i=0; i<mssec; i++){
            pointvec.push_back(p);
        }
    }

    // Store euler values at init time of function.
    double eulz,euly,eulx;
    cart.M.GetEulerZYX(eulz,euly,eulx);


    for(unsigned int i=0; i<pointvec.size(); i++){

        if(j==0){ // Up to 1000 lines.
            std::string tot=filename+std::to_string(filenr)+".txt";
            newfile.open(tot,std::ios_base::out); // out = new clean file, app = append to file
        }
        if(newfile.is_open()){

            cart.p.x(pointvec.at(i).x);
            cart.p.y(pointvec.at(i).y);
            cart.p.z(pointvec.at(i).z);

            //double procentual=((double(i)+1)/(double(pointvec.size())))*100;
            //std::cout<<"procentual:"<<procentual<<std::endl;

            //std::cout<<"check:"<<(1/double(pointvec.size()))*pointvec.size()<<std::endl;

            cart.M.DoRotX((1/double(pointvec.size()))*(degx*toRadians));
            cart.M.DoRotY((1/double(pointvec.size()))*(degy*toRadians));
            cart.M.DoRotZ((1/double(pointvec.size()))*(degz*toRadians));

            int ok=kinematic().Ik();
            if(ok){
                newfile<<                                       // std::fixed<<std::setprecision(3)<<
                                                                KDLJointCur(0)*toDegrees       <<" "<<
                                                                KDLJointCur(1)*toDegrees       <<" "<<
                                                                KDLJointCur(2)*toDegrees       <<" "<<
                                                                KDLJointCur(3)*toDegrees       <<" "<<
                                                                KDLJointCur(4)*toDegrees       <<" "<<
                                                                KDLJointCur(5)*toDegrees       <<" "<< // Radians
                                                                cart.p.x()                     <<" "<< // Hal feedbak
                                                                cart.p.y()                     <<" "<<
                                                                cart.p.z()                     <<" "<<
                                                                cart.M.GetRot().x()*toDegrees  <<" "<< // Hal feedback
                                                                cart.M.GetRot().y()*toDegrees  <<" "<<
                                                                cart.M.GetRot().z()*toDegrees  <<" "<<
                                                                std::endl;
            }
            if(!ok){
                //std::cout<<"ik error"<<std::endl;
                //newfile.close();
                //system("rm -rf ./stream/*.txt");
                //cart=temp;
                kinematic().Fk();
                kinematic().Ik();
                //return 0;
            }

            j++;

            if(j==3000){
                newfile.close();
                filenr++;
                j=0;
            }
        }
    }
    newfile.close();

    //std::cout<<"ready"<<std::endl;

    // Get ammount of files.
    ammount=0;
    struct dirent *d;
    DIR *dr;
    dr = opendir("./stream/");
    if(dr!=NULL)
    {
        //std::cout<<"List of Files & Folders:-\n";
        for(d=readdir(dr); d!=NULL; d=readdir(dr))
        {
            //std::cout<<d->d_name<<std::endl;
            ammount++;
        }
        closedir(dr);
    }
    else {
        std::cout<<"\nError Occurred!";
        return 0;
    }

    // Activate the halstream by giving it data
    return 1;
}

















