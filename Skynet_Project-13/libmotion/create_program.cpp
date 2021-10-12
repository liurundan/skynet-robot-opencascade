#include "create_program.h"

//! Make conversion's easy:
#define toRadians M_PI/180.0
#define toDegrees (180.0/M_PI)
#define toPresision std::fixed<<std::setprecision(3)

create_program::create_program()
{

}

bool create_program::point_every_ms(std::vector<Opencascade::bucket> bucketvec){

    // Remove previous performed files from the ./programstream dir.
    system("rm -rf ./stream/*.txt");
    system("rm -rf ./programstream/*.txt");
    filenr=0;
    // Create program from the machine's init position. Otherwise the current euler angles will be the euler start angles.
    kinematic().Fk_zero();
    cart=cartzero;

    for(unsigned int i=0; i<bucketvec.size(); i++){

        if(bucketvec.at(i).primitivetype=="line"){

            // Path value's:
            double vo=bucketvec.at(i).vo;
            double ve=bucketvec.at(i).ve;
            double velmax=bucketvec.at(i).velmax;
            double accmax=bucketvec.at(i).accmax;
            double xs=bucketvec.at(i).pointvec.front().X();
            double ys=bucketvec.at(i).pointvec.front().Y();
            double zs=bucketvec.at(i).pointvec.front().Z();
            double xe=bucketvec.at(i).pointvec.back().X();
            double ye=bucketvec.at(i).pointvec.back().Y();
            double ze=bucketvec.at(i).pointvec.back().Z();

            double eulerxs=bucketvec.at(i).eulervec.front().X(); //std::cout<<"eulxs:"<<eulxs*toDegrees<<std::endl;
            double eulerys=bucketvec.at(i).eulervec.front().Y(); //std::cout<<"eulys:"<<eulys*toDegrees<<std::endl;
            double eulerzs=bucketvec.at(i).eulervec.front().Z(); //std::cout<<"eulzs:"<<eulzs*toDegrees<<std::endl;
            double eulerxe=bucketvec.at(i).eulervec.back().X(); //std::cout<<"eulxs:"<<eulxe*toDegrees<<std::endl;
            double eulerye=bucketvec.at(i).eulervec.back().Y(); //std::cout<<"eulys:"<<eulye*toDegrees<<std::endl;
            double eulerze=bucketvec.at(i).eulervec.back().Z(); //std::cout<<"eulzs:"<<eulze*toDegrees<<std::endl;

            // S-curve calculated points for the given stepsize path lenght.
            std::vector<scurve::point> pointvecms;
            pointvecms=scurve().scurve_create_point_for_every_ms(velmax,
                                                                 accmax,
                                                                 vo,        //vo, vel initial
                                                                 ve,        //ve, vel end
                                                                 xs,        //x current cart pos
                                                                 ys,        //y
                                                                 zs,        //z
                                                                 xe,        //x final cart pos
                                                                 ye,
                                                                 ze);

            std::fstream newfile;
            std::string filename="./programstream/stream";
            unsigned int j=0;

            // Problem with s-curve when distance=0mm, the value is nan.
            double l=sqrt(pow(xe-xs,2)+pow(ye-ys,2)+pow(ze-zs,2));
            if(l<0.1){
                pointvecms.resize(1);
                pointvecms.back().x=cart.p.x();
                pointvecms.back().y=cart.p.y();
                pointvecms.back().z=cart.p.z();


                // When performing euler angles without cart move, we want the tcp stay's concentrated at tcp point during move.
                // We add extra waypoints in ms, with a maxdegsec parameter to limit the euler speed.
                double sec=0;
                double secx=abs((eulerxe-eulerxs)*toDegrees)/euler_maxdegsec->Pin;
                double secy=abs((eulerye-eulerys)*toDegrees)/euler_maxdegsec->Pin;
                double secz=abs((eulerze-eulerzs)*toDegrees)/euler_maxdegsec->Pin;

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
                    pointvecms.push_back(p);
                }
            }

            // Store euler values at init time of function.
            double eulz,euly,eulx;
            cart.M.GetEulerZYX(eulz,euly,eulx);

            filenr++;
            for(unsigned int i=0; i<pointvecms.size(); i++){

                if(j==0){ // Up to 1000 lines.
                    std::string tot=filename+std::to_string(filenr)+".txt";
                    newfile.open(tot,std::ios_base::out); // out = new clean file, app = append to file
                }
                if(newfile.is_open()){

                    cart.p.x(pointvecms.at(i).x);
                    cart.p.y(pointvecms.at(i).y);
                    cart.p.z(pointvecms.at(i).z);

                    //double procentual=((double(i)+1)/(double(pointvec.size())))*100;
                    //std::cout<<"procentual:"<<procentual<<std::endl;

                    //std::cout<<"check:"<<(1/double(pointvec.size()))*pointvec.size()<<std::endl;

                    cart.M.DoRotX((1/double(pointvecms.size()))*(eulerxe-eulerxs));
                    cart.M.DoRotY((1/double(pointvecms.size()))*(eulerye-eulerys));
                    cart.M.DoRotZ((1/double(pointvecms.size()))*(eulerze-eulerzs));

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
                        std::cout<<"ik error"<<std::endl;
                        // If there is a error. In future maybe add a line with divided values between the good lines.
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
        }
    }


    // Set the ammount of files to play for mainwindow.
    // Ammound is a global used variable.
    programammount=get_ammount_of_files();

    // At last. make the startpoint file.
    give_halstreamer_the_first_programline_to_start_from();

    return 1;
}

bool create_program::write_files(std::vector<scurve::point> pointvecms,std::vector<scurve::point> eulervecms,std::vector<joint> jointvecms, unsigned int fileid){

    std::cout<<"write_file mr:"<<fileid<<std::endl;

    std::fstream newfile;
    std::string filename="./programstream/stream";

    std::string tot=filename+std::to_string(fileid)+".txt";
    newfile.open(tot,std::ios_base::out); // out = new clean file, app = append to file

    if(newfile.is_open()){

        for(unsigned int i=0; i<pointvecms.size(); i++){

            newfile << toPresision <<
                       jointvecms.at(i).j0*toDegrees   <<" "<<
                       jointvecms.at(i).j1*toDegrees   <<" "<<
                       jointvecms.at(i).j2*toDegrees   <<" "<<
                       jointvecms.at(i).j3*toDegrees   <<" "<<
                       jointvecms.at(i).j4*toDegrees   <<" "<<
                       jointvecms.at(i).j5*toDegrees   <<" "<<
                       pointvecms.at(i).x              <<" "<< // Hal feedbak
                       pointvecms.at(i).y              <<" "<<
                       pointvecms.at(i).z              <<" "<<
                       eulervecms.at(i).x*toDegrees    <<" "<< // Hal feedback
                       eulervecms.at(i).y*toDegrees    <<" "<<
                       eulervecms.at(i).z*toDegrees    <<" "<<
                       std::endl;

        }
    }
    newfile.close();

    return 1;
}

unsigned int create_program::get_ammount_of_files(){
    // Set the ammount of files to play for mainwindow.
    // Ammound is a global used variable.
    unsigned int i=0;
    struct dirent *d;
    DIR *dr;
    dr = opendir("./programstream/");
    if(dr!=NULL)
    {
        //std::cout<<"List of Files & Folders:-\n";
        for(d=readdir(dr); d!=NULL; d=readdir(dr))
        {
            //std::cout<<d->d_name<<std::endl;
            i++;
        }
        closedir(dr);
    }
    else {
        std::cout<<"\nError Occurred!";
    }
    return i;
}

void create_program::give_halstreamer_the_first_programline_to_start_from(){

    // Get the startpoint from a textfile, at firstline.
    std::fstream newfile;
    std::string filename="./programstream/stream";
    std::string str; // Store the first programline in a string.

    std::string tot=filename+std::to_string(1)+".txt"; // Retrieve startpoint from fileid 1.
    newfile.open(tot,std::ios_base::in); // out = new clean file, app = append to file, in = read mode

    unsigned int i=0;
    if(newfile.is_open()){
        while(getline(newfile,str)){
            i++;
            if(i==1){
                // We have the first line now.
                break;
            }
        }
    }
    newfile.close();

    // Create new file.
    newfile.clear(); // To be sure.

    tot=filename+std::to_string(0)+".txt";
    newfile.open(tot,std::ios_base::out); // out = new clean file, app = append to file

    if(newfile.is_open()){
        newfile << str <<std::endl;
    }
    newfile.close();
}


// For offline calculation we use a instance of the machine values.
// And we use a independent inverse kinematic - forward kinematic function
//KDL::Chain temp_KDLChain=KDLChain;
//KDL::JntArray temp_KDLJointInit=KDLJointInit;
//KDL::JntArray temp_KDLJointMin=KDLJointMin;
//KDL::JntArray temp_KDLJointMax=KDLJointMax;
//KDL::JntArray jointresult;

//for(unsigned int i=0; i<bucketvec.size(); i++){

//    if(bucketvec.at(i).primitivetype=="line"){

//        // Path value's:
//        double vo=bucketvec.at(i).vo;
//        double ve=bucketvec.at(i).ve;
//        double velmax=bucketvec.at(i).velmax;
//        double accmax=bucketvec.at(i).accmax;
//        double xs=bucketvec.at(i).pointvec.front().X();
//        double ys=bucketvec.at(i).pointvec.front().Y();
//        double zs=bucketvec.at(i).pointvec.front().Z();
//        double xe=bucketvec.at(i).pointvec.back().X();
//        double ye=bucketvec.at(i).pointvec.back().Y();
//        double ze=bucketvec.at(i).pointvec.back().Z();

//        double eulxs=bucketvec.at(i).eulervec.front().X(); //std::cout<<"eulxs:"<<eulxs*toDegrees<<std::endl;
//        double eulys=bucketvec.at(i).eulervec.front().Y(); //std::cout<<"eulys:"<<eulys*toDegrees<<std::endl;
//        double eulzs=bucketvec.at(i).eulervec.front().Z(); //std::cout<<"eulzs:"<<eulzs*toDegrees<<std::endl;
//        double eulxe=bucketvec.at(i).eulervec.back().X(); //std::cout<<"eulxs:"<<eulxe*toDegrees<<std::endl;
//        double eulye=bucketvec.at(i).eulervec.back().Y(); //std::cout<<"eulys:"<<eulye*toDegrees<<std::endl;
//        double eulze=bucketvec.at(i).eulervec.back().Z(); //std::cout<<"eulzs:"<<eulze*toDegrees<<std::endl;

//        std::vector<scurve::point> pointvecms; //=scurve().scurve_create_point_for_every_ms(velmax, accmax, vo, ve, xs,  ys, zs, xe, ye, ze);
//        std::vector<scurve::point> eulervecms;
//        std::vector<joint> jointvecms;
//        //std::vector<scurve::point> pathvec;
//        //pathvec.push_back({xs,ys,zs});
//        //pathvec.push_back({xe,ye,ze});
//        //std::vector<scurve::point> pointvecms=scurve().create_point_for_every_ms_path(velmax,accmax,vo,ve,pathvec);

//        //for(unsigned int i=0; i<pointvecms.size(); i++){
//        //    std::cout<<"check pointvec at i:"<<i<<" x:"<<pointvecms.at(i).x<<" y:"<<pointvecms.at(i).y<<" z:"<<pointvecms.at(i).z<<std::endl;
//        //}

//        // Check euler maxdegsec
//        // Problem with s-curve when distance=0mm, the value is nan.
//        double l=sqrt(pow(xe-xs,2)+pow(ye-ys,2)+pow(ze-zs,2));
//        std::cout<<"l:"<<l<<std::endl;
//        if(l<0.1){
//            pointvecms.resize(1);
//            pointvecms.back().x=xs;
//            pointvecms.back().y=ys;
//            pointvecms.back().z=zs;


//            // When performing euler angles without cart move, we want the tcp stay's concentrated at tcp point during move.
//            // We add extra waypoints in ms, with a maxdegsec parameter to limit the euler speed.
//            double sec=0;
//            double secx=0, secy=0, secz=0;

//            secx=abs((eulxe-eulxs)*toDegrees)/euler_maxdegsec->Pin;
//            secy=abs((eulye-eulys)*toDegrees)/euler_maxdegsec->Pin;
//            secz=abs((eulze-eulzs)*toDegrees)/euler_maxdegsec->Pin;

//            // Get the max sec value
//            if(sec<secx){
//                sec=secx;
//            }
//            if(sec<secy){
//                sec=secy;
//            }
//            if(sec<secz){
//                sec=secz;
//            }
//            double mssec=sec*1000;
//            std::cout<<"mssec:"<<mssec<<std::endl;

//            // Resize pointvec to the needed ms value.
//            scurve::point p; // Current tcp cart calue.
//            p.x=xs;
//            p.y=ys;
//            p.z=zs;
//            for(unsigned int i=0; i<mssec; i++){
//                pointvecms.push_back(p);
//            }
//        } else {
//            pointvecms=scurve().scurve_create_point_for_every_ms(velmax, accmax, vo, ve, xs, ys, zs, xe, ye, ze);
//        }

//        // Divide the euler angles over the path for every ms:

//        for(unsigned int i=0; i<pointvecms.size(); i++){
//            //std::cout<<"pointvec at i:"<<i<<" x:"<<pointvecms.at(i).x<<" y:"<<pointvecms.at(i).y<<" z:"<<pointvecms.at(i).z<<std::endl;

//            double eulerx=eulxs+ /* start value + */ ((i/double(pointvecms.size())/* ratio */) * (eulxe-eulxs /* total radians */));
//            //std::cout<<toPresision<<"eulerx"<<eulerx*toDegrees<<std::endl;

//            double eulery=eulys+ /* start value + */ ((i/double(pointvecms.size())/* ratio */) * (eulye-eulys /* total radians */));
//            //std::cout<<toPresision<<"eulery"<<eulery*toDegrees<<std::endl;

//            double eulerz=eulzs+ /* start value + */ ((i/double(pointvecms.size())/* ratio */) * (eulze-eulzs /* total radians */));
//            //std::cout<<toPresision<<"eulerz"<<eulerz*toDegrees<<std::endl;

//            eulervecms.push_back({eulerx,eulery,eulerz}); // Parallel bucket to the pointvec
//        }

//        //std::cout<<"pointvecms size:"<<pointvecms.size()<<std::endl;
//        //std::cout<<"eulervecms size:"<<eulervecms.size()<<std::endl;

//        // When points and eulers are available for every ms. Calculate the joints for every ms with the ik (inverse kinematics) function.
//        for(unsigned int i=0; i<pointvecms.size(); i++){

//            // Has to be zero:
//            kinematic().Fk_zero(); // Initialize cartzero, machine initial position with neutral euler angels.
//            //double x=0,y=0,z=0;
//            //cartzero.M.GetEulerZYX(z,y,x);
//            //std::cout<<"cartzero init eulx:"<<x*toDegrees<<" euly:"<<y*toDegrees<<" eulz:"<<z*toDegrees<<std::endl;

//            // Set xyz values:
//            cartzero.p.x(pointvecms.at(i).x);
//            cartzero.p.y(pointvecms.at(i).y);
//            cartzero.p.z(pointvecms.at(i).z);

//            // Set euler angles:
//            cartzero.M.DoRotZ(eulervecms.at(i).z);
//            cartzero.M.DoRotY(eulervecms.at(i).y);
//            cartzero.M.DoRotX(eulervecms.at(i).x);

//            //cartzero.M.GetEulerZYX(z,y,x);
//            //std::cout<<"cartzero init eulx:"<<x*toDegrees<<" euly:"<<y*toDegrees<<" eulz:"<<z*toDegrees<<std::endl;

//            // Replace this by a virtual function.   int temp_Ik(bool ikfrominit,KDL::Chain chain, KDL::JntArray jointinit, KDL::JntArray jointmin, KDL::JntArray jointmax, KDL::Frame cart, KDL::JntArray &jointcur);
//            int ok=kinematic().temp_Ik(1,temp_KDLChain,temp_KDLJointInit,temp_KDLJointMin,temp_KDLJointMax,cartzero,jointresult);

//            joint j;
//            if(ok){
//                j.j0=jointresult(0);
//                j.j1=jointresult(1);
//                j.j2=jointresult(2);
//                j.j3=jointresult(3);
//                j.j4=jointresult(4);
//                j.j5=jointresult(5);
//                jointvecms.push_back(j);
//            }
//            if(!ok){
//                // Try it with ikfrominit=0;

////                    int next=kinematic().temp_Ik(0,temp_KDLChain,temp_KDLJointInit,temp_KDLJointMin,temp_KDLJointMax,cartzero,jointresult);
////                    if(next){
////                        j.j0=jointresult(0);
////                        j.j1=jointresult(1);
////                        j.j2=jointresult(2);
////                        j.j3=jointresult(3);
////                        j.j4=jointresult(4);
////                        j.j5=jointresult(5);
////                        jointvecms.push_back(j);
////                    }
////                    if(!next){
//                    jointvecms.push_back(j); // To stay in line we need a value
//                    std::cout<<"ik error from libmotion at i:"<<i<<" x:"<<pointvecms.at(i).x<<" y:"<<pointvecms.at(i).y<<" z:"<<pointvecms.at(i).z<<std::endl;
//                //}
//            }
//        }

//        // At this stage we have for every ms :
//        // 1. xyz cartesian point
//        // 2. zyx euler angle
//        // 3. j0-j5 joint positions
//        //
//        // Now write the text files, textstream, 1000 lines a file.

//        bool ok=write_files(pointvecms,eulervecms,jointvecms,i+1); // programid 0 is move the machine to the program startpoint.
//        if(ok){
//            std::cout<<"writing file completed"<<std::endl;
//        }

//        // The files are stored now. In mainwindow we can set the files to play off.
//    }
//}


//// Set the ammount of files to play for mainwindow.
//// Ammound is a global used variable.
//programammount=get_ammount_of_files();

//// At last. make the startpoint file.
//give_halstreamer_the_first_programline_to_start_from();

//return 1;


