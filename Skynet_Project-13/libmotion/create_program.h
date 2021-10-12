#ifndef CREATE_PROGRAM_H
#define CREATE_PROGRAM_H

#include <vector>
#include <dirent.h> // Used to find directory info

#ifdef Success
#undef Success
#endif

// libscurve
#include <scurve.h>

// libocc
#include <opencascade.h>
using namespace occ;

// libmotion
#include <create_stream.h>

// libkinematic
#include <kinematic.h>

class create_program
{
public:
    create_program();
    bool point_every_ms(std::vector<Opencascade::bucket> bucketvec);


private:
    struct joint {
        double j0=0,j1=0,j2=0,j3=0,j4=0,j5=0;
        // add external axis or more joints.
    };

    bool write_files(std::vector<scurve::point> pointvecms, std::vector<scurve::point> eulervecms, std::vector<joint> jointvecms, unsigned int fileid);
    unsigned int get_ammount_of_files();
    int filenr=0;
    void give_halstreamer_the_first_programline_to_start_from();
};

#endif // CREATE_PROGRAM_H
