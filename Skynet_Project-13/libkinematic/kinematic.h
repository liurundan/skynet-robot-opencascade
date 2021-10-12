#ifndef KINEMATIC_H
#define KINEMATIC_H

#ifndef ULAPI
#define ULAPI
#endif

#define SUCCESS 1
#undef Success //https://eigen.tuxfamily.org/bz/show_bug.cgi?id=253

#include <chainiksolverpos_lma.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainiksolvervel_pinv.hpp>
#include <chainiksolverpos_nr_jl.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <halio.h>
#include <variable.h>

//! Kdl data storage:
extern KDL::Chain KDLChain;
extern KDL::Frame cart,cartzero;
extern KDL::JntArray KDLJointInit;
extern KDL::JntArray KDLJointCur;
extern KDL::JntArray KDLJointMin;
extern KDL::JntArray KDLJointMax;

class kinematic
{
public:
    int Init();
    int Fk();
    int Fk_zero();
    int Ik();
    int temp_Ik(bool ikfrominit,KDL::Chain chain, KDL::JntArray jointinit, KDL::JntArray jointmin, KDL::JntArray jointmax, KDL::Frame cart, KDL::JntArray &jointcur);
    int Fk_tooldir(double x_in, double y_in, double z_in, double &x_out, double &y_out, double &z_out);

private:



};

#endif // KINEMATIC_H
