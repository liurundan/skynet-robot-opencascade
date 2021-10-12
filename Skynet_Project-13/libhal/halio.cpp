#include "halio.h"
#include <vector>
#include <kinematic.h>

//! Make conversion's easy:
#define toRadians M_PI/180.0
#define toDegrees (180.0/M_PI)

int comp_id=0;
float_data_t *J0_Fb;
float_data_t *J1_Fb;
float_data_t *J2_Fb;
float_data_t *J3_Fb;
float_data_t *J4_Fb;
float_data_t *J5_Fb;

float_data_t *CartX_Fb;
float_data_t *CartY_Fb;
float_data_t *CartZ_Fb;
float_data_t *EulerX_Fb;
float_data_t *EulerY_Fb;
float_data_t *EulerZ_Fb;

param_data_t *cart_stepsize;
param_data_t *euler_stepsize;
param_data_t *euler_maxdegsec;
param_data_t *joint_stepsize;
param_data_t *joint_maxdegsec;
param_data_t *tooldir_stepsize;
param_data_t *accmax;
param_data_t *velmax;

s32_data_t *streamermeat;

bit_data_t *tool0, *tool1, *tool2;

int halio::Init(){

    comp_id = hal_init("core");

    // Parameter pins
    velmax = (param_data_t*)hal_malloc(sizeof(param_data_t));
    hal_param_float_new("velmax",HAL_RW,&(velmax->Pin),comp_id);

    accmax = (param_data_t*)hal_malloc(sizeof(param_data_t));
    hal_param_float_new("accmax",HAL_RW,&(accmax->Pin),comp_id);

    cart_stepsize = (param_data_t*)hal_malloc(sizeof(param_data_t));
    hal_param_float_new("cart_stepsize",HAL_RW,&(cart_stepsize->Pin),comp_id);

    euler_stepsize = (param_data_t*)hal_malloc(sizeof(param_data_t));
    hal_param_float_new("euler_stepsize",HAL_RW,&(euler_stepsize->Pin),comp_id);

    euler_maxdegsec = (param_data_t*)hal_malloc(sizeof(param_data_t));
    hal_param_float_new("euler_maxdegsec",HAL_RW,&(euler_maxdegsec->Pin),comp_id);

    joint_stepsize = (param_data_t*)hal_malloc(sizeof(param_data_t));
    hal_param_float_new("joint_stepsize",HAL_RW,&(joint_stepsize->Pin),comp_id);

    joint_maxdegsec = (param_data_t*)hal_malloc(sizeof(param_data_t));
    hal_param_float_new("joint_maxdegsec",HAL_RW,&(joint_maxdegsec->Pin),comp_id);

    tooldir_stepsize = (param_data_t*)hal_malloc(sizeof(param_data_t));
    hal_param_float_new("tooldir_stepsize",HAL_RW,&(tooldir_stepsize->Pin),comp_id);

    // S32 pins
    streamermeat = (s32_data_t*)hal_malloc(sizeof(s32_data_t));
    hal_pin_s32_new("streamermeat",HAL_IN,&(streamermeat->Pin),comp_id);

    // Float pins
    J0_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("J0_Fb",HAL_IN,&(J0_Fb->Pin),comp_id);

    J1_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("J1_Fb",HAL_IN,&(J1_Fb->Pin),comp_id);

    J2_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("J2_Fb",HAL_IN,&(J2_Fb->Pin),comp_id);

    J3_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("J3_Fb",HAL_IN,&(J3_Fb->Pin),comp_id);

    J4_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("J4_Fb",HAL_IN,&(J4_Fb->Pin),comp_id);

    J5_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("J5_Fb",HAL_IN,&(J5_Fb->Pin),comp_id);

    CartX_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("CartX_Fb",HAL_IN,&(CartX_Fb->Pin),comp_id);

    CartY_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("CartY_Fb",HAL_IN,&(CartY_Fb->Pin),comp_id);

    CartZ_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("CartZ_Fb",HAL_IN,&(CartZ_Fb->Pin),comp_id);

    EulerX_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("EulerX_Fb",HAL_IN,&(EulerX_Fb->Pin),comp_id);

    EulerY_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("EulerY_Fb",HAL_IN,&(EulerY_Fb->Pin),comp_id);

    EulerZ_Fb = (float_data_t*)hal_malloc(sizeof(float_data_t));
    hal_pin_float_new("EulerZ_Fb",HAL_IN,&(EulerZ_Fb->Pin),comp_id);

    // Bit pins
    tool0 = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tool0",HAL_OUT,&(tool0->Pin),comp_id);

    tool1 = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tool1",HAL_OUT,&(tool1->Pin),comp_id);

    tool2 = (bit_data_t*)hal_malloc(sizeof(bit_data_t));
    hal_pin_bit_new("tool2",HAL_OUT,&(tool2->Pin),comp_id);

    int error = hal_ready(comp_id);
    if(error==0){
        std::cout << "Hal component ok" << std::endl;

        // Load hal file
        system("/opt/linuxcnc/bin/./halcmd \-f ./config/ethercat.hal");
        system("/opt/linuxcnc/bin/./halcmd start");

        return 1; //ok go on
    } else {
        std::cout << "Hal component error, performing [halrun -U] now." << std::endl;
        system("/opt/linuxcnc/scripts/./halrun -U");
        std::cout << "Restart application required." << std::endl;
        return 0; //not good, show error output in terminal.
    }

    // Normally we don't get here.
    return 1;
}





































