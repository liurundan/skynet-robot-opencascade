#ifndef HALIO_H
#define HALIO_H

#ifndef ULAPI
#define ULAPI
#endif

//! Various c++ includes:
#include <iostream>
#include <list>
#include <vector>

//! Hal:
#include "hal.h"

typedef struct {
    hal_float_t *Pin;
} float_data_t;

typedef struct {
    hal_bit_t *Pin;
} bit_data_t;

typedef struct {
    hal_float_t Pin;
} param_data_t;

typedef struct {
    hal_s32_t *Pin;
} s32_data_t;

typedef struct {
    hal_u32_t *Pin;
} u32_data_t;

extern int comp_id;
extern s32_data_t *streamermeat;
extern float_data_t *J0_Fb;
extern float_data_t *J1_Fb;
extern float_data_t *J2_Fb;
extern float_data_t *J3_Fb;
extern float_data_t *J4_Fb;
extern float_data_t *J5_Fb;

extern float_data_t *CartX_Fb;
extern float_data_t *CartY_Fb;
extern float_data_t *CartZ_Fb;
extern float_data_t *EulerX_Fb;
extern float_data_t *EulerY_Fb;
extern float_data_t *EulerZ_Fb;

extern param_data_t *cart_stepsize;
extern param_data_t *euler_stepsize;
extern param_data_t *euler_maxdegsec;
extern param_data_t *joint_stepsize;
extern param_data_t *joint_maxdegsec;
extern param_data_t *tooldir_stepsize;

extern param_data_t *accmax;
extern param_data_t *velmax;

extern bit_data_t *tool0, *tool1, *tool2;

class halio
{
public:
    int Init();
};

#endif // HALIO_H
