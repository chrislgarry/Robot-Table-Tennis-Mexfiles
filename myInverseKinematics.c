/*>>>>>>>>>>>>>>>Includes needed for MEX file>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#include "mex.h"
#include "matrix.h"
/*>>>>>>>>>>>>>>>Includes needed for inverse_kinematics.c>>>>>>>>>>>>>>>>>>*/
#include "inverse_kinematics.h"
#include "ias_common.h"
#include "ias_utilities.h"
#include "limits.h"
#include "SL_system_headers.h"
#include "SL_common.h"
#include "SL_kinematics.h"
#include "SL_user.h"
#include "SL_task_servo.h"
/*>>>>>>>>>>>>>>>>>>>>>>Helper defs>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define c_target_num    0
#define orient_num      1
#define q_des_state_num 2
#define rest_num        3
#define Jinv_w_num      4
#define gains_num       5
/*>>>>>>>>>>>>>>>>>>>>>>Start function>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

int servo_enabled;
double servo_time;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    
    /*Check for proper number of arguments---------------------------------*/
    
    if(nrhs!=8)
        mexErrMsgIdAndTxt( "MATLAB:myInverseKinematics:invalidNumInputs", "Eight inputs required.");
    
    else if(nlhs>1)
        mexErrMsgIdAndTxt( "MATLAB:myInverseKinematics:maxlhs", "Too many output arguments.");
    
    /*Instantiate C structures/inputs to myInverseKinematics() subroutine--*/
    
    SL_Cstate c_target;
	SL_quat orient;
	SL_DJstate *q_des_state;
	SL_DJstate *rest;
	Vector Jinv_w;
	Vector gains;
	double errorThresh;
	int maxIter;
    
    /*Transfer c_target from MATLAB struct type to C struct type-----------*/
    
    if(mxIsStruct(prhs[c_target_num])){
        mwSize i;
        mwSize c_target_numelements = mxGetNumberOfElements(prhs[c_target_num]);
        for(i = 0; i<c_target_numelements; i++){
            c_target.x[i+1]   = *mxGetPr(mxGetField(prhs[c_target_num], i, "x"));
            c_target.xd[i+1]  = *mxGetPr(mxGetField(prhs[c_target_num], i, "xd"));
            c_target.xdd[i+1] = *mxGetPr(mxGetField(prhs[c_target_num], i,"xdd"));
        }
    }
    else{
        mexPrintf("MATLAB:myInverseKinematics:xNotStruct","Error! Argument 1 must be a Struct Array containing n Structs, each with three fields: x, xd, xdd.");
    }
    
    /*Transfer orient from MATLAB struct type to C struct type-------------*/
    
    if(mxIsStruct(prhs[orient_num])){
        mwSize x;
        mwSize orient_numelements = mxGetNumberOfElements(prhs[orient_num]);
        for(x = 0; x<orient_numelements; x++){
            orient.q[x+1]   = *mxGetPr(mxGetField(prhs[orient_num], x, "q"));
            orient.qd[x+1]  = *mxGetPr(mxGetField(prhs[orient_num], x, "qd"));
            orient.qdd[x+1] = *mxGetPr(mxGetField(prhs[orient_num], x, "qdd"));
            orient.ad[x+1]  = *mxGetPr(mxGetField(prhs[orient_num], x, "ad"));
            orient.add[x+1] = *mxGetPr(mxGetField(prhs[orient_num], x, "add"));
        }
    }
    else{
        mexPrintf("MATLAB:myInverseKinematics:xNotStruct","Error! Argument 2 must be a Struct Array containing n Structs, each with five fields: q, qd, qdd, ad, add.");
    }
    
    /*Transfer q_des_state from MATLAB struct type to C struct type--------*/
    
    if(mxIsStruct(prhs[q_des_state_num])){
        (*q_des_state).th   = *mxGetPr(mxGetField(prhs[q_des_state_num], 0, "th"));
        (*q_des_state).thd  = *mxGetPr(mxGetField(prhs[q_des_state_num], 0, "thd"));
        (*q_des_state).thdd = *mxGetPr(mxGetField(prhs[q_des_state_num], 0, "thdd"));
        (*q_des_state).uff  = *mxGetPr(mxGetField(prhs[q_des_state_num], 0, "uff"));
        (*q_des_state).uex  = *mxGetPr(mxGetField(prhs[q_des_state_num], 0, "uex"));
    }
    else{
        mexPrintf("MATLAB:myInverseKinematics:xNotStruct","Error! Argument 3 must be a Struct Array containing n Structs, each with five fields: th, thd, thdd, uff, uex.");
    }
    
    /*Transfer rest from MATLAB struct type to C struct type---------------*/
    
    if(mxIsStruct(prhs[rest_num])){
            (*rest).th   = *mxGetPr(mxGetField(prhs[rest_num], 0, "th"));
            (*rest).thd  = *mxGetPr(mxGetField(prhs[rest_num], 0, "thd"));
            (*rest).thdd = *mxGetPr(mxGetField(prhs[rest_num], 0, "thdd"));
            (*rest).uff  = *mxGetPr(mxGetField(prhs[rest_num], 0, "uff"));
            (*rest).uex  = *mxGetPr(mxGetField(prhs[rest_num], 0, "uex"));
        }
    else{
        mexPrintf("MATLAB:myInverseKinematics:xNotStruct","Error! Argument 4 must be a Struct Array containing n Structs, each with five fields: th, thd, thdd, uff, uex.");
    }
    
    /*Transfer Jinv_w vector from MATLAB to C------------------------------*/
    
    if(mxIsDouble(prhs[Jinv_w_num])) 
        Jinv_w = mxGetPr(prhs[Jinv_w_num]);
    else
        mexErrMsgIdAndTxt("MATLAB:myInverseKinematics:xNotDouble","Error! Argument 5 must be an mxArray of type double.");
    
    /*Transfer Jinv_w vector from MATLAB to C------------------------------*/
   
    if(mxIsDouble(prhs[gains_num]))
        gains =  mxGetPr(prhs[gains_num]);
    else
        mexErrMsgIdAndTxt("MATLAB:myInverseKinematics:xNotDouble","Error! Argument 6 must be an mxArray of type double.");
    
    /*Transfer errorTresh double from MATLAB to C--------------------------*/
    
    if(mxIsDouble(prhs[6]))
        errorThresh = mxGetScalar(prhs[6]);
    else
        mexErrMsgIdAndTxt("MATLAB:myInverseKinematics:xNotDouble","Error! Argument 7 must be of type double.");
    
    /*Transfer maxIter int from MATLAB to C--------------------------------*/
    
    if(mxIsNumeric(prhs[7]))
        maxIter = (int)mxGetScalar(prhs[7]); //if causing issues, use floor()
    else
        mexErrMsgIdAndTxt("MATLAB:myInverseKinematics:xNotInt","Error! Argument 8 must be of type int.");
    
    /*Call myInverseKinematics() with C-type as args----------------------*/
    
    int return_val = myInverseKinematics(c_target, orient, q_des_state, rest, Jinv_w, gains, errorThresh, maxIter);
    
    if(return_val == FALSE)
        mexErrMsgIdAndTxt("MATLAB:myInverseKinematics:myInverseKinematics()","Error! The C subroutine myInverseKinematics() returned FALSE.");
    
    /*WARNING: Inputs to mexFunction() are READ-ONLY. q_des_state cannot be written
    to as in the C implementation, so a new structure must be created and returned.*/
    
    /*Create mxArray MATLAB structure to store return---*/
    const char    *fieldnames[] = {"th","thd","thdd","uff","uex"};
    mwSize        ndim          = mxGetNumberOfDimensions(prhs[q_des_state_num]);
    const mwSize  *dims         = mxGetDimensions(prhs[q_des_state_num]);
    int           nfields       = mxGetNumberOfFields(prhs[q_des_state_num]);
    
    plhs[0] = mxCreateStructArray(ndim, dims, nfields, fieldnames); /*Initialize Struct*/
    
    /*Transfer fields from C structure to MATLAB output structure*/
    mxSetFieldByNumber(plhs[0], 0, 0, mxCreateDoubleScalar((*q_des_state).th));
    mxSetFieldByNumber(plhs[0], 0, 1, mxCreateDoubleScalar((*q_des_state).thd));
    mxSetFieldByNumber(plhs[0], 0, 2, mxCreateDoubleScalar((*q_des_state).thdd));
    mxSetFieldByNumber(plhs[0], 0, 3, mxCreateDoubleScalar((*q_des_state).uff));
    mxSetFieldByNumber(plhs[0], 0, 4, mxCreateDoubleScalar((*q_des_state).uex));
    
    return;
}
