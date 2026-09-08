#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_1101899825002175654);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_9121434067879914978);
void pose_H_mod_fun(double *state, double *out_5107013155435799420);
void pose_f_fun(double *state, double dt, double *out_9090648939993874530);
void pose_F_fun(double *state, double dt, double *out_294061415488430612);
void pose_h_4(double *state, double *unused, double *out_2525208654296729708);
void pose_H_4(double *state, double *unused, double *out_7563366274452672869);
void pose_h_10(double *state, double *unused, double *out_2333383739813503746);
void pose_H_10(double *state, double *unused, double *out_8502106662555230246);
void pose_h_13(double *state, double *unused, double *out_3720576547851047664);
void pose_H_13(double *state, double *unused, double *out_7671103973924545946);
void pose_h_14(double *state, double *unused, double *out_931071426167433179);
void pose_H_14(double *state, double *unused, double *out_6920136942917394218);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}