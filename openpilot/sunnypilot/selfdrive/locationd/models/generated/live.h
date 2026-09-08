#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_5119314998410330295);
void live_err_fun(double *nom_x, double *delta_x, double *out_6669040642667914936);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4713063601834543483);
void live_H_mod_fun(double *state, double *out_2079584219834875270);
void live_f_fun(double *state, double dt, double *out_8790713612541053491);
void live_F_fun(double *state, double dt, double *out_7252863203599740756);
void live_h_4(double *state, double *unused, double *out_33286869564576122);
void live_H_4(double *state, double *unused, double *out_3571805064483178836);
void live_h_9(double *state, double *unused, double *out_24257071048631331);
void live_H_9(double *state, double *unused, double *out_3715413870781268634);
void live_h_10(double *state, double *unused, double *out_3930932307834878529);
void live_H_10(double *state, double *unused, double *out_3575111464275831190);
void live_h_12(double *state, double *unused, double *out_9114539772848164112);
void live_H_12(double *state, double *unused, double *out_1447651343548782959);
void live_h_35(double *state, double *unused, double *out_4485496312648423295);
void live_H_35(double *state, double *unused, double *out_4193214375873796668);
void live_h_32(double *state, double *unused, double *out_4614861069974657877);
void live_H_32(double *state, double *unused, double *out_4423842244782480530);
void live_h_13(double *state, double *unused, double *out_1190580958394188587);
void live_H_13(double *state, double *unused, double *out_3235426414362369946);
void live_h_14(double *state, double *unused, double *out_24257071048631331);
void live_H_14(double *state, double *unused, double *out_3715413870781268634);
void live_h_33(double *state, double *unused, double *out_4932824222307106731);
void live_H_33(double *state, double *unused, double *out_2945413997528286144);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}