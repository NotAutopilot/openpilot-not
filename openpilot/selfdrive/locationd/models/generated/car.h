#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_1259473718912116);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8150316124841401096);
void car_H_mod_fun(double *state, double *out_4805317391369158591);
void car_f_fun(double *state, double dt, double *out_6733672621451885418);
void car_F_fun(double *state, double dt, double *out_6738830797736072745);
void car_h_25(double *state, double *unused, double *out_8567290863275389123);
void car_H_25(double *state, double *unused, double *out_4387870834543078646);
void car_h_24(double *state, double *unused, double *out_5883556310200608214);
void car_H_24(double *state, double *unused, double *out_6560520433548578212);
void car_h_30(double *state, double *unused, double *out_3108617984083811557);
void car_H_30(double *state, double *unused, double *out_4517209781686318716);
void car_h_26(double *state, double *unused, double *out_1893055283577034088);
void car_H_26(double *state, double *unused, double *out_8129374153417134870);
void car_h_27(double *state, double *unused, double *out_3565617595614543478);
void car_H_27(double *state, double *unused, double *out_6691973093486743627);
void car_h_29(double *state, double *unused, double *out_71831932639478872);
void car_H_29(double *state, double *unused, double *out_8405335820356294660);
void car_h_28(double *state, double *unused, double *out_4509127448516205913);
void car_H_28(double *state, double *unused, double *out_6441705548790968409);
void car_h_31(double *state, double *unused, double *out_674351368390865764);
void car_H_31(double *state, double *unused, double *out_4357224872666118218);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}