#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1101899825002175654) {
   out_1101899825002175654[0] = delta_x[0] + nom_x[0];
   out_1101899825002175654[1] = delta_x[1] + nom_x[1];
   out_1101899825002175654[2] = delta_x[2] + nom_x[2];
   out_1101899825002175654[3] = delta_x[3] + nom_x[3];
   out_1101899825002175654[4] = delta_x[4] + nom_x[4];
   out_1101899825002175654[5] = delta_x[5] + nom_x[5];
   out_1101899825002175654[6] = delta_x[6] + nom_x[6];
   out_1101899825002175654[7] = delta_x[7] + nom_x[7];
   out_1101899825002175654[8] = delta_x[8] + nom_x[8];
   out_1101899825002175654[9] = delta_x[9] + nom_x[9];
   out_1101899825002175654[10] = delta_x[10] + nom_x[10];
   out_1101899825002175654[11] = delta_x[11] + nom_x[11];
   out_1101899825002175654[12] = delta_x[12] + nom_x[12];
   out_1101899825002175654[13] = delta_x[13] + nom_x[13];
   out_1101899825002175654[14] = delta_x[14] + nom_x[14];
   out_1101899825002175654[15] = delta_x[15] + nom_x[15];
   out_1101899825002175654[16] = delta_x[16] + nom_x[16];
   out_1101899825002175654[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9121434067879914978) {
   out_9121434067879914978[0] = -nom_x[0] + true_x[0];
   out_9121434067879914978[1] = -nom_x[1] + true_x[1];
   out_9121434067879914978[2] = -nom_x[2] + true_x[2];
   out_9121434067879914978[3] = -nom_x[3] + true_x[3];
   out_9121434067879914978[4] = -nom_x[4] + true_x[4];
   out_9121434067879914978[5] = -nom_x[5] + true_x[5];
   out_9121434067879914978[6] = -nom_x[6] + true_x[6];
   out_9121434067879914978[7] = -nom_x[7] + true_x[7];
   out_9121434067879914978[8] = -nom_x[8] + true_x[8];
   out_9121434067879914978[9] = -nom_x[9] + true_x[9];
   out_9121434067879914978[10] = -nom_x[10] + true_x[10];
   out_9121434067879914978[11] = -nom_x[11] + true_x[11];
   out_9121434067879914978[12] = -nom_x[12] + true_x[12];
   out_9121434067879914978[13] = -nom_x[13] + true_x[13];
   out_9121434067879914978[14] = -nom_x[14] + true_x[14];
   out_9121434067879914978[15] = -nom_x[15] + true_x[15];
   out_9121434067879914978[16] = -nom_x[16] + true_x[16];
   out_9121434067879914978[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_5107013155435799420) {
   out_5107013155435799420[0] = 1.0;
   out_5107013155435799420[1] = 0.0;
   out_5107013155435799420[2] = 0.0;
   out_5107013155435799420[3] = 0.0;
   out_5107013155435799420[4] = 0.0;
   out_5107013155435799420[5] = 0.0;
   out_5107013155435799420[6] = 0.0;
   out_5107013155435799420[7] = 0.0;
   out_5107013155435799420[8] = 0.0;
   out_5107013155435799420[9] = 0.0;
   out_5107013155435799420[10] = 0.0;
   out_5107013155435799420[11] = 0.0;
   out_5107013155435799420[12] = 0.0;
   out_5107013155435799420[13] = 0.0;
   out_5107013155435799420[14] = 0.0;
   out_5107013155435799420[15] = 0.0;
   out_5107013155435799420[16] = 0.0;
   out_5107013155435799420[17] = 0.0;
   out_5107013155435799420[18] = 0.0;
   out_5107013155435799420[19] = 1.0;
   out_5107013155435799420[20] = 0.0;
   out_5107013155435799420[21] = 0.0;
   out_5107013155435799420[22] = 0.0;
   out_5107013155435799420[23] = 0.0;
   out_5107013155435799420[24] = 0.0;
   out_5107013155435799420[25] = 0.0;
   out_5107013155435799420[26] = 0.0;
   out_5107013155435799420[27] = 0.0;
   out_5107013155435799420[28] = 0.0;
   out_5107013155435799420[29] = 0.0;
   out_5107013155435799420[30] = 0.0;
   out_5107013155435799420[31] = 0.0;
   out_5107013155435799420[32] = 0.0;
   out_5107013155435799420[33] = 0.0;
   out_5107013155435799420[34] = 0.0;
   out_5107013155435799420[35] = 0.0;
   out_5107013155435799420[36] = 0.0;
   out_5107013155435799420[37] = 0.0;
   out_5107013155435799420[38] = 1.0;
   out_5107013155435799420[39] = 0.0;
   out_5107013155435799420[40] = 0.0;
   out_5107013155435799420[41] = 0.0;
   out_5107013155435799420[42] = 0.0;
   out_5107013155435799420[43] = 0.0;
   out_5107013155435799420[44] = 0.0;
   out_5107013155435799420[45] = 0.0;
   out_5107013155435799420[46] = 0.0;
   out_5107013155435799420[47] = 0.0;
   out_5107013155435799420[48] = 0.0;
   out_5107013155435799420[49] = 0.0;
   out_5107013155435799420[50] = 0.0;
   out_5107013155435799420[51] = 0.0;
   out_5107013155435799420[52] = 0.0;
   out_5107013155435799420[53] = 0.0;
   out_5107013155435799420[54] = 0.0;
   out_5107013155435799420[55] = 0.0;
   out_5107013155435799420[56] = 0.0;
   out_5107013155435799420[57] = 1.0;
   out_5107013155435799420[58] = 0.0;
   out_5107013155435799420[59] = 0.0;
   out_5107013155435799420[60] = 0.0;
   out_5107013155435799420[61] = 0.0;
   out_5107013155435799420[62] = 0.0;
   out_5107013155435799420[63] = 0.0;
   out_5107013155435799420[64] = 0.0;
   out_5107013155435799420[65] = 0.0;
   out_5107013155435799420[66] = 0.0;
   out_5107013155435799420[67] = 0.0;
   out_5107013155435799420[68] = 0.0;
   out_5107013155435799420[69] = 0.0;
   out_5107013155435799420[70] = 0.0;
   out_5107013155435799420[71] = 0.0;
   out_5107013155435799420[72] = 0.0;
   out_5107013155435799420[73] = 0.0;
   out_5107013155435799420[74] = 0.0;
   out_5107013155435799420[75] = 0.0;
   out_5107013155435799420[76] = 1.0;
   out_5107013155435799420[77] = 0.0;
   out_5107013155435799420[78] = 0.0;
   out_5107013155435799420[79] = 0.0;
   out_5107013155435799420[80] = 0.0;
   out_5107013155435799420[81] = 0.0;
   out_5107013155435799420[82] = 0.0;
   out_5107013155435799420[83] = 0.0;
   out_5107013155435799420[84] = 0.0;
   out_5107013155435799420[85] = 0.0;
   out_5107013155435799420[86] = 0.0;
   out_5107013155435799420[87] = 0.0;
   out_5107013155435799420[88] = 0.0;
   out_5107013155435799420[89] = 0.0;
   out_5107013155435799420[90] = 0.0;
   out_5107013155435799420[91] = 0.0;
   out_5107013155435799420[92] = 0.0;
   out_5107013155435799420[93] = 0.0;
   out_5107013155435799420[94] = 0.0;
   out_5107013155435799420[95] = 1.0;
   out_5107013155435799420[96] = 0.0;
   out_5107013155435799420[97] = 0.0;
   out_5107013155435799420[98] = 0.0;
   out_5107013155435799420[99] = 0.0;
   out_5107013155435799420[100] = 0.0;
   out_5107013155435799420[101] = 0.0;
   out_5107013155435799420[102] = 0.0;
   out_5107013155435799420[103] = 0.0;
   out_5107013155435799420[104] = 0.0;
   out_5107013155435799420[105] = 0.0;
   out_5107013155435799420[106] = 0.0;
   out_5107013155435799420[107] = 0.0;
   out_5107013155435799420[108] = 0.0;
   out_5107013155435799420[109] = 0.0;
   out_5107013155435799420[110] = 0.0;
   out_5107013155435799420[111] = 0.0;
   out_5107013155435799420[112] = 0.0;
   out_5107013155435799420[113] = 0.0;
   out_5107013155435799420[114] = 1.0;
   out_5107013155435799420[115] = 0.0;
   out_5107013155435799420[116] = 0.0;
   out_5107013155435799420[117] = 0.0;
   out_5107013155435799420[118] = 0.0;
   out_5107013155435799420[119] = 0.0;
   out_5107013155435799420[120] = 0.0;
   out_5107013155435799420[121] = 0.0;
   out_5107013155435799420[122] = 0.0;
   out_5107013155435799420[123] = 0.0;
   out_5107013155435799420[124] = 0.0;
   out_5107013155435799420[125] = 0.0;
   out_5107013155435799420[126] = 0.0;
   out_5107013155435799420[127] = 0.0;
   out_5107013155435799420[128] = 0.0;
   out_5107013155435799420[129] = 0.0;
   out_5107013155435799420[130] = 0.0;
   out_5107013155435799420[131] = 0.0;
   out_5107013155435799420[132] = 0.0;
   out_5107013155435799420[133] = 1.0;
   out_5107013155435799420[134] = 0.0;
   out_5107013155435799420[135] = 0.0;
   out_5107013155435799420[136] = 0.0;
   out_5107013155435799420[137] = 0.0;
   out_5107013155435799420[138] = 0.0;
   out_5107013155435799420[139] = 0.0;
   out_5107013155435799420[140] = 0.0;
   out_5107013155435799420[141] = 0.0;
   out_5107013155435799420[142] = 0.0;
   out_5107013155435799420[143] = 0.0;
   out_5107013155435799420[144] = 0.0;
   out_5107013155435799420[145] = 0.0;
   out_5107013155435799420[146] = 0.0;
   out_5107013155435799420[147] = 0.0;
   out_5107013155435799420[148] = 0.0;
   out_5107013155435799420[149] = 0.0;
   out_5107013155435799420[150] = 0.0;
   out_5107013155435799420[151] = 0.0;
   out_5107013155435799420[152] = 1.0;
   out_5107013155435799420[153] = 0.0;
   out_5107013155435799420[154] = 0.0;
   out_5107013155435799420[155] = 0.0;
   out_5107013155435799420[156] = 0.0;
   out_5107013155435799420[157] = 0.0;
   out_5107013155435799420[158] = 0.0;
   out_5107013155435799420[159] = 0.0;
   out_5107013155435799420[160] = 0.0;
   out_5107013155435799420[161] = 0.0;
   out_5107013155435799420[162] = 0.0;
   out_5107013155435799420[163] = 0.0;
   out_5107013155435799420[164] = 0.0;
   out_5107013155435799420[165] = 0.0;
   out_5107013155435799420[166] = 0.0;
   out_5107013155435799420[167] = 0.0;
   out_5107013155435799420[168] = 0.0;
   out_5107013155435799420[169] = 0.0;
   out_5107013155435799420[170] = 0.0;
   out_5107013155435799420[171] = 1.0;
   out_5107013155435799420[172] = 0.0;
   out_5107013155435799420[173] = 0.0;
   out_5107013155435799420[174] = 0.0;
   out_5107013155435799420[175] = 0.0;
   out_5107013155435799420[176] = 0.0;
   out_5107013155435799420[177] = 0.0;
   out_5107013155435799420[178] = 0.0;
   out_5107013155435799420[179] = 0.0;
   out_5107013155435799420[180] = 0.0;
   out_5107013155435799420[181] = 0.0;
   out_5107013155435799420[182] = 0.0;
   out_5107013155435799420[183] = 0.0;
   out_5107013155435799420[184] = 0.0;
   out_5107013155435799420[185] = 0.0;
   out_5107013155435799420[186] = 0.0;
   out_5107013155435799420[187] = 0.0;
   out_5107013155435799420[188] = 0.0;
   out_5107013155435799420[189] = 0.0;
   out_5107013155435799420[190] = 1.0;
   out_5107013155435799420[191] = 0.0;
   out_5107013155435799420[192] = 0.0;
   out_5107013155435799420[193] = 0.0;
   out_5107013155435799420[194] = 0.0;
   out_5107013155435799420[195] = 0.0;
   out_5107013155435799420[196] = 0.0;
   out_5107013155435799420[197] = 0.0;
   out_5107013155435799420[198] = 0.0;
   out_5107013155435799420[199] = 0.0;
   out_5107013155435799420[200] = 0.0;
   out_5107013155435799420[201] = 0.0;
   out_5107013155435799420[202] = 0.0;
   out_5107013155435799420[203] = 0.0;
   out_5107013155435799420[204] = 0.0;
   out_5107013155435799420[205] = 0.0;
   out_5107013155435799420[206] = 0.0;
   out_5107013155435799420[207] = 0.0;
   out_5107013155435799420[208] = 0.0;
   out_5107013155435799420[209] = 1.0;
   out_5107013155435799420[210] = 0.0;
   out_5107013155435799420[211] = 0.0;
   out_5107013155435799420[212] = 0.0;
   out_5107013155435799420[213] = 0.0;
   out_5107013155435799420[214] = 0.0;
   out_5107013155435799420[215] = 0.0;
   out_5107013155435799420[216] = 0.0;
   out_5107013155435799420[217] = 0.0;
   out_5107013155435799420[218] = 0.0;
   out_5107013155435799420[219] = 0.0;
   out_5107013155435799420[220] = 0.0;
   out_5107013155435799420[221] = 0.0;
   out_5107013155435799420[222] = 0.0;
   out_5107013155435799420[223] = 0.0;
   out_5107013155435799420[224] = 0.0;
   out_5107013155435799420[225] = 0.0;
   out_5107013155435799420[226] = 0.0;
   out_5107013155435799420[227] = 0.0;
   out_5107013155435799420[228] = 1.0;
   out_5107013155435799420[229] = 0.0;
   out_5107013155435799420[230] = 0.0;
   out_5107013155435799420[231] = 0.0;
   out_5107013155435799420[232] = 0.0;
   out_5107013155435799420[233] = 0.0;
   out_5107013155435799420[234] = 0.0;
   out_5107013155435799420[235] = 0.0;
   out_5107013155435799420[236] = 0.0;
   out_5107013155435799420[237] = 0.0;
   out_5107013155435799420[238] = 0.0;
   out_5107013155435799420[239] = 0.0;
   out_5107013155435799420[240] = 0.0;
   out_5107013155435799420[241] = 0.0;
   out_5107013155435799420[242] = 0.0;
   out_5107013155435799420[243] = 0.0;
   out_5107013155435799420[244] = 0.0;
   out_5107013155435799420[245] = 0.0;
   out_5107013155435799420[246] = 0.0;
   out_5107013155435799420[247] = 1.0;
   out_5107013155435799420[248] = 0.0;
   out_5107013155435799420[249] = 0.0;
   out_5107013155435799420[250] = 0.0;
   out_5107013155435799420[251] = 0.0;
   out_5107013155435799420[252] = 0.0;
   out_5107013155435799420[253] = 0.0;
   out_5107013155435799420[254] = 0.0;
   out_5107013155435799420[255] = 0.0;
   out_5107013155435799420[256] = 0.0;
   out_5107013155435799420[257] = 0.0;
   out_5107013155435799420[258] = 0.0;
   out_5107013155435799420[259] = 0.0;
   out_5107013155435799420[260] = 0.0;
   out_5107013155435799420[261] = 0.0;
   out_5107013155435799420[262] = 0.0;
   out_5107013155435799420[263] = 0.0;
   out_5107013155435799420[264] = 0.0;
   out_5107013155435799420[265] = 0.0;
   out_5107013155435799420[266] = 1.0;
   out_5107013155435799420[267] = 0.0;
   out_5107013155435799420[268] = 0.0;
   out_5107013155435799420[269] = 0.0;
   out_5107013155435799420[270] = 0.0;
   out_5107013155435799420[271] = 0.0;
   out_5107013155435799420[272] = 0.0;
   out_5107013155435799420[273] = 0.0;
   out_5107013155435799420[274] = 0.0;
   out_5107013155435799420[275] = 0.0;
   out_5107013155435799420[276] = 0.0;
   out_5107013155435799420[277] = 0.0;
   out_5107013155435799420[278] = 0.0;
   out_5107013155435799420[279] = 0.0;
   out_5107013155435799420[280] = 0.0;
   out_5107013155435799420[281] = 0.0;
   out_5107013155435799420[282] = 0.0;
   out_5107013155435799420[283] = 0.0;
   out_5107013155435799420[284] = 0.0;
   out_5107013155435799420[285] = 1.0;
   out_5107013155435799420[286] = 0.0;
   out_5107013155435799420[287] = 0.0;
   out_5107013155435799420[288] = 0.0;
   out_5107013155435799420[289] = 0.0;
   out_5107013155435799420[290] = 0.0;
   out_5107013155435799420[291] = 0.0;
   out_5107013155435799420[292] = 0.0;
   out_5107013155435799420[293] = 0.0;
   out_5107013155435799420[294] = 0.0;
   out_5107013155435799420[295] = 0.0;
   out_5107013155435799420[296] = 0.0;
   out_5107013155435799420[297] = 0.0;
   out_5107013155435799420[298] = 0.0;
   out_5107013155435799420[299] = 0.0;
   out_5107013155435799420[300] = 0.0;
   out_5107013155435799420[301] = 0.0;
   out_5107013155435799420[302] = 0.0;
   out_5107013155435799420[303] = 0.0;
   out_5107013155435799420[304] = 1.0;
   out_5107013155435799420[305] = 0.0;
   out_5107013155435799420[306] = 0.0;
   out_5107013155435799420[307] = 0.0;
   out_5107013155435799420[308] = 0.0;
   out_5107013155435799420[309] = 0.0;
   out_5107013155435799420[310] = 0.0;
   out_5107013155435799420[311] = 0.0;
   out_5107013155435799420[312] = 0.0;
   out_5107013155435799420[313] = 0.0;
   out_5107013155435799420[314] = 0.0;
   out_5107013155435799420[315] = 0.0;
   out_5107013155435799420[316] = 0.0;
   out_5107013155435799420[317] = 0.0;
   out_5107013155435799420[318] = 0.0;
   out_5107013155435799420[319] = 0.0;
   out_5107013155435799420[320] = 0.0;
   out_5107013155435799420[321] = 0.0;
   out_5107013155435799420[322] = 0.0;
   out_5107013155435799420[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_9090648939993874530) {
   out_9090648939993874530[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_9090648939993874530[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_9090648939993874530[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_9090648939993874530[3] = dt*state[12] + state[3];
   out_9090648939993874530[4] = dt*state[13] + state[4];
   out_9090648939993874530[5] = dt*state[14] + state[5];
   out_9090648939993874530[6] = state[6];
   out_9090648939993874530[7] = state[7];
   out_9090648939993874530[8] = state[8];
   out_9090648939993874530[9] = state[9];
   out_9090648939993874530[10] = state[10];
   out_9090648939993874530[11] = state[11];
   out_9090648939993874530[12] = state[12];
   out_9090648939993874530[13] = state[13];
   out_9090648939993874530[14] = state[14];
   out_9090648939993874530[15] = state[15];
   out_9090648939993874530[16] = state[16];
   out_9090648939993874530[17] = state[17];
}
void F_fun(double *state, double dt, double *out_294061415488430612) {
   out_294061415488430612[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_294061415488430612[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_294061415488430612[2] = 0;
   out_294061415488430612[3] = 0;
   out_294061415488430612[4] = 0;
   out_294061415488430612[5] = 0;
   out_294061415488430612[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_294061415488430612[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_294061415488430612[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_294061415488430612[9] = 0;
   out_294061415488430612[10] = 0;
   out_294061415488430612[11] = 0;
   out_294061415488430612[12] = 0;
   out_294061415488430612[13] = 0;
   out_294061415488430612[14] = 0;
   out_294061415488430612[15] = 0;
   out_294061415488430612[16] = 0;
   out_294061415488430612[17] = 0;
   out_294061415488430612[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_294061415488430612[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_294061415488430612[20] = 0;
   out_294061415488430612[21] = 0;
   out_294061415488430612[22] = 0;
   out_294061415488430612[23] = 0;
   out_294061415488430612[24] = 0;
   out_294061415488430612[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_294061415488430612[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_294061415488430612[27] = 0;
   out_294061415488430612[28] = 0;
   out_294061415488430612[29] = 0;
   out_294061415488430612[30] = 0;
   out_294061415488430612[31] = 0;
   out_294061415488430612[32] = 0;
   out_294061415488430612[33] = 0;
   out_294061415488430612[34] = 0;
   out_294061415488430612[35] = 0;
   out_294061415488430612[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_294061415488430612[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_294061415488430612[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_294061415488430612[39] = 0;
   out_294061415488430612[40] = 0;
   out_294061415488430612[41] = 0;
   out_294061415488430612[42] = 0;
   out_294061415488430612[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_294061415488430612[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_294061415488430612[45] = 0;
   out_294061415488430612[46] = 0;
   out_294061415488430612[47] = 0;
   out_294061415488430612[48] = 0;
   out_294061415488430612[49] = 0;
   out_294061415488430612[50] = 0;
   out_294061415488430612[51] = 0;
   out_294061415488430612[52] = 0;
   out_294061415488430612[53] = 0;
   out_294061415488430612[54] = 0;
   out_294061415488430612[55] = 0;
   out_294061415488430612[56] = 0;
   out_294061415488430612[57] = 1;
   out_294061415488430612[58] = 0;
   out_294061415488430612[59] = 0;
   out_294061415488430612[60] = 0;
   out_294061415488430612[61] = 0;
   out_294061415488430612[62] = 0;
   out_294061415488430612[63] = 0;
   out_294061415488430612[64] = 0;
   out_294061415488430612[65] = 0;
   out_294061415488430612[66] = dt;
   out_294061415488430612[67] = 0;
   out_294061415488430612[68] = 0;
   out_294061415488430612[69] = 0;
   out_294061415488430612[70] = 0;
   out_294061415488430612[71] = 0;
   out_294061415488430612[72] = 0;
   out_294061415488430612[73] = 0;
   out_294061415488430612[74] = 0;
   out_294061415488430612[75] = 0;
   out_294061415488430612[76] = 1;
   out_294061415488430612[77] = 0;
   out_294061415488430612[78] = 0;
   out_294061415488430612[79] = 0;
   out_294061415488430612[80] = 0;
   out_294061415488430612[81] = 0;
   out_294061415488430612[82] = 0;
   out_294061415488430612[83] = 0;
   out_294061415488430612[84] = 0;
   out_294061415488430612[85] = dt;
   out_294061415488430612[86] = 0;
   out_294061415488430612[87] = 0;
   out_294061415488430612[88] = 0;
   out_294061415488430612[89] = 0;
   out_294061415488430612[90] = 0;
   out_294061415488430612[91] = 0;
   out_294061415488430612[92] = 0;
   out_294061415488430612[93] = 0;
   out_294061415488430612[94] = 0;
   out_294061415488430612[95] = 1;
   out_294061415488430612[96] = 0;
   out_294061415488430612[97] = 0;
   out_294061415488430612[98] = 0;
   out_294061415488430612[99] = 0;
   out_294061415488430612[100] = 0;
   out_294061415488430612[101] = 0;
   out_294061415488430612[102] = 0;
   out_294061415488430612[103] = 0;
   out_294061415488430612[104] = dt;
   out_294061415488430612[105] = 0;
   out_294061415488430612[106] = 0;
   out_294061415488430612[107] = 0;
   out_294061415488430612[108] = 0;
   out_294061415488430612[109] = 0;
   out_294061415488430612[110] = 0;
   out_294061415488430612[111] = 0;
   out_294061415488430612[112] = 0;
   out_294061415488430612[113] = 0;
   out_294061415488430612[114] = 1;
   out_294061415488430612[115] = 0;
   out_294061415488430612[116] = 0;
   out_294061415488430612[117] = 0;
   out_294061415488430612[118] = 0;
   out_294061415488430612[119] = 0;
   out_294061415488430612[120] = 0;
   out_294061415488430612[121] = 0;
   out_294061415488430612[122] = 0;
   out_294061415488430612[123] = 0;
   out_294061415488430612[124] = 0;
   out_294061415488430612[125] = 0;
   out_294061415488430612[126] = 0;
   out_294061415488430612[127] = 0;
   out_294061415488430612[128] = 0;
   out_294061415488430612[129] = 0;
   out_294061415488430612[130] = 0;
   out_294061415488430612[131] = 0;
   out_294061415488430612[132] = 0;
   out_294061415488430612[133] = 1;
   out_294061415488430612[134] = 0;
   out_294061415488430612[135] = 0;
   out_294061415488430612[136] = 0;
   out_294061415488430612[137] = 0;
   out_294061415488430612[138] = 0;
   out_294061415488430612[139] = 0;
   out_294061415488430612[140] = 0;
   out_294061415488430612[141] = 0;
   out_294061415488430612[142] = 0;
   out_294061415488430612[143] = 0;
   out_294061415488430612[144] = 0;
   out_294061415488430612[145] = 0;
   out_294061415488430612[146] = 0;
   out_294061415488430612[147] = 0;
   out_294061415488430612[148] = 0;
   out_294061415488430612[149] = 0;
   out_294061415488430612[150] = 0;
   out_294061415488430612[151] = 0;
   out_294061415488430612[152] = 1;
   out_294061415488430612[153] = 0;
   out_294061415488430612[154] = 0;
   out_294061415488430612[155] = 0;
   out_294061415488430612[156] = 0;
   out_294061415488430612[157] = 0;
   out_294061415488430612[158] = 0;
   out_294061415488430612[159] = 0;
   out_294061415488430612[160] = 0;
   out_294061415488430612[161] = 0;
   out_294061415488430612[162] = 0;
   out_294061415488430612[163] = 0;
   out_294061415488430612[164] = 0;
   out_294061415488430612[165] = 0;
   out_294061415488430612[166] = 0;
   out_294061415488430612[167] = 0;
   out_294061415488430612[168] = 0;
   out_294061415488430612[169] = 0;
   out_294061415488430612[170] = 0;
   out_294061415488430612[171] = 1;
   out_294061415488430612[172] = 0;
   out_294061415488430612[173] = 0;
   out_294061415488430612[174] = 0;
   out_294061415488430612[175] = 0;
   out_294061415488430612[176] = 0;
   out_294061415488430612[177] = 0;
   out_294061415488430612[178] = 0;
   out_294061415488430612[179] = 0;
   out_294061415488430612[180] = 0;
   out_294061415488430612[181] = 0;
   out_294061415488430612[182] = 0;
   out_294061415488430612[183] = 0;
   out_294061415488430612[184] = 0;
   out_294061415488430612[185] = 0;
   out_294061415488430612[186] = 0;
   out_294061415488430612[187] = 0;
   out_294061415488430612[188] = 0;
   out_294061415488430612[189] = 0;
   out_294061415488430612[190] = 1;
   out_294061415488430612[191] = 0;
   out_294061415488430612[192] = 0;
   out_294061415488430612[193] = 0;
   out_294061415488430612[194] = 0;
   out_294061415488430612[195] = 0;
   out_294061415488430612[196] = 0;
   out_294061415488430612[197] = 0;
   out_294061415488430612[198] = 0;
   out_294061415488430612[199] = 0;
   out_294061415488430612[200] = 0;
   out_294061415488430612[201] = 0;
   out_294061415488430612[202] = 0;
   out_294061415488430612[203] = 0;
   out_294061415488430612[204] = 0;
   out_294061415488430612[205] = 0;
   out_294061415488430612[206] = 0;
   out_294061415488430612[207] = 0;
   out_294061415488430612[208] = 0;
   out_294061415488430612[209] = 1;
   out_294061415488430612[210] = 0;
   out_294061415488430612[211] = 0;
   out_294061415488430612[212] = 0;
   out_294061415488430612[213] = 0;
   out_294061415488430612[214] = 0;
   out_294061415488430612[215] = 0;
   out_294061415488430612[216] = 0;
   out_294061415488430612[217] = 0;
   out_294061415488430612[218] = 0;
   out_294061415488430612[219] = 0;
   out_294061415488430612[220] = 0;
   out_294061415488430612[221] = 0;
   out_294061415488430612[222] = 0;
   out_294061415488430612[223] = 0;
   out_294061415488430612[224] = 0;
   out_294061415488430612[225] = 0;
   out_294061415488430612[226] = 0;
   out_294061415488430612[227] = 0;
   out_294061415488430612[228] = 1;
   out_294061415488430612[229] = 0;
   out_294061415488430612[230] = 0;
   out_294061415488430612[231] = 0;
   out_294061415488430612[232] = 0;
   out_294061415488430612[233] = 0;
   out_294061415488430612[234] = 0;
   out_294061415488430612[235] = 0;
   out_294061415488430612[236] = 0;
   out_294061415488430612[237] = 0;
   out_294061415488430612[238] = 0;
   out_294061415488430612[239] = 0;
   out_294061415488430612[240] = 0;
   out_294061415488430612[241] = 0;
   out_294061415488430612[242] = 0;
   out_294061415488430612[243] = 0;
   out_294061415488430612[244] = 0;
   out_294061415488430612[245] = 0;
   out_294061415488430612[246] = 0;
   out_294061415488430612[247] = 1;
   out_294061415488430612[248] = 0;
   out_294061415488430612[249] = 0;
   out_294061415488430612[250] = 0;
   out_294061415488430612[251] = 0;
   out_294061415488430612[252] = 0;
   out_294061415488430612[253] = 0;
   out_294061415488430612[254] = 0;
   out_294061415488430612[255] = 0;
   out_294061415488430612[256] = 0;
   out_294061415488430612[257] = 0;
   out_294061415488430612[258] = 0;
   out_294061415488430612[259] = 0;
   out_294061415488430612[260] = 0;
   out_294061415488430612[261] = 0;
   out_294061415488430612[262] = 0;
   out_294061415488430612[263] = 0;
   out_294061415488430612[264] = 0;
   out_294061415488430612[265] = 0;
   out_294061415488430612[266] = 1;
   out_294061415488430612[267] = 0;
   out_294061415488430612[268] = 0;
   out_294061415488430612[269] = 0;
   out_294061415488430612[270] = 0;
   out_294061415488430612[271] = 0;
   out_294061415488430612[272] = 0;
   out_294061415488430612[273] = 0;
   out_294061415488430612[274] = 0;
   out_294061415488430612[275] = 0;
   out_294061415488430612[276] = 0;
   out_294061415488430612[277] = 0;
   out_294061415488430612[278] = 0;
   out_294061415488430612[279] = 0;
   out_294061415488430612[280] = 0;
   out_294061415488430612[281] = 0;
   out_294061415488430612[282] = 0;
   out_294061415488430612[283] = 0;
   out_294061415488430612[284] = 0;
   out_294061415488430612[285] = 1;
   out_294061415488430612[286] = 0;
   out_294061415488430612[287] = 0;
   out_294061415488430612[288] = 0;
   out_294061415488430612[289] = 0;
   out_294061415488430612[290] = 0;
   out_294061415488430612[291] = 0;
   out_294061415488430612[292] = 0;
   out_294061415488430612[293] = 0;
   out_294061415488430612[294] = 0;
   out_294061415488430612[295] = 0;
   out_294061415488430612[296] = 0;
   out_294061415488430612[297] = 0;
   out_294061415488430612[298] = 0;
   out_294061415488430612[299] = 0;
   out_294061415488430612[300] = 0;
   out_294061415488430612[301] = 0;
   out_294061415488430612[302] = 0;
   out_294061415488430612[303] = 0;
   out_294061415488430612[304] = 1;
   out_294061415488430612[305] = 0;
   out_294061415488430612[306] = 0;
   out_294061415488430612[307] = 0;
   out_294061415488430612[308] = 0;
   out_294061415488430612[309] = 0;
   out_294061415488430612[310] = 0;
   out_294061415488430612[311] = 0;
   out_294061415488430612[312] = 0;
   out_294061415488430612[313] = 0;
   out_294061415488430612[314] = 0;
   out_294061415488430612[315] = 0;
   out_294061415488430612[316] = 0;
   out_294061415488430612[317] = 0;
   out_294061415488430612[318] = 0;
   out_294061415488430612[319] = 0;
   out_294061415488430612[320] = 0;
   out_294061415488430612[321] = 0;
   out_294061415488430612[322] = 0;
   out_294061415488430612[323] = 1;
}
void h_4(double *state, double *unused, double *out_2525208654296729708) {
   out_2525208654296729708[0] = state[6] + state[9];
   out_2525208654296729708[1] = state[7] + state[10];
   out_2525208654296729708[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7563366274452672869) {
   out_7563366274452672869[0] = 0;
   out_7563366274452672869[1] = 0;
   out_7563366274452672869[2] = 0;
   out_7563366274452672869[3] = 0;
   out_7563366274452672869[4] = 0;
   out_7563366274452672869[5] = 0;
   out_7563366274452672869[6] = 1;
   out_7563366274452672869[7] = 0;
   out_7563366274452672869[8] = 0;
   out_7563366274452672869[9] = 1;
   out_7563366274452672869[10] = 0;
   out_7563366274452672869[11] = 0;
   out_7563366274452672869[12] = 0;
   out_7563366274452672869[13] = 0;
   out_7563366274452672869[14] = 0;
   out_7563366274452672869[15] = 0;
   out_7563366274452672869[16] = 0;
   out_7563366274452672869[17] = 0;
   out_7563366274452672869[18] = 0;
   out_7563366274452672869[19] = 0;
   out_7563366274452672869[20] = 0;
   out_7563366274452672869[21] = 0;
   out_7563366274452672869[22] = 0;
   out_7563366274452672869[23] = 0;
   out_7563366274452672869[24] = 0;
   out_7563366274452672869[25] = 1;
   out_7563366274452672869[26] = 0;
   out_7563366274452672869[27] = 0;
   out_7563366274452672869[28] = 1;
   out_7563366274452672869[29] = 0;
   out_7563366274452672869[30] = 0;
   out_7563366274452672869[31] = 0;
   out_7563366274452672869[32] = 0;
   out_7563366274452672869[33] = 0;
   out_7563366274452672869[34] = 0;
   out_7563366274452672869[35] = 0;
   out_7563366274452672869[36] = 0;
   out_7563366274452672869[37] = 0;
   out_7563366274452672869[38] = 0;
   out_7563366274452672869[39] = 0;
   out_7563366274452672869[40] = 0;
   out_7563366274452672869[41] = 0;
   out_7563366274452672869[42] = 0;
   out_7563366274452672869[43] = 0;
   out_7563366274452672869[44] = 1;
   out_7563366274452672869[45] = 0;
   out_7563366274452672869[46] = 0;
   out_7563366274452672869[47] = 1;
   out_7563366274452672869[48] = 0;
   out_7563366274452672869[49] = 0;
   out_7563366274452672869[50] = 0;
   out_7563366274452672869[51] = 0;
   out_7563366274452672869[52] = 0;
   out_7563366274452672869[53] = 0;
}
void h_10(double *state, double *unused, double *out_2333383739813503746) {
   out_2333383739813503746[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2333383739813503746[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2333383739813503746[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_8502106662555230246) {
   out_8502106662555230246[0] = 0;
   out_8502106662555230246[1] = 9.8100000000000005*cos(state[1]);
   out_8502106662555230246[2] = 0;
   out_8502106662555230246[3] = 0;
   out_8502106662555230246[4] = -state[8];
   out_8502106662555230246[5] = state[7];
   out_8502106662555230246[6] = 0;
   out_8502106662555230246[7] = state[5];
   out_8502106662555230246[8] = -state[4];
   out_8502106662555230246[9] = 0;
   out_8502106662555230246[10] = 0;
   out_8502106662555230246[11] = 0;
   out_8502106662555230246[12] = 1;
   out_8502106662555230246[13] = 0;
   out_8502106662555230246[14] = 0;
   out_8502106662555230246[15] = 1;
   out_8502106662555230246[16] = 0;
   out_8502106662555230246[17] = 0;
   out_8502106662555230246[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_8502106662555230246[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_8502106662555230246[20] = 0;
   out_8502106662555230246[21] = state[8];
   out_8502106662555230246[22] = 0;
   out_8502106662555230246[23] = -state[6];
   out_8502106662555230246[24] = -state[5];
   out_8502106662555230246[25] = 0;
   out_8502106662555230246[26] = state[3];
   out_8502106662555230246[27] = 0;
   out_8502106662555230246[28] = 0;
   out_8502106662555230246[29] = 0;
   out_8502106662555230246[30] = 0;
   out_8502106662555230246[31] = 1;
   out_8502106662555230246[32] = 0;
   out_8502106662555230246[33] = 0;
   out_8502106662555230246[34] = 1;
   out_8502106662555230246[35] = 0;
   out_8502106662555230246[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_8502106662555230246[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_8502106662555230246[38] = 0;
   out_8502106662555230246[39] = -state[7];
   out_8502106662555230246[40] = state[6];
   out_8502106662555230246[41] = 0;
   out_8502106662555230246[42] = state[4];
   out_8502106662555230246[43] = -state[3];
   out_8502106662555230246[44] = 0;
   out_8502106662555230246[45] = 0;
   out_8502106662555230246[46] = 0;
   out_8502106662555230246[47] = 0;
   out_8502106662555230246[48] = 0;
   out_8502106662555230246[49] = 0;
   out_8502106662555230246[50] = 1;
   out_8502106662555230246[51] = 0;
   out_8502106662555230246[52] = 0;
   out_8502106662555230246[53] = 1;
}
void h_13(double *state, double *unused, double *out_3720576547851047664) {
   out_3720576547851047664[0] = state[3];
   out_3720576547851047664[1] = state[4];
   out_3720576547851047664[2] = state[5];
}
void H_13(double *state, double *unused, double *out_7671103973924545946) {
   out_7671103973924545946[0] = 0;
   out_7671103973924545946[1] = 0;
   out_7671103973924545946[2] = 0;
   out_7671103973924545946[3] = 1;
   out_7671103973924545946[4] = 0;
   out_7671103973924545946[5] = 0;
   out_7671103973924545946[6] = 0;
   out_7671103973924545946[7] = 0;
   out_7671103973924545946[8] = 0;
   out_7671103973924545946[9] = 0;
   out_7671103973924545946[10] = 0;
   out_7671103973924545946[11] = 0;
   out_7671103973924545946[12] = 0;
   out_7671103973924545946[13] = 0;
   out_7671103973924545946[14] = 0;
   out_7671103973924545946[15] = 0;
   out_7671103973924545946[16] = 0;
   out_7671103973924545946[17] = 0;
   out_7671103973924545946[18] = 0;
   out_7671103973924545946[19] = 0;
   out_7671103973924545946[20] = 0;
   out_7671103973924545946[21] = 0;
   out_7671103973924545946[22] = 1;
   out_7671103973924545946[23] = 0;
   out_7671103973924545946[24] = 0;
   out_7671103973924545946[25] = 0;
   out_7671103973924545946[26] = 0;
   out_7671103973924545946[27] = 0;
   out_7671103973924545946[28] = 0;
   out_7671103973924545946[29] = 0;
   out_7671103973924545946[30] = 0;
   out_7671103973924545946[31] = 0;
   out_7671103973924545946[32] = 0;
   out_7671103973924545946[33] = 0;
   out_7671103973924545946[34] = 0;
   out_7671103973924545946[35] = 0;
   out_7671103973924545946[36] = 0;
   out_7671103973924545946[37] = 0;
   out_7671103973924545946[38] = 0;
   out_7671103973924545946[39] = 0;
   out_7671103973924545946[40] = 0;
   out_7671103973924545946[41] = 1;
   out_7671103973924545946[42] = 0;
   out_7671103973924545946[43] = 0;
   out_7671103973924545946[44] = 0;
   out_7671103973924545946[45] = 0;
   out_7671103973924545946[46] = 0;
   out_7671103973924545946[47] = 0;
   out_7671103973924545946[48] = 0;
   out_7671103973924545946[49] = 0;
   out_7671103973924545946[50] = 0;
   out_7671103973924545946[51] = 0;
   out_7671103973924545946[52] = 0;
   out_7671103973924545946[53] = 0;
}
void h_14(double *state, double *unused, double *out_931071426167433179) {
   out_931071426167433179[0] = state[6];
   out_931071426167433179[1] = state[7];
   out_931071426167433179[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6920136942917394218) {
   out_6920136942917394218[0] = 0;
   out_6920136942917394218[1] = 0;
   out_6920136942917394218[2] = 0;
   out_6920136942917394218[3] = 0;
   out_6920136942917394218[4] = 0;
   out_6920136942917394218[5] = 0;
   out_6920136942917394218[6] = 1;
   out_6920136942917394218[7] = 0;
   out_6920136942917394218[8] = 0;
   out_6920136942917394218[9] = 0;
   out_6920136942917394218[10] = 0;
   out_6920136942917394218[11] = 0;
   out_6920136942917394218[12] = 0;
   out_6920136942917394218[13] = 0;
   out_6920136942917394218[14] = 0;
   out_6920136942917394218[15] = 0;
   out_6920136942917394218[16] = 0;
   out_6920136942917394218[17] = 0;
   out_6920136942917394218[18] = 0;
   out_6920136942917394218[19] = 0;
   out_6920136942917394218[20] = 0;
   out_6920136942917394218[21] = 0;
   out_6920136942917394218[22] = 0;
   out_6920136942917394218[23] = 0;
   out_6920136942917394218[24] = 0;
   out_6920136942917394218[25] = 1;
   out_6920136942917394218[26] = 0;
   out_6920136942917394218[27] = 0;
   out_6920136942917394218[28] = 0;
   out_6920136942917394218[29] = 0;
   out_6920136942917394218[30] = 0;
   out_6920136942917394218[31] = 0;
   out_6920136942917394218[32] = 0;
   out_6920136942917394218[33] = 0;
   out_6920136942917394218[34] = 0;
   out_6920136942917394218[35] = 0;
   out_6920136942917394218[36] = 0;
   out_6920136942917394218[37] = 0;
   out_6920136942917394218[38] = 0;
   out_6920136942917394218[39] = 0;
   out_6920136942917394218[40] = 0;
   out_6920136942917394218[41] = 0;
   out_6920136942917394218[42] = 0;
   out_6920136942917394218[43] = 0;
   out_6920136942917394218[44] = 1;
   out_6920136942917394218[45] = 0;
   out_6920136942917394218[46] = 0;
   out_6920136942917394218[47] = 0;
   out_6920136942917394218[48] = 0;
   out_6920136942917394218[49] = 0;
   out_6920136942917394218[50] = 0;
   out_6920136942917394218[51] = 0;
   out_6920136942917394218[52] = 0;
   out_6920136942917394218[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_1101899825002175654) {
  err_fun(nom_x, delta_x, out_1101899825002175654);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_9121434067879914978) {
  inv_err_fun(nom_x, true_x, out_9121434067879914978);
}
void pose_H_mod_fun(double *state, double *out_5107013155435799420) {
  H_mod_fun(state, out_5107013155435799420);
}
void pose_f_fun(double *state, double dt, double *out_9090648939993874530) {
  f_fun(state,  dt, out_9090648939993874530);
}
void pose_F_fun(double *state, double dt, double *out_294061415488430612) {
  F_fun(state,  dt, out_294061415488430612);
}
void pose_h_4(double *state, double *unused, double *out_2525208654296729708) {
  h_4(state, unused, out_2525208654296729708);
}
void pose_H_4(double *state, double *unused, double *out_7563366274452672869) {
  H_4(state, unused, out_7563366274452672869);
}
void pose_h_10(double *state, double *unused, double *out_2333383739813503746) {
  h_10(state, unused, out_2333383739813503746);
}
void pose_H_10(double *state, double *unused, double *out_8502106662555230246) {
  H_10(state, unused, out_8502106662555230246);
}
void pose_h_13(double *state, double *unused, double *out_3720576547851047664) {
  h_13(state, unused, out_3720576547851047664);
}
void pose_H_13(double *state, double *unused, double *out_7671103973924545946) {
  H_13(state, unused, out_7671103973924545946);
}
void pose_h_14(double *state, double *unused, double *out_931071426167433179) {
  h_14(state, unused, out_931071426167433179);
}
void pose_H_14(double *state, double *unused, double *out_6920136942917394218) {
  H_14(state, unused, out_6920136942917394218);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
