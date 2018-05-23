
#ifndef __KALMAN_FILTER_H_
#define __KALMAN_FILTER_H_

#define Pi 	3.1415926
//卡尔曼噪声参数
#define dt           0.012
#define R_angle      0.5
#define Q_angle      0.001
#define Q_gyro       0.003//越大越滞后


double KalmanFilter(const double ResrcData, double ProcessNiose_Q, double MeasureNoise_R);
void Angle_Calcu(short *accel, short *gyro, float *angle_temp, float *angle_final);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);
void Kalman_Filter_Z(float Accel, float Gyro);
void yijiehubu_P(float angle_m, float gyro_m);
void erjiehubu_P(float angle_m, float gyro_m);
void Erjielvbo(float angle_m,float gyro_m);

#endif
