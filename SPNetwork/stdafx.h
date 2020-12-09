// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#include "targetver.h"
#include<iostream>
#include <stdio.h>
#include <tchar.h>
#include <math.h>

// LR parameters
const int n_given = 100;
const int ite=20;
const int choose =2; 
const int T_stamp = 300; // Number of timestamps
const int i_station = 8; // Number of bi directional stations
const int k_train = 7; // Number of considered trains
const int t_arrival[]={0, 30, 60, 90, 120, 150, 180, 210, 240, 270};
const int t_interval = 10; // Length of each time interval
const int d_min = 30/t_interval; // Mimimul time intervals for dwelling
const int d_max = 90/t_interval; // Maximum time intervals for dwelling
const int d_options[5] = {3, 4, 5, 6, 7};
// Totally six segments
const int s[]={1050, 1515, 1050, 1050, 1050, 1515, 1050, 1050};
const int t_segment[]={10,  15,   10,   10,  10,   15,    10,   10};
//               0,    1,    2,    3,    4,    5,   6,     7
//const int v_l[]={2, 1, 4, 7, 10, 12, 13, 13, 13, 13, 13, 10, 7, 4, 1, 2}; // Corresponding to the 1515 distance

const int v_l[]={1, 3, 6, 9, 12, 15, 15, 15, 15, 15, 15, 12, 9, 6, 3, 1}; // Corresponding to the 1515 distance
const int s_l[]={0, 15, 60, 135, 240, 375, 525, 675, 825, 975, 1125, 1275, 1380, 1455, 1500, 1515};
const int v_s[]={1, 5, 10, 15, 15, 15, 15, 15, 10, 5, 1}; // Corresponding to the 1050 distance
const int s_s[]={0, 25, 100, 225, 375, 525, 675, 825, 950, 1025, 1050};
const int l_train=150;                               // Length of train k
//const int d_safe=20;                                // Safe margin
const int d_safe=75;                                // Safe margin
const int b_rate=1;                                 // Emergency braking rate
const int t_reaction=1;                             // Reaction time
const int w=12; 
const float percentage=1/w;
const int reaction_sc[10]={15, 16, 80, 18, 19, 10, 11, 12, 13, 14}; // Scenario-based reaction time
//const int reaction_p[3]={0.5, 0.5, 0.5}; // Scenario-based reaction time
//const int stochastic_parameters[2][11]={ 12, 15, 14, 14, 13, 13, 0, 16, 12, 13, 14,
  //                                       75, 75, 76, 60, 33, 75, 190,  0,  76, 79, 84};
const int parameter_1=5;
const int parameter_2=45;
const int parameter_9=50;
const int parameter_10=60;

const int stochastic_parameters[7][12]={ 12,  20, 17, 15, 13, 0, 16, 12, 13, 14,19, 16,
                                         70, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84, 15, parameter_10,
                                         90, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84, 15, parameter_10,                                                                                  
                                         90, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84,15, parameter_10,
                                         90, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84,15,parameter_10,
                                         90, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84,20, parameter_10,
                                         90, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84,40, parameter_10}; 
const int first_stochastic_parameters[7][12]={ 12, 20, 17, 15, 13, 0, 16, 12, 13, 14,19, 16,
                                               70, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84, 15, parameter_10,
                                               90, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84,   15,     parameter_10,                                                                             
                                               90, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84,15, parameter_10,
                                               90, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84,15, parameter_10,
                                               90, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84,15, parameter_10,
                                               91, parameter_1, parameter_2, 80, 75, 190,  0,  76, parameter_9, 84,15, parameter_10};
/*const int first_stochastic_parameters[7][11]={ 12, 19, 20, 17, 15, 13, 0, 16, 12, 13, 14,
                                               70, 15, 0, 45, 80, 75, 190,  0,  76, 79, 84,
                                               90, 15, 0, 45, 80, 75, 190,  0,  76, 79, 84,                                                                                   
                                               90, 15, 0, 45, 80, 75, 190,  0,  76, 79, 84,
                                              90, 15, 0, 45, 80, 75, 190,  0,  76, 79, 84,
                                              90, 15, 0, 45, 80, 75, 190,  0,  76, 79, 84,
                                               91, 15, 0, 46, 80, 75, 190,  0,  76, 79, 84};*/
/*const int second_stochastic_parameters[7][11]={ 12, 19, 20, 17, 15, 13, 0, 16, 12, 13, 14,
                                               70, 15, 0, 45, 80, 75, 190,  0,  76, 79, 84,
                                               90, 15, 0, 45, 80, 75, 190,  0,  76, 79, 84,                                                                                   
                                               90, 15, 0, 45, 80, 75, 190,  0,  76, 79, 84,
                                              225, 15, 0, 45, 80, 75, 190,  0,  76, 79, 84,
                                              225, 1500, 0, 45, 80, 75, 190,  0,  76, 79, 84,
                                               91, 1500, 0, 46, 80, 75, 190,  0,  76, 79, 84};*/
// Representation of stochastic parameters: First line: Reaction time; Second line: d_safe
const int depart_choices=4;
const int d_t_h=7;
/*const int train_depart_options[depart_choices][k_train]={0, 0+d_t_h, 2*d_t_h, 2*d_t_h, 4*d_t_h, 5*d_t_h, 6*d_t_h, 
	                                                     3, 3+d_t_h, 3+2*d_t_h, 3+3*d_t_h,3+ 4*d_t_h, 3+5*d_t_h, 3+6*d_t_h,
														 6, 6+d_t_h, 6+2*d_t_h, 6+3*d_t_h, 6+4*d_t_h, 6+5*d_t_h, 6+6*d_t_h, 
														 8, 8+d_t_h, 8+2*d_t_h, 8+3*d_t_h, 8+4*d_t_h, 8+5*d_t_h, 8+6*d_t_h};*/
const int train_depart_options[depart_choices][k_train]= {0, 0+d_t_h, 0+2*d_t_h, 0+3*d_t_h, 1+ 4*d_t_h, 0+5*d_t_h, 0+6*d_t_h, 
	                                                     1, 1+d_t_h, 1+2*d_t_h, 1+3*d_t_h, 1+ 4*d_t_h, 1+5*d_t_h, 1+6*d_t_h,
														 2, 2+d_t_h, 2+2*d_t_h, 2+3*d_t_h, 2+4*d_t_h, 2+5*d_t_h, 2+6*d_t_h, 
														 3, 3+d_t_h, 3+2*d_t_h, 3+3*d_t_h, 3+4*d_t_h, 3+5*d_t_h, 3+6*d_t_h};


/*const int train_depart_options[depart_choices][k_train]={0, 0+d_t_h, 2*d_t_h, 2*d_t_h, 4*d_t_h, 5*d_t_h, 6*d_t_h, 
	                                                     3, 3+d_t_h, 3+2*d_t_h, 3+3*d_t_h,3+ 4*d_t_h, 3+5*d_t_h, 3+6*d_t_h,
														 6, 6+d_t_h, 6+2*d_t_h, 6+3*d_t_h, 6+4*d_t_h, 6+5*d_t_h, 6+6*d_t_h, 
														 8, 8+d_t_h, 8+2*d_t_h, 8+3*d_t_h, 8+4*d_t_h, 8+5*d_t_h, 8+6*d_t_h};*/

/*const int train_depart_options[depart_choices][k_train]={0, 0, 2, 2, 4, 5, 6, 
	                                                     3, 3, 3+2, 3+3,3+ 4, 3+5, 3+6,
														 6, 6, 6+2, 6+3, 6+4, 6+5, 6+6, 
														 8, 8, 8+2, 8+3, 8+4, 8+5, 8+6};*/

int t_l=15;                                         // Totally 15 running intervals
int t_s=10;                                         // Totally 10 running intervals
int t_ar=12;
const int m_big=1000;
double cc= pow((double)2,(double)8);
const int dwell_state=(int)(cc);
extern int dwelling_time[];
extern int index_dwelling[][6];
//const int t_r[]={t_s, t_l, t_s, t_ar, t_s, t_l, t_s, t_ar};
const int t_r[]={10, 15, 10, 12, 10, 15, 10, 12};
//int dwelling_time[6];


int p_distance(int i, int t, int k);
int v_distance(int i, int t);
double sub1(double i);

//double f_path(int i, int j, int *k);
double f_coupling(int s, int i, int *q, int d, int *g, int k, double x);
double test_f_coupling(int s, int i, int *q, int d, int *g, int k, double x);
double first_train_f_coupling(int s, int i, int *j, int *x);
double test_first_train_f_coupling(int s, int i, int *j, int *x);

// Defination of speed profiles
// TODO: 在此处引用程序需要的其他头文件
