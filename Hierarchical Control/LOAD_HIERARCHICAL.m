close all
clear all
clc

pos_0 = [-1 -1.2 2];
lin_vel_0 = [0 0 0];
T=60;

load("POS_H.mat");
load("VEL_H.mat");
load("ACC_H.mat");
load("PSI_H.mat");


xx_new=PP_new(1,:);
yy_new=PP_new(2,:);
zz_new=PP_new(3,:);
new_tt=PP_new(4,:);

xv_new=VV_new(1,:);
yv_new=VV_new(2,:);
zv_new=VV_new(3,:);

xa_new=AA_new(1,:);
ya_new=AA_new(2,:);
za_new=AA_new(3,:);
