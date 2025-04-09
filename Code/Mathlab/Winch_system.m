% 
clear all; close all; clc;
a = 0.315;
b = 0.37;
c = 0.125;
d = 0.11;
r = 0.245+0.025;
lc = 0.335;
hfg = 0.110/2+0.025;
m = 1;
theta = 3*pi/4;
s0 = sin(theta);
c0 = cos(theta);
phi = atan((c+r*s0)/(b-r*c0));
p = pi-phi-theta;
lr =(r*s0+c)/sin(phi)-lc;
sig = atan((c-d+r*s0)/(a+r*c0));
q = theta-sig;

ln = 0.115;
e = lr-ln;
k = 551.61;
Fe = k*e;
Fg = m*9.8061;
Fe*sin(p);
Fg*c0*hfg;
Fm =(Fe*sin(p)*r+Fg*c0*hfg)/(sin(q)*r);
Tm = 0.0225*Fm