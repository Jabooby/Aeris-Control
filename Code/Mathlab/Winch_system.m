%% Required torque x-y motors (angle of 135 deg between horizontal and mast
% including only 1 motor and its opposed spring
clear all; close all; clc;
a = 0.315; % mast-base to winch (view A)
b = 0.37; % mast-base to spring bracket (view A)
c = 0.125; % mast-base height
d = 0.11; % winch height
r = 0.245+0.025; % crows-nest heigth from mast-base
lc = 0.335; % longest rope length 
hfg = 0.110/2+0.025; % mass center heigth form mast-base
m = 1; % sail mass
theta = 3*pi/4; % mast angle
s0 = sin(theta);
c0 = cos(theta);
phi = atan((c+r*s0)/(b-r*c0)); % angle between longest rope and horizontal
p = pi-phi-theta; % angle between mast and longest rope
lr =(r*s0+c)/sin(phi)-lc; % spring elongated at mast angle
sig = atan((c-d+r*s0)/(a+r*c0)); % angle between winch's rope and horizontal
q = theta-sig; % angle between mast and winch's rope
r_winch = 0.0225;

ln = 0.12; %  spring's natural length
e = lr-ln; %spring elongation
k = 333; % empirically calculated 
Fe = k*e; % elastic force
Fg = m*9.8061; % gravitational force
Fm =(Fe*sin(p)*r+Fg*c0*hfg)/(sin(q)*r); % force applicated by torque motor
T_winchxy = r_winch*Fm % required torque for winches motor

%% Required torque z-motor for critical position 90 deg and 135 deg
mu = 0.25; %  friction coefficient
r_planet = 0.0525; % radius of planet gear
fric = mu*(Fg+2*Fe*sin(phi)); %Friction force
T_z_angle45 = r_planet*fric % Required torque

d2=0.1971; % side spring to mast-base
lam=atan((r+c)/d2);
e1 = 0.05; % elongation of corner springs (measured)
e2 = 0.04; % elongation of side springs (measured)
Fe1 = k*e1*sin(phi);
Fe2 = k*e2*sin(lam);
T_z_angle90=(Fg+2*(Fe1+Fe2))*mu*r_planet