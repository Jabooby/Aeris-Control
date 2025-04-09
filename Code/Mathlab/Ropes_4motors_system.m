%% Modélisation planche à voile (câblage)
clc; close all; clear;

%% Position courante
% On obtient des coordonnées sphériques du IMU
theta = 45/360*2*pi; % 
phi = 20/360*2*pi;
% syms phi theta

r = 0.1 ; %hauteur crows nest
b = 0.1 ; %distance entre moteur et O
p = 20/360*2*pi;  %angle entre x et MO
p2 = pi-p;
p3 = p-pi;
p4 = -p;
R = 0.02; %enrouleur

% Vecteurs MO
b1 = [b*cos(p) b*sin(p) 0];
b2 = [b*cos(p2) b*sin(p2) 0];
b3 = [b*cos(p3) b*sin(p3) 0];
b4 = [b*cos(p4) b*sin(p4) 0];

% Vecteur mât
mat = [r*sin(theta)*cos(phi) r*sin(theta)*sin(phi) r*cos(theta)];

% Longueurs 
L1 = norm(b1-mat);
L2 = norm(b2-mat);
L3 = norm(b3-mat);
L4 = norm(b4-mat);


%% Positions voulues
x = 0.05;
y = 0.05;
z = sqrt(r^2-x^2-y^2);
nmat = [x y z];

nL1 = norm(b1-nmat);
nL2 = norm(b2-nmat);
nL3 = norm(b3-nmat);
nL4 = norm(b4-nmat);

%% Vitesse moteurs
vmax = 0.05;
delta1 = L1-nL1;
delta2 = L2-nL2;
delta3 = L3-nL3;
delta4 = L4-nL4;
deltamax = max(abs([delta1 delta2 delta3 delta4]));
t = deltamax/vmax;
w1 = delta1/R*t;
w2 = delta2/R*t;
w3 = delta3/R*t;
w4 = delta4/R*t;

%% Torque requis max
%disons 45 degrés

m = 1;
g = 9.8061;
Fg = m*g;
angle = pi/4;
%% Torque requis position critique
s0 = sin(angle);
c0 = cos(angle);
q = asin(b*sin(7*pi/4)/L1);
Tfg = Fg*c0*r/2;
Tm = -Tfg;
Fm = Tm/(cos(pi-q)*r);
T = Fm*R
