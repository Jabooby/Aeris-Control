m = 1;
g = 9.8061;
h = 0.2667;
b = 0.8;
r = 0.024;
Fg = m*g;


%% Torque requis position critique

syms T(theta);

s0 = sin(theta);
c0 = cos(theta);
phi = atan(h*sin(theta)/(h*cos(theta)+b/2));
Tfg = Fg*c0*h/2;


T(theta) = 0.5*Fg*c0/(cos(phi-theta+pi/2));

tension = eval(T(pi/6));

torque = tension*r



%% emplacement ressort
% on veut que le ressort soit à sa longueur naturelle lorsque la corde est
% la plus lousse (mat a angle min.)
R = 0.175/4+0.01;
hm = 0.2667;
hf = 0.108;

L = sqrt((hm+hf)^2+(b/2)^2);

q = pi/4;
a = sqrt((b/2-hm*cos(q))^2+(hf+hm*sin(q))^2);
lb=L/2;
p= acos((b/2-hm*cos(q))/a);
qa = acos(lb/2/a); %théorème triangle isocèle

qb = p-qa;
qr = 90+qa-qb;
dr = b/2-R*cos(qr)-lb*cos(qb)


