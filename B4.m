% File Name: B3.m
% Date: March 2, 2019
% Description: numerical and feasibility calculation/finding optimal solution

clc; 
clear;

global Sc;

%Input V, l, w, h, r
Inputv = [48, 0.5, 0.3, 0.3, 0.68];
V = Inputv(1);
l = Inputv(2);
w = Inputv(3);
h = Inputv(4);
r = Inputv(5);

%Parameter
rho = 800; 
mu = 0.9;
R = 1.53; 
m = 2.5; 
g = 9.81;
k = 29.4;
d = 0.2;
h_c = 0.3;

%Equation
V_c = V;
M = rho*w*l*h;
f = mu*(M+m)*g; 
T_max = V*k/R;
a_max = (4*T_max*d/2-f)/(M+m);
v = d*(V*k-f*R/(2*d))/(2*k^2);
M_x = (M+m)*g*l/2-m*a_max*h_c/2-M*a_max*(h/2+h_c);
M_y = (M+m)*g*l/2-m*v^2*h_c/(2*r)-M*v^2*(h/2+h_c)/r;
s = dubins(0,0,0,17,23,pi,r,0);
t = s/v; 
l_c = l;
w_c = w;
v_c = v;

%l_c, w_c <= 1m, V <= 48V, M_x, M_y, v_c >=0, max M, min t
outputv = [l_c, w_c, V_c, M_x, M_y, v_c, M, t];

%Objective function
E = M*60/t;

Sc = [1, 1, 1, 1, 1];
A = [];
b = [];
Aeq = [];
beq = [];
lb = [0.1, 0.01, 0.01, 0.01, 0.5];
ub = [];
fun = @optE;
nonlcon = @drone;

[x, fval, exitflag, output, lambda, grad, hessian] = fmincon(fun, Inputv, A, b, Aeq, beq, lb, ub, nonlcon);
E = 1/fval;
Sc = [0.08062257748, 8.9166933, 11.14262088, 14.91030181, 0.1039230485];
Inputv_Sc = Sc.*Inputv;
lb = lb.*Sc;
[xs, fvals, exitflags, outputs, lambdas, grads, hessians] = fmincon(fun, Inputv_Sc, A, b, Aeq, beq, lb, ub, nonlcon);
xn = xs./Sc;
Es = 1/fvals

optionsga = optimoptions('ga', 'Display', 'iter', 'PopulationSize', 500, 'MaxGenerations', 50,'PlotFcn',@gaplotbestf , 'FunctionTolerance',1e-16, 'ConstraintTolerance', 1e-16);
[xg, fvalg] = ga(fun, 5, A, b, Aeq, beq, lb, ub, nonlcon, optionsga);
Eg = 1/fvalg
