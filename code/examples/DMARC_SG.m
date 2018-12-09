% Federal University of Rio Grande do Norte
% Title: DMARC - SG
% Author: Danilo Pena
% Description: DMARC with a synchronous generator

clear
close all
clc

h = 0.0001; 
t = 0:h:30; 
n = max(size(t)); 

%% Initialization
delta_o = zeros(1,n-1);
ddelta_o = zeros(1,n-1);
phif_o = zeros(1,n-1);
Efd_o = zeros(1,n-1);
Pm_o = zeros(1,n-1);
Pg_o = zeros(1,n-1);
Pe_o = zeros(1,n-1);
Vt_o = zeros(1,n-1);
ug_o = zeros(1,n-1);
ue_o = zeros(1,n-1);
input_step = ones(1,n);

% Variables
delta = 0;
ddelta = pi;
phif = 0;
Efd = 0;
Pm = 0;
Pg = 0;
ug = 0;
ue = 0;
theta1_estimated = 0.5;
theta2_estimated = 0.9;
sigma = 1.5;
L = 0.01;
mi = 0;
ym = 0;
theta1 = 0;
theta2 = 0;

% Parameters of the synchronous generator
d = 0.006; % Damping Constant (s)
H = 3.82; % Inertia Constant (s)
xd = 1.75; % Direct Shaft Armature Reactance (pu)
xq = 1.68; % Quadrature Shaft Armature Reactor (pu)
xaf = 1.56; % Mutual Reactance of Direct Shaft (pu)
xt = 0.3; % Transmission Line Reactivity (pu)
xf = 1.665; % Reactance of Field (pu)
rf = 0.0012; % Field Strength (pu)
Te = 0.04; % Field Excitation Time Constant (s)
ke = 25; % Field Excitation Gain (dimensionless)
Tt = 0.3; % Turbine Time Constant (s)
Tg = 0.08; % Governor Time Constant (s)
f = 60; % Frequency (Hz)
V = 1; % Infinite Bus Voltage

xdl = 0.286;
%xdl = xd - ((xaf*xaf) / xf); % Direct shaft transient reactance
wo = 2*pi*f; % Synchronous speed

p1 = (wo*V*V*(xq-xdl))/(4*H*(xt+xdl)*(xt+xq));
p2 = (wo*d)/(2*H);
p3 = (wo*V*xaf)/(2*H*xf*(xt+xdl));
p4 = wo/(2*H);
p5 = (wo*rf*V*xaf)/(xf*(xt+xdl));
p6 = (wo*rf*(xt+xd))/(xf*(xt+xdl));
p7 = (wo*rf)/xf;

for  k=1:n
    % Inputs
    rg = (30*pi/180) * input_step(k);
    % Disturbances
    % if k>250001
    % rg = (5*pi/180) * input_step(k);
    % end
    % if k>500001
    % rg = (32*pi/180) * input_step(k);
    % end
    rg_o(k) = rg;

    re = 1.2* input_step(k);
    % if k>600000
    %     re = 1*input_step(k);
    % end
    % if k>850000
    %     re = 1.2*input_step(k);
    % end
    re_o(k) = re;

    % Sensitivity to Parametric Disorders and Variations
    % Disturbance = 10% of the inputs
    % rg = (30*pi/180) * input_step(k);
    % if k>550001
    %     rg = 1.1*(30*pi/180) * input_step(k); % 10%
    % end
    % rg_o(k) = rg;
    % re = 0.82* input_step(k);
    % if k>550001
    %     re = 1.1*0.82* input_step(k);  % 10%
    % end
    % re_o(k) = re;

    % Parametric variation
    % if k>250001
    %     xd = 1.1*1.75;
    %     xq = 1.1*1.68;
    %     xf = 1.1*1.665;
    % end

    % PID to load angle
    eg = rg-delta;% error between load angle and reference (PID input)
    eg_o(k)=eg;
    if k>1
    deg = (eg-eg_o(k-1))/h;
    d2eg = (deg - deg_o(k-1))/h;
    else
    deg = (eg-0)/h;
    d2eg = (deg - 0)/h;
    end
    deg_o(k) = deg;

    dug = 0.00003*d2eg + 0.07*deg + 0.2333*eg; % controller
    ug = dug*h + ug;
    ug_o(k) = ug;

    % % PI controller to field flow #################################
    % % ee = re-phif; % error between field flow and reference (PI input)
    % % ee_o(k) = ee;
    % % if k>1
    % % dee = (ee-ee_o(k-1))/h;
    % % else
    % %     dee = (ee-0)/h;
    % % end
    % % due = 0.086*dee + 0.1540*ee;
    % % ue = due*h + ue;
    % % ue_o(k) = ue;

    % DMARC to field flow
    % Reference model
    dym = - 1*ym + 1*re;
    dym_o(k) = dym;

    ym = dym*h + ym;
    ym_o(k) = ym;

    % Error
    eo = phif - ym;
    eo_o(k) = eo;  

    % Control Law
    ue = theta1*phif + theta2*re;
    ue_o(k) = ue;

    % Controller parameters
    mi = exp(-((eo*eo)/L));
    mi_o(k) = mi;
    %mi = 0;
    if mi>0 % Dual mode
        dtheta1 = (-sigma*theta1 - sigma*theta1_estimated*sign(eo*phif))/mi;
        dtheta2 = (-sigma*theta2 - sigma*theta2_estimated*sign(eo*re))/mi;

        theta1 = dtheta1*h + theta1;
        theta2 = dtheta2*h + theta2;
    else  % VS-MRAC
        theta1 = - theta1_estimated*sign(eo*phif);
        theta2 = - theta2_estimated*sign(eo*re);   
    end 

    theta1_o(k) = theta1;
    theta2_o(k) = theta2;


    % Synchronous machine
    % if(delta>2*pi)
    %     delta = delta-2*pi;
    % end
    d2delta = p1*sin(2*delta) - p2*ddelta - p3*phif*sin(delta) + p4*Pm;
    ddelta = d2delta*h + ddelta;
    delta = ddelta*h + delta;
    d2delta_o(k) = d2delta;
    ddelta_o(k) = ddelta;
    delta_o(k) = delta;


    dphif = p5*cos(delta) - p6*phif + p7*Efd;
    phif = dphif*h + phif;
    phif_o(k) = phif;

    % Excitation system
    dEfd = (-Efd + ke*ue)/Te;
    Efd = dEfd*h + Efd;
    Efd_o(k) = Efd;

    % Turbine and governor
    dPm = (-Pm + Pg)/Tt;
    Pm = dPm*h + Pm;
    Pm_o(k) = Pm;

    dPg = (-Pg + ug)/Tg;
    Pg = dPg*h + Pg;
    Pg_o(k) = Pg;

    % Power generated
    Pe = (p3*phif*sin(delta) - p1*sin(2*delta))/p4;
    Pe_o(k) = Pe;

    % Terminal voltage
    Vt = sqrt((V*xq*sin(2*delta)/(xt+xq))^2 + (((V*xdl*cos(delta))/(xt+xdl))+((xaf*xt*phif)/((xt+xdl)*xf)))^2);
    Vt_o(k) = Vt;

    % Speed
    w1 = wo - ddelta;
    w1_o(k) = w1;

    t_o(k) = k*h;
end

%% Plots

figure (1)
plot(t,delta_o)
hold on
plot(t_o,rg_o,'r')
plot(t_o,phif_o,'k')
hold on
plot(t_o,re_o,'g')

figure (2)
plot (t_o,ue_o)
hold on
plot (t_o,ug_o,'r')

% figure (3)
% plot(t_o,Vt_o,t_o,Pe_o, t_o,Pg_o, t_o,Pm)
% 
% figure (4)
% plot(t_o,Efd)