% Federal University of Rio Grande do Norte
% Title: Synchronous Generator
% Author: Danilo Pena
% Description: SG modelling

clear
close all
clc

% Parameters
d = 0.001; % Damping Constant (s)
H = 4; % Inertia Constant (s)
xd = 1.7; % Direct Shaft Armature Reactance (pu)
xq = 1.64; % Quadrature Shaft Armature Reactor (pu)
xdd = 0.2439; % Direct Axis Transient Reactivity (pu)
xaf = 1.49; % Mutual Reactance of Direct Axis (pu)
xt = 0.3; % Transmission Line Reactivity (pu)
xf = 1.65; % Reactance of Field (pu)
rf = 0.000742; % Field Strength (pu)
Te = 0.05; % Field Excitation Time Constant (s)
ke = 40; % Field Excitation Gain
Tt = 0.15; % Turbine Time Constant (s)
Tg = 0.5; % Governor Time Constant (s)
f = 60; % Frequency (Hz)
wo = 2*pi*f; % Synchronous Speed (rad / s)
V = 1; % Infinite Bus Voltage (pu)
xdd = xd - ((xaf^2) / xf); % Direct Axis Transient Reactivity (pu)

% Parameters 2
d = 0.002; % Damping Constant (s)
H = 2; % Inertia Constant (s)
xd = 1.6; % Direct Shaft Armature Reactance (pu)
xq = 1.7; % Quadrature Shaft Armature Reactor (pu)
xaf = 1.6; % Mutual Reactance of Direct Shaft (pu)
xt = 0.3; % Transmission Line Reactivity (pu)
xf = 1.6; % Reactance of Field (pu)
rf = 0.0012; % Field Strength (pu)
Te = 0.04; % Field Excitation Time Constant (s)
ke = 40; % Field Excitation Gain (dimensionless)
Tt = 0.15; % Turbine Time Constant (s)
Tg = 0.03; % Governor Time Constant (s)
f = 60; % Frequency (Hz)
V = 1; % Infinite Bus Voltage
xdd = xd - ((xaf*xaf) / xf); % Direct shaft transient reactance
wo = 2*pi*f; % Synchronous speed

p1 = (wo*(V^2)*(xq-xdd)) / (4*H*(xt+xdd)*(xt+xq));
p2 = (wo*d) / (2*H);
p3 = (wo*V*xaf) / (2*H*xf*(xt+xdd));
p4 = (wo) / (2*H);
p5 = (wo*rf*V*xaf) / (xf*(xt+xdd));
p6 = (wo*rf*(xt+xd)) / (xf*(xt+xdd));
p7 = (wo*rf) / (xf);

% Initialization
h = 0.0001;
t = 0:h:20;
n = length(t);

ue = ones(1,n); % Signal Acting on Field Excitation (pu)
ug = ones(1,n); % Signal Acting on Governor's Valve (pu)
w = pi*ones(1,n); % Frequency disturbance (rad / s)
delta = zeros(1,n); % Load Angle (rad)
Phif = zeros(1,n); % Field Flow (pu)
Pm = zeros(1,n); % Mechanical Power in the Axis (pu)
Pg = zeros(1,n); % Mechanical Output Power of the Governor (pu)
Efd = zeros(1,n); % Field Voltage (pu)
Pe = zeros(1,n); % Generated Electric Power (pu)
Vt = zeros(1,n); % Terminal voltage (pu)
% Te = Te.*ones(1,n);
% Tg = Tg.*ones(1,n);

% Initial conditions
delta0 = pi;
w0 = 0;
Phif0 = 0;
Efd0 = 0;
Pm0 = 0;
Pg0 = 0;

for k=1:n
    
    % Input
    if k>5180
        ue(k) = 1;
    else
        ue(k) = 0;
    end
    
    if k>1
        a1 = w(k-1);
        a2 = p1*sin(2*delta(k-1)) - p2*w(k-1) - p3*Phif(k-1)*sin(delta(k-1)) + p4*Pm(k-1);
        a3 = p5*cos(delta(k-1)) - p6*Phif(k-1) + p7*Efd(k-1);
        a4 = -Efd(k-1)/Te;
        a5 = (-Pm(k-1) + Pg(k-1))/Tt;
        a6 = -Pg(k-1)/Tg;
    else
        a1 = w0;
        a2 = p1*sin(2*delta0) - p2*w0 - p3*Phif0*sin(delta0) + p4*Pm0;
        a3 = p5*cos(delta0) - p6*Phif0 + p7*Efd0;
        a4 = -Efd0/Te;
        a5 = (-Pm0 + Pg0)/Tt;
        a6 = -Pg0/Tg;
    end

    A = [a1 a2 a3 a4 a5 a6]';
    B = [0 0; 0 0; 0 0; (ke/Te) 0; 0 0; 0 (1/Tg)];
    U = [ue(k); ug(k)];

    Xd = A + B*U;
    
    % Integrating Xd and calculating X
    if k>1
        delta(k) = Xd(1)*h + delta(k-1);
        w(k) = Xd(2)*h + w(k-1);
        Phif(k) = Xd(3)*h + Phif(k-1);
        Efd(k) = Xd(4)*h + Efd(k-1);
        Pm(k) = Xd(5)*h + Pm(k-1);
        Pg(k) = Xd(6)*h + Pg(k-1);
    else
        delta(k) = Xd(1)*h + delta0;
        w(k) = Xd(2)*h + w0;
        Phif(k) = Xd(3)*h + Phif0;
        Efd(k) = Xd(4)*h + Efd0;
        Pm(k) = Xd(5)*h + Pm0;
        Pg(k) = Xd(6)*h + Pg0;
    end
    
    X = [delta(k) w(k) Phif(k) Efd(k) Pm(k) Pg(k)]';
    y = [Phif(k); delta(k)];
    
    % Calculation of Electric Power Generated (Pe) and Terminal Voltage (Vt)
    Pe(k) = (p3*Phif(k)*sin(delta(k)) - p1*sin(2*delta(k))) / (p4);
    Vt(k) = sqrt(((V*xq*sin(delta(k))) / (xt+xq))^2 + (((V*xdd*cos(delta(k))) / (xt+xdd)) + ((xaf*xt*Phif(k)) / (xt*xf+xdd*xf)))^2);
end