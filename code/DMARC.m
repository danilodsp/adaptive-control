% Federal University of Rio Grande do Norte
% Title: Dual Mode Adaptive Robust Controller (DMARC)
% Author: Danilo Pena
% Description: DMARC with a first order plant

clear
close all
clc

h = 0.0001; 
t = 0:h:10; 
n = max(size(t)); 

%% Initialization

t_o = zeros(1,n-1);
dym_o = zeros(1,n-1);
ym_o =  zeros(1,n-1);
dy_o = zeros(1,n-1);
y_o = zeros(1,n-1);
eo_o = zeros(1,n-1);
dtheta1_o = zeros(1,n-1);
theta1_o = zeros(1,n-1);
dtheta2_o = zeros(1,n-1);
theta2_o = zeros(1,n-1);
u_o = zeros(1,n-1);

% Variables
input_step = ones(1,n);
u = 0;
ym = 0;
y = 0.5;
eo = 0;
am = 1;
km = 1;
ap = -1;
kp = 1;
theta1 = 0;
theta2 = 0;
theta1_estimated = 2.3;
theta2_estimated = 1.3;
sigma = 1;
L = 0.000005;
mi = 0;

%% Loops

for  k=1:n

    r = input_step(k);

    % Reference model
    dym = - am*ym + km*r;
    dym_o(k) = dym;

    ym = dym*h + ym;
    ym_o(k) = ym;

    % Plant
    dy = - ap*y + kp*u;
    dy_o(k) = dy;

    y = dy*h + y;
    y_o(k) = y;

    % Control Law
    u = theta1*y + theta2*r;
    u_o(k) = u;

    % Error
    eo = y - ym;
    eo_o(k) = eo;  

    % Controller parameters
    mi = exp(-((eo*eo)/L));
    mi_o(k) = mi;
    %mi=0;
    
        if mi>0 % Dual Mode
            dtheta1 = (-sigma*theta1 - sigma*theta1_estimated*sign(eo*y))/mi;
            dtheta2 = (-sigma*theta2 - sigma*theta2_estimated*sign(eo*r))/mi;

            theta1 = dtheta1*h + theta1;
            theta2 = dtheta2*h + theta2;
        else  % VS-MRAC
            theta1 = - theta1_estimated*sign(eo*y);
            theta2 = - theta2_estimated*sign(eo*r);
        end 

    theta1_o(k) = theta1;
    theta2_o(k) = theta2;

    t_o(k) = k*h;
end

%% Plots

figure (1)
plot(t_o,y_o,t_o,ym_o)

figure (2)
plot(t_o,theta1_o)

%figure (3)
%plot(t_o,teta2_o)

%figure (4)
%plot(t_o,u_o)

%figure (5)
%plot(t_o,eo_o)

figure (3)
plot(t_o,mi_o)

figure (4)
plot(t_o,eo_o)