% Federal University of Rio Grande do Norte
% Title: Model Reference Adaptive Control (MRAC) - Gradient method
% Author: Danilo Pena

%%
clear
close all
clc

%% Init
ts = 6;
h = 0.001;
n = ts/h;

% vector init
t_o = zeros(1,n-1);
d2ym_o =  zeros(1,n-1);
dym_o = zeros(1,n-1);
ym_o =  zeros(1,n-1);
d2y_o =  zeros(1,n-1);
dy_o = zeros(1,n-1);
y_o = zeros(1,n-1);
eo_o = zeros(1,n-1);
dtheta1_o = zeros(1,n-1);
theta1_o = zeros(1,n-1);
dtheta2_o = zeros(1,n-1);
theta2_o = zeros(1,n-1);
dtheta3_o = zeros(1,n-1);
theta3_o = zeros(1,n-1);
d2yf1_o = zeros(1,n-1);
dyf1_o = zeros(1,n-1);
yf1_o = zeros(1,n-1);
d2yf2_o = zeros(1,n-1);
dyf2_o = zeros(1,n-1);
yf2_o = zeros(1,n-1);
u_o = zeros(1,n-1);
uf_o = zeros(1,n-1);
duf_o = zeros(1,n-1);
d2uf_o = zeros(1,n-1);
ye_o = zeros(1,n-1);
ee_o = zeros(1,n-1);
    
% variables init
r = 1;
dym = 0;
ym = 0;
dy = 0;
y = 0;
eo = 0;
theta1 = -1.4;
theta2 = -8.2;
theta3 = 2.1;
a1 = 0;
a2 = -23.967;
a3 = 3.9;
dyf1 = 0;
yf1 = 0;
dyf2 = 0;
yf2 = 0;
duf = 0;
u = 0;
uf = 0;
gamma1 = 1;
gamma2 = 17.9;
gamma3 = 5;

%% Loop
for  k=1:n-1

    % Reference model
    d2ym = - 6*dym - 9*ym + 9*r;
    d2ym_o(k) = d2ym;
    
    dym = d2ym*h + dym;
    dym_o(k) = dym;
    
    ym = dym*h + ym;
    ym_o(k) = ym;
    
    % Control law
    u = theta1*dy + theta2*y + theta3*r;
    u_o(k) = u;

    % plant
    d2y = - a1*dy - a2*y + a3*u;
    d2y_o(k) = d2y;
    
    dy = d2y*h + dy;
    dy_o(k) = dy;
    
    y = dy*h + y;
    y_o(k) = y;
    
    % error
    eo = y - ym;
    eo_o(k) = eo;
    
    % filter
    d2yf1 = - 6*dyf1 - 9*yf1 - a3*dy;
    d2yf1_o(k) = d2yf1;
    
    dyf1 = d2yf1*h + dyf1;
    dyf1_o(k) = dyf1;
    
    yf1 = dyf1*h + yf1;
    yf1_o(k) = yf1;
    
    d2yf2 = - 6*dyf2 - 9*yf2 - a3*y;
    d2yf2_o(k) = d2yf2;
    
    dyf2 = d2yf2*h + dyf2;
    dyf2_o(k) = dyf2;
    
    yf2 = dyf2*h + yf2;
    yf2_o(k) = yf2;
    
    d2uf = - 6*duf - 9*uf + 9*u;
    d2uf_o(k) = d2uf;
    
    duf = d2uf*h + duf;
    duf_o(k) = duf;
    
    uf = duf*h + uf;
    uf_o(k) = uf;
       
    % estimation of Y
    ye = theta1*yf1 + theta2*yf2 + (1/theta3)*uf;
        
    % estimation error
    ee = y-ye;
    ee_o(k) = ee;
    
    % parameters estimation
    dtheta1 = gamma1*ee*yf1;
    dtheta2 = gamma2*ee*yf2;
    dtheta3 = -gamma3*ee*uf*(1/theta3^2);
    
    theta1 = dtheta1*h + theta1;
    theta1_o(k) = theta1;
    theta2 = dtheta2*h + theta2;
    theta2_o(k) = theta2;
    theta3 = dtheta3*h + theta3;
    theta3_o(k) = theta3;
    t_o(k) = k*h;
end

%% Plots
figure (1)
plot(t_o,y_o,t_o,ym_o)
figure (2)
plot(t_o,theta1_o)
figure (3)
plot(t_o,theta2_o)
figure (4)
plot(t_o,theta3_o)
figure (5)
plot(t_o,u_o)