% Federal University of Rio Grande do Norte
% Title: Model Reference Adaptive Control (MRAC) - Gradient method
% Author: Danilo Pena
% Description: MRAC for a Permanent Magnet Synchronous Generator (PMSG)

%%
clear
close all
clc

%% Init
ts = 20;
h = 0.001;
n = ts/h;

% vector
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
d2yf_o = zeros(1,n-1);
dyf_o = zeros(1,n-1);
yf_o = zeros(1,n-1);
u_o = zeros(1,n-1);
uf_o = zeros(1,n-1);
duf_o = zeros(1,n-1);
d2uf_o = zeros(1,n-1);
ye_o = zeros(1,n-1);
ee_o = zeros(1,n-1);

% variables
r = 1;
dym = 0;
ym = 0;
dy = 0;
y = 0;
eo = 0;
theta1 = 3;
a1 = 4;
dyf = 0;
yf = 0;
duf = 0;
u = 0;
uf = 0;
gamma1 = 2;
gamma2 = 50;

%% Loop
for k = 1:n-1
    % Reference model
    dym = -0.4*ym + r;
    dym_o(k) = dym;
    
    ym = dym*h + ym;
    ym_o(k) = ym;

    % Plant
    dy = - a1*y + u;
    dy_o(k) = dy;
    
    y = dy*h + y;
    y_o(k) = y;
    
    % Control law
    u = theta1*y + r;
    u_o(k) = u;
    
    % Error
    eo = ym - y;
    eo_o(k) = eo;
    
    % Filter
    dyf = - 0.4*yf - y;
    dyf_o(k) = dyf;
    
    yf = dyf*h + yf;
    yf_o(k) = yf;
    
    duf = - 0.4*uf + u;
    duf_o(k) = duf;
    
    uf = duf*h + uf;
    uf_o(k) = uf;
    
       
    % Estimation of Y
    ye = theta1*yf + uf;
        
    % Estimation error
    ee = y-ye;
    ee_o(k) = ee;
    
    % Parameters estiomation
    dtheta1 = gamma1*ee*yf;
    
    theta1 = dtheta1*h + theta1;
    theta1_o(k) = theta1;
 
    t_o(k) = k*h;
end

%% Plots
figure (1)
plot(t_o,y_o,t_o,ym_o)
figure (2)
plot(t_o,theta1_o)