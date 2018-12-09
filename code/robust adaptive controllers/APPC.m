% Federal University of Rio Grande do Norte
% Title: Adaptive Pole Placement Control
% Author: Danilo Pena
% Description: APPC with a first order plant

clear
close all
clc

h = 0.01; 
t = 0:h:8; 
n = max(size(t)); 

%% Initialization

dy_o = zeros(1,n-1);
y_o = zeros(1,n-1);
e_o = zeros(1,n-1);
u_o = zeros(1,n-1);

% Variables
r = 1;
y = 0; 
y_ch = 0;
e = 0;
de = 0;
b = 1;
a = -1;
am = 1;
a_ch = -1;
b_ch = 1;
u = 0;
gama1 = 5;
gama2 = 5;

% Controller parameter
% Characteristic Polynomial: A(s)=(s+1)^2

%% Loop

for  k=1:n
    if b_ch==0
        b_ch = 0.1;
    end

    % Plant  =>  b/(s+a)
    dy = -a*y + b*u;
    dy_o(k) = dy;
    
    y = dy*h + y;
    y_o(k) = y;
    
    % Control Law
    p1 = (2-a_ch)/b_ch;
    p0 = 1/b_ch;
    
    du = p1*de + p0*e;
    u = du*h + u;
    u_o(k) = u;
    
    % Error
    e = r - y;
    e_o(k) = e;  
    
    if (k>1)
        de = (e-e_o(k-1))/h;
    else
        de = (e-0)/h;     
    end
    de_o(k) = de;
    
    % Estimation
    dy_ch = -a_ch*y_ch + b_ch*u;
    dy_ch_o(k) = dy_ch;
    
    y_ch = dy_ch*h + y_ch;
    y_ch_o(k) = y_ch;
    
    % Estimation error
    e0 = y - y_ch;
    
    % Adaptive Law
    da_ch = - gama1*e0*y;
    a_ch = da_ch*h + a_ch;
    
    db_ch = gama2*e0*u;
    b_ch = db_ch*h + b_ch;
    
    t_o(k) = k*h;
end

%% Plots

figure (1)
plot(t_o,y_o,t_o,r)
figure (2)
plot(t_o,u_o)
%figure (3)
%plot(t_o,eo_o)
