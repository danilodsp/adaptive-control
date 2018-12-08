% Federal University of Rio Grande do Norte
% Title: Dual Mode Adaptive Robust Controller (DMARC)
% Author: Danilo Pena
% Description: DMARC with a first order plant

clear
close all
clc

h = 0.0001; 
t = 0:h:20; 
n = max(size(t)); 

%% Initialization

t_o = zeros(1,n-1);
dym_o = zeros(1,n-1);
ym_o =  zeros(1,n-1);
dy_o = zeros(1,n-1);
y_o = zeros(1,n-1);
eo_o = zeros(1,n-1);
dteta1_o = zeros(1,n-1);
teta1_o = zeros(1,n-1);
dteta2_o = zeros(1,n-1);
teta2_o = zeros(1,n-1);
u_o = zeros(1,n-1);

% Variables
input_step = ones(1,n);
u = 0;
ym = 0;
y = 0.5;
eo = 0;
am = 1;
km = 1;
ap = -1.1;
kp = 1;
teta1 = 0;
teta2 = 0;
teta1barra = 2.3;
teta2barra = 1.3;
%teta1estrela = -2;
%teta2estrela = 1;
sigma = 0.9;
L = 0.1;
mi = 1;

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
    u = teta1*y + teta2*r;
    u_o(k) = u;
    
    % Error
    eo = y - ym;
    eo_o(k) = eo;  
   
    % Controller parameters
    mi = 1- exp(-((eo*eo)/L));
    mi_o(k) = mi;
    %mi=1;
    
    if mi<0.1 && mi>=1 % MRAC
        %dteta1 = -(sigma/mi)*teta1 - (sigma/mi)*teta1barra*sign(eo*y);
        %dteta2 = -(sigma/mi)*teta2 - (sigma/mi)*teta2barra*sign(eo*r);
        dteta1 = -sigma*teta1 - sigma*teta1barra*sign(eo*y);
        dteta2 = -sigma*teta2 - sigma*teta2barra*sign(eo*r);
        teta1 = dteta1*h + teta1;
        teta2 = dteta2*h + teta2;
    else  % VS-MRAC
        teta1 = - teta1barra*sign(eo*y);
        teta2 = -teta2barra*sign(eo*r);
    end 
    
    teta1_o(k) = teta1;
    teta2_o(k) = teta2;
    
    t_o(k) = k*h;
end

%% Plots

figure (1)
plot(t_o,y_o,t_o,ym_o)

figure (2)
plot(t_o,teta1_o)

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