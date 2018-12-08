% Federal University of Rio Grande do Norte
% Title: Pole Placement Control
% Author: Danilo Pena
% Description: Pole placement control with a serie-parallel model

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
a_ch = -1.1;
b_ch = 0.9;
u = 0;
gama1 = 1;
gama2 = 1;
b0 = 0.1;
e0 = 0;
a_tio = a_ch - a;
b_tio = b_ch - b;

% Controller parameter
% Characteristic Polynomial: A(s)=(s+1)^2

%% Loop

for  k=1:n

    % Plant  =>  b/(s+a)
    dy = -a*y + b*u;
    dy_o(k) = dy;
    
    y = dy*h + y;
    y_o(k) = y;
    
    % Control Law
    u = -k*y + r;
    k_ch = (a_ch+am)/b_ch;
    
    % Serie-parallel model
    de0 = -am*e0 - a_tio*y - b_tio*u;
    e0 = de0*h + e0;
    e0_o(k) = e0;
    
    %a_tio = a_ch - a;
    %b_tio = b_ch - b;
    
    da_tio = gama1*e0*y;
    a_tio = da_tio*h + a_tio;
    
    if abs(b_ch)>b0 
        %if abs(b_ch)= b0 && sgn(b_ch)*e0*u>=0
        db_tio = gama2*e0*u;
    else 
        db_tio = 0;
    end
    
    b_tio = db_tio*h + b_tio;
    
    a_ch = a_tio + a;
    b_ch = b_tio + b;
     
    % Error
    e = r - y;
    e_o(k) = e;  
    
    if(k>1)
        de = (e-e_o(k-1))/h;
    else
        de = (e-0)/h;     
    end
    
    de_o(k)=de;
    t_o(k) = k*h;
end

%% Plots

figure (1)
plot(t_o,y_o,t_o,r)
figure (2)
plot(t_o,u_o)
%figure (3)
%plot(t_o,eo_o)