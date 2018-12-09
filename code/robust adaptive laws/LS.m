% Federal University of Rio Grande do Norte
% Title: Least Squares (LS) pure
% Author: Danilo Pena

%%
clc
clear
close all

%% Init

% Time 
h = 0.01; 
t = 0:h:5; 
m = max(size(t)); 

% Reals parameters
b0 = 3.9 * ones(1,m);  
a1 = 0 * ones(1,m); 
a0 = -23.967 * ones(1,m);

% Reference
signal = ones(1,m); % Const input
r = 1;
%signal = 1+sin(1.5*t)+sin(6*t);

% States (plant)
x1_o = zeros(1,m); 
x2_o = zeros(1,m); 
 
%gamma = [2.3 0 0; 0 4.8 0; 0 0 150]; % Const input
gamma = [1.655 0 0; 0 3.65 0; 0 0 150];
theta = [3.8; 0.05; -23.7];
b0e = theta(1); 
a1e = theta(2); 
a0e = theta(3);
z_o = zeros(1,m); 
ze_o = zeros(1,m); 
ep_o = zeros(1,m);
dep_o = zeros(1,m);
d2ep_o = zeros(1,m);
ee_o = zeros(1,m);
dee_o = zeros(1,m);
d2ee_o = zeros(1,m);
d2up_o = zeros(1,m);
dup_o = zeros(1,m);
up_o = zeros(1,m);
d2ue_o = zeros(1,m);
due_o = zeros(1,m);
ue_o = zeros(1,m);
z=0;
ze=0;

% variables
x1(1) = 0;
x2(1) = 0;
x1f(1) = 0;
x2f(1) = 0; 
dx1f(1) = 0;
dx2f(1) = 0;
dx2fp(1) = 0;
x2p(1) = 0;
dx1fp(1) = 0;
x1p(1) = 0;
dx1ff(1) = 0;
dx2ff(1) = 0;
x1up(1) = 0;
x2up(1) = 0;
x1ue(1) = 0;
x2ue(1) = 0;
p = 100*(eye(3)); 
dup(1) = 0;
up(1) = 0;
due(1) = 0;
ue(1) = 0;
yfp = 0;
dyfe = 0;
yfe = 0;

%% Loop
for i = 1:m 

    % Plant
    x2 = x2 + h*(3.9*up - 0*x2 + 23.967*x1);
    x1 = x1 + h*x2;
    y = x1;

    % vectors
    x1_o(i) = x1;
    x2_o(i) = x2;
    y_o(i) = y;

    % Filter
    x2f = x2f + h*(y - 2*x2f - x1f);
    x1f = x1f + h*x2f;
    yf = x1f;

    % vectors
    x1f_o(i) = x1f;
    x2f_o(i) = x2f;
    yf_o(i) = yf;

    % Filter
    dx2fp = dx2fp + h*(x2p - 2*dx2fp - dx1fp);
    dx1fp = dx1fp + h*dx2fp;
    dyfp = dx1fp;

    % vectors
    dx1fp_o(i) = dx1fp;
    dx2fp_o(i) = dx2fp;
    dyfp_o(i) = dyfp;

    % Filter
    x2up = x2up + h*(up - 2*x2up - x1up);
    x1up = x1up + h*x2up;
    ufp = x2up;

    % vectors
    x1up_o(i) = x1up;
    x2up_o(i) = x2up;
    ufp_o(i) = ufp;

    % Estimation
    x2ue = x2ue + h*(ue - 2*x2ue - x1ue);
    x1ue = x1ue + h*x2ue;
    ufe = x2ue;

    % vectors
    x1ue_o(i) = x1ue;
    x2ue_o(i) = x2ue;
    ufe_o(i) = ufe;

    % Estiomation
    fip = [ufp; -dyfp; -yfp]; 
    fie = [ufe; -dyfe; -yfe]; 
    z = [3.9 0 -23.967]* fip;
    ze = theta' * fie; 

    % vectors
    z_o(i) = z;
    ze_o(i) = ze;

    % Estimation error
    ns2 = fie' * fie; 
    m2 = 1 + ns2; 
    erro = (z - ze)/m2; 
    erro_o(i) = erro;

    % Parameters estimation
    p = p + h * (-(p*((fie * fie')/m2))*p);

    theta = theta + h * (p * gamma * erro * fie); 

    b0e = theta(1); 
    a1e = theta(2); 
    a0e = theta(3);
    % vectors
    a1e_o(i) = a1e;
    a0e_o(i) = a0e;
    b0e_o(i) = b0e;

    % Error
    % Plant
    ep = r-z;
    if(i>1)
        dep = (ep-ep_o(i-1))/h;
        d2ep = (dep-dep_o(i-1))/h;
    else
        dep = (ep-0)/h;
        d2ep = (dep-0)/h;
    end

    ep_o(i)=ep;
    dep_o(i)=dep;
    d2ep_o(i)=d2ep;

    % Estimation
    ee = r-ze;
    if(i>1)
        dee = (ee-ee_o(i-1))/h;
        d2ee = (dee-dee_o(i-1))/h;
    else
        dee = (ee-0)/h;
        d2ee = (dee-0)/h;
    end

    ee_o(i)=ee;
    dee_o(i)=dee;
    d2ee_o(i)=d2ee;

    % Control signal
    l1p=1;
    l0p=36-a1(1);
    p2p=((414-a0(1)-36*a1(1)+a1(1)^2)/b0(1));
    p1p=((1620-36*a0(1)+a1(1)*a0(1))/b0(1));
    p0p=2025/b0(1);

    l1e=1;
    l0e=36-a1e;
    p2e=((414-a0e-36*a1e+a1e^2)/b0e);
    p1e=((1620-36*a0e+a1e*a0e)/b0e);
    p0e=2025/b0e;

    % Plant
    d2up = -(l0p/l1p)*dup+(p2p/l1p)*d2ep+(p1p/l1p)*dep+(p0p/l1p)*ep;
    dup = d2up*h + dup;
    up = dup*h + up;

    % vectors
    d2up_o(i) = d2up;
    dup_o(i) = dup;
    up_o(i) = up;

    % Estimation
    d2ue = -(l0e/l1e)*due+(p2e/l1e)*d2ee+(p1e/l1e)*dee+(p0e/l1e)*ee;
    due = d2ue*h + due;
    ue = due*h + ue;

    % vectors
    d2ue_o(i) = d2ue;
    due_o(i) = due;
    ue_o(i) = ue;

end
 
%% Plots
% figure(1); 
% plot(t, a1, '-r','linewidth', 3);
% hold on; 
% plot(t, a1e_o, '-b','linewidth', 3); 
% xlabel('t (s)'); 
% ylabel('a1'); 
% legend('a_1 Real', 'a_1 Estimated'); 
% grid on;
 
% figure(2); 
% plot(t, a0, '-r','linewidth', 3);
% hold on; 
% plot(t, a0e_o, '-b','linewidth', 3); 
% xlabel('t (s)'); 
% ylabel('a0'); 
% legend('a_0 Real', 'a_0 Estimated'); 
% grid on; 
 
% figure(3); 
% plot(t, b0, '-r','linewidth', 3);
% hold on; 
% plot(t, b0e_o, '-b','linewidth', 3); 
% xlabel('t (s)'); 
% ylabel('b0'); 
% legend('b_0 Real', 'b_0 Estimated'); 
% grid on; 
 
figure(1);
%plot(t, z_o', '-r', 'linewidth', 3);
hold on; 
plot(t, ze_o', '-b','linewidth', 3); 
xlabel('t (s)'); 
ylabel('Saída'); 
legend('Real output', 'Estimated output'); 
grid on; 
 
figure(2); 
plot(t, erro_o, '-r','linewidth', 3);
xlabel('t (s)'); 
ylabel('Error'); 
legend('Error'); 
grid on; 
