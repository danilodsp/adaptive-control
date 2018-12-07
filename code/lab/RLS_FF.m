% Federal University of Rio Grande do Norte
% Title: Recursive Least Squares (RLS) with forgetting factor
% Author: Danilo Pena
% Description: Algorithm for parameters estimation in real-time

%%
clc
clear
close all

% inputs
t_sim = 499; % simulation time
t = 0:t_sim;
tam = length(t);
degree_y = 2; % degree of the system
degree_u = 2;
beta = 0.9; % forgetting factor
 
% Input signal
% Impulsive
%u = zeros(1,tam);u(1)=1;

% Step
%u = ones(1,tam);u(1)=0;

% Random binary
u = idinput(tam,'prbs',[0,0.1])'; % random input

%% Init
p = 1000*eye(degree_y*2,degree_u*2); % Convariance matrix
theta = [ones(1,degree_y*2)]'; % initial conditions
y = zeros(1,tam);
erro = zeros(1,tam);
yest = zeros(1,tam);
theta_real = zeros(length(theta),tam);
theta_est = zeros(length(theta),tam);

%% Loop
for k = 3:tam
    % Noise
    % mean 0 and variance 0
    %e = 0 + sqrt(0) *randn(500,1);
    % mean 0 and variance 0.25
    e = 0 + sqrt(0.25) *randn(500,1);    
    % mean 0 and variance 1
    %e = 0 + sqrt(1) *randn(500,1);

    % Plant
    y(k) = 1.5*y(k-1) - 0.7*y(k-2) + 1*u(k-1) + 0.5*u(k-2) + e(k) + 0.2*e(k-1);

    % tests
    if (k<=200)
        y(k) = 1.5*y(k-1) - 0.7*y(k-2) + 1*u(k-1) + 0.5*u(k-2) + e(k) + 0.2*e(k-1);    
     else
        y(k)= 1.3*y(k-1) - 0.4*y(k-2) + 1*u(k-1) + 0.4*u(k-2) + e(k) + 0.1* e(k-1);
     end

    % Matrix Phi (regressors)
    phi = [-y(k-1:-1:k-degree_y) u(k-1:-1:k-degree_u)]';

    % output
    yest(k) = theta'*phi;

    % estimation error
    erro(k) = y(k)-yest(k);

    % gain vector
    K = p*phi/(beta+phi'*p*phi);

    % update
    theta = theta + (K*erro(k));

    % update
    p = (1/beta)*(p-K*phi'*p);

    % real and estimated parameters
    theta_est(:,k) = theta;
    %Teta_real(:,k) = [-1.5 0.7 1 0.5];%parâmetros reais
    if (k<=200)
        theta_real(:,k) = [-1.5 0.7 1 0.5];%parâmetros reais
    else
        theta_real(:,k) = [-1.3 0.4 1 0.4];%parâmetros reais
    end;    

    % restart covariance matrix
    %     if  trace(p)<0.5
    %        mod(k,199)==0                     
    %         p=1000*eye(grau_y*2,grau_u*2);   
    %     end
    %         TRACOP(k)= trace(p);
end
        
%% Plots
plot(t,y,'linewidth',3); 
hold on;
plot (t,yest,'r-','linewidth',2); 
hold on;
plot (t,u,'m','linewidth',3); 
grid on;
title('Saídas')
legend('Real output','Estimated output','Input');
 
figure(2);
plot (theta_real','b','linewidth',3);
%axis([0 tam -2 1.5])
hold on
plot (theta_est','r-','linewidth',2);
title('Parameters')
grid on;
   
figure(3);
plot (erro','linewidth',3); 
title('Estimation error')
grid on;

figure (4);
plot (t,e,'linewidth',3)
title('Noise')
grid on;