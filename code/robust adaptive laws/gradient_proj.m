% Federal University of Rio Grande do Norte
% Title: Gradient based on instantaneous cost function
% Author: Danilo Pena

%%
clc
clear
close all

%% Init
% Time 
h = 0.01; 
t = 0:h:20; 
m = max(size(t)); 
 
% Reals parameters
b0 = 66.3 * ones(1,m);  
a1 = -1.71 * ones(1,m); 
a0 = 66.3 * ones(1,m);
 
% Reference
% input = ones(1,m); % Const input
input_signal = 1 + 5*sin(6.3*t) + 5*sin(25.2*t);

% States (plant)
x1_o = zeros(1,m); 
x2_o = zeros(1,m); 
 
% gamma = [12 0 0; 0 20 0; 0 0 10]; % Const input
gamma = [17.75 0 0; 0 20 0; 0 0 10];
theta = [66.2; -1.7; 66.2];
theta_e = [66.1; -1.6; 66.1]; 
R = .71;
z_o = zeros(1,m); 
ze_0 = zeros(1,m); 

% variables
x1(1) = 0;
x2(1) = 0;
x1f(1) = 0;
x2f(1) = 0; 
dx1f(1) = 0;
dx2f(1) = 0;
dx1ff(1) = 0;
dx2ff(1) = 0;
x1u(1) = 0;
x2u(1) = 0;

%% Loop
for i = 1:m 

    % Plant
    u = input_signal(i); 
    x2 = x2 + h*(66.3*u + 1.71*x2 - 66.3*x1);
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

    % Filter (d)
    dx2f = dx2f + h*(x2 - 2*dx2f - dx1f);
    dx1f = dx1f + h*dx2f;
    dyf = dx1f;

    % vectors
    dx1f_o(i) = dx1f;
    dx2f_o(i) = dx2f;
    dyf_o(i) = dyf;

    % Filter (d)
    dx2ff = dx2ff + h*(dyf - 2*dx2ff - dx1ff);
    dx1ff = dx1ff + h*dx2ff;
    dyff = dx1ff;

    % vectors
    dx1ff_o(i) = dx1ff;
    dx2ff_o(i) = dx2ff;
    dyff_o(i) = dyff;

    % Filter (d)
    x2u = x2u + h*(u - 2*x2u - x1u);
    x1u = x1u + h*x2u;
    uf = x2u;

    % vectors
    x1u_o(i) = x1u;
    x2u_o(i) = x2u;
    uf_o(i) = uf;

    % Estimation
    fi = [uf; dyf; yf]; 
    z = [66.3 -1.71 66.3]* fi;
    ze = theta' * fi; 

    % vectors
    z_o(i) = z;
    ze_o(i) = ze;

    % Estimation error
    ns2 = fi' * fi; 
    m2 = 1 + ns2; 
    erro = (z - ze)/m2; 
    erro_o(i) = erro;

    % Estimation
    G = (2 * (ze - z)/fi)'; 
    J = -erro * fi; 

    % Projection
    if ((theta(1) - theta_e(1))^2 + (theta(2) - theta_e(2))^2 + (theta(3) - theta_e(3))^2) - R^2 > 0 
        theta = theta + h * (-gamma * J + gamma * G * G' * inv(G' * gamma * G) * gamma * J); 
    else 
        theta = theta + h * (gamma * erro * fi); 
    end 

    b0e = theta(1); 
    a1e = theta(2); 
    a0e = theta(3); 

    % vectors
    a1e_o(i) = a1e;
    a0e_o(i) = a0e;
    b0e_o(i) = b0e;

end

disp('Error maximum:');
max(erro_o)

b0_relax = 0;
a0_relax = 0;
a1_relax = 0;
erro_relax = 0;
[~,i] = size(erro_o);

for j=i:-1:1
    if ((a0_relax == 0) && ((a0e_o(j)/a0(1))<0.9999897))
        a0_relax = 1;
        disp('stabilization time of a0:');
        (j+1)*0.01
    elseif ((a1_relax == 0) && ((a1e_o(j)/a1(1)) < 0.9999897))
        a1_relax = 1;
        disp('stabilization time of a1:');
        (j+1)*0.01
    elseif ((b0_relax == 0) && ((b0e_o(j)/b0(1)) < 0.9999897))
        b0_relax = 1;
        disp('stabilization time of b0:');
        (j+1)*0.01
    elseif ((erro_relax == 0) && (abs(erro_o(j)) > (0.0001)))
        erro_relax = 1;
        disp('stabilization time of the plant:');
        (j+1)*0.01
    end
end

%% Plots
figure(1); 
plot(t, a1, '-k','linewidth', 3);
hold on; 
plot(t, a1e_o, '-g','linewidth', 3); 
xlabel('t (s)'); 
ylabel('a1'); 
legend('a_1 real', 'a_1 estimated'); 
grid on;
 
figure(2);
plot(t, a0, '-k','linewidth', 3);
hold on; 
plot(t, a0e_o, '-g','linewidth', 3); 
xlabel('t (s)'); 
ylabel('a0'); 
legend('a_0 real', 'a_0 estimated'); 
grid on; 
 
figure(3);
plot(t, b0, '-k','linewidth', 3);
hold on; 
plot(t, b0e_o, '-g','linewidth', 3); 
xlabel('t (s)'); 
ylabel('b0'); 
legend('b_0 real', 'b_0 estimated'); 
grid on; 
 
figure(4);
plot(t, z_o', '-k', 'linewidth', 3);
hold on; 
plot(t, ze_o', '-g','linewidth', 3); 
xlabel('t (s)'); 
ylabel('Saída'); 
legend('real output', 'estimated output'); 
grid on; 
 
figure(5);
plot(t, erro_o, '-k','linewidth', 3);
xlabel('t (s)'); 
ylabel('error'); 
legend('error'); 
grid on;
