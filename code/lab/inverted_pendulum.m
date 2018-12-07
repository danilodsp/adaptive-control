% Federal University of Rio Grande do Norte
% Title: Inverted pendulum model
% Author: Danilo Pena

M = 0.5;
m = 0.127;
b = 0;
I = 0.006;
g = 9.8;
l = 0.3365;
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

%G_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);
G_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

%sys_tf = [G_cart ; G_pend];
sys_tf = [G_pend];

inputs = {'u'};
%outputs = {'x'; 'phi'};
outputs = {'phi'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)

sys_tf
% t=0:0.01:1;
% impulse(sys_tf,t);
% title('Impulsive response - Open loop')

% Kp = 100;
% Ki = 1;
% Kd = 20;
% C = pid(Kp,Ki,Kd);
% T = feedback(G_pend,C);
% t = 0:0.01:10;
% impulse(T,t)
% axis([0, 2.5, -0.2, 0.2]);
% title('Response of Pendulum Position to an Impulse Disturbance under PID Control: Kp = 100, Ki = 1, Kd = 20');