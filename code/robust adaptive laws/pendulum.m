% Federal University of Rio Grande do Norte
% Title: Pendulum model
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

% %sys_tf = [G_cart ; G_pend];
% sys_tf = [G_pend];
% 
% inputs = {'u'};
% %outputs = {'x'; 'phi'};
% outputs = {'phi'};
% 
% set(sys_tf,'InputName',inputs)
% set(sys_tf,'OutputName',outputs)
% 
% 
% sys_tf
% t=0:0.01:1;
% impulse(sys_tf,t);
% title('Resposta ao Impulso - Malha Aberta')

Gmr=9/(s^2+6*s+9);
sys_tf = [Gmr];
inputs = {'u'};
outputs = {'phi'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)

sys_tf
t=0:0.01:3;
impulse(sys_tf,t);
title('Reference model')