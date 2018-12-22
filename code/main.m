% Federal University of Rio Grande do Norte
% Title: Main simulation
% Author: Danilo Pena
% Description: Simulation of disturbances with some controllers

%{
T - Simulation time (in seconds)
controller - 'PI', 'MRAC', 'MRAC-FF', 'MRAC-FF-N', 'VS-MRAC' or 'DMARC'
disturbance - 'DELTA' (10%), 'PSI' (5%) or 'VP' (10%)
%}

%% Simulation

clear
close all
clc

T = 250;
controller = 'PI';
disturbance = 'DELTA';
disturbance_percent = 0.1;
Rg = (30*pi/180);
Re = 1.15;

[t_o, delta_o, rg_o, phif_o, re_o, ue_o, ug_o, Vt_o, Pe_o] = main_function(controller,disturbance,disturbance_percent,Rg,Re,T);

%% Plots

figure (1)
plot(t_o,delta_o,'--b','LineWidth',3)
set(gca,'fontsize',24)
hold on
grid on
plot(t_o,rg_o,'-b','LineWidth',3)
plot(t_o,phif_o,'--r','LineWidth',3)
plot(t_o,re_o,'-r','LineWidth',3)
legend('\delta','\delta*','\Psi_f','\Psi_f*','Location','East')
axis([30 120 0.4 1.3])
%axis([30 120 0.2 1.3]) % VP
xlabel('t (s)')
ylabel('(p.u.)')
set(gca,'XTick',[30:30:120])
set(gca,'XTickLabel',[20 50 80 110])

figure (2)
plot (t_o,ue_o,'-b','LineWidth',3)
set(gca,'fontsize',24)
hold on
grid on
plot (t_o,ug_o,'-r','LineWidth',3)
legend('ue','ug','Location','East')
axis([30 120 -0.2 0.6])
%axis([30 120 -0.2 0.8]) % VP
xlabel('t (s)')
ylabel('(p.u.)')
set(gca,'XTick',[30:30:120])
set(gca,'XTickLabel',[20 50 80 110])

