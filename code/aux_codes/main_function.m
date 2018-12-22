% Federal University of Rio Grande do Norte
% Title: Auxiliar code
% Author: Danilo Pena
% Description: Function for simulations with disturbances

function [t, delta, rg, phif, re, ue, ug, Vt, Pe] = main_function(controller, ...
    disturbance,disturbance_percent,Rg,Re,T)

% Sync generator

h = 0.0001;
tempo = 0:h:T;
n = max(size(tempo));

% Initialization
delta = zeros(1,n-1);
ddelta = zeros(1,n-1);
d2delta = zeros(1,n-1);
phif = zeros(1,n-1);
Efd_o = zeros(1,n-1);
Pm = zeros(1,n-1);
Pg = zeros(1,n-1);
Pe = zeros(1,n-1);
Vt = zeros(1,n-1);
ug = zeros(1,n-1);
ue = zeros(1,n-1);
rg = zeros(1,n-1);
re = zeros(1,n-1);
input_step = ones(1,n);

% Initial conditions
delta(1) = 0;
ddelta(1) = 0;
d2delta(1) = 0;
phif(1) = 0.8;
Efd = 0;
ug = 0;
ue = 0;
ym = 0;
teta1(1) = 0;
teta2(1) = 0;
eg(1) = rg(1) - delta(1);
deg(1) = (eg(1) - 0)/h;
d2eg(1) = (deg(1) - 0)/h;
ee(1) = re(1) - phif(1);
dee(1) = (ee(1) - 0)/h;

% Parameters of the synchronous generator
d = 0.006; % Damping Constant (s)
H = 3.82; % Inertia Constant (s)
xd = 1.75; % Direct Shaft Armature Reactance (pu)
xq = 1.68; % Quadrature Shaft Armature Reactor (pu)
xaf = 1.56; % Mutual Reactance of Direct Shaft (pu)
xt = 0.3; % Transmission Line Reactivity (pu)
xf = 1.665; % Reactance of Field (pu)
rf = 0.0012; % Field Strength (pu)
Te = 0.04; % Field Excitation Time Constant (s)
ke = 25; % Field Excitation Gain (dimensionless)
Tt = 0.3; % Turbine Time Constant (s)
Tg = 0.08; % Governor Time Constant (s)
f = 60; % Frequency (Hz)
V = 1; % Infinite Bus Voltage
xdl = 0.286;
%xdl = xd - ((xaf * xaf) / xf); % Direct shaft transient reactance
wo = 2*pi*f; % Synchronous speed
xdk = xd;
xqk = xq;
xfk = xf;

% Controller parameters
% gama1 = 0.2;              %MRAC
% gama2 = 0.2;              %MRAC
% gama1 = 1.2;              %MRAC - forgetting factor
% gama2 = 1.2;              %MRAC - forgetting factor

% model 1
%teta1barra=0.0079;         %MRAC - forgetting factor and normalization
%teta2barra=0.054;          %MRAC - forgetting factor and normalization
%sigma=0.98;                %MRAC - forgetting factor and normalization

% model 2
teta1barra=0.012;           %MRAC - forgetting factor and normalization
teta2barra=0.05;            %MRAC - forgetting factor and normalization
sigma=0.4;                  %MRAC - forgetting factor and normalization

% test
% teta1barra=0.007;           %MRAC - forgetting factor and normalization
% teta2barra=0.5;             %MRAC - forgetting factor and normalization
% sigma=0.038;                %MRAC - forgetting factor and normalization

%sigma = 1.2;               %MARC - forgetting factor
%sigma = 0.025;             %DMARC
L = 0.0005;
%mi=0;  
%teta1barra=0.0595;         %DMARC
%teta2barra=0.00537;        %DMARC

for  k=2:n
    
    % Input
    rg(k) = Rg*input_step(k);
    if (strcmp(disturbance,'DELTA'))
        ref = 1.22*input_step(k);
        ref_P = 0.399*input_step(k);
        if k>400000
            rg(k) = (1-disturbance_percent)*Rg*input_step(k);
            ref_P = 0.3485*input_step(k);
            ref = 1.2*input_step(k);
        end
        if k>750000
            rg(k) = 1*Rg*input_step(k);
            ref_P = 0.399*input_step(k);
            ref = 1.22*input_step(k);
        end
        if k>1100000
            rg(k) = (1-disturbance_percent)*Rg*input_step(k);
            ref_P = 0.3485*input_step(k);
            ref = 1.2*input_step(k);
        end
        if k>1450000
            rg(k) = 1*Rg*input_step(k);
            ref_P = 0.399*input_step(k);
            ref = 1.24*input_step(k);
        end
        if k>1800000
            rg(k) = (1-disturbance_percent)*Rg*input_step(k);
            ref_P = 0.3485*input_step(k);
            ref = 1.22*input_step(k);
        end
        if k>2150000
            rg(k) = 1*Rg*input_step(k);
            ref_P = 0.399*input_step(k);
            ref = 1.24*input_step(k);
        end
    end

    re(k) = Re*input_step(k);
    if (strcmp(disturbance,'PSI'))
        ref = 1.219*input_step(k);
        ref_P = 0.3912*input_step(k);
        if k>400000
            re(k) = (1-disturbance_percent)*Re*input_step(k);
            ref = 1.193*input_step(k);
            ref_P = 0.3433*input_step(k);
        end
        if k>750000
            re(k) = 1*Re*input_step(k);
            ref = 1.216*input_step(k);
            ref_P = 0.3912*input_step(k);
        end
        if k>1100000
            re(k) = (1-disturbance_percent)*Re*input_step(k);
            ref = 1.193*input_step(k);
            ref_P = 0.3433*input_step(k);
        end
        if k>1450000
            re(k) = 1*Re*input_step(k);
            ref = 1.216*input_step(k);
            ref_P = 0.3912*input_step(k);
        end
        if k>1800000
            re(k) = 1*Re*input_step(k);
            ref = 1.216*input_step(k);
            ref_P = 0.3433*input_step(k);
        end
        if k>2150000
            re(k) = (1+disturbance_percent)*Re*input_step(k);
            ref = 1.239*input_step(k);
            ref_P = 0.3912*input_step(k);
        end
    end

    if (strcmp(disturbance,'VP'))
        ref = 1.22*input_step(k);
        ref_P = 0.4*input_step(k);
        if k>400000
            ref = 1.263*input_step(k);
            ref_P = 0.5225*input_step(k);
            xd = (1-disturbance_percent)*xdk;
            xq = (1-disturbance_percent)*xqk;
            xf = (1-disturbance_percent)*xfk;
        end
        if k>750000
            ref = 1.318*input_step(k);
            ref_P = 0.6746*input_step(k);
            xd = (1-2*disturbance_percent)*xdk;
            xq = (1-2*disturbance_percent)*xqk;
            xf = (1-2*disturbance_percent)*xfk;
        end
        if k>1100000
            ref = 1.263*input_step(k);
            ref_P = 0.5225*input_step(k);
            xd = (1-disturbance_percent)*xdk;
            xq = (1-disturbance_percent)*xqk;
            xf = (1-disturbance_percent)*xfk;
        end
        if k>1450000
            ref = 1.22*input_step(k);
            ref_P = 0.4*input_step(k);
            xd = 1*xdk;
            xq = 1*xqk;
            xf = 1*xfk;
        end
        if k>1800000
            ref = 1.263*input_step(k);
            ref_P = 0.5225*input_step(k);
            xd = (1-disturbance_percent)*xdk;
            xq = (1-disturbance_percent)*xqk;
            xf = (1-disturbance_percent)*xfk;
        end
        if k>2150000
            ref = 1.22*input_step(k);
            ref_P = 0.4*input_step(k);
            xd = 1*xdk;
            xq = 1*xqk;
            xf = 1*xfk;
        end
    end

    ref_o(k) = ref;
    refP_o(k) = ref_P;

    p1 = (wo*V*V*(xq-xdl))/(4*H*(xt+xdl)*(xt+xq));
    p2 = (wo*d)/(2*H);
    p3 = (wo*V*xaf)/(2*H*xf*(xt+xdl));
    p4 = wo/(2*H);
    p5 = (wo*rf*V*xaf)/(xf*(xt+xdl));
    p6 = (wo*rf*(xt+xd))/(xf*(xt+xdl));
    p7 = (wo*rf)/xf;

     %PID controller for load angle
    eg(k) = rg(k) - delta(k-1);% error between load angle and reference (PID input)
    deg(k) = (eg(k) - eg(k-1))/h;
    d2eg(k) = (deg(k) - deg(k-1))/h;

    %dug = 0.00005*d2eg + 0.01*deg + 0.9333*eg; % controller 1
    dug(k) = 0.0001*d2eg(k) + 0.02*deg(k) + 1*eg(k); % controller 2
    %dug(k) = 0.085*d2eg(k) + 0.421*deg(k) + 0.521*eg(k); % controller 3
    ug(k) = dug(k)*h + ug(k-1);

    if (strcmp(controller,'PI'))
        % PI Controller for Field Flow
        ee(k) = re(k) - phif(k-1);% error between field flow and reference (PI input)
        dee(k) = (ee(k) - ee(k-1))/h;
        %due(k) = 0.086*dee(k) + 0.08*ee(k); % controller 1
        due(k) = 0.52*dee(k) + 0.793*ee(k); % controller 2
        ue(k) = due(k)*h + ue(k-1);
    end

    if (~strcmp(controller,'PI'))
        % DMARC Controller for Field Flow
        % Reference model
        dym(k) = - 1*ym(k-1) + 1*re(k);

        ym(k) = dym(k)*h + ym(k-1);

        % error
        eo = phif(k-1) - ym(k);

        % Control Law
        ue(k) = teta1(k-1)*phif(k-1) + teta2(k-1)*re(k);
    end

    if (strcmp(controller,'MRAC')) 
        % Estimation (MRAC)
        dteta1 = -gama1*eo*phif(k-1);
        dteta2 = -gama2*eo*re(k);

        teta1(k) = dteta1*h + teta1(k-1);
        teta2(k) = dteta2*h + teta2(k-1);
    end

    if (strcmp(controller,'MRAC-FF')) 
        % forgetting factor
        dteta1 = -sigma*teta1(k-1) - gama1*eo*phif(k-1);
        dteta2 = -sigma*teta2(k-1) - gama2*eo*re(k);

        teta1(k) = dteta1*h + teta1(k-1);
        teta2(k) = dteta2*h + teta2(k-1);
    end

    if (strcmp(controller,'MRAC-FF-N')) 
        % forgetting factor and normalization
        dteta1 = -sigma*teta1(k-1) - sigma*teta1barra*sign(eo*phif(k-1));
        dteta2 = -sigma*teta2(k-1) - sigma*teta2barra*sign(eo*re(k));

        teta1(k) = dteta1*h + teta1(k-1);
        teta2(k) = dteta2*h + teta2(k-1);
    end

    if (strcmp(controller,'VS-MRAC')) 
        % VS-MRAC
        teta1(k) = -teta1barra*sign(eo*phif(k-1));
        teta2(k) = -teta2barra*sign(eo*re(k));   
    end

    if (strcmp(controller,'DMARC')) 
        % controller parameters of DMARC
        mi = exp(-((eo*eo)/L));
        mi_o(k) = mi;
        mi=0;
        if mi>0 % Dual mode
        dteta1 = (-sigma*teta1(k-1) - sigma*teta1barra*sign(eo*phif(k-1)))/(mi+0.0001);
        dteta2 = (-sigma*teta2(k-1) - sigma*teta2barra*sign(eo*re(k)))/(mi+0.0001);

        teta1(k) = dteta1*h + teta1(k-1);
        teta2(k) = dteta2*h + teta2(k-1);  
        else  % VS
        teta1(k) = -teta1barra*sign(eo*phif(k-1));
        teta2(k) = -teta2barra*sign(eo*re(k));   
        end 
    end

    % synchronous machine
    % if(delta>2*pi)
    %     delta = delta-2*pi;
    % end
    d2delta(k) = p1*sin(2*delta(k-1)) - p2*ddelta(k-1) - p3*phif(k-1)*sin(delta(k-1)) + p4*Pm(k-1);
    ddelta(k) = d2delta(k)*h + ddelta(k-1);
    delta(k) = ddelta(k)*h + delta(k-1);

    dphif = p5*cos(delta(k-1)) - p6*phif(k-1) + p7*Efd(k-1);
    phif(k) = dphif*h + phif(k-1);

    % Excitation system
    dEfd(k) = (-Efd(k-1) + ke*ue(k))/Te;
    Efd(k) = dEfd(k)*h + Efd(k-1);

    % Turbine and governor
    dPm(k) = (-Pm(k-1) + Pg(k-1))/Tt;
    Pm(k) = dPm(k)*h + Pm(k-1);

    dPg(k) = (-Pg(k-1) + ug(k))/Tg;
    Pg(k) = dPg(k)*h + Pg(k-1);

    % Power generated
    Pe(k) = (p3*phif(k)*sin(delta(k)) - p1*sin(2*delta(k)))/p4;

    % Terminal voltage
    Vt(k) = sqrt((V*xq*sin(delta(k))/(xt+xq))^2 + (((V*xdl*cos(delta(k)))/(xt+xdl))+((xaf*xt*phif(k))/((xt+xdl)*xf)))^2);

    % Speed
    %w1(k) = wo - ddelta(k);

    t(k) = k*h;
end

end