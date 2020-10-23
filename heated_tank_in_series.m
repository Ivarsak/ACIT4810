%Project, heated tank in series

%% Defining constants

UA = 10000;  %Heat transfer capacity * Area
V = 100;     %Tank volume [L]
rho = 1000;  %Liquid density [g/L]

q1 = 50;        %Inlet flowrate q1 [L/min]
q2 = 50;        %Inlet flowrate q2 [L/min]
q_out = q1+q2;  %Outlet flowrate q_out [L/min]

w1 = rho*q1;    %Inlet mass flowrate w1 [kg/s]
w2 = rho*q2;    %Inlet mass flowrate w2 [kg/s]

w_out = (w1 + w2);  %Outlet mass flowrate w_out [kg/s]

T1 = 4;             %Inlet temperature T1 [deg]
T2 = 6;             %Inlet temperature T2 [deg]
C = 4.2;            %Heat capacity constant for water [J/g/K] 
Th = 152.5;         %Heater temperature Th [deg]

%% Steady state values
T3_0 = (((q1*T1/V) + (q2*T2/V)))*(V/q_out) %Steady state






