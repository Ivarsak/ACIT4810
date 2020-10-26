%Project, heated tank in series

%% Defining constant parameters

UA = 10000;  %Heat transfer capacity * Area
V = 100;     %Tank volume [L]
rho = 1000;  %Liquid density [g/L]

q1 = 10;            %Inlet flowrate q1 [L/min]
q2 = 10;            %Inlet flowrate q2 [L/min]
q_out = q1+q2;      %Outlet flowrate q_out [L/min]

w1 = rho*q1;        %Inlet mass flowrate w1 [kg/s]
w2 = rho*q2;        %Inlet mass flowrate w2 [kg/s]
w_out = (w1 + w2);  %Outlet mass flowrate w_out [kg/s]

T2 = 6;             %Inlet temperature T2 [deg]
C = 2.4;            %Heat capacity constant for water [J/g/K] 


%% Steady state values

T1_0 = 4;             %Inlet temperature T1 [deg]
Th_0 = 152.5;         %Heater temperature Th [deg]
T3_0 = (((q1*T1_0/V) + (q2*T2/V)))*(V/q_out) %Estimating steady state value for T3

%Estimating steady state value for T4
syms T4_0

T4_eq = (w_out*(T3_0 - T4_0))/(rho*V) + (UA*(Th_0 - T4_0))/(rho*V*C) == 0
 
T4_0 = double(solve(T4_eq, T4_0)) 

%% Linearization

a_11 = -q_out/V;
a_12 = 0;
a_21 = - q_out/V - UA/(C*V*rho);
a_22 = - q_out/V - UA/(C*V*rho);

b_11 = q1/V;
b_12 = T2/V;
b_13 = 0;
b_21 = 0;
b_22 = 0;
b_23 = UA/(C*V*rho);


Am = [a_11 a_12;
      a_21 a_22]

Bm = [b_11 b_12 b_13;
      b_21 b_22 b_23]
  
Cm=[1 0;0 1];
Dm=0;

model_lin=ss(Am,Bm,Cm,Dm);


%% Test procedure
%test procedure
%start simulation
t0=0;
%decrease 
t1=30;
%bring back to nominal
t2=60;
%increase
t3=90;
%bring back to nominal
t4=120;
%end simulation
t5=150;

%changes in cooling medium temperature Tc
T1_1=-0.5; %deg
T1_2=+0.5; %deg
T1_3=+0.5; %deg
T1_4=-0.5; %deg


%changes in cooling medium temperature Tc
q2_1=-2; %L/min
q2_2=+2; %L/min
q2_3=+2; %L/min
q2_4=-2; %L/min


%changes in cooling medium temperature Tc
Th_1=-5; %deg
Th_2=+5; %deg
Th_3=+5; %deg
Th_4=-5; %deg

