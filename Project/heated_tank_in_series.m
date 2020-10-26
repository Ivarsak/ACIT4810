%Project, heated tank in series

%% Defining constant parameters

%UA = 10000;  %Heat transfer capacity * Area
UA=5*10^4;
V = 100;     %Tank volume [L]
rho = 1000;  %Liquid density [g/L]

q1 = 50;            %Inlet flowrate q1 [L/min]
q2 = 50;            %Inlet flowrate q2 [L/min]
q_out = q1+q2;      %Outlet flowrate q_out [L/min]

w1 = rho*q1;        %Inlet mass flowrate w1 [kg/s]
w2 = rho*q2;        %Inlet mass flowrate w2 [kg/s]
w_out = (w1 + w2);  %Outlet mass flowrate w_out [kg/s]

T2 = 6;             %Inlet temperature T2 [deg]
C = 2.4;            %Heat capacity constant for water [J/g/K] 


%% Steady state values

T1_0 = 4;             %Inlet temperature T1 [deg]
Th_0 = 80;         %Heater temperature Th [deg]
T3_0 = (((q1*T1_0/V) + (q2*T2/V)))*(V/q_out); %Estimating steady state value for T3

%Estimating steady state value for T4
syms T4_0

T4_eq = (w_out*(T3_0 - T4_0))/(rho*V) + (UA*(Th_0 - T4_0))/(rho*V*C) == 0;
 
T4_0 = double(solve(T4_eq, T4_0)); 

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
      a_21 a_22];

Bm = [b_11 b_12 b_13;
      b_21 b_22 b_23];
  
Cm=[1 0;0 1];
Dm=[0 0 0; 0 0 0];

model_lin=ss(Am,Bm,Cm,Dm);

%% Transfer function models
tf(model_lin);

T3_T1_den = 0.1;
T4_T1_den = -0.02417;
T3_q2_den = 0.06;
T4_q2_den = -0.0145;
T4_Th_den = 0.04167;

T3_T1_num = [1 0.2];
T4_T1_num = [1 0.4417 0.04833];
T3_q2_num = [1 0.2];
T4_q2_num = [1 0.4417 0.04833];
T4_Th_num = [1 0.2417];

T3_T1 = tf(T3_T1_den, T3_T1_num)
T4_T1 = tf(T4_T1_den, T4_T1_num)
T3_q2 = tf(T3_q2_den, T3_q2_num)
T4_q2 = tf(T4_q2_den, T4_q2_num)
T4_Th = tf(T4_Th_den, T4_Th_num)

% Plant for MPC controller
plant = [T3_T1 T3_q2 0;
         T4_T1 T4_q2 T4_Th];
     
%% Skogestads PID Tuning Method

Tc = 0.1
%PID tuning parameters for T3 controller
[Kc_1, Ti_1] = skogestad(T3_T1_den, T3_T1_num(2),0,Tc);

%PID tuning parameters for T4 controller
[Kc_2, Ti_2] = skogestad(T4_Th_den, T4_Th_num(2),0,Tc);

%% MPC control
%define the third input signal to plant as "Measured Disturbance"
plant1 = setmpcsignals(plant,'MD',[2]);

T_settling = (19.5604 + 26.7741 + 16.1857 )/3;
%horizon for prediction and control
%settling time stepinfo(TF11), stepinfo(TF22),... 
Ts = 0.5;
N = 20; %average settling time for open loop responses MV-CV
M = N/2; %M=N/2
P = N+M; %P=N+M

%constraints for the manipulated (input) variables
MV1 = struct('Min',-50,'Max',50);
MV2 = struct('Min',-200,'Max',200);
MV=[MV1 MV2];

%constraints for the controlled (output) variables
%not used in case B
%T
%OV1 = struct('Min',-5,'Max',5 );
%cA
%OV2 = struct('Min',-0.2,'Max',0.2);
%OV=[OV1 OV2];

%Weights
%Q equal weighting between controlled variables
Q=[1 10];
%Ru zero weighting for the values of the manipulated variables (u)
Ru=[1 1];
%Rd weighting for the changes in the manipulated variables (du)
Rd=[1 1];
W=struct('ManipulatedVariables',Ru,'ManipulatedVariablesRate',Rd,'OutputVariables',Q);

%specifies MPC controller with prediction horizon (p), control horizon (m), and input, input increment, and output weights (W). 
%and the properties of manipulated variables (MV), output variables (OV), and input disturbance variables (DV).
%MPCobj = mpc(Plant,Ts,p,m,W,MV,OV,DV)
mpcB = mpc(plant1,Ts,P,M,W,MV);

%creates a controller state object compatible with the controller object, MPCobj,
xmpcB = mpcstate(mpcB);


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
q2_1=-60; %L/min
q2_2=+60; %L/min
q2_3=+60; %L/min
q2_4=-60; %L/min


%changes in cooling medium temperature Tc
Th_1=-5; %deg
Th_2=+5; %deg
Th_3=+5; %deg
Th_4=-5; %deg

