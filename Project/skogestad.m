
%% Skogestad tuning method

%Output: Kc = gain, T_i = integral time
%Input: kp = transfer function gain, tau = transfer function timeconsant,
%theta = transfer function timedelay (dead-time)

function [Kc, T_i] = skogestad(Kp, tau, theta, tc1)
       
     Kc = tau/(Kp*(tc1 + theta));         %Estimating proportional gain
     T_i = min(tau, 4*(tc1 + theta));     %Estimating integral time
end







