function u = fcn(e,t)
%% Define Variables
% Control Gains
Kp=4;
Ki=0;
umax = 8;
% Memory variables
persistent I_past;
persistent t_past;
if isempty(I_past), I_past=0; end;
if isempty(t_past), t_past=0; end;
%% Calculate Controller output
% sample time
Ts = t - t_past;
% Integrator
I = I_past+e*Ts;
% PI control calculation
u = Kp*e+Ki*I;
% anti-windup
if abs(u)>umax,
u = sign(u)*umax;
e = sign(e)*min(umax/Kp, abs(e));
I = (u-Kp*e)/Ki;
end;
I_past=I;
t_past=t;
