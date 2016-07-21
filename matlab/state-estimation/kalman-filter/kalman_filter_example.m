% KALMAN_FILTER - updates a system state vector estimate based upon an
%                 observation, using a discrete Kalman filter.
%
% File inspired from "Learning the Kalman Filter" by Michael C. Kleder
% http://ch.mathworks.com/matlabcentral/fileexchange/5377-learning-the-kalman-filter
%
% FORM OF EQUATIONS USED IN THIS FILE
%
% STATE UPDATE
%
% x = Ax + Bu + w  
%
% MEASUREMENT UPDATE
% z = Hx + v       

% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%       v ~ N(0,R) meaning v is gaussian noise with covariance R
%
% VECTOR VARIABLES:
%
% s.x = state vector estimate. In the input struct, this is the
%       "a priori" state estimate. In the output struct,
%       this is the "a posteriori" state estimate.
% s.z = observation vector
% s.u = input control vector, optional (defaults to zero).
%
% MATRIX VARIABLES:
%
% state_.A = state transition matrix (defaults to identity).
% state_.P = covariance of the state vector estimate. In the input struct,
%       this is "a priori," and in the output it is "a posteriori."
%       (required unless autoinitializing as described below).
% state_.B = input matrix, optional (defaults to zero).
% state_.Q = process noise covariance (defaults to zero).
% state_.R = measurement noise covariance (required).
% state_.H = observation matrix (defaults to identity).
%
% NORMAL OPERATION:
%
% (1) define all state definition fields: A,B,H,Q,R
% (2) define intial state estimate: x,P
% (3) obtain observation and control vectors: z,u
% (4) call the filter to obtain updated state estimate: x,P
% (5) return to step (3) and repeat
%
% INITIALIZATION:
%
% If an initial state estimate is unavailable, then it can be obtained
% from the first observation as follows, provided that there are the
% same number of observable variables as state variables. This "auto-
% intitialization" is done automatically if state_.x is absent or NaN.
%
% x = inv(H)*z
% P = inv(H)*R*inv(H')
%
% This is mathematically equivalent to setting the initial state estimate
% covariance to infinity.
%
% SCALAR EXAMPLE (Automobile Voltimeter):
%
% % Define the system as a constant of 12 volts:
clear state_
state_.x = 12;
state_.A = 1;
% Define a process noise (stdev) of 2 volts as the car operates:
state_.Q = 2^2; % variance, hence stdev^2
% Define the voltimeter to measure the voltage itself:
state_.H = 1;
% Define a measurement error (stdev) of 2 volts:
state_.R = 2^2; % variance, hence stdev^2
% Do not define any system input (control) functions:
state_.B = 0;
state_.u = 0;
% Do not specify an initial state:
state_.x = nan;
state_.P = nan;
% Generate random voltages and watch the filter operate.
tru=[]; % truth voltage
for t=1:20
   tru(end+1) = randn*2+12;
   state_(end).z = tru(end) + randn*2; % create a measurement
   state_(end+1)=kalman_filter(state_(end)); % perform a Kalman filter iteration
end
figure
hold on
grid on
% plot measurement data:
hz=plot([state_(1:end-1).z],'r.');
% plot a-posteriori state estimates:
hk=plot([state_(2:end).x],'b-');
ht=plot(tru,'g-');
legend([hz hk ht],'observations','Kalman output','true voltage',0)
title('Automobile Voltimeter Example')
hold off