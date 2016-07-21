function state_ = kalman_filter(state_)
%
%   Syntax:
%   state_ = kalman_filter(state_)
%
%   Inputs:
%   state_  : the "a priori" state
%
%   Outputs:
%   state_ : the "a posteriori" state estimate
%
%   Inspired from: Learning the Kalman Filter by Michael C. Kleder
%   http://ch.mathworks.com/matlabcentral/fileexchange/5377-learning-the-kalman-filter
%
%

% set defaults for absent fields:
if ~isfield(state_,'x'); state_.x=nan*z; end
if ~isfield(state_,'P'); state_.P=nan; end
if ~isfield(state_,'z'); error('Observation vector missing'); end
if ~isfield(state_,'u'); state_.u=0; end
if ~isfield(state_,'A'); state_.A=eye(length(x)); end
if ~isfield(state_,'B'); state_.B=0; end
if ~isfield(state_,'Q'); state_.Q=zeros(length(x)); end
if ~isfield(state_,'R'); error('Observation covariance missing'); end
if ~isfield(state_,'H'); state_.H=eye(length(x)); end

if isnan(state_.x)
   % initialize state estimate from first observation
   if diff(size(state_.H))
      error('Observation matrix must be square and invertible for state autointialization.');
   end
   state_.x = inv(state_.H)*state_.z;
   state_.P = inv(state_.H)*state_.R*inv(state_.H'); 
else
   
   % DISCRETE KALMAN FILTER STEPS
   
   % Prediction for state vector and covariance:
   state_.x = state_.A*state_.x + state_.B*state_.u;
   state_.P = state_.A * state_.P * state_.A' + state_.Q;

   % Compute Kalman gain factor:
   K = state_.P*state_.H'*inv(state_.H*state_.P*state_.H'+state_.R);

   % Correction based on observation:
   state_.x = state_.x + K*(state_.z-state_.H*state_.x);
   state_.P = state_.P - K*state_.H*state_.P;
   
   % Note that the desired result, which is an improved estimate
   % of the sytem state vector x and its covariance P, was obtained
   % in only five lines of code, once the system was defined. 

end

return
