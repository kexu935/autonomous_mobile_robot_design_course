function [x,P]= unscented_kalman_filter(fstate,x,P,hmeas,z,Q,R)
%
%   Syntax:
%   [x,P]= unscented_kalman_filter(fstate,x,P,hmeas,z,Q,R)
%   Unscented Kalman Filter implementation for nonlinear dynamic systems
%   returns state estimate, x and state covariance, P 
%   for nonlinear dynamic system 
%   (for simplicity, noises are assumed as additive):
%           x_k+1 = f(x_k) + w_k
%           z_k   = h(x_k) + v_k
%   where   w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%           v ~ N(0,R) meaning v is gaussian noise with covariance R
%   Inputs:   
%   f:      function handle for f(x)
%   x:      "a priori" state estimate
%   P:      "a priori" estimated state covariance
%   h:      function handle for h(x)
%   z:      current measurement
%   Q:      process noise covariance 
%   R:      measurement noise covariance
% 
%   Outputs:   
%   x:      "a posteriori" state estimate
%   P:      "a posteriori" state covariance
%
%   This file is based on https://github.com/Piyush3dB/NN-Kalman/
%


L=numel(x);                                 %numer of states
m=numel(z);                                 %numer of measurements

alpha=1e-3;                                 %default, tunable
ki=0;                                       %default, tunable
beta=2;                                     %default, tunable

lambda=alpha^2*(L+ki)-L;                    %scaling factor
c=L+lambda;                                 %scaling factor

Wm=[lambda/c 0.5/c+zeros(1,2*L)];           %weights for means
Wc=Wm;
Wc(1)=Wc(1)+(1-alpha^2+beta);               %weights for covariance
c=sqrt(c);
X=sigmas(x,P,c);                            %sigma points around x
[x1,X1,P1,X2]=ut(fstate,X,Wm,Wc,L,Q);          %unscented transformation of process
% X1=sigmas(x1,P1,c);                         %sigma points around x1
% X2=X1-x1(:,ones(1,size(X1,2)));             %deviation of X1
[z1,Z1,P2,Z2]=ut(hmeas,X1,Wm,Wc,m,R);       %unscented transformation of measurments
P12=X2*diag(Wc)*Z2';                        %transformed cross-covariance
K=P12*inv(P2);
x=x1+K*(z-z1);                              %state update
P=P1-K*P12';                                %covariance update

function [y,Y,P,Y1]=ut(f,X,Wm,Wc,n,R)
%Unscented Transformation
%Input:
%        f: nonlinear map
%        X: sigma points
%       Wm: weights for mean
%       Wc: weights for covraiance
%        n: numer of outputs of f
%        R: additive covariance
%Output:
%        y: transformed mean
%        Y: transformed smapling points
%        P: transformed covariance
%       Y1: transformed deviations

L=size(X,2);
y=zeros(n,1);
Y=zeros(n,L);
for k=1:L                   
    Y(:,k)=f(X(:,k));       
    y=y+Wm(k)*Y(:,k);       
end
Y1=Y-y(:,ones(1,L));
P=Y1*diag(Wc)*Y1'+R;          

function X=sigmas(x,P,c)
%Sigma points around reference point
%Inputs:
%       x: reference point
%       P: covariance
%       c: coefficient
%Output:
%       X: Sigma points

N = numel(x);
A = c*chol(P)';
Y = x(:,ones(1,N));
X = [x Y+A Y-A]; 