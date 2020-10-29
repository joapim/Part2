M=[6.8177e6, 0, 0; 0, 7.8784e6, -2.5955e6; 0, -2.5955e6, 3.57e9]; % mass
invM=inv(M);
D=diag([2.6486e5, 8.8164e5, 3.3774e8]); % damping
h=0.1; %sampling time

omega=diag([2*pi/10,2*pi/10,2*pi/10]); % omega and lambda used in Aw
lambda=diag([0.08,0.08,0.08]);
Aw=[zeros(3), eye(3); -omega^2, -2*lambda*omega]; % system

Cw=[zeros(3), eye(3)]; %meassurements

Kw=diag([0.01,0.01,0.01]); % used in Ew
Ew=[zeros(3); Kw]; % disturbance

Tb=diag([1000,1000,1000]); % zero-mean Gaussian white noise vector
invTb=inv(Tb);
Eb=eye(3); % diagonal scaling matrix 

Q=diag([ 10^4 10^4 10^3 10^9 10^9 10^9]);
R=diag([1,1,0.1]);

% EKF is based on a nonlinear model (eq. 7.12-7.13)
B=[zeros(6,3);zeros(3,3);zeros(3,3);invM];
E=[Ew, zeros(6,3); zeros(3), zeros(3); zeros(3), Eb; zeros(3), zeros(3)];
H=[Cw, eye(3), zeros(3), zeros(3)];

xBar=zeros(15,1); % A priori state 
pBar=zeros(15); % covariance matrix estimates 

save('initObserverInput.mat');

