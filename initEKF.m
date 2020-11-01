x_bar=zeros(15,1); % A priori state 
p_bar=zeros(15); % covariance matrix estimates 

M = [6.8177e6, 0, 0; % mass
    0, 7.8784e6, -2.5955e6;
    0, -2.5955e6, 3.57e9]; 

invM = inv(M);

h = 0.1; % sampling time
omega_i = 2*pi/10; % wave frequency
lambda_i = 0.08; % relative damping ratio
K_i = 0.01; % constant

omega = [omega_i, 0, 0; % omega used in A
         0, omega_i, 0;
         0, 0 ,omega_i]; 

lambda = [lambda_i, 0, 0; % lambda used in A
        0, lambda_i, 0;
        0, 0, lambda_i];

K = [K_i, 0, 0; % used in E
      0, K_i, 0;
      0, 0, K_i]; 

A = [zeros(3), eye(3); % system
      -omega^2, -2*lambda*omega]; 

B=[zeros(6,3); % (eq. 7.12-7.13)
   zeros(3,3);
   zeros(3,3);
   invM];

C = [zeros(3), eye(3)]; %meassurements

D = [2.6486e5, 0, 0; % damping
    0, 8.8164e5, 0;
    0, 0, 3.3774e8]; 

Ew = [zeros(3); % disturbance
      K]; 
  
E = [Ew, zeros(6,3); % (eq. 7.12-7.13)
    zeros(3), zeros(3);
    zeros(3), eye(3);
    zeros(3), zeros(3)];
  
H=[C, eye(3), zeros(3), zeros(3)]; % (eq. 7.12-7.13)

Q = [10^4, 0, 0, 0, 0, 0; % process noise covariance matrix
     0, 10^4, 0, 0, 0, 0;
     0, 0, 10^3, 0, 0, 0;
     0, 0, 0, 10^9, 0, 0;
     0, 0, 0, 0, 10^9, 0;
     0, 0, 0, 0, 0, 10^9];

R = [1, 0,0; % meassurement noise covariance matrix
     0, 1, 0;
     0, 0, 0.1];
 
T = [1000, 0,0; % zero-mean Gaussian white noise vector
      0, 1000, 0;
      0, 0, 1000]; 

invT = inv(T);

save('initEKF.mat');

