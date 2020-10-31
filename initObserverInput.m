function ObserverInput=initObserverInput()
M=[6.8177e6, 0, 0; 0, 7.8784e6, -2.5955e6; 0, -2.5955e6, 3.57e9];
invM=inv(M);
D=diag([2.6486e5, 8.8164e5, 3.3774e8]);
h=0.1;
omega=diag([2*pi/10,2*pi/10,2*pi/10]);
lambda=diag([0.08,0.08,0.08]);

Kw=diag([0.01,0.01,0.01]);
Aw=[zeros(3), eye(3); -omega^2, -2*lambda*omega];
Cw=[zeros(3), eye(3)];
Ew=[zeros(3); Kw];

Tb=diag([1000,1000,1000]);
invTb=inv(Tb);
Eb=eye(3);%diag([10^4,10^4,10^4]);

Q=diag([ 10^4 10^4 10^3 10^9 10^9 10^9]);
R=diag([1,1,0.1]);

B=[zeros(6,3);zeros(3,3);zeros(3,3);invM];
E=[Ew, zeros(6,3); zeros(3), zeros(3); zeros(3), Eb; zeros(3), zeros(3)];
H=[Cw, eye(3), zeros(3), zeros(3)];

xBar=zeros(15,1);
pBar=zeros(15);

ObserverInput.M=M;
ObserverInput.invM=invM;
ObserverInput.D=D;
ObserverInput.h=h;
ObserverInput.Aw=Aw;
ObserverInput.invTb=invTb;
ObserverInput.Tb=Tb;
ObserverInput.Q=Q;
ObserverInput.R=R;
ObserverInput.B=B;
ObserverInput.E=E;
ObserverInput.H=H;
ObserverInput.xBar=xBar;
ObserverInput.pBar=pBar;

save('initObserverInput.mat');
end
