close all

load('supply.mat')

heading = 0:10:350;

motion = 1;


figure()
[X,Y]=meshgrid(heading,vessel.motionRAO.w )
surf(X,Y,vessel.motionRAO.amp{1, motion}(:,:,1))
xlabel('Heading [deg]')
ylabel('w [rad/s]')

figure()
[X,Y]=meshgrid(heading,2*pi./vessel.motionRAO.w)
surf(X,Y,vessel.motionRAO.amp{1, motion}(:,:,1))
xlabel('Heading [deg]')
ylabel('T [s]')