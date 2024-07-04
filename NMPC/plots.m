%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Poses=[-1                           -1.2                        2;
%         9                           6                           1.5;
%         9                           -3.5                        1.5;
%        -3.75                        -5.5                        2;
%        -6                          -6                           1.5;
%        -3.75                        -6.5                        1;
%         4                           -0.8                        1.5;
%        -2.5                         6                           1.5;  
%        -1                           -1.2                        2;
%         9                           6                           1.5;
%         9                           -3.5                        1.5;
%        -3.75                        -5.5                        2;
%        -6                          -6                           1.5;
%        -3.75                        -6.5                        1;
%         4                           -0.8                        1.5;
%        -2.5                         6                           1.5;  
%        -1                           -1.2                        2;
%         9                           6                           1.5;
%         9                           -3.5                        1.5;
%        -3.75                        -5.5                        2;
%        -6                          -6                           1.5;
%        -3.75                        -6.5                        1;
%         4                           -0.8                        1.5;
%        -2.5                         6                           1.5;  
%        -1                           -1.2                        2];
% 

load("WORKSPACE.mat")

figure(1)
plot3(Poses(:,1),Poses(:,2),Poses(:,3),'*r')
hold on;
plot3(X_REF,Y_REF,Z_REF,'r')
plot3(X,Y,Z,'g')
title("3D trajectory tracking: Nonlinear MPC")
legend("Gates centers","Reference","Quadcopter trajectory using Nonlinear MPC")
hold off

figure(2)
plot(Time,X_REF,'r')
hold on
plot(Time,X,'g')
title("Position along X and reference")
legend("X ref.","X")
xlabel("time (s)")
ylabel("X (m)")
hold off

figure(3)
plot(Time,Y_REF,'r')
hold on
plot(Time,Y,'g')
title("Position along Y and reference")
legend("Y ref.","Y")
xlabel("time (s)")
ylabel("Y (m)")
hold off

figure(4)
plot(Time,Z_REF,'r')
hold on
plot(Time,Z,'g')
title("Position along Z and reference")
legend("Z ref.","Z")
xlabel("time (s)")
ylabel("Z (m)")
hold off

figure(5)
plot(Time,PSI_REF,'r')
hold on
plot(Time,PSI,'g')
title("Psi and reference")
legend("Psi ref.","Psi")
xlabel("time (s)")
ylabel("Psi (rad)")
hold off

figure(6)
plot(tU,U1)
hold on
plot(tU,U2,'g')
hold on
plot(tU,U3,'r')
hold on
plot(tU,U4,'m')
title("Control inputs")
legend("U1","U2","U3","U4")
xlabel("time (s)")
ylabel("u (N)")
hold off