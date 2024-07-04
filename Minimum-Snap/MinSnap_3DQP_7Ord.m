clear 
close all
clc

%% Waypoints (3 laps generation)

Poses=[-1                           -1.2                        2;
        9                           6                           1.5;
        9                           -3.5                        1.5;
       -3.75                        -5.5                        2;
       -6                          -6                           1.5;
       -3.75                        -6.5                        1;
        4                           -0.8                        1.5;
       -2.5                         6                           1.5;  
       -1                           -1.2                        2;
        9                           6                           1.5;
        9                           -3.5                        1.5;
       -3.75                        -5.5                        2;
       -6                          -6                           1.5;
       -3.75                        -6.5                        1;
        4                           -0.8                        1.5;
       -2.5                         6                           1.5;  
       -1                           -1.2                        2;
        9                           6                           1.5;
        9                           -3.5                        1.5;
       -3.75                        -5.5                        2;
       -6                          -6                           1.5;
       -3.75                        -6.5                        1;
        4                           -0.8                        1.5;
       -2.5                         6                           1.5;  
       -1                           -1.2                        2];

N_Poses=length(Poses);
Ts=0.01;

v0 = [0,0,0];
a0 = [0,0,0];
v1 = [0,0,0];
a1 = [0,0,0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% sample mid points
% Define n-1 intermediate points in between the poses

r = 1;  %% corridor r
step = r;
new_waypts = Poses(1,:);
Index=[1];

for i=2:(length(Poses))
    x1 = Poses(i-1,1);
    y1 = Poses(i-1,2);
    z1 = Poses(i-1,3);
    x2 = Poses(i,1);
    y2 = Poses(i,2);
    z2 = Poses(i,3);
    n = ceil(hypot(z1-z2,sqrt((y1-y2)^2+(x1-x2)^2))/step)+1;
    sample_pts = [linspace(x1,x2,n)' linspace(y1,y2,n)' linspace(z1,z2,n)'];
    new_waypts = [new_waypts; sample_pts(2:end,:)];
    Index=[Index length(new_waypts)];
end

% figure(1)
% plot3(new_waypts(:,1),new_waypts(:,2),new_waypts(:,3))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

new_waypts=new_waypts';
T=40;
[ts,x] = arrangeT(new_waypts,T);
new_waypts=new_waypts';

%% Trajectory Plan

n_order=7;

[polys_x,Aieq,bieq,Aeq,beq] = MS_QP(new_waypts(:,1),Index,ts,n_order,v0(1),a0(1),v1(1),a1(1),r);
[polys_y] = MS_QP(new_waypts(:,2),Index,ts,n_order,v0(2),a0(2),v1(2),a1(2),r);
[polys_z] = MS_QP(new_waypts(:,3),Index,ts,n_order,v0(3),a0(3),v1(3),a1(3),r);

%% Visualization

figure
plot3(new_waypts(:,1),new_waypts(:,2),new_waypts(:,3),'.g');hold on
plot3(Poses(:,1),Poses(:,2),Poses(:,3),'*r');hold on;

for i=1:length(new_waypts)
    plot_CUBE(new_waypts(i,:)',r);
end

title('minimum snap trajectory','FontSize',22);
tt = 0:0.01:T;

xx = polys_vals(polys_x,ts,tt,0);
yy = polys_vals(polys_y,ts,tt,0);
zz = polys_vals(polys_z,ts,tt,0);
PP=[xx;yy;zz;tt];

xv = polys_vals(polys_x,ts,tt,1);
yv = polys_vals(polys_y,ts,tt,1);
zv = polys_vals(polys_z,ts,tt,1);
VV=[xv;yv;zv;tt];

xa = polys_vals(polys_x,ts,tt,2);
ya = polys_vals(polys_y,ts,tt,2);
za = polys_vals(polys_z,ts,tt,2);
AA=[xa;ya;za;tt];

plot3(xx,yy,zz,'r');

figure
plot(tt,xx,'g')
hold on
plot(tt,yy,'r')
hold on
plot(tt,zz,'blue')
title("Position")
xlabel("t(s)")
ylabel("m")
legend('x','y','z')

figure
plot(tt,xv,'g')
hold on
plot(tt,yv,'r')
hold on
plot(tt,zv,'blue')
title("Velocity")
xlabel("t(s)")
ylabel("m/s")
legend('velocity along x','velocity along y','velocity along z')

figure
plot(tt,xa,'g')
hold on
plot(tt,ya,'r')
hold on
plot(tt,za,'blue')
title("Acceleration")
xlabel("t(s)")
ylabel("m/s^2")
legend('acc. along x','acc. along y','acc. along z')

err = [xx(1,end)-new_waypts(end,1) yy(1,end)-new_waypts(end,2) zz(1,end)-new_waypts(end,3)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FUNCTIONS

function plot_CUBE(center,r)
        p1 = center+[-r;-r;-r];
        p2 = center+[-r;r;-r];
        p3 = center+[r;r;-r];
        p4 = center+[r;-r;-r];
        p5 = center+[-r;-r;r];
        p6 = center+[-r;r;r];
        p7 = center+[r;r;r];
        p8 = center+[r;-r;r];
        plot_line(p1,p2);
        plot_line(p2,p3);
        plot_line(p3,p4);
        plot_line(p4,p1);
        plot_line(p5,p6);
        plot_line(p6,p7);
        plot_line(p7,p8);
        plot_line(p8,p5);
        plot_line(p1,p5);
        plot_line(p2,p6);
        plot_line(p3,p7);
        plot_line(p4,p8);
    end
    
    function plot_line(p1,p2)
        a = [p1(:),p2(:)];    
        plot3(a(1,:),a(2,:),a(3,:),'b');
    end

%% QP Solver

function [polys,Aieq,bieq,Aeq,beq] = MS_QP(Poses,Index,ts,n_order,v0,a0,ve,ae,r)
        
        n_poly = length(Poses)-1;
        n_coef = n_order+1;

        % compute Q
        Q_all = [];
        for i=1:n_poly
            Q_all = blkdiag(Q_all,computeQ(n_order,3,ts(i),ts(i+1)));
        end
        b_all = zeros(size(Q_all,1),1);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% PASSING THROUGH ALL GATES CENTERS and CONTINUITY CONSTRAINTS
        
        % Sfruttando la conoscenza degli indici degli istanti di tempo
        % corrispondenti al passaggio per i gate centers, si impone:
        % - continuità in pos, vel e acc per ogni punto intermedio e gate
        % centers;
        % - Posizione desiderata ai gate centers;
        % - Vel e acc desiderate a inizio e fine.

        Aeq = [];
        beq = [];
        j=2;

        for i=1:length(Poses)
            if i==1
                Aeq = [ [1 ts(i)^1 ts(i)^2 ts(i)^3 ts(i)^4 ts(i)^5 ts(i)^6 ts(i)^7], zeros(1,(n_poly-i)*n_coef);                %p0
                        [0 1 2*ts(i)^1 3*ts(i)^2 4*ts(i)^3 5*ts(i)^4 6*ts(i)^5 7*ts(i)^6], zeros(1,(n_poly-i)*n_coef);          %v0
                        [0 0 2 6*ts(i)^1 12*ts(i)^2 20*ts(i)^3 30*ts(i)^4 42*ts(i)^5], zeros(1,(n_poly-i)*n_coef)];              %a0
                beq = [Poses(i) v0 a0 ];
            elseif i==length(Poses)
                Aeq = [ Aeq;
                        zeros(1,n_coef*(i-2)), [1 ts(i)^1 ts(i)^2 ts(i)^3 ts(i)^4 ts(i)^5 ts(i)^6 ts(i)^7]; %pe
                        zeros(1,n_coef*(i-2)), [0 1 2*ts(i)^1 3*ts(i)^2 4*ts(i)^3 5*ts(i)^4 6*ts(i)^5 7*ts(i)^6]; %ve
                        zeros(1,n_coef*(i-2)), [0 0 2 6*ts(i)^1 12*ts(i)^2 20*ts(i)^3 30*ts(i)^4 42*ts(i)^5]]; %ae]
                beq = [beq Poses(i) ve ae ];
            elseif i==Index(j)
                Aeq = [ Aeq;
                        zeros(1,n_coef*(i-2)), [1 ts(i)^1 ts(i)^2 ts(i)^3 ts(i)^4 ts(i)^5 ts(i)^6 ts(i)^7], zeros(1,(n_poly-i+1)*n_coef);  %p_i
                        zeros(1,n_coef*(i-2)), [1 ts(i)^1 ts(i)^2 ts(i)^3 ts(i)^4 ts(i)^5 ts(i)^6 ts(i)^7], -[1 ts(i)^1 ts(i)^2 ts(i)^3 ts(i)^4 ts(i)^5 ts(i)^6 ts(i)^7], zeros(1,(n_poly-i)*n_coef); %0 : Position Continuity
                        zeros(1,n_coef*(i-2)), [0 1 2*ts(i)^1 3*ts(i)^2 4*ts(i)^3 5*ts(i)^4 6*ts(i)^5 7*ts(i)^6], -[0 1 2*ts(i)^1 3*ts(i)^2 4*ts(i)^3 5*ts(i)^4 6*ts(i)^5 7*ts(i)^6],zeros(1,(n_poly-i)*n_coef); %0 : Speed Continuity
                        zeros(1,n_coef*(i-2)), [0 0 2 6*ts(i)^1 12*ts(i)^2 20*ts(i)^3 30*ts(i)^4 42*ts(i)^5], -[0 0 2 6*ts(i)^1 12*ts(i)^2 20*ts(i)^3 30*ts(i)^4 42*ts(i)^5], zeros(1,(n_poly-i)*n_coef)]; %0 : Acc. Continuity
                        
                beq = [ beq Poses(i) 0 0 0]; 
                j=j+1;
            else
                Aeq = [ Aeq;
                        zeros(1,n_coef*(i-2)), [1 ts(i)^1 ts(i)^2 ts(i)^3 ts(i)^4 ts(i)^5 ts(i)^6 ts(i)^7], -[1 ts(i)^1 ts(i)^2 ts(i)^3 ts(i)^4 ts(i)^5 ts(i)^6 ts(i)^7], zeros(1,(n_poly-i)*n_coef); %0 : Position Continuity
                        zeros(1,n_coef*(i-2)), [0 1 2*ts(i)^1 3*ts(i)^2 4*ts(i)^3 5*ts(i)^4 6*ts(i)^5 7*ts(i)^6], -[0 1 2*ts(i)^1 3*ts(i)^2 4*ts(i)^3 5*ts(i)^4 6*ts(i)^5 7*ts(i)^6],zeros(1,(n_poly-i)*n_coef); %0 : Speed Continuity
                        zeros(1,n_coef*(i-2)), [0 0 2 6*ts(i)^1 12*ts(i)^2 20*ts(i)^3 30*ts(i)^4 42*ts(i)^5], -[0 0 2 6*ts(i)^1 12*ts(i)^2 20*ts(i)^3 30*ts(i)^4 42*ts(i)^5], zeros(1,(n_poly-i)*n_coef)]; %0 : Acc. Continuity
                 
                beq = [ beq 0 0 0];
            end

         end

        beq=beq';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % % % p0=Poses(1);
        % % % p1=Poses(2);
        % % % p2=Poses(3);
        % % % p3=Poses(4);
        % % % p4=Poses(5);
        % % % p5=Poses(6);
        % % % p6=Poses(7);
        % % % p7=Poses(8);
        % % % pe=Poses(9);
        % % % 
        % % % Aeq = [[1 ts(1)^1 ts(1)^2 ts(1)^3 ts(1)^4 ts(1)^5 ts(1)^6 ts(1)^7], zeros(1,(n_poly-1)*n_coef);          %p0
        % % %        [0 1 2*ts(1)^1 3*ts(1)^2 4*ts(1)^3 5*ts(1)^4 6*ts(1)^5 7*ts(1)^6], zeros(1,(n_poly-1)*n_coef);    %v0
        % % %        [0 0 2 6*ts(1)^1 12*ts(1)^2 20*ts(1)^3 30*ts(1)^4 42*ts(1)^5], zeros(1,(n_poly-1)*n_coef);        %a0
        % % % 
        % % %        [1 ts(2)^1 ts(2)^2 ts(2)^3 ts(2)^4 ts(2)^5 ts(2)^6 ts(2)^7], zeros(1,(n_poly-1)*n_coef);          %p1
        % % %        [0 1 2*ts(2)^1 3*ts(2)^2 4*ts(2)^3 5*ts(2)^4 6*ts(2)^5 7*ts(2)^6], -[0 1 2*ts(2)^1 3*ts(2)^2 4*ts(2)^3 5*ts(2)^4 6*ts(2)^5 7*ts(2)^6],zeros(1,(n_poly-2)*n_coef); %0
        % % %        [0 0 2 6*ts(2)^1 12*ts(2)^2 20*ts(2)^3 30*ts(2)^4 42*ts(2)^5], -[0 0 2 6*ts(2)^1 12*ts(2)^2 20*ts(2)^3 30*ts(2)^4 42*ts(2)^5], zeros(1,(n_poly-2)*n_coef); %0
        % % %         zeros(1,n_coef), [1 ts(2)^1 ts(2)^2 ts(2)^3 ts(2)^4 ts(2)^5 ts(2)^6 ts(2)^7], zeros(1,(n_poly-2)*n_coef); %p1
        % % % 
        % % %         zeros(1,n_coef), [1 ts(3)^1 ts(3)^2 ts(3)^3 ts(3)^4 ts(3)^5 ts(3)^6 ts(3)^7], zeros(1,(n_poly-2)*n_coef); %p2
        % % %         zeros(1,n_coef), [0 1 2*ts(3)^1 3*ts(3)^2 4*ts(3)^3 5*ts(3)^4 6*ts(3)^5 7*ts(3)^6], -[0 1 2*ts(3)^1 3*ts(3)^2 4*ts(3)^3 5*ts(3)^4 6*ts(3)^5 7*ts(3)^6],zeros(1,(n_poly-3)*n_coef); %0
        % % %         zeros(1,n_coef), [0 0 2 6*ts(3)^1 12*ts(3)^2 20*ts(3)^3 30*ts(3)^4 42*ts(3)^5], -[0 0 2 6*ts(3)^1 12*ts(3)^2 20*ts(3)^3 30*ts(3)^4 42*ts(3)^5], zeros(1,(n_poly-3)*n_coef); %0
        % % %         zeros(1,n_coef*2), [1 ts(3)^1 ts(3)^2 ts(3)^3 ts(3)^4 ts(3)^5 ts(3)^6 ts(3)^7], zeros(1,(n_poly-3)*n_coef); %p2
        % % % 
        % % %         zeros(1,n_coef*2), [1 ts(4)^1 ts(4)^2 ts(4)^3 ts(4)^4 ts(4)^5 ts(4)^6 ts(4)^7], zeros(1,(n_poly-3)*n_coef); %p3
        % % %         zeros(1,n_coef*2), [1 ts(4)^1 ts(4)^2 ts(4)^3 ts(4)^4 ts(4)^5 ts(4)^6 ts(4)^7], -[1 ts(4)^1 ts(4)^2 ts(4)^3 ts(4)^4 ts(4)^5 ts(4)^6 ts(4)^7], zeros(1,(n_poly-4)*n_coef); %p3
        % % %         zeros(1,n_coef*2), [0 1 2*ts(4)^1 3*ts(4)^2 4*ts(4)^3 5*ts(4)^4 6*ts(4)^5 7*ts(4)^6], -[0 1 2*ts(4)^1 3*ts(4)^2 4*ts(4)^3 5*ts(4)^4 6*ts(4)^5 7*ts(4)^6],zeros(1,(n_poly-4)*n_coef); %0
        % % %         zeros(1,n_coef*2), [0 0 2 6*ts(4)^1 12*ts(4)^2 20*ts(4)^3 30*ts(4)^4 42*ts(4)^5], -[0 0 2 6*ts(4)^1 12*ts(4)^2 20*ts(4)^3 30*ts(4)^4 42*ts(4)^5], zeros(1,(n_poly-4)*n_coef); %0
        % % %         zeros(1,n_coef*3), [1 ts(4)^1 ts(4)^2 ts(4)^3 ts(4)^4 ts(4)^5 ts(4)^6 ts(4)^7], zeros(1,(n_poly-4)*n_coef); %p3
        % % % 
        % % %         zeros(1,n_coef*3), [1 ts(5)^1 ts(5)^2 ts(5)^3 ts(5)^4 ts(5)^5 ts(5)^6 ts(5)^7], zeros(1,(n_poly-4)*n_coef); %p4
        % % %         zeros(1,n_coef*3), [1 ts(5)^1 ts(5)^2 ts(5)^3 ts(5)^4 ts(5)^5 ts(5)^6 ts(5)^7], -[1 ts(5)^1 ts(5)^2 ts(5)^3 ts(5)^4 ts(5)^5 ts(5)^6 ts(5)^7], zeros(1,(n_poly-5)*n_coef); %p4
        % % %         zeros(1,n_coef*3), [0 1 2*ts(5)^1 3*ts(5)^2 4*ts(5)^3 5*ts(5)^4 6*ts(5)^5 7*ts(5)^6], -[0 1 2*ts(5)^1 3*ts(5)^2 4*ts(5)^3 5*ts(5)^4 6*ts(5)^5 7*ts(5)^6],zeros(1,(n_poly-5)*n_coef); %0
        % % %         zeros(1,n_coef*3), [0 0 2 6*ts(5)^1 12*ts(5)^2 20*ts(5)^3 30*ts(5)^4 42*ts(5)^5], -[0 0 2 6*ts(5)^1 12*ts(5)^2 20*ts(5)^3 30*ts(5)^4 42*ts(5)^5], zeros(1,(n_poly-5)*n_coef); %0
        % % %         zeros(1,n_coef*4), [1 ts(5)^1 ts(5)^2 ts(5)^3 ts(5)^4 ts(5)^5 ts(5)^6 ts(5)^7], zeros(1,(n_poly-5)*n_coef); %p4
        % % % 
        % % %         zeros(1,n_coef*4), [1 ts(6)^1 ts(6)^2 ts(6)^3 ts(6)^4 ts(6)^5 ts(6)^6 ts(6)^7], zeros(1,(n_poly-5)*n_coef); %p5
        % % %         zeros(1,n_coef*4), [0 1 2*ts(6)^1 3*ts(6)^2 4*ts(6)^3 5*ts(6)^4 6*ts(6)^5 7*ts(6)^6], -[0 1 2*ts(6)^1 3*ts(6)^2 4*ts(6)^3 5*ts(6)^4 6*ts(6)^5 7*ts(6)^6],zeros(1,(n_poly-6)*n_coef); %0
        % % %         zeros(1,n_coef*4), [0 0 2 6*ts(6)^1 12*ts(6)^2 20*ts(6)^3 30*ts(6)^4 42*ts(6)^5], -[0 0 2 6*ts(6)^1 12*ts(6)^2 20*ts(6)^3 30*ts(6)^4 42*ts(6)^5], zeros(1,(n_poly-6)*n_coef); %0
        % % %         zeros(1,n_coef*5), [1 ts(6)^1 ts(6)^2 ts(6)^3 ts(6)^4 ts(6)^5 ts(6)^6 ts(6)^7], zeros(1,(n_poly-6)*n_coef); %p5
        % % % 
        % % %         zeros(1,n_coef*5), [1 ts(7)^1 ts(7)^2 ts(7)^3 ts(7)^4 ts(7)^5 ts(7)^6 ts(7)^7], zeros(1,(n_poly-6)*n_coef); %p6
        % % %         zeros(1,n_coef*6), [1 ts(7)^1 ts(7)^2 ts(7)^3 ts(7)^4 ts(7)^5 ts(7)^6 ts(7)^7], zeros(1,(n_poly-7)*n_coef); %p6
        % % % 
        % % %         zeros(1,n_coef*6), [1 ts(8)^1 ts(8)^2 ts(8)^3 ts(8)^4 ts(8)^5 ts(8)^6 ts(8)^7], zeros(1,(n_poly-7)*n_coef); %p7
        % % %         zeros(1,n_coef*7), [1 ts(8)^1 ts(8)^2 ts(8)^3 ts(8)^4 ts(8)^5 ts(8)^6 ts(8)^7]; %p7
        % % % 
        % % %         zeros(1,n_coef*7), [1 ts(9)^1 ts(9)^2 ts(9)^3 ts(9)^4 ts(9)^5 ts(9)^6 ts(9)^7]; %pe
        % % %         zeros(1,n_coef*7), [0 1 2*ts(9)^1 3*ts(9)^2 4*ts(9)^3 5*ts(9)^4 6*ts(9)^5 7*ts(9)^6]; %ve
        % % %         zeros(1,n_coef*7), [0 0 2 6*ts(9)^1 12*ts(9)^2 20*ts(9)^3 30*ts(9)^4 42*ts(9)^5]]; %ae
        % % % 
        % % %  beq = [p0 v0 a0 p1 0 0 p1 p2 0 0 p2 p3 0 0 0 p3 p4 0 0 0 p4 p5 0 0 p5 p6 p6 p7 p7 pe ve ae]'; 
        % % % 
        % % % Quanto scritto sopra non stabilisce la continuità su tutto in
        % quanto si è implementato il metodo ricorsivo, resta a scopo
        % dimostrativo.

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % CORRIDOR CONSTRAINTS: WIP

        Aieq = [];
        bieq = [];

        for i=2:length(Poses)-1

            Aieq = [Aieq;
                    zeros(1,n_coef*(i-1)), [1 ts(i)^1 ts(i)^2 ts(i)^3 ts(i)^4 ts(i)^5 ts(i)^6 ts(i)^7], zeros(1,(n_poly-i)*n_coef);
                    zeros(1,n_coef*(i-1)), -[1 ts(i)^1 ts(i)^2 ts(i)^3 ts(i)^4 ts(i)^5 ts(i)^6 ts(i)^7], zeros(1,(n_poly-i)*n_coef)];

            bieq = [bieq; Poses(i)+r; r-Poses(i)];

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        options = optimoptions('quadprog','MaxIterations',200);
        Poses = quadprog(Q_all,b_all,Aieq,bieq,Aeq,beq,[],[],[],options);
        
        polys = reshape(Poses,n_coef,n_poly);
    
    end

%% Compute Q

    function Q = computeQ(n,r,t1,t2)
        T = zeros((n-r)*2+1,1);
        for i = 1:(n-r)*2+1
            T(i) = t2^i-t1^i;
        end
        Q = zeros(n);
        for i = r+1:n+1
            for j = i:n+1
                k1 = i-r-1;
                k2 = j-r-1;
                k = k1+k2+1;
                Q(i,j) = prod(k1+1:k1+r)*prod(k2+1:k2+r)/k*T(k);
                Q(j,i) = Q(i,j);
            end
        end
    end

%% Arrange T

function [ts,x] = arrangeT(waypts,T)
    x = waypts(:,2:end) - waypts(:,1:end-1);
    dist = sum(x.^2,1).^0.5;
    k = T/sum(dist);
    ts = [0 cumsum(dist*k)];
end

%% poly_vals & poly_val

function val = poly_val(poly,t,r)
    val = 0;
    n = length(poly)-1;
    if r<=0
        for i=0:n
            val = val+poly(i+1)*t^i;
        end
    else
        for i=r:n
            a = poly(i+1)*prod(i-r+1:i)*t^(i-r);
            val = val + a;
        end
    end
end

function vals = polys_vals(polys,ts,tt,r)
idx = 1;
N = length(tt);
vals = zeros(1,N);
for i = 1:N
    t = tt(i);
    if t<ts(idx)
        vals(i) = 0;
    else
        while idx<length(ts) && t>ts(idx+1)+0.0001
            idx = idx+1;
        end
        vals(i) = poly_val(polys(:,idx),t,r);
    end
end
end

