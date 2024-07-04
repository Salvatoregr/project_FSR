function x_dot = NMPC_XFcn(states,u)

    % STATES
    x=states(1);
    y=states(2);
    z=states(3);
    x_d=states(4);
    y_d=states(5);
    z_d=states(6);
    phi=states(7);
    theta=states(8);
    psi=states(9);
    phi_d=states(10);
    theta_d=states(11);
    psi_d=states(12);
    
    % INPUTS
    u1=u(1);
    u2=u(2);
    u3=u(3);
    u4=u(4);
    
    % PARAMS
    m=0.85;
    d=2*0.15;
    J=diag([2.5 2.1 4.3])*10^-3;
    c=0.22;
    g=9.81;
    
    % DYNAMIC EQUATIONS
    x_dd=sin(theta)/m*(u1+u2+u3+u4);
    y_dd=-sin(phi)/m*(u1+u2+u3+u4);
    z_dd=cos(phi)*cos(theta)/m*(u1+u2+u3+u4)-g;
    phi_dd=d/J(1,1)*(u2-u4);
    theta_dd=d/J(2,2)*(u1-u3);
    psi_dd=c/J(3,3)*(-u1+u2-u3+u4);

    x_dot=[x_d,y_d,z_d,x_dd,y_dd,z_dd,phi_d,theta_d,psi_d,phi_dd,theta_dd,psi_dd]';
end


