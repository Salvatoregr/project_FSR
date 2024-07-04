% Caricamento delle variabili dal workspace
err_R = out.err_R.signals.values;
err_W = out.err_W.signals.values;
err_p = out.err_p.signals.values;
pos = out.pos.signals.values;
taub = out.tau_b.signals.values;
uT = out.uT.signals.values;
traj = out.traj.signals.values;
pos_d = out.pos_d.signals.values;
t = out.traj.time;

% Aggiustamento delle dimensioni per le variabili err_R, err_W, err_p e pos
err_R = err_R(1:length(t), :);
err_W = err_W(1:length(t), :);
err_p = err_p(1:length(t), :);
pos = pos(1:length(t), :);
pos_d = pos(1:length(t), :);

% Plot degli errori
figure;
plot(t, err_R(:,1), 'LineWidth', 1.5);
hold on;
plot(t, err_R(:,2), 'LineWidth', 1.5);
plot(t, err_R(:,3), 'LineWidth', 1.5);
plot(t, err_p(:,1), 'LineWidth', 1.5);
plot(t, err_p(:,2), 'LineWidth', 1.5);
plot(t, err_p(:,3), 'LineWidth', 1.5);
title('Errors');
xlabel('Time (s)');
ylabel('error');
legend('err_R_x', 'err_R_y', 'err_R_z', 'err_P_x', 'err_P_y', 'err_P_z');
hold off;

% Plot della posizione (con tre vettori separati)
figure;
plot(t, pos(:,1), 'LineWidth', 1.5);
hold on
plot(t, pos_d(:,1), 'LineWidth', 1.5);
title('Position along X and reference');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x','x_{ref}')
hold off

figure
plot(t, pos(:,2), 'LineWidth', 1.5);
title('Position along Y and reference');
xlabel('Time (s)');
ylabel('Position Y (m)');
hold on
plot(t, pos_d(:,2), 'LineWidth', 1.5);
legend('y','y_{ref}')
hold off

figure
plot(t, pos(:,3), 'LineWidth', 1.5);
title('Position along Z and reference');
xlabel('Time (s)');
ylabel('Position Z (m)');
hold on
plot(t, pos_d(:,3), 'LineWidth', 1.5);
legend('z','z_{ref}')
hold off

% Plot della tau_b
figure;
plot(t, taub(1:length(t), :), 'LineWidth', 1.5);
title('Tau_b (N)');
xlabel('Time (s)');
ylabel('tau_b (Nm)');
legend(arrayfun(@(x) sprintf('tau_b_%d', x), 1:size(taub, 2), 'UniformOutput', false));

% Plot della uT
figure;
plot(t, uT(1:length(t), :), 'LineWidth', 1.5);
title('uT');
xlabel('Time (s)');
ylabel('uT');
legend(arrayfun(@(x) sprintf('u_T', x), 1:size(uT, 2), 'UniformOutput', false));

% Waypoints
waypoints = [-1   -1.2  2;
              9    6    1.5;
              9   -3.5  1.5;
             -3.75 -5.5  2;
             -6   -6    1.5;
             -3.75 -6.5  1;
              4   -0.8  1.5;
             -2.5  6    1.5]; 

% Plot della traiettoria in 3D con waypoints
figure;
plot3(traj(:,1), traj(:,2), traj(:,3), 'LineWidth', 1.5);
hold on;
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
hold off;
title('3D Trajectory with Waypoints');
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Trajectory', 'Waypoints');
