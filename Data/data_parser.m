%% Reader for the C++ Variable Wing Sweep State Files
file_name = "vw_step_high_k.txt";

data = readmatrix(file_name);
time = data(:, 1);
as_count = data(:, 2);
phi = data(:, 3:5);
phi_dot = data(:, 6:8);

phi_z = phi(:, 3);
phi_dot_z = phi_dot(:, 3);

figure(1)
plot(time, phi_z);
xlabel('Time (s)', FontSize=20)
ylabel('$\phi_z$ (rad)', 'Interpreter', 'latex', FontSize=20)
title('$\phi_z$ Over Time', 'Interpreter', 'latex', FontSize=20)


figure(2)
plot(time, phi_dot_z)
xlabel('Time (s)', FontSize=20)
ylabel('$\dot{\phi}_z$ (rad/s)', 'Interpreter', 'latex', FontSize=20)
title('$\dot{\phi}_z$ Over Time', 'Interpreter', 'latex', FontSize=20)