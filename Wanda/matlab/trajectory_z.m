clc; clear all; close all; format compact;

C = sym( 'c', [1,7], 'real' );

%% Define constraint equations

% Start at 0
t = 0;
Eq(1) = C(1)*t^0 + C(2)*t^1 + C(3)*t^2 + C(4)*t^3 + C(5)*t^4 + C(6)*t^5 + C(7)*t^6 == 0;

% End at 0
t = 1;
Eq(2) = C(1)*t^0 + C(2)*t^1 + C(3)*t^2 + C(4)*t^3 + C(5)*t^4 + C(6)*t^5 + C(7)*t^6 == 0;

% Reach the peak in the middle
t = 0.5;
Eq(3) = C(1)*t^0 + C(2)*t^1 + C(3)*t^2 + C(4)*t^3 + C(5)*t^4 + C(6)*t^5 + C(7)*t^6 == 1;

% Start with no velocity
t = 0;
Eq(4) = C(2)*t^0 + 2*C(3)*t^1 + 3*C(4)*t^2 + 4*C(5)*t^3 + 5*C(6)*t^4 + 6*C(7)*t^5 == 0;

% End with no velocity
t = 1;
Eq(5) = C(2)*t^0 + 2*C(3)*t^1 + 3*C(4)*t^2 + 4*C(5)*t^3 + 5*C(6)*t^4 + 6*C(7)*t^5 == 0;

% Start with no acceleration
t = 0;
Eq(6) = 2*C(3)*t^0 + 2*3*C(4)*t^1 + 3*4*C(5)*t^2 + 4*5*C(6)*t^3 + 5*6*C(7)*t^4 == 0;

% End with no acceleration
t = 1;
Eq(7) = 2*C(3)*t^0 + 2*3*C(4)*t^1 + 3*4*C(5)*t^2 + 4*5*C(6)*t^3 + 5*6*C(7)*t^4 == 0;

solution = solve( Eq, C );

C1 = solution.c1
C2 = solution.c2
C3 = solution.c3
C4 = solution.c4
C5 = solution.c5
C6 = solution.c6
C7 = solution.c7

%% Plot the trajectory to validate it
t_span = 0:0.001:1;
z_traj = C1*t_span.^0 + C2*t_span.^1 + C3*t_span.^2 + C4*t_span.^3 + C5*t_span.^4 + C6*t_span.^5 + C7*t_span.^6;
z_vel = C2*t_span.^0 + 2*C3*t_span.^1 + 3*C4*t_span.^2 + 4*C5*t_span.^3 + 5*C6*t_span.^4 + 6*C7*t_span.^5;
z_accel = 2*C3*t_span.^0 + 2*3*C4*t_span.^1 + 3*4*C5*t_span.^2 + 4*5*C6*t_span.^3 + 5*6*C7*t_span.^4;

figure()
grid on
hold on
xlabel('Phase')
ylabel('Foot Position Z (m)')
plot(t_span, z_traj, 'LineWidth', 2)

figure()
grid on
hold on
xlabel('Phase')
ylabel('Foot Velocity Z (m/s)')
plot(t_span, z_vel, 'LineWidth', 2)

figure()
grid on
hold on
xlabel('Phase')
ylabel('Foot Acceleration Z (m/s^2)')
plot(t_span, z_accel, 'LineWidth', 2)

