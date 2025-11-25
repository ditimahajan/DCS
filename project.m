clear; close all; clc;

%% ---------------- 1) Orbit & continuous model -----------------
mu = 3.986004418e14;    % m^3/s^2
Re = 6371000;           % m
h0 = 500000;            % chosen altitude (m) - match your earlier runs
r0 = Re + h0;
n = sqrt(mu / r0^3);
fprintf('Mean motion n = %.6e rad/s\n', n);

A = [0 1; 3*n^2 0];
B = [0; 1];
C = [1 0];
D = 0;

%% ---------------- 2) Discretize (ZOH) -------------------------
Ts = 1.0;                % sampling time [s]
sysc = ss(A,B,C,D);
sysd = c2d(sysc, Ts, 'zoh');
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C;

%% ---------------- 3) Augment integrator (discrete LQI) ----------
% Use integrator state that accumulates error (r - y)
% Augmentation consistent with discrete-time integrator: z[k+1] = z[k] + Ts*(r - y)
Aa = [Ad, zeros(2,1); -Ts*Cd, 1];
Ba = [Bd; 0];

%% ---------------- 4) LQI tuning (balanced) ----------------------
% These values are tuned to produce a balanced/realistic result (~1.7 m/s DV)
Q = diag([5e5, 2e3, 5e4]);   % [x, xdot, integrator]
R = 5e-4;                    % control penalty

K = dlqr(Aa, Ba, Q, R);
Kx = K(:,1:2); Ki = K(:,3);
fprintf('LQI gains: Kx = [%.3e  %.3e], Ki = %.3e\n', Kx(1), Kx(2), Ki);

%% ---------------- 5) Simulation parameters ----------------------
Tsim = 900;               % total simulation time (s) - matches your screenshots
N = round(Tsim / Ts);
t = (0:N-1)*Ts;

% disturbance (small constant; you may add pulses)
dist = zeros(1,N);
% optional small pulse (example): dist(380:383) = -1e-6;   % if you want to emulate
dist_const = 1e-6;         % baseline disturbance magnitude
dist(:) = dist_const;      % constant small disturbance

% thruster limits & smoothing
u_max = 2e-3;              % [m/s^2] (tuned to match earlier plots)
u_min = -u_max;
u_rate_limit = 5e-4;       % m/s^2 per second (max change per sample)
u_filter_tau = 2.0;        % LPF time constant (s)
K_aw = 0.05;               % anti-windup back-calculation gain

% initial conditions
x = [100; 0];              % initial radial offset 100 m like your screenshots
z = 0;                     % integrator initial
r_ref = 0;

% fuel / spacecraft parameters (used to estimate fuel mass)
m0 = 200;                  % initial mass [kg], set to satellite mass (example)
Isp = 300;                 % [s] specific impulse
g0 = 9.80665;              % m/s^2
ve = Isp * g0;             % effective exhaust velocity [m/s]

% logs
X = zeros(2, N); U = zeros(1,N); Z = zeros(1,N); CumDV = zeros(1,N);
Mfuel = zeros(1,N);        % remaining mass after each burn (estimate)
Mfuel(1) = m0;

% internal variables
u_filt = 0;

%% ---------------- 6) Simulation loop ----------------------------
for k = 1:N
    % measurement
    y = C * x;
    % control law (unsaturated)
    u_unsat = -Kx * x - Ki * z;
    % saturate
    u_sat = min(max(u_unsat, u_min), u_max);
    % back-calculation anti-windup on integrator
    z = z + Ts * (r_ref - y) + K_aw*(u_sat - u_unsat);
    % rate limit (delta u per sample)
    max_du = u_rate_limit * Ts;
    du = u_sat - u_filt;
    if du > max_du
        u_step = u_filt + max_du;
    elseif du < -max_du
        u_step = u_filt - max_du;
    else
        u_step = u_sat;
    end
    % LPF smoothing
    alpha = Ts / (u_filter_tau + Ts);
    u_filt = (1-alpha)*u_filt + alpha*u_step;
    u_applied = u_filt;
    % plant update (discrete)
    x = Ad * x + Bd * (u_applied + dist(k));
    % log
    X(:,k) = x;
    U(k) = u_applied;
    Z(k) = z;
    if k==1
        CumDV(k) = abs(U(k)) * Ts;
    else
        CumDV(k) = CumDV(k-1) + abs(U(k)) * Ts;
    end
    % estimate fuel consumed so far using ideal rocket equation
    % Use delta-v to date = CumDV(k). Using m(t) = m0 * exp(-dv/ve).
    m_remain = m0 * exp(-CumDV(k) / ve);
    Mfuel(k) = m_remain;
end

total_dv = CumDV(end);
fuel_used = m0 - Mfuel(end);
fprintf('Total Δv (approx) = %.6f m/s\n', total_dv);
fprintf('Estimated fuel used (ideal rocket eq.) = %.6f kg (m0 = %.1f kg, Isp = %.1f s)\n', fuel_used, m0, Isp);

%% ---------------- 7) Combined figure (all curves together) ----------
figure('Name','Radial LQI - All plots','NumberTitle','off','Position',[100 100 900 900]);

% Subplot 1: radial position
subplot(4,1,1);
plot(t, X(1,:), 'LineWidth', 1.5); grid on;
ylabel('x (m)'); title('Radial position x(t)');

% Subplot 2: radial velocity
subplot(4,1,2);
plot(t, X(2,:), 'LineWidth', 1.5); grid on;
ylabel('xdot (m/s)'); title('Radial velocity \dot{x}(t)');

% Subplot 3: control accel (show in mm/s^2 for readability)
subplot(4,1,3);
plot(t, U*1e3, 'LineWidth', 1.5); grid on;
ylabel('u (mm/s^2)'); title(sprintf('Control acceleration u(t)  |  Total Δv = %.6f m/s', total_dv));

% Subplot 4: cumulative Δv and fuel remaining (dual y-axis)
subplot(4,1,4);
yyaxis left;
plot(t, CumDV, 'LineWidth', 1.5); grid on;
ylabel('Cumulative \Delta v (m/s)');
yyaxis right;
plot(t, Mfuel, '--', 'LineWidth', 1.5);
ylabel('Estimated remaining mass (kg)');
xlabel('Time (s)');
title(sprintf('Cumulative Δv and estimated remaining mass (m0=%.0f kg, Isp=%.0f s)', m0, Isp));
legend('Cumulative \Delta v','Remaining mass','Location','best');

% make plots tight and readable
linkaxes(findall(gcf,'Type','axes'),'x');

%% ---------------- 8) Save results & figures -------------------------
outdir = fullfile(pwd, 'sat_control_results_all');
if ~exist(outdir, 'dir')
    mkdir(outdir);
end
saveas(gcf, fullfile(outdir, 'all_plots_combined.png'));
save(fullfile(outdir, 'radial_LQI_all_results.mat'), 'X','U','CumDV','Mfuel','t','K','Kx','Ki','Q','R');

fprintf('Saved plots and workspace to: %s\n', outdir);