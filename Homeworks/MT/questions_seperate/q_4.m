%% _*PROBLEM 4 (25 Pts.)*_
% Problem setup

% Plant dynamics: x[k+1] = 0.9 x[k] + 2 u[k],  y[k]   = x[k]
A = 0.9;
B = 2.0;
C = 1.0;
T = 0.05;                % sample time
Tf = 6;                  % simulation time
Nsim = round(Tf/T);


Np = 8;       % MPC prediction horizon
Nc = 4;       % MPC control horizon

Q = 1;        % Cost weights
R = 0.1;      % Cost weights


u_max  = 0.55; % Constraints
du_max = 0.30; % Constraints


F = zeros(Np,1); % F vector
Ap = A;
for i = 1:Np
    F(i) = C * Ap;
    Ap = Ap * A;
end

H = zeros(Np, Nc); % H matrix
for i = 1:Np
    for j = 1:Nc
        if j <= i
            Aexp = 1;
            for k = 1:(i-j)
                Aexp = Aexp * A;
            end
            H(i,j) = C * (Aexp * B);
        end
    end
end

% S matrix
S = tril(ones(Nc));

% Phi matrix
Phi = H * S;

Qbar = Q * eye(Np);
Rbar = R * eye(Nc);

options = optimoptions('quadprog','Display','off');
% (a) Design MPC, reference is step with magnitude 2

r_step = 2;

x = 0;        % state
u_prev = 0;   % previous control

% For 4(d): store first predicted output y_hat[k+1|k]
yhat_hist_a = zeros(Nsim,1);
yhat_hist_b = zeros(Nsim,1);
yhat_hist_c = zeros(Nsim,1);

u_hist_a = zeros(Nsim,1);
ref_hist_a = zeros(Nsim,1);

for k = 1:Nsim
    
    % Reference over prediction horizon
    r_vec = r_step * ones(Np,1);
    
    % b_k
    u_vec = u_prev * ones(Nc,1);
    b_k = F*x + H*u_vec;
    
    % QP
    Hqp = Phi.' * Qbar * Phi + Rbar;
    fqp = Phi.' * Qbar * (b_k - r_vec);
    
    % Constraints
    A_rate = [ eye(Nc); -eye(Nc) ];
    b_rate = [ du_max*ones(Nc,1); du_max*ones(Nc,1) ];
    
    A_u = [ S; -S ];
    b_u = [ u_max*ones(Nc,1) - u_vec;
            u_max*ones(Nc,1) + u_vec ];
    
    Aineq = [A_rate; A_u];
    bineq = [b_rate; b_u];
    
    % Solve
    dU = quadprog(Hqp, fqp, Aineq, bineq, [], [], [], [], [], options);
    
    % Predicted output horizon for case (a)
    Ypred_a = b_k + Phi*dU;
    yhat_hist_a(k) = Ypred_a(1);     % one–step ahead prediction

    du = dU(1);
    u = u_prev + du;
    
    % Plant update
    x = A*x + B*u;
    y = x;
    
    % Log
    y_hist_a(k) = y;
    u_hist_a(k) = u;
    ref_hist_a(k) = r_step;
    
    u_prev = u;
end

t_mpc = (0:Nsim-1)*T;

figure; 
plot(t_mpc, y_hist_a,'b','LineWidth',1.4); hold on;
plot(t_mpc, ref_hist_a,'r--','LineWidth',1.4);
xlabel('Time [s]'); ylabel('y[k]');
title('4(a) — MPC Tracking (Step Reference = 2)');
legend('y[k]','Reference'); grid on;

figure;
stairs(t_mpc, u_hist_a,'b','LineWidth',1.4);
xlabel('Time [s]'); ylabel('u[k]');
title('4(a) — MPC Control Input');
grid on;
% (b) Part a controller with measurement noise

noise_amp = 0.15;   % Visible amplitude

x = 0;
u_prev = 0;

y_hist_b = zeros(Nsim,1);
u_hist_b = zeros(Nsim,1);
ref_hist_b = zeros(Nsim,1);

for k = 1:Nsim

    r_vec = r_step * ones(Np,1);

    u_vec = u_prev * ones(Nc,1);
    b_k = F*x + H*u_vec;

    Hqp = Phi.'*Qbar*Phi + Rbar;
    fqp = Phi.'*Qbar*(b_k - r_vec);

    A_rate = [ eye(Nc); -eye(Nc) ];
    b_rate = [ du_max*ones(Nc,1); du_max*ones(Nc,1) ];

    A_u = [ S; -S ];
    b_u = [ u_max*ones(Nc,1) - u_vec;
            u_max*ones(Nc,1) + u_vec ];

    Aineq = [A_rate; A_u];
    bineq = [b_rate; b_u];

    dU = quadprog(Hqp, fqp, Aineq, bineq, [], [], [], [], [], options);

    % Predicted output horizon for case (b)
    Ypred_b = b_k + Phi*dU;
    yhat_hist_b(k) = Ypred_b(1);


    du = dU(1);
    u = u_prev + du;

    % True plant
    x_true = A*x + B*u;

    % Measurement
    y_meas = x_true + noise_amp*randn;

    % The controller uses y_meas as state estimate
    x = y_meas;

    y_hist_b(k) = y_meas;
    u_hist_b(k) = u;
    ref_hist_b(k) = r_step;

    u_prev = u;
end

figure; 
plot(t_mpc, y_hist_b,'b','LineWidth',1.4); hold on;
plot(t_mpc, ref_hist_b,'r--','LineWidth',1.4);
xlabel('Time [s]'); ylabel('y[k]');
title('4(b) — MPC Tracking with Measurement Noise (Step Reference = 2)');
legend('Measured y[k]','Reference'); grid on;

figure;
stairs(t_mpc, u_hist_b,'b','LineWidth',1.4);
xlabel('Time [s]'); ylabel('u[k]');
title('4(b) — MPC Control Input with Measurement Noise');
grid on;
%  (c) Same construct at part b but reference is a sine wave r[k] = sin(0.1*pi/3 * k)

x = 0;
u_prev = 0;

y_hist_c = zeros(Nsim,1);
u_hist_c = zeros(Nsim,1);
ref_hist_c = zeros(Nsim,1);

for k = 1:Nsim
    
    % Sine reference Np steps ahead
    ref_horizon = sin( (0.1*pi/3) * (k : k+Np-1) ).';
    
    u_vec = u_prev * ones(Nc,1);
    b_k = F*x + H*u_vec;

    Hqp = Phi.'*Qbar*Phi + Rbar;
    fqp = Phi.'*Qbar*(b_k - ref_horizon);

    A_rate = [ eye(Nc); -eye(Nc) ];
    b_rate = [ du_max*ones(Nc,1); du_max*ones(Nc,1) ];

    A_u = [ S; -S ];
    b_u = [ u_max*ones(Nc,1) - u_vec;
            u_max*ones(Nc,1) + u_vec ];

    Aineq = [A_rate; A_u];
    bineq = [b_rate; b_u];

    dU = quadprog(Hqp, fqp, Aineq, bineq, [], [], [], [], [], options);
    
    % Predicted output horizon for case (c)
    Ypred_c = b_k + Phi*dU;
    yhat_hist_c(k) = Ypred_c(1);

    du = dU(1);
    u = u_prev + du;

    x_true = A*x + B*u;
    y_meas = x_true + noise_amp*randn;

    x = y_meas;

    y_hist_c(k) = y_meas;
    u_hist_c(k) = u;
    ref_hist_c(k) = ref_horizon(1);

    u_prev = u;
end

figure; 
plot(t_mpc, y_hist_c,'b','LineWidth',1.4); hold on;
plot(t_mpc, ref_hist_c,'r--','LineWidth',1.4);
xlabel('Time [s]'); ylabel('y[k]');
title('4(c) — MPC Tracking with Noise (Sine Reference)');
legend('Measured y[k]','Reference'); grid on;

figure;
stairs(t_mpc, u_hist_c,'b','LineWidth',1.4);
xlabel('Time [s]'); ylabel('u[k]');
title('4(c) — MPC Control Input (Sine Reference + Noise)');
grid on;

% (d) Reference tracking vs predicted output

figure;

subplot(3,1,1);
plot(t_mpc, ref_hist_a, 'r--', 'LineWidth', 1.3); hold on;
plot(t_mpc, y_hist_a,   'b',   'LineWidth', 1.4);
plot(t_mpc, yhat_hist_a,'g',   'LineWidth', 1.1);
grid on;
title('(a): Step reference, noiseless');
ylabel('y[k]');
legend('Reference r[k]', 'Measured y[k]', 'yhat_{k+1|k}', 'Location', 'best');

subplot(3,1,2);
plot(t_mpc, ref_hist_b, 'r--', 'LineWidth', 1.3); hold on;
plot(t_mpc, y_hist_b,   'b',   'LineWidth', 1.4);
plot(t_mpc, yhat_hist_b,'g',   'LineWidth', 1.1);
grid on;
title('(b): Step reference + measurement noise');
ylabel('y[k]');
legend('Reference r[k]', 'Measured y[k]', 'yhat_{k+1|k}', 'Location', 'best');

subplot(3,1,3);
plot(t_mpc, ref_hist_c, 'r--', 'LineWidth', 1.3); hold on;
plot(t_mpc, y_hist_c,   'b',   'LineWidth', 1.4);
plot(t_mpc, yhat_hist_c,'g',   'LineWidth', 1.1);
grid on;
title('(c): Sine reference + measurement noise');
xlabel('Time [s]');
ylabel('y[k]');
legend('Reference r[k]', 'Measured y[k]', 'yhat_{k+1|k}', 'Location', 'best');