close all hidden;
clearvars -except;
clc;

rng(418);                       %seed for reproducibility
%% _*PROBLEM 1 (25 Pts.)*_
%% 1.1 Data generation

Tsampling = 0.01;      
Tfinal = 40;            
t = 0:Tsampling:(Tfinal - Tsampling);     %to produce exactly N=4000 samples
N = numel(t);

Tzero = 2;                                % 2 seconds zero input (segmentation for bias estimation)
Nz = round(Tzero / Tsampling);            % number of zero samples
% PRBS input

dwell = 0.2;
L = round(dwell / Tsampling);
nLev = ceil((N - Nz) / L);                %PRBS only after zero segment

%first producing levels
lev = zeros(nLev, 1);
for k = 1:nLev
    if rand > 0.5
        lev(k) = 1;
    else
        lev(k) = -1;
    end
end

%then dwelling them for the input
u_prbs = zeros(N - Nz, 1);                    
idx = 1;                                       
for k = 1:nLev
    for j = 1:L
        if idx > (N - Nz)                     
            break;
        end
        u_prbs(idx) = lev(k);                  
        idx = idx + 1;
    end
end

u = [zeros(Nz,1); u_prbs];                 % zero input + PRBS
u_t = u;                                   % ZOH-held input
% Plant & Sensor

s  = tf('s');                                                  
Gp = 5/(s^2 + 1.2*s + 5);                                      %plant transfer function
Gs = 1/(0.15*s + 1);                                           %sensor dynamics
y_plant = lsim(Gp, u, t, 'zoh');                               %defining "zoh" at lsim for both plant and the sensor
y_sens  = lsim(Gs, y_plant, t, 'zoh');
% Bias, noise, spikes

bias  = -0.25;                                                  %sensor bias
sigma = 0.05;                                                   %zero-mean gaussian for measurement noise
y_raw = y_sens + bias + sigma*randn(size(y_sens));                                                   
idx_spike = randsample((Nz+2):(N-1), 25);                       %spikes only in PRBS region       
y_raw(idx_spike) = y_raw(idx_spike) + 0.5 * sign(randn(numel(idx_spike), 1));    %adding twenty five 0.5 magnitude spikes
% Plots

figure;
plot(t, u); grid on; xlabel('Time [s]'); ylabel('u[k]');    %u[k] input plotting
title('Input u[k] vs Time');

figure;
stairs(t, u_t, 'LineWidth', 1); grid on;                   %u(t) ZOH plot
xlabel('Time [s]'); ylabel('u(t)');
title('Continuous-time ZOH Input u(t)');

figure; 
plot(t, y_raw); grid on; xlabel('Time [s]'); ylabel('y[k]'); %raw y[k] plotting with bias, noise, spikes
title('Raw Output y[k] vs Time');
%% 1.2 Data cleaning
% (a) spike
% We detect spikes using the Median Absolute Deviation method,a any sample that 
% deviates more than 4*mean_abs_dev from the median is considered a spike.

y = y_raw;
spikes = false(size(y));   
med = median(y); 
mad_val = median(abs(y - med));  

for i = (Nz+1):N                             % do not detect spikes inside the zero-input segment
    if abs(y(i) - med) > 4 * mad_val
        spikes(i) = true;
    end
end

% replace spikes by NaN (marking them for interpolation)
y(spikes) = NaN;

% linear interpolation
for i = 2:N-1
    if isnan(y(i))
        y(i) = (y(i-1) + y(i+1)) / 2;
    end
end
% (b) Bias 

bias_val = mean(y(1:Nz));                   %proper bias estimation
y = y - bias_val;
fprintf('Removed bias: %.4f\n', bias_val);
% (c) Moving average
% To reduce measurement noise, we apply a moving average filter. For each sample, 
% movmean replaces y[k] with the average of the previous M samples. We use M = 
% 5, which smooths the noise without distorting the dynamics. The same filter 
% is also applied to the input signal because filtered input will be used in the 
% next problem.

M = 5;                                  % number of steps used
y_clean = movmean(y, M);                % used built-in func. instead of hardcoding
u_filter = movmean(u, M);
% Raw and Clean Plot

figure;
plot(t, y_raw, 'r'); hold on;
plot(t, y_clean, 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('y');
legend('Raw','Cleaned');
title('Raw vs Cleaned Output');
grid on;



save('q1_data.mat', 'u_filter', 'y_clean', 'Tsampling', 't');

