%% _*PROBLEM 2 (25 Pts.)*_

load('q1_data.mat');


% Consequent NAN's may occure (which our hardcoded linear interpolation is not available to overcome).
% Because spike detection is hardcoded (not with the built-in MATLAB function) this minor error is inevitable. 
% Remove remaining NaNs from filtered signals, at spike interpolation (with built-in function)

u_filter = fillmissing(u_filter, 'linear');
y_clean  = fillmissing(y_clean, 'linear');
% (a) Split data into training (first 25 s) and validation (last 15 s) , estimate FIR parameters for nb = 40 and nb = 120

% Split data
train_end = round(25 / Tsampling);  % 25 seconds - 2500 sample
train_u = u_filter(1:train_end);
train_y = y_clean(1:train_end);

val_u   = u_filter(train_end+1 : end);
val_y   = y_clean(train_end+1 : end);
t_val   = t(train_end+1 : end);


%nb = 40
nb1 = 40;
Ntr = length(train_u);

Phi40 = zeros(Ntr-nb1, nb1);
y40   = zeros(Ntr-nb1, 1);

for k = nb1+1:Ntr
    for i = 1:nb1
        Phi40(k-nb1, i) = train_u(k - i);
    end
    y40(k-nb1) = train_y(k);
end

b40 = Phi40 \ y40;


%nb = 120
nb2 = 120;
Phi120 = zeros(Ntr-nb2, nb2);
y120   = zeros(Ntr-nb2, 1);

for k = nb2+1:Ntr
    for i = 1:nb2
        Phi120(k-nb2, i) = train_u(k - i);
    end
    y120(k-nb2) = train_y(k);
end

b120 = Phi120 \ y120;
% (b) Prediction on validation set

% Full length filtered input from problem 1
Nu = length(u_filter);

% Prediction nb = 40
yhat40_full = zeros(Nu,1);
for k = 1:Nu
    s = 0;
    for i = 1:nb1
        if (k-i) >= 1
            s = s + b40(i) * u_filter(k-i);
        end
    end
    yhat40_full(k) = s;
end

% Prediction nb = 120
yhat120_full = zeros(Nu,1);
for k = 1:Nu
    s = 0;
    for i = 1:nb2
        if (k-i) >= 1
            s = s + b120(i) * u_filter(k-i);
        end
    end
    yhat120_full(k) = s;
end

% Extract validation portion
yhat40  = yhat40_full(train_end+1:end);
yhat120 = yhat120_full(train_end+1:end);

% Plot nb = 40
figure;
plot(t_val, val_y, 'r'); hold on;
plot(t_val, yhat40, 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Output');
legend('Measured y[k]', 'ŷ[k] (nb=40)', 'Location', 'best');
title('Validation — FIR Model nb = 40');
grid on;

% Plot nb = 120
figure;
plot(t_val, val_y, 'r'); hold on;
plot(t_val, yhat120, 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Output');
legend('Measured y[k]', 'ŷ[k] (nb=120)', 'Location', 'best');
title('Validation — FIR Model nb = 120');
grid on;
% (c) RMSE and Fit 

rmse40  = sqrt(mean((val_y - yhat40 ).^2));
rmse120 = sqrt(mean((val_y - yhat120).^2));

fit40  = (1 - norm(val_y - yhat40 )/norm(val_y - mean(val_y))) * 100;
fit120 = (1 - norm(val_y - yhat120)/norm(val_y - mean(val_y))) * 100;

fprintf('\n VALIDATION METRICS \n');
fprintf('nb =  40 → RMSE = %.4f, Fit = %.2f %%\n', rmse40,  fit40);
fprintf('nb = 120 → RMSE = %.4f, Fit = %.2f %%\n', rmse120, fit120);


save('q2_data.mat', 'u_filter', 'y_clean', 'train_u', 'train_y', ...
     'val_y', 't_val', 'Tsampling', 'train_end');
