%% _*PROBLEM 3 (25 Pts.)*_

% Using the same data from problem 2
load('q2_data.mat');


arx_pairs = [1 40;      % Model 1: short output history, same nb as FIR for comparision
             2 40;      % Model 2: more poles, same nb
             2 80];     % Model 3: more poles and longer input history

numModels = 3
rmse_arx  = zeros(numModels, 1);
fit_arx   = zeros(numModels, 1);

Nu_total  = length(u_filter);
Ntr       = length(train_u);    % number of training samples

for m = 1:numModels
    
    na = arx_pairs(m, 1);       % number of past outputs
    nb = arx_pairs(m, 2);       % number of past inputs
    
    % First usable index computation (need at least n_a past outputs and nb past inputs)
    k0  = max(na, nb) + 1;
    Ktr = Ntr - k0 + 1;         % number of training rrows
    
    % Build regression matrix and output vector y_arx on training data
    Phi   = zeros(Ktr, na + nb);
    y_arx = zeros(Ktr, 1);
    
    for k = k0:Ntr
        % past outputs
        Phi(k-k0+1, 1:na) = -train_y(k-1:-1:k-na).';
        
        % past inputs
        Phi(k-k0+1, na+1:end) = train_u(k-1:-1:k-nb).';
        
        % current output
        y_arx(k-k0+1) = train_y(k);
    end
    
    % Least-squares estimate (theta)
    theta = Phi \ y_arx;
    a_arx = theta(1:na);
    b_arx = theta(na+1:end);
    
    % One step ahead prediction, just like in the lecture sldies
    yhat_full = zeros(Nu_total, 1);
    
    for k = k0:Nu_total
        y_past = y_clean(k-1:-1:k-na);
        u_past = u_filter(k-1:-1:k-nb);
        
        % y^[k]
        yhat_full(k) = -a_arx.' * y_past + b_arx.' * u_past;
    end
    
    % Take only validation part
    yhat_val = yhat_full(train_end+1:end);
    
    % Compute RMSE and FIT on validation data
    rmse_arx(m) = sqrt(mean((val_y - yhat_val).^2));
    fit_arx(m)  = (1 - norm(val_y - yhat_val) / norm(val_y - mean(val_y))) * 100;
    
    % plot for each na,nb pair
    figure;
    plot(t_val, val_y, 'b'); hold on;
    plot(t_val, yhat_val, 'r--', 'LineWidth', 1.5);

    xlabel('Time [s]'); ylabel('Output');
    legend('Measured y[k]', ...
           sprintf('y^[k] (na=%d, nb=%d)', na, nb), ...
           'Location', 'best');
    title(sprintf('Validation — ARX Model (na = %d, nb = %d)', na, nb));
    grid on;
    
end

% Compare models
[bestFit, bestIdx] = max(fit_arx);
best_na = arx_pairs(bestIdx, 1);
best_nb = arx_pairs(bestIdx, 2);

fprintf('\n ARX Model Comparision (validation set)\n');
for m = 1:numModels
    fprintf('(na,nb) = (%2d, %3d)   RMSE = %.4f, Fit = %.2f %%\n', ...
        arx_pairs(m,1), arx_pairs(m,2), rmse_arx(m), fit_arx(m));
end
fprintf('Best pair based on validation FIT: (na, nb) = (%d, %d), Fit = %.2f %%\n', ...
        best_na, best_nb, bestFit);
