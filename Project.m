close all

%% System 1

s=tf('s');
tf_G = 20/(s*(s+2)*(s+5));
figure
bode(tf_G)

figure
n = nyquistplot(tf_G);
setoptions(n,'ShowFullContour', 'off')
grid on

%% P1 - Question 01

% Actuator 1 - Dead zone

A = linspace(0,5,1000);

for i=1:length(A)
    N_A_actuator1(i) = 2-(4/pi)*(asin(1/(2*A(i))) + (1/(2*A(i)))*sqrt(1-1/(4*A(i)^2)));
end

inv_N_A_actuator1 = -1./N_A_actuator1;

figure (1)
plot(A, N_A_actuator1)


% Actuator 2 - Saturation with Dead zone

for i=1:length(A)
    N_A_actuator2(i) = 4/(pi*A(i))*sqrt(1-1/A(i)^2);
end

inv_N_A_actuator2 = -1./N_A_actuator2;

figure (2)
plot(A, N_A_actuator2)


% Actuator 3 - Histeresis relay

for i=1:length(A)
    N_A_actuator3(i) = 2/(pi*A(i))*exp(-1i*asin(2/A(i)));
end

inv_N_A_actuator3 = -1./N_A_actuator3;

figure (3)
plot(A, N_A_actuator3)

% Nyquist plot
figure (4)
n = nyquistplot(tf_G);
setoptions(n,'ShowFullContour', 'off')
hold on
plot( real(inv_N_A_actuator1) , imag(inv_N_A_actuator1), LineWidth=2)
plot( real(inv_N_A_actuator2) , imag(inv_N_A_actuator2), LineWidth=2)
plot( real(inv_N_A_actuator3) , imag(inv_N_A_actuator3), LineWidth=2)
legend('System 1', 'Actuator 1', 'Actuator 2', 'Actuator 3')
axis([-1.5 -0.5 -5 10])
hold off

%% P1 - Question 02

% Popov Criteria

% Define the frequency range
omega = logspace(-3, 4, 100000);  % Frequency range (logarithmic scale)

% Define the transfer function G(jw)
Gjw = 20 ./ (-1j * omega.^3 - 7 * omega.^2 + 1j * 10 * omega);  % Transfer function G(jw)

% Real and Imaginary parts of G(jw)
X = real(Gjw);
Y = imag(Gjw).*omega;

% Plot Imag(Gjw)*w x Real(Gjw)
figure;
plot(X, Y, 'LineWidth',2);
axis([min(X)/2 max(X)/2 min(Y)/2+0.5 max(Y)/2+0.5]);
grid on;
title('System G(j \omega)');
xlabel('Re(G(j\omega))');
ylabel('Im(G(j\omega)) \cdot \omega');

% Define q values - slope of Popov line (1/q)
q_values = linspace(0,1,20); % Example slopes for Popov line
K_max_values = zeros(size(q_values)); % Store max K for each lambda

% Loop through each slope q
for i = 1:length(q_values)
    q = q_values(i);
    
    % Possible gains
    K_range = linspace(0, 4, 100); % Range of K to test
    valid_K = 0; % Store the largest gain
    
    % Evaluate the Popov inequality
    for K = K_range
        Popov_inequality = X - q * Y + 1/K;
        
        % Check if the inequality is satisfied for all frequencies
        if all(Popov_inequality > 0)
            valid_K = K;
        else
            break
        end
    end
    
    % Store the largest valid K for this slope
    K_max_values(i) = valid_K;
end


cmap = jet(length(K_max_values)); % Create a colormap based on the number of K_max_values

% Loop through each K in K_max_values
legend_entries = cell(1,size(K_max_values,2));
legend_entries{1} = sprintf('System G');

figure
plot(X, Y, 'LineWidth',2);
hold on
for i = 1:length(K_max_values)
    K = K_max_values(i);  % Get the current K value
    q = q_values(i);
    
    % Normalize K to map to the colormap
    normalized_K = (K - min(K_max_values)) / (max(K_max_values) - min(K_max_values));  % Normalize K for colormap
    
    % Calculate the Popov line
    line = (X + 1 / K) ./ q;
    
    % Plot the line with color corresponding to K
    plot(X, line, 'LineWidth', 0.5, 'Color', cmap(round(normalized_K * (length(K_max_values) - 1)) + 1, :));  % Color varies with K

    % Add a legend entry for the current K value
    legend_entries{i+1} = sprintf('K = %.3f, q = %.3f', K, q);
end

% Add the legend with all entries
legend(legend_entries, 'Location', 'Best');

% Axis properties
axis([min(X)/2 max(X)/2 min(Y)/2+0.5 max(Y)/2+0.5]);
grid on;
xlabel('Re(G(j\omega))');
ylabel('Im(G(j\omega)) \cdot \omega');
hold off;

% Plot effect of q on Maximum Gain K
figure;
plot(q_values, K_max_values, '-o', 'LineWidth', 2);
xlabel('q');
ylabel('Maximum K');
title('Effect of q on maximum K');
grid on;

% Plot K max line
[k_max, idx] = max(K_max_values);
line = (X + 1 / k_max) ./ q_values(idx);

figure
plot(X, Y, 'LineWidth',2);
hold on
plot(X, line, 'LineWidth', 1);
plot(-1/k_max, 0, 'bo')
hold off
legend('System G', 'Popov Line', sprintf('K = %.3f, q = %.3f', k_max, q_values(idx)))
axis([min(X)/2 max(X)/2 min(Y)/2+0.5 max(Y)/2+0.5]);
grid on;
xlabel('Re(G(j\omega))');
ylabel('Im(G(j\omega)) \cdot \omega');

% Conclusions
fprintf('P1 - Question 02\n')
fprintf('Popov Criteria\n');
fprintf('Interval K - [0, %.3f]\n', max(K_max_values));

%% P2 - Question 01


%% P2 - Question 02


%% P2 - Question 03

% --------- Specific case
% Control law 
Phi = 1.5;
eta = 1;
K = [0.25 0.25 -1];

% Intervalo de Simulação
interval = [0 8];
x0 = [2 2 2 0];

% Solver ODE45
[t, y] = ode45(@(t, x) SystemProblem02(t, x, @(x) ControlLawSlidingSurface(x, Phi, eta, K)), interval, x0);

s = zeros(size(y,1),1);
for i=1:size(y,1)
    s(i) = y(i,1)+y(i,2)+y(i,3);
end

% Plot results
figure;
subplot(3, 1, 1);
plot(t, y(:, 1:3));
xlabel('Time (s)');
ylabel('States');
legend('x_1', 'x_2', 'x_3');

subplot(3, 1, 2);
plot(t, y(:, 4)); % Plota a ação de controle
xlabel('Time (s)');
ylabel('Control Action');

subplot(3, 1, 3);
plot(t, s); % Plota a ação de controle
xlabel('Time (s)');
ylabel('Sliding surface (s)');


% --------- Changing the value of Phi
% Control law 
Phi_values = [0.1 1 2 4];

figure

% Preallocate legend entries for each subplot
legend_entries = cell(1, length(Phi_values));

% Create subplots
subplot(3, 1, 1);
hold on;

subplot(3, 1, 2);
hold on;

subplot(3, 1, 3);
hold on;


for i = 1:length(Phi_values)
    Phi = Phi_values(i);
    eta = 0.1;
    K = [0.25 0.25 -1];
    
    % Intervalo de Simulação
    interval = [0 8];
    x0 = [2 2 2 0];
    
    % Solver ODE45
    [t, y] = ode45(@(t, x) SystemProblem02(t, x, @(x) ControlLawSlidingSurface(x, Phi, eta, K)), interval, x0);
    
    s = zeros(size(y,1),1);
    for j = 1:size(y,1)
        s(j) = y(j,1) + y(j,2) + y(j,3);
    end
    
    % Plot x_2
    subplot(3, 1, 1);
    plot(t, y(:, 2));
    
    % Plot control action
    subplot(3, 1, 2);
    plot(t, y(:, 4));
    
    subplot(3, 1, 3);
    plot(t, s);

    % Update legend entries
    legend_entries{i} = sprintf('\\Phi = %.1f', Phi); % Use LaTeX for Φ
end

subplot(3, 1, 1);
legend(legend_entries, 'Location', 'Best');
xlabel('Time (s)');
ylabel('State x_2');
grid on;

subplot(3, 1, 2);
legend(legend_entries, 'Location', 'Best');
xlabel('Time (s)');
ylabel('Control Action');
grid on;

subplot(3, 1, 3);
legend(legend_entries, 'Location', 'Best');
xlabel('Time (s)');
ylabel('Sliding surface (s)');
grid on;

hold off;

% --------- Changing the value of Eta
% Control law 
eta_values = [0.1 0.5 1 1.5];

figure

% Preallocate legend entries for each subplot
legend_entries = cell(1, length(Phi_values));

% Create subplots
subplot(3, 1, 1);
hold on;

subplot(3, 1, 2);
hold on;

subplot(3, 1, 3);
hold on;


for i = 1:length(eta_values)
    eta = eta_values(i);
    Phi = 1;
    K = [0.25 0.25 -1];
    
    % Intervalo de Simulação
    interval = [0 8];
    x0 = [2 2 2 0];
    
    % Solver ODE45
    [t, y] = ode45(@(t, x) SystemProblem02(t, x, @(x) ControlLawSlidingSurface(x, Phi, eta, K)), interval, x0);
    
    s = zeros(size(y,1),1);
    for j = 1:size(y,1)
        s(j) = y(j,1) + y(j,2) + y(j,3);
    end
    
    % Plot x_2
    subplot(3, 1, 1);
    plot(t, y(:, 2));
    
    % Plot control action
    subplot(3, 1, 2);
    plot(t, y(:, 4));
    
    subplot(3, 1, 3);
    plot(t, s);

    % Update legend entries
    legend_entries{i} = sprintf('\\eta = %.1f', eta);
end

subplot(3, 1, 1);
legend(legend_entries, 'Location', 'Best');
xlabel('Time (s)');
ylabel('State x_2');
grid on;

subplot(3, 1, 2);
legend(legend_entries, 'Location', 'Best');
xlabel('Time (s)');
ylabel('Control Action');
grid on;

subplot(3, 1, 3);
legend(legend_entries, 'Location', 'Best');
xlabel('Time (s)');
ylabel('Sliding surface (s)');
grid on;

hold off;


