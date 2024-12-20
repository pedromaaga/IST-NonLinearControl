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

%% Question 01

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

%%
sol = vpasolve([eq1,eq2],[w,x])

G(sol.w)
N(sol.x)

