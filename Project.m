close all

% consider the output feedback control of a given LTI system
s=tf('s')
tf_G= zpk(-1 , [-4 , -2 , 1] , 10 )

%%
figure
bode(tf_G)

figure
n = nyquistplot(tf_G);
setoptions(n,'ShowFullContour', 'off')
grid on


%%
figure
t = 0:0.01:3;
u= 0.01*sin(t);
lsim( tf_G , u , t)

figure
impulse(tf_G,[1,5])

figure
step(tf_G,[1,5])


%% Actuator 1 - Dead zone
close all

S_1=0.5 %delta
k=2

X=-1e1:1e-5:1e1; %-2:0.1:1;

S_X=S_1./X;
N_deadzone = k - 2*k/pi*( asin(S_X) + S_X.*sqrt( 1-(S_X).^2 ) );
figure
plot(X,N_deadzone)


inv_N_deadzone = -1./N_deadzone;

figure
hold on
n = nyquistplot(tf_G);
setoptions(n,'ShowFullContour', 'off')
grid on
plot( real(inv_N_deadzone) , imag(inv_N_deadzone),"DisplayName", "-1/N" ) %,'o-'

%%

syms G(w)  N(x)


G(w) = 10*(1i*w+1)/((1i*w+4)*(1i*w+2)*(1i*w-1))
assume(w,"real")


N(x) = k - 2*k/pi*( asin(S_1/x) + S_1/x.*sqrt( 1-(S_1/x).^2 ) )
assume(x,"real")



eq1= imag(G(w))==0
sol1 = vpasolve(eq1,w,[0.1,100])

eq2= N(x)*real(G(w)) == 1



%% Actuator 2 - Saturation with Dead zone
close all

S_2=1 %delta
M_2=1

X=-1e1:1e-5:1e1; %-2:0.1:1;

S_X=S_2./X;
N_saturation = 4*M_2*sqrt(1-(S_X).^2)./(pi.*X);
figure
plot(X,N_saturation)

inv_N_saturation = -1./N_saturation;

figure
hold on
n = nyquistplot(tf_G);
setoptions(n,'ShowFullContour', 'off')
grid on
plot( real(inv_N_saturation) , imag(inv_N_saturation),"DisplayName", "-1/N" ) %,'o-'


%% Actuator 3 - Histeresis relay
%close all

h=2
M_3=0.5

X=-1e1:1e-5:1e1; %-2:0.1:1;

% N_histeresis= 4*M*sqrt(1-(h/X)^2)/(pi*X)-i*4*h*M/(*pi*X^2) % if X>h , else =0
% inv_N_histeresis = -1./N_histeresis;

% Calculate N_histeresis for X <= h, else set it to 0
N_histeresis = zeros(size(X)); % Initialize with zeros
valid_indices =  X >= h; % Logical array where X is within [-h, h]
N_histeresis(valid_indices) = 4 * M_3 * sqrt(1 - (h ./ X(valid_indices)).^2) ./ (pi * X(valid_indices)) ...
                              - 1i * 4 * h * M_3 ./ (pi * X(valid_indices).^2); % Only calculate for X within the range

figure
plot(X,N_histeresis)

% Inverse of N_histeresis (handle division by zero)
inv_N_histeresis = zeros(size(N_histeresis)); % Initialize with zeros
non_zero_indices = N_histeresis ~= 0; % Avoid division by zero
inv_N_histeresis(non_zero_indices) = -1 ./ N_histeresis(non_zero_indices);

figure
hold on
n = nyquistplot(tf_G);
setoptions(n,'ShowFullContour', 'off')
grid on
plot( real(inv_N_histeresis) , imag(inv_N_histeresis),"DisplayName", "-1/N" ) %,'o-'



sol = vpasolve([eq1,eq2],[w,x])

G(sol.w)
N(sol.x)

