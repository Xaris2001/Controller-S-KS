clc;clear all; close all;
% Define the plant transfer function
G = tf(4, conv([1 -1], conv([0.02 1], [0.02 1])));

% Define the weighting functions for the hinfsyn function
Wp1 = tf([(1/1.5) 10], [1 10^(-4)]);      % Performance weight 1
Wp2 = tf([(1/1.5) 10], [1 10^(-4)]);      % Performance weight 2
Wu = 1; % unweighted performance

P = augw(G,Wp1,Wp2,Wu);
% Compute the optimal controller
[K,CL,gamma,info] = hinfsyn(P,1,1);

% Plot the closed-loop step response
sys_cl = feedback(G*K,1);
step(sys_cl);

% Compute the performance metrics
w = logspace(-3,3,1000);
wc = bandwidth(sys_cl)
% [mag,phase] = bode(K*G,w);
% figure;
% bode (K*G,w);
sys_cl = feedback(G*K,1);
t = 0:0.001:10;
r = ones(size(t));
[y,t] = lsim(sys_cl,r,t);
plot(t,y);
xlabel('Time (sec)');
ylabel('Output');
title('Step response of the closed-loop system');

A = K(1:size(K,1)/2, 1:size(K,2)/2);
[Kmax, ~] = norm(K, inf);
num=[4]; den =[0.0004 0.04 1 -1];
[A,B,C,D]=tf2ss(num,den);
A_max = maxk(A,1);
A_max=A_max(1,3);
sys_max =A_max*Kmax;

sys= K*G;
[Gm,Pm,Wcg,Wcp] = margin(sys_cl)
Ms = max(20*log10(1/max(gamma))) % sensitivity peak
Mt = 20*log10(abs(1/(1+sys_max))) % complementary sensitivity peak
Gm_dB = 20*log10(Gm) % gain margin
Pm_dB = Pm % phase margin
