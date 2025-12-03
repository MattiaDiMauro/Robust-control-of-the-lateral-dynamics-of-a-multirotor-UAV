clear
close all
clc

%% NOMINAL MODELLING
% Lateral dynamics: 

% Stability nominal values
Y_v_nom = -0.1068;               
Y_p_nom = 0.1192;              
L_p_nom = -2.6478;                     

% Control nominal values
Y_d_nom = -10.1647;             
L_d_nom = 450.7085;         

g = 9.81;

% State Space Model
A_nom = [Y_v_nom Y_p_nom g; 0 L_p_nom 0; 0 1 0];
B_nom = [Y_d_nom L_d_nom 0]';
C_nom = [0 1 0; 0 0 1];
D_nom = [0 0]';
 
% Nominal Plant
sys_ld_nom = ss(A_nom,B_nom,C_nom,D_nom)
sys_ld_nom.u = '\delta_{lat}';
sys_ld_nom.y = {'p','\phi'};


% Plants analysis of the transfer function delta_{lat} --> p
G_delta_lat_p_nom = tf(sys_ld_nom(1))

figure
pzplot(sys_ld_nom(1));
grid on
title('Tf delta lat to p')
legend('Nominal model','location','southeast')

z_delta_lat_p = tzero(sys_ld_nom(1))
p_delta_lat_p = pole(sys_ld_nom(1))

% Plants analysis of the transfer function delta_{lat} --> phi
G_delta_lat_phi_nom = tf(sys_ld_nom(2))

figure
pzplot(sys_ld_nom(2));
grid on 
title('Tf delta lat to phi')
legend('Nominal model','location','southeast')

z_delta_lat_phi = tzero(sys_ld_nom(2))
p_delta_lat_phi = pole(sys_ld_nom(2))

% Bode diagram
figure 
bode(sys_ld_nom)
grid on
legend('Nominal model','location','southwest')

% Step response
figure 
step(sys_ld_nom, 0:0.01:60)
grid on
legend('Nominal model','location','southwest')

%% DESIGN ATTITUDE CONTROL LOOP

% R_phi is a P controller:
R_phi = tunablePID('R_phi','P'); 
R_phi.u = 'e_\phi';
R_phi.y = 'p_0';

% R_p is a PID controller:
R_p = tunablePID2('R_p','PID');
R_p.c.Value = 0;                                                               
R_p.c.Free = false;                                                            
R_p.b.Value = 1;                                                               
R_p.b.Free = false;                                                             
R_p.Tf.Value = 0.02;                                                            
R_p.Tf.Free = false;                                                            
R_p.u = {'p_0','p'};
R_p.y = {'\delta_{lat}'};

% Define the outer loop
SumOuter = sumblk('e_\phi = \phi_0 - \phi');

% Construct the closed loop
CL_tun = connect(SumOuter,R_phi,R_p,sys_ld_nom,'\phi_0',{'\phi','p'});

% Nominal Perfomance:
% Complementary sensitivity tf (F_2) and Sensitivity tf (S_2).
omega_n = 10;         
xi = 0.9;    
s = tf('s');
F_2 = omega_n^2/(s^2 + 2*s*xi*omega_n + omega_n^2);
S_2 = 1-F_2;
figure
step(F_2)
grid on
stepinfo(F_2)

% Bode diagram of F_2 and S_2
figure
bodemag(F_2)
hold on
bodemag(S_2)
grid on
legend('F_2','S_2')

% Performance Weight (W_p) 
M = 2;
A = 1e-2;
omega_b = 0.45*omega_n;

W_p = (s/M + omega_b)/(s + A*omega_b);

figure
bodemag(F_2)
hold on
bodemag(S_2)
grid on
bodemag(1/W_p)
legend('F_2','S_2','1/W_p', 'location','southeast')

% Control effort limitation 
alpha = deg2rad(20)/0.05;
w_act = 10;

W_q = alpha*(s+w_act*1e-3)/(s+w_act);

figure
bodemag(1/W_q)
grid on

%% TUNING REGULATORS
% Define the input and output of the weight blocks
W_p.u = 'e_\phi';
W_p.y = 'z_1';

W_q.u = '\delta_{lat}';
W_q.y = 'z_2';

% Include them in the closed loop
CL0 = connect(R_p, R_phi, sys_ld_nom, SumOuter, W_p, W_q,...
    '\phi_0',{'z_1','z_2'},{'\delta_{lat}','e_\phi','\phi'});

% Extracts tuned blocks
opt = hinfstructOptions('Display','final','RandomStart',10);
[CL,gamma,info] = hinfstruct(CL0,opt);
showTunable(CL)
R_phi_tuned = getBlockValue(CL,'R_phi');
R_p_tuned   = getBlockValue(CL,'R_p');

% Read the gains
Kp_phi = R_phi_tuned.Kp;
Kp_p   = R_p_tuned.Kp;
Ki_p   = R_p_tuned.Ki;
Kd_p   = R_p_tuned.Kd;

% Substitute the gains into R_p and R_phi
R_p.Kp.Value = Kp_p;
R_p.Ki.Value = Ki_p; 
R_p.Kd.Value = Kd_p; 
R_phi.Kp.Value = Kp_phi;

% Define S, F, Q, L functions
F = getIOTransfer(CL,'\phi_0','\phi');
figure
step(F)
hold on
step(F_2)
grid on
legend('hinfstruct','second order', 'location','southeast')
stepinfo(F)

S = getIOTransfer(CL,'\phi_0','e_\phi');
figure
bodemag(S)
grid on
hold on
bodemag(1/W_p)
legend('S','1/W_p','location','southeast')


Q = getIOTransfer(CL,'\phi_0','\delta_{lat}');
figure
bodemag(Q)
grid on
hold on
bodemag(1/W_q)
legend('Q','1/W_q','location','southeast')

L = (1-S)/S;
figure
margin(L)

% Control effort limitation
t = linspace(0,6,10^4);
u = 0*(t<=1) + 10*(t>1 & t<= 3) - 10*(t>3 & t<= 5) + 0*(t>5);
u = deg2rad(u);
figure
lsim(Q,u,t);
hold on
grid on


%% UNCERTAIN MODELLING
% Uncertainties 
unc_Y_v = 3*4.26;
unc_Y_p = 3*2.03;
unc_L_p = 3*2.01;
unc_Y_d = 3*1.37;
unc_L_d = 3*0.81;

% Stability Derivatives
Y_v = ureal('Y_v',Y_v_nom,'Perc',unc_Y_v);
Y_p = ureal('Y_p',Y_p_nom,'Perc',unc_Y_p);
L_p = ureal('L_p',L_p_nom,'Perc',unc_L_p);

% Control Derivatives
Y_d = ureal('Y_d',Y_d_nom,'Perc',unc_Y_d);
L_d = ureal('L_d',L_d_nom,'Perc',unc_L_d);

% State Space Model
A = [Y_v Y_p g; 0 L_p 0; 0 1 0];
B = [Y_d L_d 0]';
C = [0 1 0; 0 0 1];
D = [0 0]';

% Uncertain Plant
sys_ld = ss(A,B,C,D)
sys_ld.u = '\delta_{lat}';
sys_ld.y = {'p','\phi'};
 
% Plants analysis of the transfer function delta_{lat} --> p with uncertain
figure
pzplot(sys_ld(1));
hold on 
pzplot(sys_ld_nom(1));
grid on
title('Tf delta lat to p with uncertain')
legend('Uncertain model','Nominal model','location','east')

% Plants analysis of the transfer function delta_{lat} --> phi with uncertain
figure
pzplot(sys_ld(2));
hold on 
pzplot(sys_ld_nom(2));
grid on 
title('Tf delta lat to phi with uncertain')
legend('Uncertain model','Nominal model','location','east')

% Bode diagram
figure 
bode(sys_ld)
hold on
bode(sys_ld_nom)
grid on
legend('Uncertain model','Nominal model','location','northwest')

% Step response
figure 
step(sys_ld, 0:0.01:45)
hold on
step(sys_ld_nom, 0:0.01:45)
grid on
legend('Uncertain model','Nominal model','location','northwest')

%% ROBUST STABILITY
% Extract the M-Delta form
CL_unc = connect(SumOuter,R_phi,R_p,sys_ld,'\phi_0',{'\phi','p'});
[M,Delta,BlkStruct] = lftdata(CL_unc); 

M.InputName(1:2) = {'w1','w2'}; % Uncertainty inputs
M.OutputName(1:2) = {'z1','z2'}; % Uncertainty outputs
Mmu = M({'z1','z2'}, {'w1','w2'}); % Consider only inputs and outputs associated with Delta Block

% Define the frequency range for analysis
omega = logspace(-3, 3, 10000);         
mu_RS_values = zeros(size(omega));
margin_RS_values = zeros(size(omega));

% Save the mu and sigma values in two vectors
fprintf('Analyzing robust stability:\n');
for k = 1:length(omega)    
    Mmu_jw = squeeze(freqresp(Mmu, omega(k))); % We produce freq resp for M
    [mu_est, ~] = mussv(Mmu_jw, BlkStruct);    % Mmu_jw is frq format, providing pointwise values for mu for each frequency 
    mu_RS_values(k) = real(mu_est(1)); 
    margin_RS_values(k) = 1 / mu_RS_values(k);
end
SIGMA = max(sigma(Mmu,omega)); % take the maximum singular value

% Print the maximum mu and minimum margin 
mu_RS_peak = max(mu_RS_values);
margin_RS = 1 / mu_RS_peak;
fprintf('Peak μ_RS = %.4f\n', mu_RS_peak);
fprintf('Stability margin = %.4f\n', margin_RS);
if mu_RS_peak < 1
    fprintf('Robustly stable control system\n');
else
    fprintf('NOT robustly stable system\n');
end

% Plot results
figure('Name','Robust Stability Analysis: mu');
loglog(omega, mu_RS_values, 'b-', 'LineWidth', 2); 
hold on;
loglog(omega, ones(size(omega)), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('\omega [rad/s]');
ylabel('\mu_\Delta(M(j\omega))');
title('Robust Stability Analysis: mu');
legend('\mu_{RS}', 'threshold = 1', 'Location', 'best');
ylim([0, max(1.1, 1.2*mu_RS_peak)]);

% Plot results
figure('Name','Robust Stability Analysis: margin');
loglog(omega, margin_RS_values, 'b-', 'LineWidth', 2);
grid on;
hold on;
loglog(omega, ones(size(omega)), 'r--', 'LineWidth', 1.5);
xlabel('\omega [rad/s]');
ylabel('margin_\Delta(M(j\omega))');
title('Robust Stability Analysis: margin');
legend('margin_{RS}','threshold = 1','Location', 'best');
ylim([0, 100]);

% Plot results
figure('Name','Robust Stability Analysis: sigma');
loglog(omega, SIGMA, 'b-', 'LineWidth', 2);
grid on;
hold on;
loglog(omega, ones(size(omega)), 'r--', 'LineWidth', 1.5);
xlabel('\omega [rad/s]');
ylabel('\sigma_\Delta(M(j\omega))');
title('Robust Stability Analysis: sigma');
legend('\sigma_{RS}','threshold = 1', 'Location', 'best');
ylim([0, 1.2]);


%% ROBUST PERFORMANCE
% Define the weight blocks as uncertain blocks
gamma_test = 1;  % Maximum absolute gain of uncertain dynamics at all frequencies
Wp_delta = ultidyn('Wp_delta_perf', [1 1], 'Bound', gamma_test); % Define uncertain block for performance
Wq_delta = ultidyn('Wq_delta_perf', [1 1], 'Bound', gamma_test); % Define uncertain block for control effort limit.

% Substitute the deterministic weight with the uncertain ones 
Wp_block = series(W_p, Wp_delta);   % z1 = W_p * Wp_delta * e_phi
Wq_block = series(W_q, Wq_delta);   % z2 = W_q * Wq_delta * delta_lat

% Define the input and output of the weight blocks
Wp_block.u = 'e_\phi';
Wp_block.y = 'z_1';
Wq_block.u = '\delta_{lat}';
Wq_block.y = 'z_2';

% Include them in the closed loop
CL_weighted = connect(R_p, R_phi, sys_ld, SumOuter, Wp_block, Wq_block, ...
                     '\phi_0', {'z_1', 'z_2'});

% Extract the M-Delta form
[M_weighted, Delta_weighted, BlkStruct_weighted] = lftdata(CL_weighted); 

% Uncertain inputs and outputs
M_weighted.InputName(1:4) = {'w1perf','w2perf','w3perf','w4perf' };
M_weighted.OutputName(1:4) = {'z1perf','z2perf','z3perf','z4perf'};
Mmuperf = M_weighted({'z1perf','z2perf','z3perf','z4perf'}, {'w1perf','w2perf','w3perf','w4perf'}); 
  
mu_RP_values = zeros(size(omega));
margin_RP_values = zeros(size(omega));

% Save the mu and margin values in two vectors
fprintf('Analyzing robust performance:\n');
for k = 1:length(omega)
        M_RP_jw = squeeze(freqresp(Mmuperf, omega(k))); 
        [mup_est, ~] = mussv(M_RP_jw, BlkStruct_weighted);
        mu_RP_values(k) = real(mup_est(1));
        margin_RP_values(k)=1/mu_RP_values(k);
end

% Print the maximum mu and minimum sigma
mu_RP_peak = max(mu_RP_values);
fprintf('Peak μ_RP = %.4f\n', mu_RP_peak);
fprintf('Peak sigma_RP = %.4f\n', 1 / mu_RP_peak);
if mu_RP_peak < 1
    fprintf('Guaranteed Robust Performance\n');
else
    fprintf('Robust Performance NOT guaranteed\n');
end

% Plot risultati
figure('Name','Robust Performance Analysis: mu');
loglog(omega, mu_RP_values, 'b-', 'LineWidth', 2); 
hold on;
loglog(omega, ones(size(omega)), 'r--', 'LineWidth', 1.5);
grid on;
xlabel('\omega [rad/s]');
ylabel('\mu_{RP}');
title('Robust Performance Analysis: mu');
legend('\mu_{RP}', 'threshold = 1', 'Location', 'best');
ylim([0, min(5, 1.2*mu_RP_peak)]);

% Plot results
figure('Name','Robust Performance Analysis: sigma');
loglog(omega, margin_RP_values, 'b-', 'LineWidth', 2); 
grid on;
hold on;
loglog(omega, ones(size(omega)), 'r--', 'LineWidth', 1.5);
xlabel('\omega [rad/s]');
ylabel('margin_\Delta(M(j\omega))');
title('Robust Performance Analysis: margin');
legend('margin_{RP}', 'threshold = 1', 'Location', 'best');
ylim([0, 100]);

%% MONTE CARLO
% Define the number of MC iterations evaluating the means and the standard deviation of Q and S
N = 700; 
Hinf_CE = [];
Hinf_S = [];
for j = 1:N
    sys_ld_rand = usample(sys_ld);
    CL_MC = connect(R_p, R_phi, sys_ld_rand, SumOuter, W_p, W_q,...
    '\phi_0',{'z_1','z_2'},{'\delta_{lat}','e_\phi','\phi'});
    
    S_MC = getIOTransfer(CL_MC,'\phi_0','e_\phi');
    Q_MC = getIOTransfer(CL_MC,'\phi_0','\delta_{lat}');
    Hinf_CE(j) = hinfnorm(W_q*Q_MC);
    mean_Hinf_CE(j) = mean(Hinf_CE);
    std_Hinf_CE(j) = std(Hinf_CE);
    Hinf_S(j) = hinfnorm(W_p*S_MC);
    mean_Hinf_S(j) = mean(Hinf_S);
    std_Hinf_S(j) = std(Hinf_S);
end

% Plot the results
figure
plot((1:N), Hinf_S);
grid on;
hold on;
plot((1:N), Hinf_CE);
plot((1:N), ones(N));
title('Hinf_S, Hinf_{CE}');
xlabel('Iterations');
ylabel('Hinf_{value}');
legend('Hinf_S', 'Hinf_{CE}', 'threshold = 1', 'Location', 'best');

figure
plot((1:N), mean_Hinf_S);
xlabel('Iterations');
ylabel('Mean_{Hinf_S}');
title('Mean_{Hinf_S}');
grid on;
legend('mean_Hinf_S', 'Location', 'best');

figure
plot((1:N), mean_Hinf_CE);
xlabel('Iterations');
ylabel('Mean_{Hinf_Q}');
title('Mean_{Hinf_Q}');
grid on;
legend('mean_Hinf_Q', 'Location', 'best');

figure
plot((1:N), std_Hinf_CE);
xlabel('Iterations');
ylabel('std_{Hinf}');
title('Std_{Hinf}');
hold on
plot((1:N), std_Hinf_S);
grid on;
legend('std_Hinf_S', 'std_Hinf_Q', 'Location', 'best');

%% 
% Pick Nnew as number of MC iterations  
Nnew = 300; 

% Plot the S and Q functions with MC simulation
figure;
for j = 1:Nnew
    sys_ld_rand = usample(sys_ld);
    CL_MC = connect(R_p, R_phi, sys_ld_rand, SumOuter, W_p, W_q,...
    '\phi_0',{'z_1','z_2'},{'\delta_{lat}','e_\phi','\phi'});
    
    S_MC = getIOTransfer(CL_MC,'\phi_0','e_\phi');
    bodemag(S_MC,'b')
    Q_MC = getIOTransfer(CL_MC,'\phi_0','\delta_{lat}');
    bodemag(Q_MC,'r')
    grid on
    hold on
end
bodemag(1/W_p,'g')
bodemag(1/W_q,'k')
legend('Q_MC', 'S_MC', 'location','northeast');
title('Monte Carlo simulation for performance');

figure;
for j = 1:Nnew
    sys_ld_rand = usample(sys_ld);
    CL_MC = connect(R_p, R_phi, sys_ld_rand, SumOuter, W_p, W_q,...
    '\phi_0',{'z_1','z_2'},{'\delta_{lat}','e_\phi','\phi'});
    Q_MC = getIOTransfer(CL_MC,'\phi_0','\delta_{lat}');
    lsim(Q_MC,u,t);
    hold on 
    grid on
end
title('Monte Carlo simulation for control effort');