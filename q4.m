%% Q4) Outstanding Level: Grid-connected Mode and Islanded Mode Switch
% This script contains the code to run completed_microinverter_setup.slx
% Streamlined config script for full implementation

Ts = 1e-6;
Tes = 20*Ts; % electrical sampling time
Tcs = 100*Ts;

% Grid voltage
V_rms = 400;
V_dc = V_rms * sqrt(3) * sqrt(2);

% PLL
Wn = 2*pi*200;
DF = 0.707;
Ki_pll = (Wn^2);
Kp_pll = 2*DF / sqrt(1/Ki_pll);
f_center = 50;

% Damped LC filter
L1_filter = 1.5e-3;
L2_filter = 1.0e-3;
Rd_filter = 0.2;
C1_filter = 2e-6;
% Source: docs/inverter_output_LCL_Filter.pdf
F_damped_filter = 1/(2*pi) * sqrt((L1_filter+L2_filter)/(C1_filter*L1_filter*L2_filter));

% RL load
P_load = 10e3 / 3;
pf_load = 0.95;
V_rms = 400;

f = 50; 
w = 2*pi*f;

R_load = (V_rms^2) * (pf_load^2) / P_load;
L_load = sqrt((R_load * V_rms^2)/P_load - R_load^2) / w;

% Controller parameters
% NOTE: We approximate the equivalent LC filter for the LCL damped filter
L_filter = 1.25e-3;
C_filter = 2e-6;
F_approx_filter = 1/(2*pi*sqrt(L_filter*C_filter));

% Current PI controller
Wn = 2*pi*100;
%DF = 0.707;
DF = 3.5;
Ki_current = L_filter*Wn^2;
Kp_current = 2*L_filter*DF*Wn;

% Voltage PI controller
Wn = 2*pi*30;
%DF = 0.707;
DF  = 10.5;
Ki_voltage = C_filter*Wn^2;
Kp_voltage = 2*C_filter*DF*Wn;

% Power PI controller
Ki_power = 100;
Kp_power = 0;

%% Plot response of our system
% Get out transfer functions
s = tf('s');
Z_C = 1/(s*C_filter);
Z_RL = (L2_filter + L_load)*s + R_load;
Z_RC = Z_C + Rd_filter;
Z_L = s*L1_filter;

Z_RLC_parallel = 1/(1/Z_C + 1/Z_RL);

% T1 = I_inductor / V_in
T1 = 1/(Z_L + Z_RLC_parallel);

% T2 = Vo/Vin
T2 = Z_RLC_parallel / (Z_L + Z_RLC_parallel);
T2 = minreal(T2);

% T3 = I_load / Vin
% I_load = Vo/Z_RL
T3 = T2 * 1/Z_RL;
T3 = minreal(T3);

% H = I_inductor / V_outupt
% H = I_inductor / V_in * V_in / V_output
H = T1 / T2;
H = minreal(H);

% Controller responses
G1 = Kp_current + Ki_current/s;
G2 = Kp_voltage + Ki_voltage/s;

% T4 is our I1/Ir closed loop response for current controller
T4 = G1/(L1_filter*s + G1);
T4 = minreal(T4);

% T5 is our V0/Vr closed loop response for outer voltage controller
T5 = G1*G2/(H*L1_filter*s + s*C_filter*G1 + G1*G2);
T5 = minreal(T5);

% T6 is our simplified response for outer voltage controller
T6 = G2/(C_filter*s + G2);
T6 = minreal(T6);

% Response of power control
T7 = (Kp_power*s + Ki_power) / ((Kp_power+1)*s + Ki_power);

% Plot the responses
dt = linspace(0, 0.1, 10000);
[dy1,dt1] = step(T4, dt);
[dy2,dt2] = step(T5, dt);
[dy3,dt3] = step(T6, dt);
[dy4,dt4] = step(T7, dt);

step_info_1 = stepinfo(dy1, dt1);
step_info_2 = stepinfo(dy2, dt2);
step_info_3 = stepinfo(dy3, dt3);
step_info_4 = stepinfo(dy4, dt4);

figure(1);
clf(1);
hold on;
plot(dt1, dy1);
plot(dt2, dy2);
plot(dt3, dy3);
plot(dt4, dy4);
hold off;
grid on;
xlabel("Time (seconds)");
ylabel("Amplitude");
legend("Current control", "Real voltage control", "Naive voltage control", "Power control");