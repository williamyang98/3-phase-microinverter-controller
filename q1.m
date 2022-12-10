%% Q1) Basic Level: Phase Lock Loop (PLL)
Ts = 1e-6;
Tes = 20*Ts; % electrical sampling time
Tcs = 100*Ts;

%% 3 phase pll design
% Wn = sqrt(AKi)
% ζ = sqrt(A/Ki) * Kp/2
% A = 240 * sqrt(2);

% The output of the DQ transform is
% D = Acos(theta_in - theta_out)
% Q = Asin(theta_in - theta_out)

% A is the peak to peak voltage of the 3 phase input

% We can use the small angle approximation as
% Q = theta_in - theta_out = E(s)

% Forward transfer function is given as
% G(s) = (Kp + Ki/s) * 1/s
% G(s) = (Kp*s + Ki) / s^2

% Y(s) = E(s)G(s)
% E(s) = A[X(s)-Y(s)]
% Y(s) = AX(s)G(s) - AY(s)G(s)
% Y(s)/X(s) = AG(s) / [1 + AG(s)]
% Y(s)/X(s) = A*Kp*(s + Ki/Kp) / [s^2 + A*Kp*s + A*Ki]

% Wn = sqrt(A*Ki)
% 2ζWn = A*Kp
% ζ = sqrt(A/Ki) * Kp/2

% We compensate for the amplitude in our phase detector
A = 1;
Wn = 2*pi*100;
DF = 0.707;

Ki_pll = (Wn^2) / A;
Kp_pll = 2*DF / sqrt(A/Ki_pll);

%% 3 phase filter and load
F_load = 50; 
F_switch = 1e3;
V_rms = 400;
w = 2*pi*F_load;

% cutoff of LC filter is f = 1/(2*pi*sqrt(LC))

% Power rating of capacitor is given as VAR (50 to 400 kVAR)
% Z = 1/(jwC)
% S = V^2 / Z
% S = V^2 * jwC
% |S| = V^2 * wC
S_max = 100e3;
C_filter_max = S_max / (V_rms^2 * w) * 0.05;

% Current rating of inductor (series and saturation)
% Ztotal = jwL + 1/jwC
% V_dc = 1.414 * V_rms = 565.6 V
% Ipp = V_dc / Ztotal


L_filter = 40e-3;
C_filter = 100e-6;

V_dc = sqrt(2) * sqrt(3) * V_rms;

% Calculate ripple current
% V = L di/dt
% di = V/L * dt
% Ipp_ripple = 2*V_dc/(LFsw)
I_pp = 2*V_dc/(L_filter*F_switch);

% Cutoff of the filter
f3db_filter = 1/(2*pi*sqrt(L_filter*C_filter));

%% 3 phase load
% the parameters of our load
% We have 3 phases, so for each RL load on each phase
P_load = 10e3 / 3;
pf_load = 0.95;
F_load = 50; 
V_rms = 400;
w = 2*pi*F_load;

% Z = R + jwL

% S = VI
% S = V^2 / Z
% S = (V^2)/[R^2 + (wL)^2] * (R - jwL)

% |S| = V^2 / sqrt(R^2 + (wL)^2)

% P = VI cos(theta)
% Q = VI sin(theta)

% Constraint 1: Real power
% P = R * (V^2)/(R^2 + (wL)^2) 

% Constraint 2: Power factor
% pf = cos(theta) = P/|S|
% pf = R / sqrt(R^2 + (wL)^2)

% pf^2 = R^2 / (R^2 + (wL)^2)
% P / pf^2 = V^2 / R 
% R = (V^2 * pf^2) / P

% (wL)^2 = (R*V^2)/P - R^2
% L = sqrt([R*(V^2)/P - R^2]/w^2)

R_load = (V_rms^2) * (pf_load^2) / P_load;
L_load = sqrt((R_load * V_rms^2)/P_load - R_load^2) / w;

S = V_rms^2 / (1j*w*L_load + R_load);

%% DQ controller for current controlled inverter
% To get plant equations apply DQ transform to KVL equation
% dT/dt T^-1 = [0 w 0; -w 0 0; 0 0 0]
% Refer to PDF: Conventional current control for inverters.pdf

% Plant equations
% Idq = inductor current, Vdq1 = input voltage, Vdq0 = output voltage
% Vd1 = Ls*Id - wL*Iq + Vd0
% Vq1 = Ls*Iq + wL*Id + Vq0

% Controller equations
% Vd1 = -wL*Iq + Vd0 + Gc(s)*(Idr - Id)
% Vq1 =  wL*Id + Vq0 + Gc(s)*(Iqr - Iq)

% Closed loop response
% Id/Idr = Gc(s)/[Ls + G(s)] = T(s)
% Iq/Iqr = Gc(s)/[Ls + G(s)] = T(s)

% With a PI controller
% G(s) = (Kp*s + Ki)/s
% T(s) = (Kp/L) * [s + (Ki/Kp)] / [s^2 + s*(Kp/L) + (Ki/L)]
Wn = 2*pi*100;
%DF = 0.707;
DF = 2.2;

% Wn^2 = Ki/L
Ki_current = L_filter*Wn^2;

% 2*DF*Wn = Kp/L
Kp_current = 2*L_filter*DF*Wn;

Kd_current = 0.001;

%Kp_current = 100;
%Ki_current = 0;

%% DQ controller for voltage controlled inverter
% Plant equations
% Id1 = inductor current, Idq1 = load current, Vdq0 = output voltage
% Id1 = sCVd0 - wCVq0 + Id0
% Iq1 = sCVq0 + wCVd0 + Iq0

% Controller equations
% Id1 = -wC*Vq0 + Id0 + Gc(s)*(Vdr - Vd0)
% Iq1 =  wC*Vd0 + Iq0 + Gc(s)*(Vqr - Vq0)

% Closed loop response
% Vd0/Vdr = Gc(s)/[Cs + Gc(s)] = T(s)
% Vq0/Vqr = Gc(s)/[Cs + Gc(s)] = T(s)

% With a PI controller
% Gc(s) = (Kp*s + Ki)/s
% T(s) = (Kp/C) * [s + (Ki/Kp)] / [s^2 + s*(Kp/C) + (Ki/C)]
Wn = 2*pi*30;
%DF = 0.707;
DF  = 1.5;

% Wn^2 = Ki/C
Ki_voltage = C_filter*Wn^2;

% 2*DF*Wn = Kp/C
Kp_voltage = 2*C_filter*DF*Wn;

%Ki_voltage = 0;
%Kp_voltage = 0.05;

%% Issue with DQ voltage controller
% We can't control Id1 and Iq1 directly, only through Vdq1 
% Consider the plant equations for current controller
% The output of our voltage controller will be the new current reference

% As long as the time constant of the current controller is smaller than
% the voltage controller, we can ignore it's effects

% Plant equation that we can control
% Idq1 = inductor current, Vdq1 = input voltage, Vdq0 = output voltage
% Vd1 = Ls*Id1 - wL*Iq1 + Vd0
% Vq1 = Ls*Iq1 + wL*Id1 + Vq0

% Voltage controller equations
% Idr = -wC*Vq0 + Id0 + G2(s)*(Vdr - Vd0)
% Iqr =  wC*Vd0 + Iq0 + G2(s)*(Vqr - Vq0)

% Current Controller equations
% Vd1 = -wL*Iq1 + Vd0 + G1(s)*(Idr - Id1)
% Vq1 =  wL*Id1 + Vq0 + G1(s)*(Iqr - Iq1)

% Cascaded controller equations
% Vd1 = -wL*Iq1 + Vd0 + G1(s)[-Id1 - wC*Vq0 + Id0] + G1(s)G2(s)*(Vdr - Vd0)
% Vq1 =  wL*Id1 + Vq0 + G1(s)[-Iq1 + wC*Vd0 + Iq0] + G1(s)G2(s)*(Vqr - Vq0)

% Closed loop response
% Ls*Id1 = + G1(s)[-Id1 - wC*Vq0 + Id0] + G1(s)G2(s)*(Vdr - Vd0)
% Ls*Iq1 = + G1(s)[-Iq1 + wC*Vd0 + Iq0] + G1(s)G2(s)*(Vqr - Vq0)

% Substitute the current plant equations (which we can't directly effect)
% Id1 = sCVd0 - wCVq0 + Id0
% Iq1 = sCVq0 + wCVd0 + Iq0

% Ls*Id1 = + G1(s)[-sCVd0] + G1(s)G2(s)*(Vdr - Vd0)
% Ls*Iq1 = + G1(s)[-sCVq0] + G1(s)G2(s)*(Vqr - Vq0)

% Ls*Id1 = -G1(s)*sC*Vd0 + G1(s)G2(s)*(Vdr - Vd0)
% Ls*Iq1 = -G1(s)*sC*Vq0 + G1(s)G2(s)*(Vdr - Vq0)

% Note: Both our equations use terms from the same d/q components
% Here Idq1/Vdq0 = H(s), which is determined by the type of load we have

% Equations for both dq components
% Ls*H(s)*V0 = -G1(s)*sC*V0 + G1(s)G2(s)*(Vr - V0)
% V0*[H(s)*Ls + sC*G1(s) + G1(s)G2(s)] = G1(s)G2(s)Vr
% V0/Vr = G1(s)G2(s)/[H(s)*Ls + sC*G1(s) + G1(s)G2(s)]

%% Get out transfer functions
s = tf('s');
Z_RL = L_load*s + R_load;
Z_C = 1/(s*C_filter);
Z_L = s*L_filter;

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
T4 = G1/(L_filter*s + G1);
T4 = minreal(T4);

% T5 is our V0/Vr closed loop response for outer voltage controller
T5 = G1*G2/(H*L_filter*s + s*C_filter*G1 + G1*G2);
T5 = minreal(T5);

% T6 is our simplified response for outer voltage controller
T6 = G2/(C_filter*s + G2);
T6 = minreal(T6);

%% Plot the responses
dt = linspace(0, 0.1, 10000);
[dy1,dt1] = step(T4, dt);
[dy2,dt2] = step(T5, dt);
[dy3,dt3] = step(T6, dt);

step_info_1 = stepinfo(dy1, dt1);
step_info_2 = stepinfo(dy2, dt2);
step_info_3 = stepinfo(dy3, dt3);

figure(1);
clf(1);
hold on;
plot(dt1, dy1);
plot(dt2, dy2);
plot(dt3, dy3);
hold off;
grid on;
xlabel("Time (seconds)");
ylabel("Amplitude");
legend("Current control", "Real voltage control", "Naive voltage control");

%% Design an FIR filter for filtering the current and voltage measuremnts
% We have a switching frequency that presents as noise on current
% LC filter doesn't filter out all of the noise

N = 50;
F_stop = F_switch/2;
Fs = 1/Tes;

B_LPF_1 = fir1(N, F_stop/(Fs/2));
fvtool(B_LPF_1, 'Fs', Fs, 'Color', 'White');

%% Can we design a better voltage controller?
% Plant equations
% Idq1 = inductor current, Idq0 = load current, Vdq1 = input voltage, Vdq0 = output voltage
% Vd1 = sLId1 - wLIq1 + Vd0
% Vq1 = sLIq1 + wLId1 + Vq0
% Id1 = sCVd0 - wCVq0 + Id0
% Iq1 = sCVq0 + wCVd0 + Iq0

% Controller parameters
% We have inputs: Vdq1
% We measure Idq1, Idq0, Vdq0
% 