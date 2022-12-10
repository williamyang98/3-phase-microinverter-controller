%% Q2) Satisfactory Level: Grid-Forming Inverters
% Correct for zero in second order response
L_filter = 40e-3;
C_filter = 100e-6;

Wn = 2*pi*200;
DF = 0.707;
% Wn^2 = Ki/L
Ki_current = L_filter*Wn^2;
% 2*DF*Wn = Kp/L
Kp_current = 2*L_filter*DF*Wn;

%% Plot transfer function
s = tf('s');
G1 = Kp_current + Ki_current/s;
T1 = G1/(L_filter*s + G1);
T1 = minreal(T1);

G2 = Kp_current + Ki_current/s + s*0.001;
T2 = G2/(L_filter*s + G2);

dt = linspace(0, 0.02, 10000);
dy1 = step(T1, dt);
dy2 = step(T2, dt);

figure(1);
clf(1);
hold on;
plot(dt, dy1);
plot(dt, dy2);
hold off;
xlabel("Time (seconds)");
ylabel("Amplitude");
grid on;
legend("Original", "Corrected");