%% Q3) High Level: Grid-Forming and Grid-Feeding Inverters 
% Calculate power from supply when grid feeding
L = 10e-3;
f = 50;
w = 2*pi*f;

ZL = 1j*w*L;

Vrms = 400;

k = 1 + 1.156e-2;
phi = 5 * pi/180;

Vgrid = Vrms;
Vs = k * Vrms * exp(1j*phi);

Is = (Vs-Vgrid) / ZL;

S = Vs*Is;
pf = cos(angle(S));

PQ = [real(S), imag(S)];


