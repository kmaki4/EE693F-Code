M = 16;
theta = diag(exp(1j*(rand(M, 1)*2*pi)));

C = (abs(g)').*theta.*abs(h)+h_SISO;
N_o = -100;

Pt = linspace(0,40,1000);

p = Pt.*(abs(C)).^2/N_o;

R = log2(1+p);

plot(Pt, R)