clear all;
close all;

load("RIS_Channels4.mat")
%h = RIS to transmitter
%g = receiver to RIS
%h_SISO = receiver to transmitter

M = size(h,1);
theta = diag(exp(1j*(rand(M, 1)*2*pi)));

C = (abs(g)')*theta*abs(h)+h_SISO;
N_o = db2pow(-100);
Pt = db2pow(linspace(0,40,100));
R= zeros(1,size(Pt,2));

for Z = 1:size(Pt,2)
    p = Pt(Z).*abs(C).^2./N_o;
    l = log2(1+p);
    R(Z) = mean(l, "all");
end

plot(pow2db(Pt), R)
temp = pow2db(Pt);
save("N_256_H_1.5.mat","temp","R")
grid on;


