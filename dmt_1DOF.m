%Transmissibility plots
% F.Cegla 07/10/2020
%===============================

MR = 1.55;

k = 80000 * MR^2;
m = 50;
zeta=[0.4*MR^2];

omega_n = sqrt(k/m);

f_max = 25; % Hz
df = 0.5;
f = 0:df:f_max;

omega = f*2*pi;

y = 50; % mm

r=linspace(0,omega(end)/omega_n,length(omega));
T=zeros(length(zeta),length(r));
phase=zeros(length(zeta),length(r));

for count=1:length(zeta)
T(count,:)=sqrt((2.*zeta(count).*r).^2+1)./sqrt((1-r.^2).^2+(2.*zeta(count).*r).^2);
phase(count,:)=atan(2.*zeta(count).*r.^3./((1-r.^2)+(2.*zeta(count).*r).^2));
end

% Amplitude of displacement of sprung mass, x:
x = T*y;
subplot(1,2,1)
plot(f,x)
xlabel('f (Hz)')
ylabel('x (mm)')

%Force, frequency
F = k.*x.*sqrt((1-r.^2).^2 + (2*zeta*r).^2)/10^6;

subplot(1,2,2)
plot(f,F);
xlabel('f (Hz)')
ylabel('F (kN)')
% figure
% for count=1:length(zeta)
%     subplot(2,1,1)
%     plot(r,T(count,:))
%     if count==1
%         hold
%     end
%     subplot(2,1,2)
%     plot(r,(unwrap(phase(count,:).*2)./2)./pi.*180)
%     if count==1
%         hold
%     end
% end
% xlabel('Normalised frequency (\omega/\omega_n)')
% ylabel('Phase (degrees)')
% subplot(2,1,1)
% ylabel('Magnitude')
% legend('\zeta=0.05','\zeta=0.1','\zeta=0.25','\zeta=0.5','\zeta=01')
