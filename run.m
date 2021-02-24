clear
clc

% Vertical Strut Bending:
E = 70e9;           %Pa (Young's modulus of strut material)
I = 37.2;           %cm^4 (second moment of area of strut section)
I = 125;            %cm^4 (second moment of area of strut section)
I = I*(1/100)^4;    %m^4
y = 40e-4/2;        %m
y = 90e-3/2;        %m
L = 1400;           %mm (length of vertical strut)
h_a = 710;          %mm (actuator height at nominal position)
%With supporting A-frame:
Z_N = 940;          %mm
%Z_N sweep:
% Z_N = 200:10:1400;    %mm
% Z_N = 560:1:570;
Z_N = Z_N/1000;     %m


% Force magnitudes defined such that:
% <0 => Suspension arm in compression
% >0 => Suspension arm in tension

%Upper wishbone inboard front
UIFR = [178.51 250.87 281.96]; %Position
UIFV = [-0.54695 0.82885 0.11772]; %Direction
UIFM = -793.314409; %magnitude

%Upper wishbone inboard rear
UIRR = [-177.74 281.38 264.58]; %Position
UIRV = [0.514318814 0.836795346 0.187749054]; %Direction
UIRM = -560.3454574; %magnitude

%Lower wishbone inboard front
LIFR = [176.98 252.3 124.89]; %Position
LIFV = [-0.477138521 0.877673666 0.045030746]; %Direction
LIFM = 4464.581782; %Magnitude

%Lower wishbone inboard rear
LIRR = [-175.12 281.87 126.41]; %Position
LIRV = [0.549395053 0.834465564 0.042805352]; %Direction
LIRM = 2995.749875; %Magnitude

%Rocker pivot
RPR = [30 158.4 459.5]; %Position
RPV = [0 -0.609313 0.79293]; %Direction
RPM = 12706;%Magnitude

%Damper to chassis
DCR = [30 185.78 271.56]; %Position
DCV = [0 -0.533056104 0.846079896]; %Direction
DCM = -6036.413019; %Magnitude

%Trackrod inboard
TIR = [75.89 230 201]; %Position
TIV = [0 0.992417851 0.122909761]; %Direction
TIM = -775.356131; %Magnitude

%Resultant position
R = [0 0 0];

%construct matrix of hardpoint positions relative to the resultant position
Pos = [UIFR-R; UIRR-R; LIFR-R; LIRR-R; RPR-R; DCR-R; TIR-R];
%convert into m
Pos = Pos./1000;

%construct matrix of hardpoint force vectors
Dir = [UIFV; UIRV; LIFV; LIRV; RPV; DCV; TIV];

%construct array of force magnitudes
Mag = [UIFM; UIRM; LIFM; LIRM; RPM; DCM; TIM];

%construct array of points at which forces act on the mounting plate
hardpoints = [UIFR; UIRR; LIFR; LIRR; RPR; DCR; TIR];

%Plot force vectors
figure(1)
for i = 1:length(hardpoints)
    quiver3(hardpoints(i,1), hardpoints(i,2), hardpoints(i,3), Dir(i,1)*sign(Mag(i)), Dir(i,2)*sign(Mag(i)), Dir(i,3)*sign(Mag(i)), abs(Mag(i)))
hold on
end
xlabel('x')
ylabel('y')
zlabel('z')

%construct array of force components
Comp = Dir(:,:).*Mag(:,1);

%sum moments about R due to forces
%M1 = cross(Pos(1,:),Comp(1,:));
M = cross(Pos(:,:),Comp(:,:));

%find the total moment at the resultant point
Mt = sum(M);

%sum of all the forces on the mounting plate
Ft = sum(Comp);

%extract resultant force in y-direction
Fy = Ft(2);

%extract moment about the x-axis
Mx = Mt(1);

%position vectors of the bearing locations
%x and z seperation of the bearing locations
Xsep = 600; %356.25;
Zsep = 300;

%bearing location "centre"
B = [0 200 200];

%matrix of bearing locations
Bloc = [B(1)+Xsep/2 B(2) B(3)-Zsep/2; B(1)+Xsep/2 B(2) B(3)+Zsep/2; B(1)-Xsep/2 B(2) B(3)+Zsep/2; B(1)-Xsep/2 B(2) B(3)-Zsep/2];

%matrix of position vectors from resultant location to bearing locations
Bpos = [Bloc(1,:)-R; Bloc(2,:)-R; Bloc(3,:)-R; Bloc(4,:)-R];
Bpos = Bpos./1000;

%solve for bearing forces F = [F1; F2] = [Bottom; Top]
M = [1 1; -2*Bpos(1,3) -2*Bpos(2,3)];
F = inv(M)*[-Fy; -Mx];

%section to plot moments across the plate using iteration

%% Vertical Strut Bending
n = 100;     % number of distance discretised points
z = linspace(0, L, n)/1000;    %m

B(3) = B(3) + h_a;  %mm (add vertical separation of origin from ground)
% Without supporting 45deg strut:
% F_R = sum(F); %N
% M_R = F(1)*(B(3)-Zsep/2) + F(2)*(B(3)+Zsep/2);  %Nmm
% M_R = M_R/1000;     %Nm
% 
% C1 = 0;
% C2 = 0;
% for i=1:length(z)
%     if z(i) < (B(3) - Zsep/2)/1000
%         v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + C1*z(i) + C2)/(E*I);    %m
%     elseif z(i) < (B(3) + Zsep/2)/1000
%         v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/6*F(1)*(z(i)-(B(3)-Zsep/2)/1000).^3 + C1*z(i) + C2)/(E*I);    %m
%     else
%         v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/6*F(1)*(z(i)-(B(3)-Zsep/2)/1000).^3 + 1/6*F(2)*(z(i)-(B(3)+Zsep/2)/1000).^3 + C1*z(i) + C2)/(E*I);    %m
%     end
% end

% With supporting 45deg strut:
stress = [];
e2 = [];    % height difference between bearings due to deflection (in the y-direction) 
for z_N = Z_N
    % 4 unknowns (incl. M_N):
    % d2v/dz2 = 0:
%     Q = [[1 1 0 0];
%         [0 z_N 1 1];
%         [-1/6*z_N^3 0 0.5*z_N^2 0];
%         [-z_N 0 1 0]];
%     R_mat = Q \ [F(1)+F(2); F(1)*(B(3) - Zsep/2)/1000 + F(2)*(B(3) + Zsep/2)/1000; -1/6*F(1)*(z_N - (B(3) - Zsep/2)/1000)^3; -F(1)*(z_N - (B(3) - Zsep/2)/1000)];
    % dv/dz = 0:
    Q = [[1 1 0 0];
        [0 z_N 1 1];
        [-1/6*z_N^3 0 0.5*z_N^2 0];
        [-0.5*z_N^2 0 z_N 0]];

    % 3 unknowns (excl. M_N):
%     M_N = 0;
    % BC: dv/dz = 0:
%     Q = [[1 1 0];
%         [0 z_N 1];
%         [-0.5*z_N^2 0 z_N]];
%     R_mat = Q \ [F(1)+F(2); F(1)*(B(3) - Zsep/2)/1000 + F(2)*(B(3) + Zsep/2)/1000; 0];
%     F_R = R_mat(1);
%     N = R_mat(2);
%     M_R = R_mat(3);
    % BC: v(z_N) = 0:
%     Q = [[1 1 0];
%         [0 z_N 1];
%         [-1/6*z_N^3 0 0.5*z_N^2]];
%     R_mat = Q \ [F(1)+F(2); F(1)*(B(3) - Zsep/2)/1000 + F(2)*(B(3) + Zsep/2)/1000; 0];
%     F_R = R_mat(1);
%     N = R_mat(2);
%     M_R = R_mat(3);

    C1 = 0;
    C2 = 0;
    v = zeros(1, length(z));
    if z_N < (B(3) - Zsep/2)/1000
        R_mat = Q \ [F(1)+F(2); F(1)*(B(3) - Zsep/2)/1000 + F(2)*(B(3) + Zsep/2)/1000; 0; 0];
        F_R = R_mat(1);
        N = R_mat(2);
        M_R = R_mat(3);
        M_N = R_mat(4);
        for i=1:length(z)
            if z(i) < z_N
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i);   %Nm
            elseif z(i) < (B(3) - Zsep/2)/1000
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/2*M_N*(z(i)-z_N).^2 - 1/6*N*(z(i) - z_N).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i) + M_N - N*(z(i) - z_N);   %Nm
            elseif z(i) < (B(3) + Zsep/2)/1000
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/2*M_N*(z(i)-z_N).^2 - 1/6*N*(z(i) - z_N).^3 + 1/6*F(1)*(z(i)-(B(3)-Zsep/2)/1000).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i) + M_N - N*(z(i) - z_N) + F(1)*(z(i) - (B(3)-Zsep/2)/1000);   %Nm
            else
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/2*M_N*(z(i)-z_N).^2 - 1/6*N*(z(i) - z_N).^3 + 1/6*F(1)*(z(i)-(B(3)-Zsep/2)/1000).^3 + 1/6*F(2)*(z(i)-(B(3)+Zsep/2)/1000).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i) + M_N - N*(z(i) - z_N) + F(1)*(z(i) - (B(3)-Zsep/2)/1000) + F(2)*(z(i) - (B(3)+Zsep/2)/1000);   %Nm
            end
        end
    elseif z_N < (B(3) + Zsep/2)/1000
        R_mat = Q \ [F(1)+F(2); F(1)*(B(3) - Zsep/2)/1000 + F(2)*(B(3) + Zsep/2)/1000; -1/6*F(1)*(z_N - (B(3) - Zsep/2)/1000)^3; -0.5*F(1)*(z_N - (B(3) - Zsep/2)/1000)^2];
        F_R = R_mat(1);
        N = R_mat(2);
        M_R = R_mat(3);
        M_N = R_mat(4);
        for i=1:length(z)
            if z(i) < (B(3) - Zsep/2)/1000
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i);   %Nm
            elseif z(i) < z_N
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/6*F(1)*(z(i)-(B(3)-Zsep/2)/1000).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i) + F(1)*(z(i) - (B(3)-Zsep/2)/1000);   %Nm
            elseif z(i) < (B(3) + Zsep/2)/1000
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/2*M_N*(z(i)-z_N).^2 - 1/6*N*(z(i) - z_N).^3 + 1/6*F(1)*(z(i)-(B(3)-Zsep/2)/1000).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i) + M_N - N*(z(i) - z_N) + F(1)*(z(i) - (B(3)-Zsep/2)/1000);   %Nm
            else
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/2*M_N*(z(i)-z_N).^2 - 1/6*N*(z(i) - z_N).^3 + 1/6*F(1)*(z(i)-(B(3)-Zsep/2)/1000).^3 + 1/6*F(2)*(z(i)-(B(3)+Zsep/2)/1000).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i) + M_N - N*(z(i) - z_N) + F(1)*(z(i) - (B(3)-Zsep/2)/1000) + F(2)*(z(i) - (B(3)+Zsep/2)/1000);   %Nm
            end
        end
    else
        R_mat = Q \ [F(1)+F(2); F(1)*(B(3) - Zsep/2)/1000 + F(2)*(B(3) + Zsep/2)/1000; -1/6*(F(1)*(z_N - (B(3) - Zsep/2)/1000)^3 + F(2)*(z_N - (B(3) + Zsep/2)/1000)^3); -0.5*(F(1)*(z_N - (B(3) - Zsep/2)/1000)^2 + F(2)*(z_N - (B(3) + Zsep/2)/1000)^2)];
        F_R = R_mat(1);
        N = R_mat(2);
        M_R = R_mat(3);
        M_N = R_mat(4);
        for i=1:length(z)
            if z(i) < (B(3) - Zsep/2)/1000
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i);   %Nm
            elseif z(i) < (B(3) + Zsep/2)/1000
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/6*F(1)*(z(i)-(B(3)-Zsep/2)/1000).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i) + F(1)*(z(i) - (B(3)-Zsep/2)/1000);   %Nm
            elseif z(i) < z_N
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/6*F(1)*(z(i)-(B(3)-Zsep/2)/1000).^3 + 1/6*F(2)*(z(i)-(B(3)+Zsep/2)/1000).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i) + F(1)*(z(i) - (B(3)-Zsep/2)/1000) + F(2)*(z(i) - (B(3)+Zsep/2)/1000);   %Nm
            else
                v(i) = (1/2*M_R*z(i).^2 - 1/6*F_R*z(i).^3 + 1/6*F(1)*(z(i)-(B(3)-Zsep/2)/1000).^3 + 1/6*F(2)*(z(i)-(B(3)+Zsep/2)/1000).^3 + 1/2*M_N*(z(i)-z_N).^2 - 1/6*N*(z(i)-z_N).^3 + C1*z(i) + C2)/(E*I);    %m
                M_b(i) = M_R - F_R*z(i) + M_N - N*(z(i) - z_N) + F(1)*(z(i) - (B(3)-Zsep/2)/1000) + F(2)*(z(i) - (B(3)+Zsep/2)/1000);   %Nm
            end
        end
    end
    stress = [stress; M_b.*y/I];              %Pa

    % Plot deflection
    figure(2)
    plot(z,v*1000)
    xlabel('z (m)')
    ylabel('v (mm)')
    % Plot bearing positions
    B_ind = round(n*[(B(3)-Zsep/2)/L, (B(3) + Zsep/2)/L]);
    z_b = [];
    v_b = [];
    for b = B_ind
        z_b = [z_b, z(b)];  % m
        v_b = [v_b, v(b)];  % m
    end
    hold on
    scatter(z_b, v_b*1000)
    plot(z(b), v(b))
    
    figure(3)
    plot(z, stress/10^6)
    xlabel('z (mm)')
    ylabel('stress (MPa)')

    e2 = [e2, 1000*abs(v_b(1) - v_b(2))];   %mm
end
[e2_min, e2_min_ind] = min(e2);
z_N = Z_N(e2_min_ind);  %m

disp('e2 = ' + string(e2_min) + ' mm at ' + string(z_N*1000) + ' mm');