clc

load('inboard_hardpoints.mat');
load('length.mat');
load('rocker_unit_vec.mat');
load('l_rocker.mat');
load('unit_dir_vec.mat')
load('M.mat');   % Force-Moment Matrix

%[Fz,~] = max(out.Fr);
Fz = [0,0,5000];

F = [0,0,Fz(3),0.6*Fz(3),0,0]';  % Load at tyre contact patch

%% Calculate Suspension Arm Loads
inverted = inv(M);
T = inv(M)*F;    % Tensions in suspension members (tension +ve, compression -ve)

disp('Front UWB Arm = ' + string(T(1)) + ' N');
disp('Rear UWB Arm = ' + string(T(2)) + ' N');
disp('Front LWB Arm = ' + string(T(3)) + ' N');
disp('Rear LWB Arm = ' + string(T(4)) + ' N');
disp('Push Rod = ' + string(T(5)) + ' N');
disp('Track Rod = ' + string(T(6)) + ' N');

%% T5 Transform

l_pr_dir = rocker_unit_vec(1,:);
l_pr = l_pr_dir.*l_rocker(1);

l_rc_dir = rocker_unit_vec(2,:);
l_rc = l_rc_dir.*l_rocker(2);

T5_dir = unit_dir_vec(5,:);

T5_prime_dir = rocker_unit_vec(3,:);

T5_prime = T(5)*(cross(l_pr,(T5_dir - dot(T5_dir, l_pr_dir).*l_pr_dir)))./(cross(l_rc,(T5_prime_dir - dot(T5_prime_dir, l_rc_dir).*l_rc_dir)));

T(5) = -1*T5_prime(1);      % x -1 since direction is reversed between acting on rocker and plate (-ve in compression).
unit_dir_vec(5,:) = T5_prime_dir(1,:);    

T5_prime = T(5)*T5_prime_dir;     
disp('T5_prime = ' + string(T(5)) + ' N');

%% Plot Suspension Arms
for i = 1:6
    quiver3(inboard_hardpoints(i,1), inboard_hardpoints(i,2), inboard_hardpoints(i,3), -unit_dir_vec(i,1), -unit_dir_vec(i,2), -unit_dir_vec(i,3), length(i))
    hold on
end

quiver3(30, 88.56, 425.87, -T5_prime_dir(1), -T5_prime_dir(2), -T5_prime_dir(3), l_rocker(3))

%% Calculate Bearing Loads - Not Working (all matrices singular)
H = 300;    % mm
W = 356.25;    % mm

z_mean = mean([mean([inboard_hardpoints(1,3), inboard_hardpoints(2,3)]), mean([inboard_hardpoints(3,3),inboard_hardpoints(4,3)])]);   % mm
z_u = (z_mean + H/2)/1000;  % m
z_L = (z_mean - H/2)/1000;  % m
y = 0.6;                    % m
x = (W/2)/1000;             % m

% A = [[2 0 0 2 0 0],
%     [0 2 0 0 2 0],
%     [0 0 2 0 0 2],
%     [0 -2*z_u 2*y 0 -2*z_L 2*y],
%     [-2*z_u 0 0 -2*z_L 0 0],
%     [-2*y 0 0 -2*y 0 0]];

A = [[4 0 0 0 0 0],
    [0 1 1 1 1 0],
    [0 0 0 0 0 4],
    [0 -z_u -z_u -z_L -z_L 0],
    [-2*(z_u+z_L) 0 0 0 0 0],
    [-4*y x -x x -x 0]];

B = M;
for i = 1:6
    B(4,i) = -(inboard_hardpoints(i,2)/1000*unit_dir_vec(i,3)-inboard_hardpoints(i,3)/1000*unit_dir_vec(i,2));
    B(5,i) = (inboard_hardpoints(i,3)/1000*unit_dir_vec(i,1)-inboard_hardpoints(i,1)/1000*unit_dir_vec(i,3));
    B(6,i) = -(inboard_hardpoints(i,1)/1000*unit_dir_vec(i,2)-inboard_hardpoints(i,2)/1000*unit_dir_vec(i,1));
end

%R = A\B*T

%% Bearing Loads Simplified
h_w = 232.41;  % mm

% Test to see influence of push rod force:
%T(5) = 0;
% Test to see influence of track rod force:
%T(6) = 0;

% Add T7 (rocker pivot reaction force):
T(7) = 12706;    % N
unit_dir_vec(7,:) = [0, -0.609313, 0.79293];
inboard_hardpoints(7,:) = [30, 158.4, 459.5];       % mm

% Ry_L = (sum(T.*unit_dir_vec(:,2))*(h_w/1000 + (H/2)/1000) - dot((T.*unit_dir_vec(:,2)),inboard_hardpoints(:,2)/1000))/(H/1000);
% Ry_U = sum(T.*unit_dir_vec(:,2)) - Ry_L;

% disp('Ry_U = ' + string(Ry_U) + ' N');
% disp('Ry_L = ' + string(Ry_L) + ' N');

% Sense-Checking
Fy = T.*-unit_dir_vec(:,2);
Fy(5,:) = T(5).*unit_dir_vec(5,2);
Fy(7,:) = T(7).*unit_dir_vec(7,2);
Fy_sum = sum(Fy)
% Fy_reac = Ry_L + Ry_U

Mx = Fy.*inboard_hardpoints(:,3)/1000
Mx_sum = sum(Mx)
% Mx_reac = (Ry_U*(h_w+H/2)+Ry_L*(h_w-H/2))/1000

% Check innacuracy by neglecting effect of Fz on Mx
Fz = T.*-unit_dir_vec(:,3);
Fz(5,:) = T(5).*unit_dir_vec(5,3);
Fz(7,:) = T(7).*unit_dir_vec(7,3);
Fz_sum = sum(Fz)

Mx_z = Fz.*inboard_hardpoints(:,2)/1000     %Contribution of Fz to Mx
Mx_z_sum = sum(Mx_z)

%Total resultant moment due to suspension arm loads:
Mx_res = Mx_sum + Mx_z_sum

Ry_L = (-Fy_sum*(h_w + H/2)/1000 - Mx_sum)/(H/1000);
Ry_U = -Fy_sum - Ry_L;

%T(5) = 0;   % T5, pushrod is in y-z plane so has no influence on x-forces
Tx = T.*unit_dir_vec(:,1);
Tz = T.*unit_dir_vec(:,3);

Rx_L = (dot(Tz, inboard_hardpoints(:,1)/1000) - dot(Tx, inboard_hardpoints(:,3)/1000) + z_u*dot(T, inboard_hardpoints(:,1)/1000))/(2*(z_L-z_u));
Rx_U = -(0.5*dot(T, inboard_hardpoints(:,1)/1000) + Rx_L);

disp('Rx_U = ' + string(Rx_U) + ' N');
disp('Rx_L = ' + string(Rx_L) + ' N');

% Sense-Checking
Fx = sum(Tx) + 2*Rx_L + 2*Rx_U
My = 2*z_u*Rx_U + 2*z_L*Rx_L + dot(Tx, inboard_hardpoints(:,3)/1000) - dot (Tz, inboard_hardpoints(:,1)/1000)