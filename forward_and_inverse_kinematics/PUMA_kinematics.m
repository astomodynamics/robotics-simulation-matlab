%{
%% PUMA forward and inverse kinematics Code

Written by Tomohiro Sasaki
%}

global la  lb  lc alpha1  alpha2  alpha3
la = 1. ;
lb = .2 ;
lc = 1. ;
alpha1 = deg2rad(-90);
alpha2 = deg2rad(0);
alpha3 = deg2rad(0);

%% Problem 1  a) (Forward kinematics)
FigNum = 1;
%% case 1
CaseNum = 1;
theta1 = deg2rad(0);
theta2 = deg2rad(-90);
theta3 = deg2rad(90);
[T01, T12, T23] = plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum);

%% Case 2
CaseNum = 2;
theta1 = deg2rad(45);
theta2 = deg2rad(-60);
theta3 = deg2rad(120);
[T01, T12, T23] = plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum);

%% Case 3
CaseNum = 3;
theta1 = deg2rad(45);
theta2 = deg2rad(60);
theta3 = deg2rad(-120);
[T01, T12, T23] = plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum);

%% Case 4
CaseNum = 4;
theta1 = deg2rad(90);
theta2 = deg2rad(45);
theta3 = deg2rad(90);
[T01, T12, T23] = plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum);

%% Case 5
CaseNum = 5;
theta1 = deg2rad(180);
theta2 = deg2rad(-30);
theta3 = deg2rad(-60);
[T01, T12, T23] = plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum);

%% Problem 1 b) (Inverse kinematics)
% for example final position
re =[1, 0.2, 1, 1]';

syms theta1sol theta2sol theta3sol
eq1 = cos(theta1sol)*cos(theta2sol)*re(1) + sin(theta1sol)*cos(theta2sol)*re(2) ...
    - sin(theta2sol)*re(3) == lc*cos(theta3sol) + la;
eq2 = -cos(theta1sol)*sin(theta2sol)*re(1) - sin(theta1sol)*sin(theta2sol)*re(2) ...
    - cos(theta2sol)*re(3) == lc*sin(theta3sol);
eq3 = -sin(theta1sol)*re(1) + cos(theta1sol)*re(2) == lb;

S=vpasolve([eq1 eq2 eq3],[theta1sol theta2sol theta3sol],[0 -pi/4 pi/4]);
    % sol =>
    % theta1 = rad2deg(0); theta2 = rad2deg(-90); theta3 = rad2deg(90);
S=vpasolve([eq1 eq2 eq3],[theta1sol theta2sol theta3sol],[0 pi/4 -pi/4]);
    % sol =>
    % theta1 = rad2deg(0); theta2 = rad2deg(0); theta3 = rad2deg(-90);
S=vpasolve([eq1 eq2 eq3],[theta1sol theta2sol theta3sol],[pi  pi/4 pi/4]);
    % sol =>
    % theta1 = rad2deg(202.6199); theta2 = rad2deg(-90); theta3 = rad2deg(-90);
S=vpasolve([eq1 eq2 eq3],[theta1sol theta2sol theta3sol],[pi  pi/4 -pi/4]);
    % sol =>
    % theta1 = rad2deg(202.6199); theta2 = rad2deg(-180); theta3 = rad2deg(-90);

theta1 = S.theta1sol/pi*180;
theta2 = S.theta2sol/pi*180;
theta3 = S.theta3sol/pi*180;

S.theta1sol;
S.theta2sol;
S.theta3sol;


%% Problem 1 b) (Inverse kinematics)
re = [0.3, 0.3, 0.1]';
FigNum = 2;
syms theta1sol theta2sol theta3sol
eq1 = cos(theta1sol)*cos(theta2sol)*re(1) + sin(theta1sol)*cos(theta2sol)*re(2) ...
    - sin(theta2sol)*re(3) == lc*cos(theta3sol) + la;
eq2 = -cos(theta1sol)*sin(theta2sol)*re(1) - sin(theta1sol)*sin(theta2sol)*re(2) ...
    - cos(theta2sol)*re(3) == lc*sin(theta3sol);
eq3 = -sin(theta1sol)*re(1) + cos(theta1sol)*re(2) == lb;

CaseNum = 1;
S = vpasolve([eq1 eq2 eq3],[theta1sol theta2sol theta3sol],[0 -pi/4 pi/4]);
%     sol =>
%     theta1 = 0.294515; theta2 = 4.646112;  theta3 = 2.751832 rad
%     theta1 = 16.874494; theta2 = 266.202605;  theta3 = 157.668355 deg

theta1 = wrapTo2Pi(double(S.theta1sol));
theta2 =  wrapTo2Pi(double(S.theta2sol));
theta3 =  wrapTo2Pi(double(S.theta3sol));

disp('========')
X = sprintf('theta1 = %f; theta2 = %f;  theta3 = %f rad',(theta1), (theta2), (theta3));
disp(X)
Y = sprintf('theta1 = %f; theta2 = %f;  theta3 = %f deg', rad2deg(theta1), rad2deg(theta2), rad2deg(theta3));
disp(Y)
[T01, T12, T23] = plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum);



CaseNum = 2;
S = vpasolve([eq1 eq2 eq3],[theta1sol theta2sol theta3sol],[0 -pi/4 pi/4+pi]);
% sol=> 
% theta1 = 0.294515; theta2 = 1.114759;  theta3 = 3.531353 rad
% theta1 = 16.874494; theta2 = 63.870960;  theta3 = 202.331645 deg

theta1 = wrapTo2Pi(double(S.theta1sol));
theta2 =  wrapTo2Pi(double(S.theta2sol));
theta3 =  wrapTo2Pi(double(S.theta3sol));

disp('========')
X = sprintf('theta1 = %f; theta2 = %f;  theta3 = %f rad',(theta1), (theta2), (theta3));
disp(X)
Y = sprintf('theta1 = %f; theta2 = %f;  theta3 = %f deg', rad2deg(theta1), rad2deg(theta2), rad2deg(theta3));
disp(Y)
[T01, T12, T23] = plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum);



CaseNum = 3;
S = vpasolve([eq1 eq2 eq3],[theta1sol theta2sol theta3sol],[pi -pi/4 pi/4]);
% sol->
% theta1 = 4.417873; theta2 = 2.026834;  theta3 = 2.751832 rad
% theta1 = 253.125506; theta2 = 116.129040;  theta3 = 157.668355 deg

theta1 = wrapTo2Pi(double(S.theta1sol));
theta2 =  wrapTo2Pi(double(S.theta2sol));
theta3 =  wrapTo2Pi(double(S.theta3sol));

disp('========')
X = sprintf('theta1 = %f; theta2 = %f;  theta3 = %f rad',(theta1), (theta2), (theta3));
disp(X)
Y = sprintf('theta1 = %f; theta2 = %f;  theta3 = %f deg', rad2deg(theta1), rad2deg(theta2), rad2deg(theta3));
disp(Y)
[T01, T12, T23] = plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum);



CaseNum = 4;
S = vpasolve([eq1 eq2 eq3],[theta1sol theta2sol theta3sol],[pi -pi/4 3/2*pi]);
% sol=>
% theta1 = 4.417873; theta2 = 4.778666;  theta3 = 3.531353 rad
% theta1 = 253.125506; theta2 = 273.797395;  theta3 = 202.331645 deg

theta1 = wrapTo2Pi(double(S.theta1sol));
theta2 =  wrapTo2Pi(double(S.theta2sol));
theta3 =  wrapTo2Pi(double(S.theta3sol));

disp('========')
X = sprintf('theta1 = %f; theta2 = %f;  theta3 = %f rad',(theta1), (theta2), (theta3));
disp(X)
Y = sprintf('theta1 = %f; theta2 = %f;  theta3 = %f deg', rad2deg(theta1), rad2deg(theta2), rad2deg(theta3));
disp(Y)
[T01, T12, T23] = plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum);


%% Problem 1 c)
FigNum = 3;
k_span = [0:1:12]';

syms theta1sol theta2sol theta3sol

for k = 1:length(k_span)
    CaseNum = k;
    re = [0.5+0.2*cos(2*k*pi/12), 0.2*sin(2*k*pi/12), 0]'

    eq1 = cos(theta1sol)*cos(theta2sol)*re(1) + sin(theta1sol)*cos(theta2sol)*re(2) ...
    - sin(theta2sol)*re(3) == lc*cos(theta3sol) + la;
    eq2 = -cos(theta1sol)*sin(theta2sol)*re(1) - sin(theta1sol)*sin(theta2sol)*re(2) ...
        - cos(theta2sol)*re(3) == lc*sin(theta3sol);
    eq3 = -sin(theta1sol)*re(1) + cos(theta1sol)*re(2) == lb;
    
    S = vpasolve([eq1 eq2 eq3],[theta1sol theta2sol theta3sol],[0 0 pi/2]);
    theta1 = wrapTo2Pi(double(S.theta1sol));
    theta2 =  wrapTo2Pi(double(S.theta2sol));
    theta3 =  wrapTo2Pi(double(S.theta3sol));
    plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum)
    
end





