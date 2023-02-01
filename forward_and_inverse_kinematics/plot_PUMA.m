function [T01, T12, T23] = plot_PUMA(theta1, theta2, theta3, FigNum, CaseNum)
global la  lb  lc alpha1  alpha2  alpha3

% setup homogeneous transformation (Denavit-Hatenberg)
% rotation matrix of frame 1 w.r.t. frame 0
R01 = rotz(theta1);
T01 = rotm2tform(R01);

% rotation matrix of frame 2 w.r.t. frame 1
R12 = rotx(alpha1) * rotz(theta2) ;
T12 = rotm2tform(R12);

% rotation matrix of frame 3 w.r.t. frame 2
R23 = rotx(alpha2) * rotz(theta3);
T23 = rotm2tform(R23);
T23(1:3,4) = [la, 0, 0]';

% rotation matrix of frame 4 w.r.t. frame 3
T34 = rotm2tform(rotx(alpha3) * rotz(0.));
T34(1:3,4) = [lc, 0, lb]';

T03 = T01 * T12 * T23;

%Endpoint w.r.t. frame 3
r3 = [lc;0;lb;1];
r0 = T03 * r3; % position of effector
% r0 = [0.5657, 0.8485, 0, 1.0000]';

% position of elbow
rm=T03*[0;0;lb;1];
% rm = [0.2121, 0.4950, 0.8660, 1.0000]'

T04 = T01 * T12 * T23 * T34;


figure(FigNum)
hold on
if CaseNum < 2
    plot3([0.1 -0.1 -0.1 0.1 0.1],[0.1 0.1 -0.1 -0.1 0.1],[0 0 0 0 0],'g','DisplayName',"base square"); %base square
end
FigName = "sol" + string(CaseNum);
plot3([0 T03(1,4) rm(1) r0(1)],[0 T03(2,4) rm(2) r0(2)],[0 T03(3,4) rm(3) r0(3)],'DisplayName', FigName);
plot3(rm(1),rm(2),rm(3),'rv','DisplayName',"elbow");
plot3(r0(1),r0(2),r0(3),'ro','DisplayName',"effecter");

axis equal
% legend()
axis([-1 1 -1 1 -0.5 1.5])
grid
hold off
view(-30,30)

end

