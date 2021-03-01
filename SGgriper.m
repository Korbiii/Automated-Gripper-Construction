function [SG] = SGgriper

rad_axles = 2.5;
height_grip = 50;
grip_length = 30;
second_axle_distance = 10;

PL_bottom = PLconvexhull([PLcircle(rad_axles+2);PLtrans(PLcircle(rad_axles+1),[0 height_grip/2])]);
PL_bottom = CPLbool('-',PL_bottom,[PLcircle(rad_axles);NaN NaN;PLtrans(PLcircle(rad_axles),[0 second_axle_distance])]);
PL_top = PLconvexhull([PLtrans(PLcircle(rad_axles+1),[0 height_grip/2]);PLtrans(PLcircle(1),[grip_length height_grip])]);
PL = CPLbool('+',PL_bottom,PL_top);

SG = SGofCPLz(PL,8);
height_SG = max(SG.VL(:,3));

H_b = [rotz(0) [0;0;height_SG/2]; 0 0 0 1];
H_f = [rotz(0) [0;second_axle_distance;height_SG/2]; 0 0 0 1];
SG = SGTset(SG,'B',H_b);
SG = SGTset(SG,'F',H_f);
SGTplot(SG);
end