function [SG] = SGconnRod
clf;
rad_axles = 2.5;
length_conn = 20;

PL_rod = CPLconvexhull([PLcircle(rad_axles+2);PLtrans(PLcircle(rad_axles+2),[length_conn 0])]);
SG = SGofCPLz(PL_rod,4);



height_SG = 0;
H_b = [roty(180) [0;0;height_SG/2]; 0 0 0 1];
H_f = [roty(180) [length_conn;0;height_SG/2]; 0 0 0 1];
SG = SGTset(SG,'F',H_f);
SG = SGTset(SG,'B',H_b);


end