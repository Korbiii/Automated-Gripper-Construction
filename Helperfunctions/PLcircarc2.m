%%  [PL] = PLcircarc2(PL,radius)
%	=== INPUT PARAMETERS ===

%	=== OUTPUT RESULTS ======
function [PL] = PLcircarc2(PL)
v1 = PL(1,:)-PL(2,:);
v1 = v1/norm(v1);
v2 = PL(3,:)-PL(2,:);
v2 = v2/norm(v2);

v1_n = v1*rot(pi/2);
v2_n = v2*rot(pi/2);

cp = cross2edges(PL(1,:),v1_n,PL(3,:),v2_n);
dist = min([pdist2(PL(1,:),cp),pdist2(PL(3,:),cp)]);

v3 = PL(1,:)-cp;
v4 = PL(3,:)-cp;
v5 = [1 0];

phi_1 = -acos2(v3,v5);
phi_2 = -acos2(v4,v3);

PL_circle = PLtrans(PLcircseg(dist,'',phi_1,phi_1+phi_2),cp);
PL = PL_circle;

end