function [SG] = SGServomount
rad_axles = 2.5;
connection_rad= 13;
num_conn_points = 3;

PL_connection = CPLconvexhull([PLcircle(rad_axles+2);PLtrans(PLcircle(rad_axles+2),[0 -connection_rad])]);
PL_connection = [PL_connection;NaN NaN;PLtrans(PLcircle(rad_axles),[0 -connection_rad])];

PL_full_conn = PL_connection;
for i=1:num_conn_points-1
	PL_full_conn = CPLbool('+',PL_full_conn,PLtransR(PL_connection,rot((2*pi)/num_conn_points*i)));
end
SG = SGofCPLz(PL_full_conn,4);

height_SG = max(SG.VL(:,3));
H_b = [roty(180) [0;0;0]; 0 0 0 1];
SG = SGTset(SG,'B',H_b);

for i=0:num_conn_points-1
	pos_gripper = PLtransR([0 -connection_rad],i*(2/3*pi));
	H_f = [rotz(180)*rotz((360/num_conn_points)*i) [pos_gripper';height_SG]; 0 0 0 1];
	SG = SGTset(SG,append('F',num2str(i+1)),H_f);
end
end


%%  [SG] = SGstack(SGs)
%	=== INPUT PARAMETERS ===
%   dir:        Direction of stacking y,x,z
%	SGs:        Array of SGs

%	=== OUTPUT RESULTS ======
%	SG:         SG of stacked SGs
function [SG] = SGstack(dir,varargin)
SG = varargin{1};
for i=2:nargin-1
    switch dir
        case 'z'
            SG = SGcat(SG,SGontop(varargin{i},SG));            
        case 'y'
            SG = SGcat(SG,SGinfront(varargin{i},SG));       
        case 'x'
            SG = SGcat(SG,SGleft(varargin{i},SG));
    end
end
end