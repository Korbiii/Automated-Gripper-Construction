function [SG] = SGconnection2
rad_axles = 2.5;
connection_length = 13;

PL_side = CPLbool('+',PLcircle(rad_axles+2),PLtrans(PLsquare(connection_length,4),[connection_length/2 -rad_axles]));
PL_side = CPLbool('-',PL_side,PLcircle(rad_axles));
SG_side = SGofCPLz(PL_side,8);

SG_hole = SGofCPLz(PLcircle(rad_axles),10);
SG_hole = SGtransrelSG(SG_hole,SG_side,'rotx',pi/2,'centerz','centery','transx',connection_length-rad_axles-2);

SG = SGbool('-',SG_side,SG_hole);


height_SG = max(SG.VL(:,3));
H_b = [rotz(0) [0;0;height_SG/2]; 0 0 0 1];
H_f = [rotx(90)*rotz(-90) [connection_length-rad_axles-2;-(((rad_axles*2)-4)/2);height_SG/2]; 0 0 0 1];
SG = SGTset(SG,'F',H_f);
SG = SGTset(SG,'B',H_b);
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