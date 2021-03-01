function [SG] = SGconnection1
clf;
rad_axles = 2.5;
connection_length = 10;

PL_out = PLconvexhull([PLcircle(rad_axles+2);PLtrans(PLcircle(rad_axles+2),[connection_length 0])]);
PL_out = CPLbool('-',PL_out,[PLcircle(rad_axles);NaN NaN;PLtrans(PLcircle(rad_axles),[connection_length 0])]);

PL_in = CPLbool('-',PL_out,PLcircle(rad_axles+3));
PL_in = CPLbool('-',PL_in,PLtrans(PLsquare((rad_axles+2)*2,(rad_axles+2)*2,3),[connection_length 0]));
SG_in = SGofCPLz(PL_in,8);
SG_out = SGofCPLz(PL_out,3);
SG = SGstack('z',SG_out,SG_in,SG_out);

height_SG = max(SG.VL(:,3));
H_f = [rotz(0) [0;0;height_SG/2]; 0 0 0 1];
H_b = [rotz(0) [connection_length;0;height_SG/2]; 0 0 0 1];
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