function [SG] = SGUnterarm()
clf;
CPL_cable_slot = PLkidney(22.5,15,pi);
CPL_screw_holes = CPLcopyradial(PLcircle(1.6),10.5,8);
PLlayer1 = [PLcircle(40);NaN NaN;PLcircle(37.7);NaN NaN;PLcircle(19);NaN NaN;PLcircle(7.5)];
PLlayer1 = CPLbool('-',PLlayer1,CPL_cable_slot);
PLlayer1 = CPLbool('-',PLlayer1,CPL_screw_holes);
SGlayer1 = SGofCPLz(PLlayer1,6.325);

PLlayer2 = [PLcircle(40);NaN NaN;PLcircle(37.7);NaN NaN;PLcircle(29.7)];
PLlayer2 = CPLbool('-',PLlayer2,CPL_cable_slot);
PLlayer2 = CPLbool('-',PLlayer2,CPL_screw_holes);
SGlayer2 = SGofCPLz(PLlayer2,11.2);

PLlayer3 = [PLcircle(40)];
PLlayer3 = CPLbool('-',PLlayer3,CPL_cable_slot);
PLlayer3 = CPLbool('-',PLlayer3,CPL_screw_holes);
SGlayer3 = SGofCPLz(PLlayer3,11.2);

PLlayer4 = [PLcircle(40);NaN NaN;PLcircle(22.5)];
SGlayer4 = SGofCPLz(PLlayer4,15);

SG = SGstack('z',SGlayer1,SGlayer2,SGlayer3,SGlayer4);
SGplot(SG);


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