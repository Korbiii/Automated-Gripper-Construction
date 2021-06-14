%%  [SG] = SGstack(SGs)
%	=== INPUT PARAMETERS ===
%   dir:        Direction of stacking y,x,z
%	SGs:        Array of SGs

%	=== OUTPUT RESULTS ======
%	SG:         SG of stacked SGs
function [SG] = SGstack2(dir,varargin)
SG = varargin{1};
for i=2:nargin-1
	if(isfield(varargin{i},'Tname'))
		H_T = SGTget(varargin{i},'F');
		ref_point = varargin{i}.VL(1,:);
	end
	switch dir
		case 'z'
			SG_2 = SGontop(varargin{i},SG);
			SG = SGcat(SG,SG_2);
		case 'y'
			SG = SGcat(SG,SGinfront(varargin{i},SG));
		case 'x'
			SG = SGcat(SG,SGleft(varargin{i},SG));
	end
	if(isfield(varargin{i},'Tname'))
		dif = SG_2.VL(1,:)-ref_point;
		H_T(1:3,4) = H_T(1:3,4) + dif';
		SG = SGTset(SG,'F',H_T);
	end
end
end