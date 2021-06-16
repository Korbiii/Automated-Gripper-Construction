%%  [SG] = SGstack(SGs)
%	=== INPUT PARAMETERS ===
%   dir:        Direction of stacking y,x,z
%	SGs:        Array of SGs

%	=== OUTPUT RESULTS ======
%	SG:         SG of stacked SGs
function [SG] = SGstack(dir,varargin)
SG = varargin{1};
Ts = {};
ref_points = [];
for i=2:nargin-1
	if(isfield(varargin{i},'Tname'))
		Tname = varargin{i}.Tname;
		ref_point = varargin{i}.VL(1,:);
		for j=1:size(Tname,2)
			Ts{end+1}=SGTget(varargin{i},Tname{j});
		end
	end	
    switch dir
		case 'z'
			SG_2 = SGontop(varargin{i},SG);
		case 'y'
			SG_2 = SGinfront(varargin{i},SG);
		case 'x'
			SG_2 = SGleft(varargin{i},SG);
		case 'z-'
			SG_2 = SGunder(varargin{i},SG);
		case 'y-'
			SG_2 = SGbehind(varargin{i},SG);
		case 'x-'
			SG_2 = SGright(varargin{i},SG);
	end
	SG = SGcat(SG,SG_2);
	if(isfield(varargin{i},'Tname'))
		dif = SG_2.VL(1,:)-ref_point;
		for j=1:size(Tname,2)			
			Ts{j}(1:3,4) = Ts{j}(1:3,4) + dif';
			SG = SGTset(SG,Tname{j},Ts{j});
		end
		Ts = {};
	end
	
	
end
end