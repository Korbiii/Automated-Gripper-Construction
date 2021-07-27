% SGcatF(SG1,SG2,[SG3],...) - returns concatenation of two or more solids or sets of
%	while preserving all inserted Frames.
% 
%	Compared to the normal SGcat function this function is designed to
%	preserve all Frames of the inputted SGs instead of just from the first
%	one.% 
% 
%   SG = SGcatF(SG1,SG2,[SG3],...)
%   === INPUT PARAMETERS ===
%   SG1:		First SG to concatenate
%	SG2:		Second SG to concatenate
%	...			More SG to concatenate
%   === OUTPUT RESULTS ======
%   SG :		Concatenated SG of all input SGs with all Frames
function [SG] = SGcatF(varargin)
num_inputs = nargin;
if num_inputs == 1
	temp = varargin{1};
	varargin = {};
	for i=1:size(temp,2)
		varargin{end+1} = temp{i};
	end
	num_inputs = size(temp,2);
end

SG = varargin{1};
Ts = {};
for i=2:num_inputs
	if(isfield(varargin{i},'Tname'))
		Tname = varargin{i}.Tname;
		for j=1:size(Tname,2)
			Ts{end+1}=SGTget(varargin{i},Tname{j});
		end			
	end	
	
	SG = SGcat(SG,varargin{i});
	
	if(isfield(varargin{i},'Tname'))
		for j=1:size(Tname,2)
			SG = SGTset(SG,Tname{j},Ts{j});
		end		
		Ts = {};
	end
end
end

