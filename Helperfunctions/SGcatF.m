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

