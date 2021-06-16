function [SG] = SGcatF(varargin)
SG = varargin{1};
Ts = {};
for i=2:nargin
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

