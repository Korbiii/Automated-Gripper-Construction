% SGaxle([R,lengths]) - returns SGs of axles used for grippers
% 
%	Function creates an array of axle SGs based on a list of lenghts
% 
% 
%   SGs = SGaxle([R,lengths])
%   === INPUT PARAMETERS ===
%   R :			Radius of axles
%	lengths:	Array of Axls lengths
%   === OUTPUT RESULTS ======
%   SGs :		Array of all Axle SGs 
function SGs = SGaxle(varargin)

R = 3; if nargin >=1 && ~isempty(varargin{1}) R = varargin{1}; end
lengths = 25; if nargin >=2 && ~isempty(varargin{2}) lengths = varargin{2}; end
SGs = {};
for i=1:size(lengths,2)
	SG_back = SGof2CPLsz(PLcircle(R-0.1),PLcircle(R-0.2),5);
	SG_main_part = SGofCPLz(PLcircle(R-0.2),lengths(i)-6);
	SG_front = SGof2CPLsz(PLcircle(R-0.2),PLcircle(R-1),1);
	SG = SGstack('z',SG_back,SG_main_part,SG_front);
	SGs{end+1} = SG;
end
end