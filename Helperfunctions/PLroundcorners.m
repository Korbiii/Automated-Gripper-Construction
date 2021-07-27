% PLroundcorners(PL,indices,varargin) - returns a PL after rounding
%	specified corners
% 
%	This function automates the process of rounding off corners, that has
%	to be done manually normally.
% 
%  [PL] = PLroundcorners(PL,indices,varargin)
%	=== INPUT PARAMETERS ===
%	PL:				Contour of PL you to search throguh
%	indices:		Position of corner that should be rounded
%	radius:			Radius or individual radii of the cornerarc
%	connection:		Gives the possibility to create different orientated arcs
%	round:			default = 1; If 0 chamfers instead of arcs.
%	=== OUTPUT RESULTS ======
%	PL:				PL after rounding off specified corners 
function [PL] = PLroundcorners(PL,corner_numbers,varargin)
radius = ones(1,size(corner_numbers,2)); if nargin>=3 && ~isempty(varargin{1}); radius=varargin{1}; end
connection = []; if nargin >=4 && ~isempty(varargin{2}); connection = varargin{2}; end
round = 1; if nargin >=5 && ~isempty(varargin{3}); round = varargin{3}; end
if(size(radius,1)==1)
    radius = repmat(radius,1,size(corner_numbers,2));
end
try
    PL_save = CPLselectinout(PL,1);
catch
    PL_save =[];
end
PL = CPLselectinout(PL,0);
corners = {};
for i=1:size(corner_numbers,2)
    if corner_numbers(i) == 1
        v1 = PL(corner_numbers(i)+1,:)-PL(corner_numbers(i),:);
        v1 = v1/norm(v1);
        v2 = PL(size(PL,1),:)-PL(corner_numbers(i),:);
        v2 = v2/norm(v2);
    elseif corner_numbers(i) == size(PL,1)
        v1 = PL(1,:)-PL(corner_numbers(i),:);
        v1 = v1/norm(v1);
        v2 = PL(corner_numbers(i)-1,:)-PL(corner_numbers(i),:);
        v2 = v2/norm(v2);
    else
        v1 = PL(corner_numbers(i)+1,:)-PL(corner_numbers(i),:);
        v1 = v1/norm(v1);
        v2 = PL(corner_numbers(i)-1,:)-PL(corner_numbers(i),:);
        v2 = v2/norm(v2);
	end	
    following_point = PL(corner_numbers(i),:)+(v1*abs(radius(i)));
    trailing_point = PL(corner_numbers(i),:)+(v2*abs(radius(i)));
	if round
		corners{end+1} = PLcircarc2([trailing_point;PL(corner_numbers(i),:);following_point]);
	else
		corners{end+1} = [trailing_point;following_point];
	end
    [is_member,pos] = ismember(corner_numbers(i),abs(connection));
    if is_member
        if connection(pos) <0
            corners{end} = PLmirror0(corners{end},[trailing_point;PL(corner_numbers(i),:)],1);
        else
            corners{end} = PLmirror0(corners{end},[following_point;PL(corner_numbers(i),:)],1);
        end
    end
end
for i=1:size(corner_numbers,2)
    if corner_numbers(i) == 1
        PL = [corners{i};PL(corner_numbers(i)+1:end,:)];
    elseif corner_numbers(i)==size(PL,1)
        PL = [PL(1:corner_numbers(i)-1,:);corners{i}];
    else
        PL = [PL(1:corner_numbers(i)-1,:);corners{i};PL(corner_numbers(i)+1:end,:)];
    end
    corner_numbers = corner_numbers + size(corners{i},1)-1;
    
end
if ~isempty(PL_save)
    PL = [PL;NaN NaN;PL_save];
end
end