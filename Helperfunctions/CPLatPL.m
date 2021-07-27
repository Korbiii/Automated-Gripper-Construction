% CPLatPL(CPL_in,PL) - returns CPL of CPLs
% 
%	Function places CPL at given positions from a Point List.
% 
% 
%   [CPL] = CPLatPL(CPL_in,PL)
%   === INPUT PARAMETERS ===
%   CPL_in :	CPL that should be placed at points of PL
%	PL:			PL that contains positions for CPL
%   === OUTPUT RESULTS ======
%   CPL :		Final CPL
function [CPL] = CPLatPL(CPL_in,PL)

CPL = [];
for i=1:size(PL,1)
	CPL = CPLbool('+',CPL,PLtrans(CPL_in,PL(i,:)));
end

end