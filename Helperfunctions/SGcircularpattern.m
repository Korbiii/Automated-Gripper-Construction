% SGcircularpattern(SG,n,angle) - returns a circular pattern of a SG
% 
%	This function is just there to build a simple circular patter of an SG
%	given an angle and number of copies.
% 
% 	[SG] = SGcircularpattern(SG,n,angle)
%	=== INPUT PARAMETERS ===
%	SG:     SG for circular pattern
%	n:      number of copies
%   angle:  angle between copies
%	=== OUTPUT RESULTS ======
%   SG:     Combined SG of copies
function [SG] = SGcircularpattern(SG,n,angle)
for i=1:n
   SG = SGcat(SG,SGtransR(SG,rot(0,0,angle)));
end
end