% SGslicebool(SG2slice,SG_object) - returns the result of a substraction
%	operation of two solids 
%
%	This function bools two solids through a number of slices.
% 
%	[SG] = SGslicebool(SG2slice,SG_object)
%	=== INPUT PARAMETERS ===
%	SG2slice:			SG that the object will be cut out of
%	SG_object:			SG of object that is used to cut out solid
%	=== OUTPUT RESULTS ======
%	SG:					Resulting SG from bool operation
function [SG] = SGslicebool(SG2slice,SG_object)
res = 0.15;
SG = [];		
start_H_2slice = min(SG2slice.VL(:,3))+0.01;
start_H_Object =min(SG_object.VL(:,3))+0.01;
end_H_Object = max(SG_object.VL(:,3))-0.01;
end_H_2slice = max(SG2slice.VL(:,3))-0.01;

if start_H_2slice<start_H_Object
	SG = SGcut(SG2slice,start_H_Object);
	height = start_H_Object;
else	
	height = start_H_2slice;
end

if end_H_Object<end_H_2slice
	end_Height =end_H_Object;
else
	end_Height = end_H_2slice;
end
h = waitbar(0,'Please wait...');
steps = (end_Height-height)/res;
step =1;
while height < end_Height
	CPL_slice = CPLofSGslice2(SG2slice,height);
	CPL_slice_object =  CPLofSGslice2(SG_object,height);	
	CPL_new = CPLbool('-',CPL_slice,CPL_slice_object);
	if isempty(CPL_new)
		SGslice = SGbox([0.001 0.001 res]);
	else
		CPL_ps = polyshape(CPL_new);
		SGslice = SGofCPLz(CPL_ps.Vertices,res);
	end
	height =height+res;
	if isempty(SG)
		SG = SGtrans(SGslice,[0 0 height]);
	else
		SG = SGcat(SG,SGontop(SGslice,SG,-0.000001));
	end
	step = step+1;
	waitbar(step / steps);
end
close(h);
if end_H_Object < end_H_2slice
	[~,SG_top] = SGcut(SG2slice,end_H_Object);
	SG = SGcat(SG,SGontop(SG_top,SG,-0.000001));
end
end
