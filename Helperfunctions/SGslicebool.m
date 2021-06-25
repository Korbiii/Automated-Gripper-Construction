function [SG] = SGslicebool(SG2slice,SG_object)
res = 0.3;
SG = [];		
start_height = min(SG2slice.VL(:,3))+0.01;
start_H_Object =min(SG_object.VL(:,3))+0.01;
end_H_Object = max(SG_object.VL(:,3));
end_Height = max(SG2slice.VL(:,3))-0.01;

if start_height<start_H_Object
	SG = SGcut(SG2slice,start_height);
end

height = start_height;

while height < end_Height	
	CPL_slice = CPLofSGslice2(SG2slice,height);
	CPL_slice_object =  CPLofSGslice2(SG_object,height);	
	CPL_new = CPLbool('-',CPL_slice,CPL_slice_object);
	if isempty(CPL_new)
		SGslice = SGbox([0.001 0.001 res]);
	else
		SGslice = SGofCPLz(CPL_new,res);
	end
	height =height+res;
	if isempty(SG)
		SG = SGtrans(SGslice,[0 0 height]);
	else
		SG = SGcat(SG,SGontop(SGslice,SG));
	end
end
if end_H_Object < end_Height
	[~,SG_top] = SGcut(SG2slice,end_H_Object);
	SG = SGcat(SG,SGontop(SG_top,SG));
end

end
