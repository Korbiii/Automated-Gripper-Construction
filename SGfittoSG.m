function [SG] = SGfittoSG(SG,SG2fit,dir,stop_value)
idx = ismembertol(SG.VL(:,3),min(SG.VL(:,3)));
idx = find(idx == 1);
for i=1:size(idx,1)
	cp = crosspointVLFL2(SG2fit.VL,SG2fit.FL,SG.VL(idx(i),:),dir);
	if ~isempty(cp)
		SG.VL(idx(i),:) = cp(1,:);
		cp = [];
	else
		if dir(1)~=0			
			SG.VL(idx(i),1) = stop_value;
		elseif dir(2)~=0
			SG.VL(idx(i),2) = stop_value;
		else			
			SG.VL(idx(i),3) = stop_value;
		end
	end
end
end







