function [SG] = SGfittoOutsideCPL(SG,CPL,dir)
% CPL = CPLaddauxpoints(CPL,0.25);

switch dir
	case 'x-'
	
	case 'x+'
		idx = ismembertol(SG.VL(:,1),max(SG.VL(:,1)));
		idx = find(idx == 1);
		relevant_points = CPL(CPL(:,1)>=max(SG.VL(:,1)),:);
		for i=1:size(idx)
			dif = abs(relevant_points(:,2)-SG.VL(idx(i),2));
			[min_idx,~] = find(dif == min(dif));
			SG.VL(idx(i),1) = relevant_points(min_idx,1);
		end
	case 'y-'
		idx = find(SG.VL(:,2) == min(SG.VL(:,2)));
		relevant_points = CPL(CPL(:,2)<min(SG.VL(:,2)),:);
		for i=1:size(idx)
			dif = abs(relevant_points(:,1)-SG.VL(i,1));
			[min_idx,~] = find(dif == min(dif));
			SG.VL(idx(i),2) = relevant_points(min_idx,2);
		end
	case 'y+'
		idx = ismembertol(SG.VL(:,2),max(SG.VL(:,2)));
		idx = find(idx == 1);
		relevant_points = CPL(CPL(:,2)>=max(SG.VL(:,2)),:);
		for i=1:size(idx)
			dif = abs(relevant_points(:,1)-SG.VL(idx(i),1));
			[min_idx,~] = find(dif == min(dif));
			SG.VL(idx(i),2) = relevant_points(min_idx,2);
		end
end


end