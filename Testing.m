
clf;
CPLplot(CPL);
CPL_new = CPL;
axis equal;
cp_s = [];
added_points = 0;
dev = 0.5;


lengthCPL = size(CPL,1);
CPL_length = [CPL;CPL(1,:)];
distances = diag(pdist2(CPL_length,CPL_length),1);
[~,start] = find(max(distances));
start = mod(start,lengthCPL)+1;
offS = 0;

for i=start:start+lengthCPL
	
	v_a = CPL(mod(i-2-offS,lengthCPL)+1,:)-CPL(mod(i-1-offS,lengthCPL)+1,:);
	v_b = CPL(mod(i+1,lengthCPL)+1,:)-CPL(mod(i+2,lengthCPL)+1,:);
	
	cross_point = cross2edges2(CPL(mod(i-2-offS,lengthCPL)+1,:),v_a,CPL(mod(i+1,lengthCPL)+1,:),v_b);
	dis1 = pdist2(cross_point,CPL(mod(i-2-offS,lengthCPL)+1,:));
	dis2 = pdist2(cross_point,CPL(mod(i+1,lengthCPL)+1,:));
	
	if  mod(i+1,lengthCPL)+1 < mod(i-1,lengthCPL)+1
		if mod(i+1,lengthCPL)+1 == 1			
			CPL_new = [CPL(mod(i+1,lengthCPL)+1:mod(i-1-offS,lengthCPL)+1,:);cross_point];
		else
			CPL_new = [cross_point;CPL(mod(i+1,lengthCPL)+1:mod(i-1-offS,lengthCPL)+1,:)];
		end	
	else
		CPL_new = [CPL(1:mod(i-1-offS,lengthCPL)+1,:);cross_point;CPL(mod(i+1,lengthCPL)+1:end,:)];
	end	
	
	CPL_new = PLroundcorners(CPL_new,mod(i-offS,lengthCPL)+1,min(dis1,dis2));
	
	[row, col] = find(isnan(CPL_new));
	if sum(row)>0 
		continue;
	end
	
	if mod(i+2,lengthCPL)+1 < mod(i-2-offS,lengthCPL)+1
		CPL_before = [CPL(mod(i-2-offS,lengthCPL)+1:end,:);CPL(1:mod(i+2,lengthCPL)+1,:)];
	else
		CPL_before = CPL(mod(i-2-offS,lengthCPL)+1:mod(i+2,lengthCPL)+1,:);
	end
	
	CPL_before = CPLaddauxpoints(CPL_before,0.5);
	
	[~,idx]= ismember(CPL_before,CPL(mod(i+2,lengthCPL)+1,:),'rows');	
	idx = find(idx);
	CPL_before = CPL_before(1:idx,:);
	close_enough = 1;
	CPLplot(CPL_new);
	CPL_new_more = CPLaddauxpoints(CPL_new,0.5);
	offS = offS+1;
	for j=1:size(CPL_before,1)
		min_dis = min(pdist2(CPL_before(j,:),CPL_new_more));
		if min_dis>dev
			close_enough=0;
			offS = 0;
			break;
		end
	end	
	
	if ~isempty(cross_point) && close_enough 
		CPLplot(CPL_new,'b');
	end
end




