%%   [S_coords]=CPLui2([snap_value,CPL_ref,SG_ref,phi_SG])
%	=== INPUT PARAMETERS ===
%	snap_value:	Value that clicked points snap to; 0 for no snapping
%	CPL_ref:	Reference CPL to build around
%	SG_ref:		Referemce SG to build around
%	phi_SG:     Angle to rotate SG
%   x_ref:      Referenceline in x-Direction
%   y_ref:      Referenceline in y_Direction
%   snap_deg    Degree to what lines snap to; default = 45°;-1 = free
%	=== OUTPUT RESULTS ======
%	S_coords:	String of coordinate
%   CPL:        CPL of contour
function [CPL] = CPLfromimgTrace(varargin)
I = imread('C:\Users\Korbi\Desktop\testt.jpg');
sizex =  size(I,1);
factor = 100/sizex;
I = imresize(I, factor);

CPL = [];
limits = [-30 30 -10 10];
refresh()

while true
    [p,in] = input();
    if in == 3 break; end;
	if in == 1
		p(1);
		if ~isempty(CPL)
			x_diff = CPL(:,1)-p(1);
			y_diff = CPL(:,2)-p(2);
			
			idx_x = find(abs(x_diff)<1);
			idx_y = find(abs(y_diff)<1);
			
			if ~isempty(idx_x)
				p(1) = CPL(idx_x(1),1);
			end
			if ~isempty(idx_y)
				p(2) = CPL(idx_y(1),2);
			end
		else
			
		end
		
		CPL = [CPL;p(1) p(2)];
    elseif in == 114
        CPL(end,:) =[];
    end
    refresh()
end
close(gcf);
CPLplot(CPL);
    function refresh()
        clf;
        CPLplot(CPL);
        view(0,-90);grid on;		
		h = image(I); 
		uistack(h,'bottom');
		axis equal
	end
	function [p,in] = input()
		p = [];		
		[x,y,in]=ginput(1);
		if in == 3; return; end
		p = [x y];
    end
end