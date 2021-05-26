function SG = SGtoolholderUI()
clf;
inputStr = input('Choose the way to input your Object! IMG/STL/BSH ','s');

switch inputStr
	case 'IMG'
		imagePath = imgetfile();
		imageAct = imread(imagePath);
		sizex =  size(imageAct,1);
		factor = 1000/sizex;
		imageAct= imresize(imageAct, factor);
		CPL = traceImage(imageAct);
		height = input('Height of your object ');
		SG_object = SGofCPLz(CPL,height);
	case 'STL'
		SG_object = SGreadSTL();
	case 'BSH'
		inputStr = input('What basic shape is your object square/circle ','s');
		switch inputStr
			case 'square'				
				width = input('Width of your object ');
				depth = input('Depth of your object ');
				height = input('Height of your object ');
				SG_object = SGbox([width depth height]);
			case 'circle'
				radius = input('Radius of your object ');
				length = input('Length of your object ');
				SG_object = SGcylinder(radius,length);
			otherwise
				error("WRONG INPUT");
		end
		
	otherwise		
		error("WRONG INPUT");
end
clf;
SG_object = SGtrans0(SG_object);
SGplot(SG_object);

% SG_object = SGtrans0(SG_object);
% [limx,limy,limz] =sizeVL(SG_object);
% 
% SG_adapter = SGtrans0(SGofCPLz(PLsquare(30),20));
% 
% SGplot(SG_object);
% SGplot(SG_adapter);
% xlim([-limx limx]);
% ylim([-limy limy]);
% zlim([-limz limz]);
% view(90,0);
% 
% while true
% 	[~,~,in]=ginput(1);
% 	switch in
% 		case 31
% 			SG_object = SGtrans(SG_object,[0 0 -1]);
% 		case 30
% 			SG_object = SGtrans(SG_object,[0 0 +1]);
% 		case 28
% 			SG_object = SGtrans(SG_object,[0 -1 0]);
% 		case 29
% 			SG_object = SGtrans(SG_object,[0 1 0]);
% 		case 101
% 			break;
% 		otherwise		
% 	end	
% 	clf;
% 	SGplot(SG_object);
% 	SGplot(SG_adapter);
% 	xlim([-limx limx]);
% 	ylim([-limy limy]);
% 	zlim([-limz limz]);
% 	view(90,0);
% end
% 
% CPL_outside = PLsquare(20,30);
% SG_outside = SGofCPLzdelaunayGrid(CPL_outside,1,1,0.5);
% idx = find(SG_outside.VL(:,3)==1);
% SG_outside = SGtransrelSG(SG_outside,SG_adapter,'roty',pi/2,'aligntop','alignright');
% 
% for i=1:size(idx,1)
% 	CPL_temp = CPLofSGslice(SG_object,SG_outside.VL(idx(i),3));
% 	CPL_temp = CPLaddauxpoints(CPL_temp,0.5);
% 	
% 	
% 	
% 	x=1;
% end
% SGplot(SG_outside);

end

function [CPL_out] = traceImage(i_file)

CPL_out = [];
CPLplot([0 0]);
imshow(i_file);
axis equal;
view(2);

while true
	[p,in] = inputFIG();
	if in == 3 break; end;
	
	
	if ~isempty(CPL_out)
			x_diff = CPL_out(:,1)-p(1);
			y_diff = CPL_out(:,2)-p(2);
			
			idx_x = find(abs(x_diff)<10);
			idx_y = find(abs(y_diff)<10);
			
			if ~isempty(idx_x)
				p(1) = CPL_out(idx_x(1),1);
			end
			if ~isempty(idx_y)
				p(2) = CPL_out(idx_y(1),2);
			end
	end
	
	CPL_out = [CPL_out;p];
	
	CPLplot(CPL_out);
	hi = imshow(i_file);
	uistack(hi,'down');
	axis equal;
	view(2);
end

y_values = nchoose2(CPL_out(:,2));
y_value_diff = abs(y_values(:,1)-y_values(:,2)); 
min_idx = find(min(y_value_diff));

idx_1 = find(CPL_out(:,2) == y_values(min_idx,1));
idx_2 = find(CPL_out(:,2) == y_values(min_idx,2));

CPLplot([CPL_out(idx_1,:);CPL_out(idx_2,:)],'b',5);
CPLplot(CPL_out);
hi = imshow(i_file);
uistack(hi,'bottom');
axis equal;
view(2);


realLength = input('What is the real Length of the blue line ');
virtLength = pdist2(CPL_out(idx_1,:),CPL_out(idx_2,:));

factor = realLength/virtLength;
CPL_out = CPL_out*factor;

	function [p,in] = inputFIG()
		p = [];
		[x,y,in]=ginput(1);
		if in == 3; return; end
		p = [x y];
	end
end



