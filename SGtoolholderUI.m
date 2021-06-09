function SG = SGtoolholderUI()
clf;
inputStr = '';
while ~ismember(inputStr, {'IMG','STL','BSH'})
	inputStr = input('Choose the way to input your Object! IMG/STL/BSH ','s');
end
height=0;
width = 0;
depth = 0;
radius = 0;
length =0;
switch inputStr
	case 'IMG'
		imagePath = imgetfile();
		imageAct = imread(imagePath);
		sizex =  size(imageAct,1);
		factor = 1000/sizex;
		imageAct= imresize(imageAct, factor);
		CPL = traceImage(imageAct);
		
		while height<=0
			height = input('Height of your object ');
		end
		SG_object = SGofCPLz(CPL,height);
		SG_object = SGtransrelSG(SG_object,'','rotx',-pi/2,'rotz',pi/2);
	case 'STL'
		SG_object = SGreadSTL();
	case 'BSH'
		inputStr = input('What basic shape is your object box/cylinder/sphere ','s');
		switch inputStr
			case 'box'
				while width <=0
					width = input('Width of your object ');
				end
				while depth <=0
					depth = input('Depth of your object ');
				end
				while height <=0
					height = input('Height of your object ');
				end
				SG_object = SGbox([width depth height]);
			case 'cylinder'
				while radius <=0
					radius = input('Radius of your object ');
				end
				while length <=0
					length = input('Length of your object ');
				end
				SG_object = SGcylinder(radius,length);
			case 'sphere'
				while radius <=0
					radius = input('Radius of your object ');
				end
				SG_object = SGsphere(radius);
			otherwise
				error("WRONG INPUT");
		end
		
	otherwise		
		error("WRONG INPUT");
end
close;
SG_object = SGtrans0(SG_object);


inputStr = '';
while ~ismember(inputStr, {'Tool','Com','Mech','Para'})
	inputStr = input('Choose the gripper type! Tool/Com/Mech/Para ','s');
end
switch inputStr
	case 'Tool'
		size = 50;
		SG_gripper_sil = SGLCLtoolholder(size);
		SG_gripper_sil = SGtransrelSG(SG_gripper_sil,'','rotz',pi/2,'rotx',-pi/2,'centery');
		SGplot(SG_gripper_sil);
		SGplot(SG_object);
		input_pm = input('+ for bigger - for smaller x for good ','s');		
		while input_pm~= 'x'
			switch input_pm
				case '+'
					size=size+1;
				case '-'
					size=size-1;
			end
			SG_gripper_sil = SGLCLtoolholder(size);
			SG_gripper_sil = SGtransrelSG(SG_gripper_sil,'','rotz',pi/2,'rotx',-pi/2,'centery');
			SGplot(SG_gripper_sil);
			SGplot(SG_object);
			input_pm = input('+ for bigger - for smaller x for good ','s');
		end
	case 'Com'
		SG_gripper_sil = SGcompConfig(SG_object);
	case 'Mech'
		SG_gripper_sil = SGmechGripper();
		SG_gripper_sil = SGtransrelSG(SG_gripper_sil,'','rotx',pi,'rotz',pi/2,'alignbottom');
	case 'Para'
		SG_gripper_sil = SGparrallelGripper();
		SG_gripper_sil = SGtransrelSG(SG_gripper_sil,'','rotz',pi/2,'aligntop');
	otherwise 
		error('SOMETHING WENT HOOOOORRRRIIIIBLLLE WRONG');		
end
clf;
SGplot(SG_gripper_sil);	
	 
object = SGplot(SG_object);
object.Tag = 'object';
view(90,0);

while true
	[~,~,in]=ginput(1);
	switch in
		case 31
			SG_object = SGtrans(SG_object,[0 0 -1]);
		case 30
			SG_object = SGtrans(SG_object,[0 0 +1]);
		case 28
			SG_object = SGtrans(SG_object,[0 -1 0]);
		case 29
			SG_object = SGtrans(SG_object,[0 1 0]);	
		case 119 %w
			SG_object.VL = SG_object.VL*rotx(deg2rad(10));
		case 115 %a
			SG_object.VL = SG_object.VL*rotx(deg2rad(-10));
		case 101 
			break;
		otherwise		
	end
	patch = findall(gcf, 'Tag', 'object');
	delete(patch);
	object = SGplot(SG_object);
	object.Tag = 'object';
	
end

switch inputStr
	case 'Tool'
		SG_gripper_sil = SGbool3('-',SG_gripper_sil,SG_object);
	case 'Com'
	case 'Mech'
		SG_gripper_sil = SGmechGripper('SG_object',SGtransrelSG(SGmirror(SG_object,'xy'),'','transz',160,'rotz',pi/2));
	case 'Para'
	otherwise 
		error('SOMETHING WENT HOOOOORRRRIIIIBLLLE WRONG');		
end

		SGwriteSTL(SG_gripper_sil);


end


function [SG] = SGcompConfig(SG_object)
SG = SGgripper();
SG = SGtrans(SG,[0 0 -80]);
SGplot(SG_object);
SG_grip = SGplot(SG);
SG_grip.Tag = 'gripper';

gripper_numbers = 0;
while ~ismember(gripper_numbers, [1,2,3,4])
	gripper_numbers = input('How many fingers should your gripper have? 1-4 ');
end
SG = SGgripper('gripper_numbers',gripper_numbers);
SG = SGtrans(SG,[0 0 -80]);
patch = findall(gcf, 'Tag', 'gripper');
delete(patch);
SG_grip = SGplot(SG);

grip_radius = 0;
while grip_radius <= 0
	grip_radius = input('What should the radius of your gripper be?');
end
SG = SGgripper('gripper_numbers',gripper_numbers);
SG = SGtrans(SG,[0 0 -80]);
patch = findall(gcf, 'Tag', 'gripper','Radius',grip_radius);
delete(patch);
SG_grip = SGplot(SG);

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

num_points = size(CPL_out,1);
line_dis =0;
start_point = 0;
for i=0:num_points-1
	line_dis_temp = pdist2(CPL_out(i+1,:),CPL_out(mod(i+1,num_points)+1,:));
	if line_dis_temp > line_dis
		start_point = i+1;
		line_dis =line_dis_temp;
	end	
end

CPLplot([CPL_out(start_point,:);CPL_out(mod(start_point,num_points)+1,:)],'b',5);
CPLplot(CPL_out);
hi = imshow(i_file);
uistack(hi,'bottom');
axis equal;
view(2);


realLength = input('What is the real Length of the blue line ');
virtLength = line_dis;

factor = realLength/virtLength;
CPL_out = CPL_out*factor;

	function [p,in] = inputFIG()
		p = [];
		[x,y,in]=ginput(1);
		if in == 3; return; end
		p = [x y];
	end
end



