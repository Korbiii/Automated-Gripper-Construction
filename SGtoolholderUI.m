function SG = SGtoolholderUI()
clf;
inputStr = '';
while ~ismember(inputStr, {'IMG','STL','BSH','CPL','SG'})
	inputStr = input('Choose the way to input your Object! IMG/STL/BSH/SG ','s');
end
[height,width,depth,radius,length]=deal(0);

H_Base_frame = [rotz(90) [0;0;0]; 0 0 0 1];
SG_base_box = SGbox(0.1);
SG_base_box = SGTset(SG_base_box,'BaseBox',H_Base_frame);

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
	case 'SG'
		[baseName, folder] = uigetfile();
		SG_object = load(fullfile(folder, baseName));
	case 'CPL'		
		inputStr = '';
		[baseName, folder] = uigetfile();
		CPL_object = load(fullfile(folder, baseName));
		while ~ismember(inputStr, {'S','E','s','e'})
			inputStr = input('Extruson (e) or Shaft (s)','s');
		end
		switch inputStr
			case {'e','E'}
				while height <=0
					height = input('Height of your object ');
				end
				SG_object = SGofCPLz(CPL_object,height);
			case {'s','S'}
				SG_object = SGofCPLrot(CPL_object,'',false);
		end		
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
H_Object = [rotx(0) [0;0;0]; 0 0 0 1];
SG_object = SGTset(SG_object,'Object',H_Object);

inputStr = '';
[~,~,~,~,~,gripper_options] = SGendeffectors('');
while ~ismember(inputStr, gripper_options)
	disp('Gripper options are: ');
	disp(string(gripper_options));
	inputStr = input('Choose the gripper type!' ,'s');
end

[SG_gripper_sil,SG_grippers,~,inputsO,inputsG] = SGendeffectors(inputStr);
SG_gripper_sil = SGtransrelSG(SG_gripper_sil,SG_base_box,'alignT',{'GripperT','BaseBox'});
SG_grippers = SGtransrelSG(SG_grippers,SG_gripper_sil,'alignTz',{'GripperT','GripperT'});

clf;
gripper = SGplot(SG_grippers);
gripper.Tag = 'gripper';

gripper_base = SGplot(SG_gripper_sil);
gripper_base.Tag = 'base';

SG_object = SGtransrelSG(SG_object,SG_grippers,'alignT',{'Object','ObjectPos'});
object = SGplot(SG_object);
object.Tag = 'object';
view(90,0);


while true
	[~,~,in]=ginput(1);
	if in == 120
		SG_object = SGTset(SG_object,'ObjectPos',SGTget(SG_grippers,'ObjectPos'));
		break;
	end
	
	for k=1:size(inputsO,1)
		if in == inputsO{k,3}
			 SG_object = SGtransrelSG(SG_object,SG_object,inputsO{k,1},inputsO{k,2});
		elseif in == inputsO{k,4} 
			 SG_object = SGtransrelSG(SG_object,SG_object,inputsO{k,1},-inputsO{k,2});
		end			
	end
	
	for k=1:size(inputsG,1)
		if in == inputsG{k,4}
			 inputsG{k,2}=inputsG{k,2}+inputsG{k,3};
			 [SG_gripper_sil,SG_grippers] = SGendeffectors(inputStr,inputsG);
		elseif in == inputsG{k,5} 			
			 inputsG{k,2}=inputsG{k,2}-inputsG{k,3};
			 [SG_gripper_sil,SG_grippers] = SGendeffectors(inputStr,inputsG);
		end			
		SG_gripper_sil = SGtransrelSG(SG_gripper_sil,SG_base_box,'alignT',{'GripperT','BaseBox'});
		SG_grippers = SGtransrelSG(SG_grippers,SG_gripper_sil,'alignTz',{'GripperT','GripperT'});
	end
	
	patch = findall(gcf, 'Tag', 'gripper');
	delete(patch);
	gripper = SGplot(SG_grippers);
	gripper.Tag = 'gripper';
	
	patch = findall(gcf, 'Tag', 'base');
	delete(patch);
	gripper_base = SGplot(SG_gripper_sil);
	gripper_base.Tag = 'base';
	
	patch = findall(gcf, 'Tag', 'object');
	delete(patch);
	object = SGplot(SG_object);
	object.Tag = 'object';	
	
	view(90,0);
	
end

[~,~,SG_final] = SGendeffectors(inputStr,inputsG,SG_object);
clf;
SGplot(SG_final);
SGwriteSTL(SG_final);


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



