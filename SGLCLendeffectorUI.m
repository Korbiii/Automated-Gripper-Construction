%%   SG = LCLRobot()
%    === INPUT PARAMETERS ===
%    === OUTPUT RESULTS ======
%    
function SG = SGLCLendeffectorUI()
close all

options = {'img','stl','bsh','cpl','sg','rmx'};
options_desc = {'Trace image of object','Load STL of object','Basic Shape of object','Load CPL of object','Load SG of object','Remix STL with adapter'};
inputStr = options{listdlg('ListString',options_desc,'SelectionMode','single','PromptString','Choose what you want to do')};

inputStr = lower(inputStr);
switch inputStr
	case 'img'
		imagePath = imgetfile();
		imageAct = imread(imagePath);
		sizex =  size(imageAct,1);
		factor = 1000/sizex;
		imageAct= imresize(imageAct, factor);
		CPL = traceImage(imageAct);	
		height = str2double(inputdlg('Height of your object?'));
		SG_object = SGofCPLz(CPL,height);
		SG_object = SGtransrelSG(SG_object,'','rotx',-pi/2,'rotz',pi/2);
		SG = placeObject(SG_object);	
	case 'stl'
		SG_object = SGreadSTL();
		SG = placeObject(SG_object);
	case 'sg'
		[baseName, folder] = uigetfile();
		SG_object = load(fullfile(folder, baseName));
		SG = placeObject(SG_object);	
	case 'cpl'		
		[baseName, folder] = uigetfile();
		CPL_object = load(fullfile(folder, baseName));
		option = questdlg('Extrusion or Rotatiuon','CPL2SG','Extrusion','Rotation','');
		switch option			
			case 'Extrusion'				
				height = str2double(inputdlg('Height of your object?'));
				SG_object = SGofCPLz(CPL_object,height);
			case 'Rotation'
				SG_object = SGofCPLrot(CPL_object,'',false);
		end		
		SG = placeObject(SG_object);
	case 'bsh'
		options = {'Box','Cylinder','Sphere'};
		inputStr = options{listdlg('ListString',options,'SelectionMode','single','PromptString','Choose what your object resembles')};
		switch inputStr
			case options(1)
				prompt = {'Width of Box:','Depth of Box:','Height of Box:'};
				output_dlg = str2double(inputdlg(prompt));				
				answer = questdlg('Do you want grip geometry?','Added Geometry','Yes','No','No');
				switch answer 
					case 'No'	
						SG_object = SGbox([output_dlg(1) output_dlg(2) output_dlg(3)]);
					case 'Yes'	
						CPL_square = CPLaddauxpoints(PLsquare(output_dlg(1)+0.9,output_dlg(2)+1),5);
						CPL_remove = CPLatPL(PLcircle(0.5),CPL_square);
						CPL_square = CPLbool('-',CPL_square,CPL_remove);
						SG_object = SGofCPLz(CPL_square,output_dlg(3));
				end				
			case options(2)
				prompt = {'Radius of Cylinder:','Height of Cylinder:'};
				output_dlg = str2double(inputdlg(prompt));
				
				answer = questdlg('Do you want grip geometry?','Added Geometry','Yes','No','No');
				switch answer 
					case 'No'	
						SG_object = SGcylinder(output_dlg(1),output_dlg(2));
					case 'Yes'	
						CPL_circle = CPLaddauxpoints(PLcircle(output_dlg(1)+0.9),5);
						CPL_remove = CPLatPL(PLcircle(0.5),CPL_circle);
						CPL_circle = CPLbool('-',CPL_circle,CPL_remove);
						SG_object = SGofCPLz(CPL_circle,output_dlg(2));
				end			
			case options(3)
				prompt = {'Radius of Sphere:'};
				output_dlg = str2double(inputdlg(prompt));					
				SG_object = SGsphere(output_dlg(1));
	
			otherwise
				error("WRONG INPUT");
		end
		SG = placeObject(SG_object);	
	case 'rmx'
		SG = remixAdapter();
	otherwise
		error("WRONG INPUT");
end
if nargout == 0
	clf;
	SGplot(SG);
	SGwriteSTL(SG);
end
view(3)


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

function [SG] = remixAdapter()
view_switch = 0;
t_anno = 1;
factor = 1;
inputs = {'transy',1*factor,29,28;'transz',1*factor,30,31;'roty',pi/2,115,119;'rotx',pi/2,113,101;'rotz',pi/2,97,100};
[SG_adapter,CPL_adapter] = SGconnAdaptersLCL('adapter_type','rotLock','cable',0);
SG_adapter = SGtransrelSG(SG_adapter,'','aligntop',-10);

SG_remix = SGreadSTL();
SG_remix = SGtransrelSG(SG_remix,'','center','alignbottom');

SGfigure;
f_anno=SGfigureannotation("v: Change view" + newline + "Arrow Keys: Move Part" + newline + "WA/SD/QE : Rotate Part"+ newline + "x : Confirm Position"+ newline + "x : Toggle overlay",'y',10,0.8);

[~,sy,~] = sizeVL(SG_adapter.VL);
VLplot([0 -sy/2 0;0 sy/2 0]);

P_adapter = SGplot(SG_adapter);
P_adapter.Tag = 'adapter';

P_remix = SGplot(SG_remix);
P_remix.Tag = 'remix';

view(90,0);

while true
	[~,~,in]=ginput(1);
	if in == 120
		break;
	end	
	for k=1:size(inputs,1)
		if in == inputs{k,3}
			SG_remix = SGtransrelSG(SG_remix,SG_remix,inputs{k,1},inputs{k,2});
		elseif in == inputs{k,4}
			SG_remix = SGtransrelSG(SG_remix,SG_remix,inputs{k,1},-inputs{k,2});
		end
	end
	if in == 118
		if view_switch			
			inputs = {'transy',1*factor,29,28;'transz',1*factor,30,31;'roty',pi/2,115,119;'rotx',pi/2,113,101;'rotz',pi/2,97,100};			
			view(90,0);
		else	
			inputs = {'transx',1*factor,29,28;'transy',1*factor,30,31;'roty',pi/2,115,119;'rotx',pi/2,113,101;'rotz',pi/2,97,100};
			view(0,90);
		end
		view_switch = ~view_switch;
	end		
	if in == 116
		if t_anno == 1
			delete(f_anno);
		else			
			f_anno=SGfigureannotation("v: Change view" + newline + "Arrow Keys: Move Part" + newline + "WA/SD/QE : Rotate Part"+ newline + "x : Confirm Position",'y',10,0.8);
		end
		t_anno = ~t_anno;
	end
		
	patch_tmp = findall(gcf, 'Tag', 'adapter');
	delete(patch_tmp);
	P_adapter = SGplot(SG_adapter);
	P_adapter.Tag = 'adapter';
	
	patch_tmp = findall(gcf, 'Tag', 'remix');
	delete(patch_tmp);
	P_remix = SGplot(SG_remix);
	P_remix.Tag = 'remix';	
end

if min(SG_remix.VL(:,3)) < 0
	CPL_remix = CPLconvexhull(CPLofSGslice2(SG_remix,0));
	[~,SG_remix] = SGcut(SG_remix,0);	
else
	height = min(SG_remix.VL(:,3))+0.1;
	CPL_remix = CPLconvexhull(CPLofSGslice2(SG_remix,height));
end
	SG_connection = SGof2CPLsz(CPLaddauxpoints(CPL_adapter,0.5),CPLaddauxpoints(CPL_remix,0.5),10,'center');
	SG = SGstack('z',SG_adapter,SG_connection,SG_remix);

end

function [SG_final] = placeObject(SG_object)
factor = 1;
H_Base_frame = [rotz(90) [0;0;0]; 0 0 0 1];
SG_base_box = SGbox(0.1);
SG_base_box = SGTset(SG_base_box,'BaseBox',H_Base_frame);

close;

SG_object = SGtrans0(SG_object);
H_Object = [rotx(0) [0;0;0]; 0 0 0 1];
SG_object = SGTset(SG_object,'Object',H_Object);

[~,~,~,~,~,gripper_options] = SGendeffectors('');
inputStr = gripper_options{listdlg('ListString',gripper_options,'SelectionMode','single','PromptString','Choose what your object resembles')};

[SG_gripper_sil,SG_grippers,~,inputsO,inputsG] = SGendeffectors(inputStr);
SG_gripper_sil = SGtransrelSG(SG_gripper_sil,SG_base_box,'alignT',{'GripperT','BaseBox'});
SG_grippers = SGtransrelSG(SG_grippers,SG_gripper_sil,'alignTz',{'GripperT','GripperT'});
SG_grippers.alpha = 0.9;
close all;
Help_string = "V: Change view" + newline + "Object:   ";
for h = 1:size(inputsO,1)	
	key1 = upper(getKeyCharFromASCII(inputsO{h,3}));
	key2 = upper(getKeyCharFromASCII(inputsO{h,4}));
	Help_string = Help_string + inputsO{h,1} + ": " + key1+"\"+ key2 + "      ";
end
Help_string = Help_string + "Endeffector:   ";
for h = 1:size(inputsG,1)
	key1 = upper(getKeyCharFromASCII(inputsG{h,4}));
	key2 = upper(getKeyCharFromASCII(inputsG{h,5}));
	Help_string = Help_string + inputsG{h,1} + ": " + key1+"\"+key2 + "      ";
end
Help_string = Help_string + newline + "X : Confirm Position";
Help_string = Help_string + newline + "T : Toggle Annotation";
Help_string = Help_string + newline + "F/G : Increase Decrease Factor";
SGfigure;
f_anno = SGfigureannotation(Help_string,'y',10,0.75);

gripper = SGplot(SG_grippers);
gripper.Tag = 'gripper';

gripper_base = SGplot(SG_gripper_sil);
gripper_base.Tag = 'base';

SG_object = SGtransrelSG(SG_object,SG_grippers,'alignT',{'Object','ObjectPos'});
object = SGplot(SG_object);
object.Tag = 'object';

view(90,0);
view_switch = 0;
t_anno = 1;
while true
	[~,~,in]=ginput(1);
	if in == 120
		SG_object = SGTset(SG_object,'ObjectPos',SGTget(SG_grippers,'ObjectPos'));
		break;
	end	
	for k=1:size(inputsO,1)
		if in == inputsO{k,3}
			SG_object = SGtransrelSG(SG_object,SG_object,inputsO{k,1},inputsO{k,2}*factor);
		elseif in == inputsO{k,4}
			SG_object = SGtransrelSG(SG_object,SG_object,inputsO{k,1},-inputsO{k,2}*factor);
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
	if in == 118; view_switch = ~view_switch; end	
	if in == 102; factor = factor*2; end
	if in == 103; factor = factor/2; end
	if in == 116
		if t_anno == 1			
			delete(f_anno);
		else			
			f_anno = SGfigureannotation(Help_string,'y',10,0.75);
		end		
		t_anno = ~t_anno;
	end		
	
	patch = findall(gcf, 'Tag', 'gripper');
	delete(patch);	
	SG_grippers.alpha = 0.9;
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
	if view_switch
		view(3);
	else
		view(90,0);
	end
	
end
inputsG(end+1,:) = {'output',1,'','',''};
[~,~,SG_final] = SGendeffectors(inputStr,inputsG,SG_object);
end

function [key_char] = getKeyCharFromASCII(value)
	if ~ismember(value,[29,28,30,31])
		key_char = char(value);
	else
		switch value
			case 29
				key_char = 'Right';
			case 28				
				key_char = 'Left';
			case 30
				key_char = 'Up';
			case 31
				key_char = 'Down';
		end
	end

end

function [CPL_out] = traceImage(i_file)

CPL_out = [];
CPLplot([0 0]);
imshow(i_file);
axis equal;
view(2);

while true
	[p,in] = inputFIG();
	if in == 3; break; end
	if ~isempty(CPL_out)
			x_diff = CPL_out(:,1)-p(1);
			y_diff = CPL_out(:,2)-p(2);
			
			idx_x = find(abs(x_diff)<2);
			idx_y = find(abs(y_diff)<2);
			
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


realLength = str2double(inputdlg('What is the real Length of the blue line '));
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



