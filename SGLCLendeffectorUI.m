%%   SG = LCLRobot()
%    === INPUT PARAMETERS ===
%    === OUTPUT RESULTS ======
%    
function SG = SGLCLendeffectorUI()
close all

dlg_size = [400 100];
options = {'object','rmx'};
options_desc = {'I have an object/tool for which I need a gripper/toolholder','I have a STL I want to attach to my robot'};
inputStr = options{listdlg('ListString',options_desc,'SelectionMode','single','PromptString','What do you want to do?','ListSize',dlg_size)};
switch inputStr
	case 'object'
		options = {'bsh','img','stl','cpl','sg','sgc'};
		options_desc = {'My Object is a basic shape (e.g. Cylinder)','I have a picture of my object','I have an STL file of my Object','I have a CPL of my object','I have a SG of my Object','I have a SGcommand String'};
		inputStr = options{listdlg('ListString',options_desc,'SelectionMode','single','PromptString','In what form do you want to input your Object?','ListSize',dlg_size)};
end

inputStr = lower(inputStr);
switch inputStr
	case 'img'
		imagePath = imgetfile();
		imageAct = imread(imagePath);
		sizex =  size(imageAct,1);
		factor = 1000/sizex;
		imageAct= imresize(imageAct, factor);
		CPL = traceImage(imageAct);	
		height = str2double(inputdlg('What is the HEIGHT of your object? (in mm)','Height',[1 35],{'20'}));
		SG_object = SGofCPLz(CPL,height);
		SG_object = SGtransrelSG(SG_object,'','rotx',-pi/2,'rotz',pi/2);
		SG = placeObject(SG_object);	
	case 'stl'
		SG_object = SGreadSTL();
		SG = placeObject(SG_object);
	case 'sg'
		name = inputdlg('What is the name of your SG in the workspace');
		SG_object = evalin('base',name{1});
		SG = placeObject(SG_object);	
	case 'cpl'		
		name = inputdlg('What is the name of your CPL in the workspace');
		CPL_object = evalin('base',name{1});
		option = questdlg('Extrusion or Rotatiuon','CPL2SG','Extrusion','Rotation','');
		switch option			
			case 'Extrusion'				
				height = str2double(inputdlg('What is the HEIGHT of your object? (in mm)','Height',[1 35],{'20'}));
				SG_object = SGofCPLz(CPL_object,height);
			case 'Rotation'
				SG_object = SGofCPLrot(CPL_object,'',false);
		end		
		SG = placeObject(SG_object);
	case 'bsh'
		options = {'Box','Cylinder','Sphere','Conic Section'};
		inputStr = options{listdlg('ListString',options,'SelectionMode','single','PromptString','What is your object resembling?','ListSize',dlg_size)};
		switch inputStr
			case options(1)
				prompt = {'WIDTH of your Box in mm:','DEPTH of your box in mm:','HEIGHT of your box in mm:'};		
				output_dlg = str2double(inputdlg(prompt,'Box Dimensions',[1 35],{'20','20','20'}));				
				answer = questdlg('Do you want added geometry to bettrer grip your object?','Grip','Yes','No','No');
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
				prompt = {'DIAMETER of your Cylinder in mm:','HEIGHT of your Cylinder in mm:'};
				output_dlg = str2double(inputdlg(prompt,'Cylinder Dimensions',[1 35],{'20','20'}));			
				answer = questdlg('Do you want added geometry to bettrer grip your object?','Grip','Yes','No','No');
				switch answer 
					case 'No'	
						SG_object = SGcylinder(output_dlg(1)/2,output_dlg(2));
					case 'Yes'	
						CPL_circle = CPLaddauxpoints(PLcircle((output_dlg(1)/2)+0.35),5);
						CPL_remove = CPLatPL(PLcircle(0.5),CPL_circle);
						CPL_circle = CPLbool('-',CPL_circle,CPL_remove);
						SG_object = SGofCPLz(CPL_circle,output_dlg(2));
				end			
			case options(3)
				prompt = {'DIAMETER of your Sphere in mm:'};
				output_dlg = str2double(inputdlg(prompt,'Cylinder Dimensions',[1 35],{'20'}));					
				SG_object = SGsphere(output_dlg(1));
			case options(4)
				prompt = {'DIAMETER of lower circle in mm:','DIAMETER of upper circle in mm:','Height of cone section in mm:'};
				output_dlg = str2double(inputdlg(prompt,'Cone Dimensions',[1 35],{'20','30','100'}));
				SG_object = SGof2CPLsz(PLcircle(output_dlg(2)/2),PLcircle(output_dlg(1)/2),output_dlg(3));
	
			otherwise
				error("WRONG INPUT");
		end
		SG = placeObject(SG_object);	
	case 'sgc'
		command = inputdlg('What is the name of your CPL in the workspace');
		SG_object = SGofCPLcommand(command{1});
		SG = placeObject(SG_object);
	case 'rmx'
		SG = remixAdapter();
	otherwise
		error("WRONG INPUT");
end
if nargout == 0
	clf;
	SGfigure;
	SGplot(SG);
	SGwriteSTL(SG);
	VLFLplotlight
end
view(3);

end

function [SG] = remixAdapter()
fig_pos = [100 100 1280 720];
view_switch = 0;
t_anno = 1;
factor = 1;
rot_val = pi/2;
tran_val = 1;
inputs = {'transy',1*factor,29,28;'transz',1*factor,30,31;'roty',pi/2,115,119;'rotx',pi/2,113,101;'rotz',pi/2,97,100};
[SG_adapter,CPL_adapter] = SGconnAdaptersLCL('adapter_type','rotLock','cable',0);
SG_adapter = SGtransrelSG(SG_adapter,'','aligntop',-10);

SG_remix = SGreadSTL();
SG_remix = SGtransrelSG(SG_remix,'','center','alignbottom');

SGfigure;
set(gcf, 'Position',fig_pos);
f_anno_size = 140;
f_anno_string = "Use the following keys to position your object" + newline ...
	+ "V: Change view" + newline...
	+"Arrow Keys: Move Part" + newline ...
	+ "Rotate Object: (x-Axis) A/D (y-Axis) W/S (z-Axis) Q/E"+ newline ...
	+ "X : Confirm Position"+ newline ...
	+ "T : Toggle overlay"+newline+ ...
	"(Parts below the red line will be cut off your object)";
f_anno=SGfigureannotation(f_anno_string,'y',10,1-(f_anno_size/fig_pos(4)));
p_anno = plotannotation({"F/G: Increase/Decrease :"," ","Rotation: +/-" + num2str(rad2deg(factor*rot_val))+ "°","Translation: +/-" + num2str(factor*tran_val)+ " mm"},'Color','b'); 

annotation('textbox',[.8 .5 .1 .1],'String',"Parts below red line"+ newline+ "will be cut off your object",'Color','r'); 

[~,sy,~] = sizeVL(SG_adapter.VL);
VLplot([0 -sy/2 0;0 sy/2 0]);

P_adapter = SGplot(SG_adapter);
P_adapter.Tag = 'adapter';

P_remix = SGplot(SG_remix);
P_remix.Tag = 'remix';
set(gca,'visible','off')
view(90,0);
while true		
	[~,~,in]=myginput(1,'crosshair');
	if isempty(in); continue; 	end
	if in == 120
		break;
	end	
	for k=1:size(inputs,1)
		if in == inputs{k,3}
			SG_remix = SGtransrelSG(SG_remix,SG_remix,inputs{k,1},inputs{k,2}*factor);
		elseif in == inputs{k,4}
			SG_remix = SGtransrelSG(SG_remix,SG_remix,inputs{k,1},-inputs{k,2}*factor);
		end
	end
	if in == 118
		view_switch = mod(view_switch+1,3);
		if view_switch==0		
			inputs = {'transy',1*factor,29,28;'transz',1*factor,30,31;'roty',pi/2*factor,115,119;'rotx',pi/2*factor,113,101;'rotz',pi/2*factor,97,100};			
			view(90,0);
		elseif view_switch == 1
			inputs = {'transx',1*factor,29,28;'transy',1*factor,30,31;'roty',pi/2*factor,115,119;'rotx',pi/2*factor,113,101;'rotz',pi/2*factor,97,100};
			view(0,90);
		elseif view_switch == 2
			view(45,45);
		end
	end		
	if in == 116
		if t_anno == 1
			delete(f_anno);
		else			
			delete(p_anno);
			pos = get(gcf, 'Position');
			f_anno=SGfigureannotation(f_anno_string,'y',10,1-(f_anno_size/pos(4)));
			p_anno = plotannotation({"F/G: Increase/Decrease :"," ","Rotation: +/-" + num2str(rad2deg(factor*rot_val))+ "°","Translation: +/-" + num2str(factor*tran_val)+ " mm"},'Color','b'); 
		end
		t_anno = ~t_anno;
	end
	
	if in == 102 || in ==  103
		delete(p_anno);
		switch in
			case 102
				factor = factor*2;
			case 103
				factor = factor/2;
		end
		p_anno = plotannotation({"F/G: Increase/Decrease :"," ","Rotation: +/-" + num2str(rad2deg(factor*rot_val))+ "°","Translation: +/-" + num2str(factor*tran_val)+ " mm"},'Color','b'); 
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
fig_pos = [100 100 1280 720];
dlg_size = [400 100];
factor = 1;
rot_val = pi/2;
tran_val = 1;
servo_name = 'sm40bl';
last_servo_name =servo_name;
attach_dof = 'rotLock';
last_attach_dof = attach_dof;
attach_servo = 'sm40bl';
last_attach_servo = attach_servo;
H_Base_frame = [rotz(90) [0;0;0]; 0 0 0 1];
SG_base_box = SGbox(0.1);
SG_base_box = SGTset(SG_base_box,'BaseBox',H_Base_frame);

close;
SG_object = SGtrans0(SG_object);
H_Object = [rotx(0) [0;0;0]; 0 0 0 1];
SG_object = SGTset(SG_object,'Object',H_Object);

[~,~,~,~,~,gripper_options] = SGendeffectors('');
inputStr = gripper_options{listdlg('ListString',gripper_options,'SelectionMode','single','PromptString','Choose what your object resembles','ListSize',dlg_size)};

[SG_gripper_sil,SG_grippers,~,inputsO,inputsG] = SGendeffectors(inputStr);
SG_gripper_sil = SGtransrelSG(SG_gripper_sil,SG_base_box,'alignT',{'GripperT','BaseBox'});
SG_grippers = SGtransrelSG(SG_grippers,SG_gripper_sil,'alignT',{'GripperT','GripperT'});
SG_grippers = SGcolorfaces(SG_grippers,'g');
SG_grippers.alpha = 0.5;

close all;
Help_string = "USE THE FOLLOWING KEYS TO ADJUST OBJECT POSITION AND ENDEFFECTOR GEOMETRY" + newline;
Help_string = Help_string + "Move Object: Arrow Keys "+ newline +"    Rotate Object: (x-Axis) A/D (y-Axis) W/S (z-Axis) Q/E";
Help_string = Help_string +newline+ "Endeffectoroptions:   ";
for h = 1:size(inputsG,1)
	key1 = upper(getKeyCharFromASCII(inputsG{h,4}));
	key2 = upper(getKeyCharFromASCII(inputsG{h,5}));
	Help_string = Help_string + inputsG{h,1} + ": " + key1+"\"+key2 + "      ";
end
Help_string = Help_string + newline + "M: Change Servo/ C: Change Connector ";
Help_string = Help_string + newline +  "V: Change view";
Help_string = Help_string + newline + "T : Toggle Annotation";
Help_string = Help_string + newline + "X : Confirm Position";
SGfigure;
set(gcf, 'Position',fig_pos);
f_anno_size = 140;
f_anno = SGfigureannotation(Help_string,'y',10,1-(f_anno_size/fig_pos(4)));
p_anno = plotannotation({"F/G: Increase/Decrease :"," ","Rotation: +/-" + num2str(rad2deg(factor*rot_val))+ "°","Translation: +/-" + num2str(factor*tran_val)+ " mm"},'Color','b'); 



gripper = SGplot(SG_grippers);
gripper.Tag = 'gripper';

gripper_base = SGplot(SG_gripper_sil);
gripper_base.Tag = 'base';

SG_object = SGtransrelSG(SG_object,SG_grippers,'alignT',{'Object','ObjectPos'});
object = SGplot(SG_object);
object.Tag = 'object';

view(90,0);
set(gca,'visible','off')
view_switch = 0;
t_anno = 1;
last_inputsG = inputsG;
err_anno = [];
while true
	[~,~,in]=myginput(1,'crosshair');	
	if isempty(in); continue; end
	delete(err_anno);
	change = 0;
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
			inputsG{k,2}=inputsG{k,2}+(inputsG{k,3}*factor);
			change = 1;
		elseif in == inputsG{k,5}
			inputsG{k,2}=inputsG{k,2}-(inputsG{k,3}*factor);
			change = 1;
		end
	end
	if in == 118
		view_switch = mod(view_switch+1,3);
		if view_switch == 0			
			inputsO(strcmpi(inputsO,'transy')) = {'transz'};
			inputsO(strcmpi(inputsO,'transx')) = {'transy'};
			pos_rotx = strcmpi(inputsO,'rotx');
			pos_roty = strcmpi(inputsO,'roty');
			inputsO(pos_rotx) = {'roty'};			
			inputsO(pos_roty) = {'rotx'};
			inputsO(pos_roty,2) = {-inputsO{pos_roty,2}};
			view(90,0);
		elseif view_switch == 1
			inputsO(strcmpi(inputsO,'transy')) = {'transx'};
			pos_rotx = strcmpi(inputsO,'rotx');
			pos_roty = strcmpi(inputsO,'roty');
			inputsO(pos_roty) = {'rotx'};
			inputsO(pos_rotx) = {'roty'};
			inputsO(pos_rotx,2) = {-inputsO{pos_rotx,2}};
			view(0,0);
		elseif view_switch == 2
			inputsO(strcmpi(inputsO,'transz')) = {'transy'};
			view(45,45);
		end		
	end
	if in == 102 || in == 103
		delete(p_anno);
		switch in
			case 102				
				factor = factor*2;
			case 103				
				factor = factor/2;
		end
		p_anno = plotannotation({"F/G: Increase/Decrease :"," ","Rotation: +/-" + num2str(rad2deg(factor*rot_val))+ "°","Translation: +/-" + num2str(factor*tran_val)+ " mm"},'Color','b'); 
	end
	if in == 116
		if t_anno == 1			
			delete(f_anno);
		else	
			delete(p_anno);
			f_anno = SGfigureannotation(Help_string,'y',10,1-(f_anno_size/fig_pos(4)));
			p_anno = plotannotation({"F/G: Increase/Decrease :"," ","Rotation: +/-" + num2str(rad2deg(factor*rot_val))+ "°","Translation: +/-" + num2str(factor*tran_val)+ " mm"},'Color','b'); 
		end		
		t_anno = ~t_anno;
	end		
	if in == 109 %m
		servo_options = readtable('Servos.xlsx');
		servo_options = table2cell(servo_options(:,1)); 
		servo_name = servo_options{listdlg('ListString',servo_options,'SelectionMode','single','PromptString','Choose the motor for the dof')};
		change = 1;
	end	
	if in == 99 %c
		[~,~,dof_options] = SGdofsLCL();
		attach_dof = dof_options{listdlg('ListString',dof_options,'SelectionMode','single','PromptString','Which connector do you want?')};
		servo_options = readtable('Servos.xlsx');
		servo_options = table2cell(servo_options(:,1));
		attach_servo = servo_options{listdlg('ListString',servo_options,'SelectionMode','single','PromptString','Choose the motor for the dof')};
		change = 1;
	end
	if change
		try
			[SG_gripper_sil,SG_grippers] = SGendeffectors(inputStr,inputsG,'',servo_name,attach_dof,attach_servo);
			SG_gripper_sil = SGtransrelSG(SG_gripper_sil,SG_base_box,'alignT',{'GripperT','BaseBox'});
			SG_grippers = SGtransrelSG(SG_grippers,SG_gripper_sil,'alignT',{'GripperT','GripperT'});
		catch
			err_anno = annotation('textbox', [0.4, 0, 0.1, 0.1], 'String', "Change was not possible! Reverted!",'Color','r','linestyle','none');
			inputsG = last_inputsG;
			attach_dof = last_attach_dof;
			attach_servo = last_attach_servo;
			servo_name = last_servo_name;
		end
	end

	patch = findall(gcf, 'Tag', 'gripper');
	delete(patch);	
	SG_grippers = SGcolorfaces(SG_grippers,'g');
	SG_grippers.alpha = 0.5;
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
	
	last_inputsG = inputsG;
	last_attach_dof = attach_dof;
	last_attach_servo =attach_servo;
	last_servo_name= servo_name;
	
	
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
t_anno = 1;
CPL_out = [];
SGfigure;
CPLplot([0 0]);
imshow(i_file);
fig_pos =get(gcf, 'Position');
f_anno=SGfigureannotation("CLICK AROUND THE OUTLINE OF YOUR OBJECT"+ newline +"Z : Remove Last Point"+ newline +"T : Toggle Annotation"+ newline + "X : Confirm Outline",'y',10,1-(80/fig_pos(4)));
axis equal;
view(2);

while true
	[p,in] = inputFIG();
	if in == 120; break; end
	if in == 116
		if t_anno == 1
			delete(f_anno);
		else
			f_anno=SGfigureannotation("CLICK AROUND THE OUTLINE OF YOUR OBJECT"+ newline +"Z : Remove Last Point"+ newline +"T : Toggle Annotation"+ newline + "X : Confirm Outline",'y',10,1-(80/fig_pos(4)));
		end
		t_anno = ~t_anno;
	end
	if in == 122
		CPL_out = CPL_out(1:end-1,:);
		CPLplot(CPL_out);
		hi = imshow(i_file);
		uistack(hi,'down');
		axis equal;
		view(2);
	end
	if in == 1
		if t_anno
			delete(f_anno);
		end
		delete(f_anno);
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
		if t_anno
			f_anno=SGfigureannotation("CLICK AROUND THE OUTLINE OF YOUR OBJECT"+ newline +"Z : Remove Last Point"+ newline +"T : Toggle Annotation"+ newline + "X : Confirm Outline",'y',10,1-(80/fig_pos(4)));
		end
	end
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


realLength = str2double(inputdlg('What is the real Length of the blue line in mm '));
virtLength = line_dis;

factor = realLength/virtLength;
CPL_out = CPL_out*factor;

	function [p,in] = inputFIG()
		p = [];
		[x,y,in]=myginput(1,'crosshair');
		if in == 3; return; end
		p = [x y];
	end
end



