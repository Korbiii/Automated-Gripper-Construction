%SGLCLtoolCasing(varargin) - returns the SGs to create a tool casing for
%	the LCL robot.
%
%	This function returns the SGs and STLs for a 3D printable tool casing
%	for the LCL robot.
%
%   [SG_base,SG_main_body,SG_complete,inputsObject,inputsGripper] = SGLCLtoolCasing(varargin)
%   === INPUT PARAMETERS ===
%   'width':				Width of tool casing
%	'height':				Height of tool casing
%   'thickness':			Thickness of tool casing
%   'output':				If set function writes STLs
%	'c_input':				Cell array of above inputs e.g. {'grip_H',50;'output'}
%   === OUTPUT RESULTS ======
%   SG:						SG of gripper main body
%	SG_main_body:			SG of variable  part of tool casing
%	SG_final:				SG complete gripper
%	inputsObject:			Input array for object manipulation
%	inputsGripper:			Input array for casing manipulation
function [SG_base,SG_main_body,SG_complete,inputsObject,inputsGripper] = SGLCLtoolCasing(varargin)
conn_servo = 'sm40bl';
conn_type = 'rotLock';
width_holder = 50;
height_holder = 50;
thickness = 50;
output = 0;

inputsObject = {'transy',1,29,28;'transz',2,30,31;'roty',pi/2,115,119;'rotx',pi/2,97,100;'rotz',-pi/2,101,113};
inputsGripper = {'width',50,2,43,45;'height',50,3,104,106;'thickness',50,2,107,108};

if ~isempty(varargin)
	if strcmp(varargin{1},'c_inputs')
		temp = varargin{2};
		varargin(1) = [];
		varargin(1) = [];
		for k=1:size(temp,1)
			varargin{end+1} = temp{k,1};
			varargin{end+1} = temp{k,2};
		end
	end
end

i_idx = 1;
SG_object = {};
while i_idx<=size(varargin,2)
	if ~ischar(varargin{i_idx})
		i_idx = i_idx+1; 
		continue; 
	end
	switch varargin{i_idx}
		case 'width'
			width_holder = varargin{i_idx+1};
		case 'height'
			height_holder = varargin{i_idx+1};
		case 'thickness'
			thickness = varargin{i_idx+1};
		case 'SG_object'
			SG_object = varargin{i_idx+1};
		case 'conn_type'
			conn_type = varargin{i_idx+1};					
			i_idx = i_idx+1;	
		case 'conn_servo'
			conn_servo = varargin{i_idx+1};		
			i_idx = i_idx+1;	
		case 'output'
			output = 1;
	end
	i_idx = i_idx+1;
end

CPL_base = PLroundcorners(PLsquare(width_holder,height_holder),[3,4],width_holder*0.05);


PL_hole_positions =PLsquare(width_holder-10,height_holder-10);
PL_hole_positions = CPLaddauxpoints(PL_hole_positions,20);
CPL_screw_TH = CPLatPL(PLcircle(1.6),PL_hole_positions);
CPL_screw_HH = CPLatPL(PLcircle(3),PL_hole_positions);
CPL_base_TH = CPLbool('-',CPL_base,CPL_screw_TH);
CPL_base_HH = CPLbool('-',CPL_base,CPL_screw_HH);

SG_fixed_side = SGofCPLz(CPL_base_TH,thickness/2);
SG_variable_side_in = SGofCPLz(CPL_base_TH,5);
SG_variable_side_out = SGofCPLz(CPL_base_HH,(thickness/2)-5);

SG_variable_side = SGcat(SG_variable_side_in,SGontop(SG_variable_side_out,SG_variable_side_in,-0.01));

SG_fixed_side = SGtransrelSG(SG_fixed_side,'','rotx',pi/2,'transy',thickness/2);
SG_variable_side = SGtransrelSG(SG_variable_side,'','rotx',pi/2);

SG_main_body = SGcat(SG_fixed_side,SG_variable_side);

height_max = max(SG_main_body.VL(:,3));
T_Object = [rotx(0) [0;0;height_max]; 0 0 0 1];
SG_main_body = SGTset(SG_main_body,'ObjectPos',T_Object);

height_min = min(SG_main_body.VL(:,3));
T_connection_top = [rotx(0) [0;0;height_min]; 0 0 0 1];
SG_main_body = SGTset(SG_main_body,'GripperT',T_connection_top);

if ~isempty(SG_object)
	SG_object = SGtransrelSG(SG_object,SG_main_body,'alignTz',{'ObjectPos','ObjectPos'});	
	
	SG_fixed_side = SGslicebool(SG_fixed_side,SG_object);
	SG_variable_side = SGslicebool(SGtransrelSG(SG_variable_side,'','rotx',-pi/2),SGtransrelSG(SG_object,'','rotx',-pi/2));	
	
	SG_variable_side = SGtransrelSG(SG_variable_side,'','rotx',pi/2);
	SG_main_body = SGcatF(SG_fixed_side,SG_variable_side);		
end

[SG_connector, CPL_connector] = SGconnAdaptersLCL('adapter_type',conn_type,'servo',conn_servo,'cable',0);
CPL_connector = CPLaddauxpoints(CPL_connector,0.5);
CPL_base = CPLaddauxpoints(PLsquare(width_holder,thickness),0.5);
SG_connection = SGof2CPLsz(CPL_connector,CPL_base,10);

SG_base = SGstack('z',SG_connector,SG_connection);

height_max = max(SG_base.VL(:,3));
T_Connection_bot = [rotx(180) [0;0;height_max]; 0 0 0 1];
SG_base = SGTset(SG_base,'GripperT',T_Connection_bot);

SG_complete = SGstack('z',SG_connector,SG_connection,SG_main_body);
SG_fixed_side = SGstack('z',SG_connector,SG_connection,SG_fixed_side);

height_max = max(SG_complete.VL(:,3));
T_Object = [rotx(0) [0;0;height_max]; 0 0 0 1];
SG_complete = SGTset(SG_complete,'ObjectPos',T_Object);

height_max = min(SG_complete.VL(:,3));
H_GripperPos = [rotx(0) [0;0;height_max+20]; 0 0 0 1];
SG_complete = SGTset(SG_complete,'GripperT',H_GripperPos);

if nargout == 0
	clf;
	SGplot(SG_complete);
	SGwriteSTL(SG_complete);
	SGwriteSTL(SG_base);
end
if output
	fnames = {};
	fnames{end+1} = SGwriteSTL(SG_variable_side,"Tool Casing Variable Side");
	fnames{end+1} = SGwriteSTL(SG_fixed_side,"Tool Casing Fixed Side");
	SGsSaveToFolder(fnames);
end


end