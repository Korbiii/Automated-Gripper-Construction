%%   [SG_base,SG_main_body,SG_complete,inputsObject,inputsGripper] = SGLCLtoolholder([variable_name, value])
%    === INPUT PARAMETERS ===
%    'width':				Width of toolholder
%	 'height':				Height of toolholder
%    'length':				Length of toolholder
%    'conn_type':			Connection type. e.g. 'rotLock','z'
%	 'servo':				Servo used for connection bar rotLock	
%    'output':				If set function writes STLs
%	 'c_input':				Cell array of above inputs e.g. {'width',20;'height',30}
%    === OUTPUT RESULTS ======
%    SG_base:				SG of toolholder main body
%	 SG_main_body:			SG of variable toolholder body
%	 SG_final:				SG complete toolholder
%	 inputsObject:			Input array for object manipulation
%	 inputsGripper:			Input array for gripper manipulation
function [SG_base,SG_main_body,SG_complete,inputsObject,inputsGripper] = SGLCLtoolholder(varargin)
servo_name = 'sm40bl';
adapter_type = 'rotLock';
width_holder = 50;
length_holder = 50;
height_holder = 50;

inputsObject = {'transx',1,52,54;'transy',1,29,28;'transz',2,30,31;'roty',pi/2,115,119;'rotz',pi/2,113,101;'rotx',pi/2,97,100};
inputsGripper = {'width',50,2,43,45;'height',20,3,104,106;'length',40,3,107,108};

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
output = 0;
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
		case 'length'
			length_holder = varargin{i_idx+1};
		case 'SG_object'
			SG_object = varargin{i_idx+1};
		case 'conn_type'
			SG_object = varargin{i_idx+1};					
			i_idx = i_idx+1;	
		case 'conn_servo'
			SG_object = varargin{i_idx+1};		
			i_idx = i_idx+1;	
		case 'output'
			output = 1;
	end
	i_idx = i_idx+1;
end




CPL_base = PLroundcorners(PLsquare(width_holder,length_holder),[1,2,3,4],10);
SG_base = SGofCPLzchamfer(CPL_base,height_holder*2,height_holder*0.05);
[~,SG_main_body] = SGcut(SG_base,height_holder);

height_max = max(SG_main_body.VL(:,3));
T_Object = [rotx(0) [0;0;height_max]; 0 0 0 1];
SG_main_body = SGTset(SG_main_body,'ObjectPos',T_Object);

height_min = min(SG_main_body.VL(:,3));
T_connection_top = [rotx(0) [0;0;height_min]; 0 0 0 1];
SG_main_body = SGTset(SG_main_body,'GripperT',T_connection_top);

if ~isempty(SG_object)
	SG_object = SGtransrelSG(SG_object,SG_main_body,'alignTz',{'ObjectPos','ObjectPos'});	
	SG_main_body=SGslicebool(SG_main_body,SG_object);
end

[SG_connector, CPL_connector] = SGconnAdaptersLCL('adapter_type',adapter_type,'servo',servo_name,'cable',0);
CPL_connector = CPLaddauxpoints(CPL_connector,0.5);
CPL_base = CPLaddauxpoints(CPL_base,0.5);
SG_connection = SGof2CPLsz(CPL_connector,CPL_base,10);

SG_base = SGstack('z',SG_connector,SG_connection);

height_max = max(SG_base.VL(:,3));
T_Connection_bot = [rotx(0) [0;0;height_max]; 0 0 0 1];
SG_base = SGTset(SG_base,'GripperT',T_Connection_bot);

SG_complete = SGstack('z',SG_connector,SG_connection,SG_main_body);

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
end
SG_main_body.alpha = 0.45;


end