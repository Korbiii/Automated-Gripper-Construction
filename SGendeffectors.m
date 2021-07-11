%%   [SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG,gripper_options] = SGendeffectors([inputs, SG_object])
%    === INPUT PARAMETERS ===
%    inputs:				inputArray to pass through to gripper function
%	 SG_object:				SG of object to grip to pass through to gripper function
%    === OUTPUT RESULTS ======
%    SG_gripper_sil:		SG of gripper main body
%	 SG_grippers:			SG of variable gripper parts with reduced alpha value
%	 inputsO:				Input array for object manipulation
%	 inputsG:				Input array for gripper manipulation
%	 gripper_options:		Returns field of existing endeffector options
function [SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG,gripper_options] = SGendeffectors(endE_nameStr,varargin)
inputs = {}; if nargin>=2 && ~isempty(varargin{1}); inputs=varargin{1}(:,1:2); end
SG_object = []; if nargin>=3 && ~isempty(varargin{2}); SG_object=varargin{2}; end
servo_name = 'sm40bl'; if nargin>=4 && ~isempty(varargin{3}); servo_name=varargin{3}; end
conn_dof = 'rotLock'; if nargin>=5 && ~isempty(varargin{4}); conn_dof=varargin{4}; end
conn_servo = 'sm40bl'; if nargin>=6 && ~isempty(varargin{5}); conn_servo=varargin{5}; end
[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = deal([]);


gripper_options = {'Simple Toolholder','Mechanical Gripper','Parallel Gripper','Compliant Gripper','Tool Casing'};

switch endE_nameStr
	case gripper_options(1)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGLCLtoolholder();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGLCLtoolholder('c_inputs',inputs,'SG_object',SG_object,'conn_type',conn_dof,'conn_servo',conn_servo);
		end
	case gripper_options(2)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGmechGripper();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGmechGripper('c_inputs',inputs,'SG_object',SG_object,'conn_type',conn_dof,'conn_servo',conn_servo);
		end
	case gripper_options(3)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGparrallelGripper();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGparrallelGripper('c_inputs',inputs,'SG_object',SG_object,'servo',servo_name,'conn_type',conn_dof,'conn_servo',conn_servo);
		end
	case gripper_options(4)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGcompliantGripper();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGcompliantGripper('c_inputs',inputs,'servo',servo_name,'conn_type',conn_dof,'conn_servo',conn_servo);
		end
	case gripper_options(5)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGLCLtoolcasing();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGLCLtoolcasing('c_inputs',inputs,'SG_object',SG_object,'conn_servo',servo_name,'conn_type',conn_dof);
		end
	otherwise 
		disp("Gripper options");
end



end