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
[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = deal([]);


gripper_options = {'Simple Toolholder','Mechanical Gripper','Parallel Gripper','Compliant Gripper','Tool Casing'};

switch endE_nameStr
	case gripper_options(1)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGLCLtoolholder();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGLCLtoolholder('c_inputs',inputs,'SG_object',SG_object);
		end
	case gripper_options(2)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGmechGripper();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGmechGripper('c_inputs',inputs,'SG_object',SG_object);
		end
	case gripper_options(3)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGparrallelGripper();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGparrallelGripper('c_inputs',inputs,'SG_object',SG_object);
		end
	case gripper_options(4)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGcompliantGripper();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGcompliantGripper('c_inputs',inputs);
		end
	case gripper_options(5)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGLCLtoolcasing();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGLCLtoolcasing('c_inputs',inputs,'SG_object',SG_object);
		end
	otherwise 
		disp("Gripper options");
end



end