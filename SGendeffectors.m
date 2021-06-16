function [SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG,gripper_options] = SGendeffectors(endE_nameStr,varargin)
inputs = {}; if nargin>=2 && ~isempty(varargin{1}); inputs=varargin{1}(:,1:2); end
SG_object = []; if nargin>=3 && ~isempty(varargin{2}); SG_object=varargin{2}; end
[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = deal([]);


gripper_options = {'Tool','Mech','Para','Comp'};

switch endE_nameStr
	case gripper_options(1)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGLCLtoolholder();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGLCLtoolholder('c_inputs',inputs,'SG_object',SG_object,'output');
		end
	case gripper_options(2)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGmechGripper();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGmechGripper('c_inputs',inputs,'SG_object',SG_object,'output');
		end
	case gripper_options(3)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGparrallelGripper();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGparrallelGripper('c_inputs',inputs,'SG_object',SG_object,'output');
		end
	case gripper_options(4)
		if isempty(inputs)
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGgripper();
		else
			[SG_gripper_sil,SG_grippers,SG_final,inputsO,inputsG] = SGgripper('c_inputs',inputs,'output');
		end
	otherwise 
		disp("Gripper options");
end



end