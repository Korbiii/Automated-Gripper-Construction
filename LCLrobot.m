function [SG] = LCLrobot(varargin)

arm_length = 150;
dof_array = ['z','x'];
servo_array = {'sm40bl','sm40bl','sm40bl'};
gripper_type = 1;
gripper_servo = 'sm40bl';

for f=1:size(varargin,2)
if ~ischar(varargin{f}), continue; end
      switch varargin{f}
          case 'arm_length'
              arm_length = varargin{f+1};     
          case 'driven_gripper'
              gripper_type = 1;
          case 'mechanical_gripper'
              gripper_type = 2;
          case 'gripper_servo'
              gripper_servo = varargin{f+1};
      end   
end

SGs = {};
SGs{end+1} = SGUnterarm(arm_length,dof_array(1),servo_array{1});

for i=2:size(dof_array,2)
    if dof_array(i) == 'x'
        SGs{end+1} = SGLCLzdof(servo_array{i},dof_array(i-1),servo_array{i-1});
    elseif dof_array(i) == 'z'
        SGs{end+1} = SGRotatingattach(servo_array{i},dof_array(i-1),servo_array{i-1});
    end
end

if gripper_type ~= 0
    if gripper_type == 1
        SG_gripper = SGgripper(dof_array(end));
    elseif gripper_type == 2
        % TODO
    end
    SGs{end+1} = SG_gripper;
end


SGc = SGTchain(SGs);
SGplot(SGc);
end