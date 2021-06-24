%%   SG = LCLRobot()
%    === INPUT PARAMETERS ===
%    === OUTPUT RESULTS ======
%    
function [SG] = LCLrobot(varargin)
close all

[~,~,dof_options] = SGdofsLCL();
servo_options = readtable('Servos.xlsx');
servo_options = table2cell(servo_options(:,1));

load('STL/Base.mat');
load('STL/Shoulder.mat');
load('STL/UpperArm.mat');

phis = [0 pi -pi/4 -pi/4];
SGs = {SG_base,SG_shoulder,SG_upper_Arm};
SGc = SGTchain(SGs,[0 pi -pi/4]);
SGplot(SGc);
view(3);


arm_length = str2double(inputdlg('How long should the lower arm be?'));
SGs{end+1} = SGUnterarm(arm_length,'z','sm40bl');
SGc = SGTchain(SGs,phis);
clf;
SGplot(SGc);
view(3);

last_dof = dof_options{listdlg('ListString',dof_options,'SelectionMode','single','PromptString','Choose the lower arm degree of freedom')};
SGs{end} = SGUnterarm(arm_length,last_dof,'sm40bl');
SGc = SGTchain(SGs,phis);
clf;
SGplot(SGc);
view(3);

last_servo = servo_options{listdlg('ListString',servo_options,'SelectionMode','single','PromptString','Choose the motor for the dof')};
SGs{end} = SGUnterarm(arm_length,last_dof,last_servo);
SGc = SGTchain(SGs,phis);
clf;
SGplot(SGc);
view(3);


next_dof = listdlg('ListString',dof_options,'SelectionMode','single','PromptString','Choose the next dof');
while ~isempty(next_dof)
	next_dof = dof_options{next_dof};
	next_servo = servo_options{listdlg('ListString',servo_options,'SelectionMode','single','PromptString','Choose the motor for the dof')};
		
	SGs{end+1} = SGdofsLCL('attach_dof',last_dof,'dof',next_dof,'servo',next_servo,'attach_servo',last_servo);
	phis = [phis 0];
	SGc = SGTchain(SGs,phis);
	clf;
	SGplot(SGc);
	view(3);
	
	last_servo = next_servo;
	last_dof = next_dof;
	next_dof = listdlg('ListString',dof_options,'SelectionMode','single','PromptString','Choose the next dof');	
end

SGs{end+1} = SGdofsLCL('attach_dof',last_dof,'dof','rotLock','attach_servo',last_servo,'cable');
SGc = SGTchain(SGs,[phis 0]);
if nargout == 0
	clf;
	SGplot(SGc);
	view(3);
end


end