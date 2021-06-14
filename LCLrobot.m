%%   SG = LCLRobot(input1,)
%    === INPUT PARAMETERS ===
%    input1 : 
%
%    === OUTPUT RESULTS ======
%    SG : 
%
function [SG] = LCLrobot(varargin)
 clf;

load('STL/Base.mat');
load('STL/Shoulder.mat');
load('STL/UpperArm.mat');

phis = [0 pi -pi/4 -pi/4];


SGs = {SG_base,SG_shoulder,SG_upper_Arm};

SGc = SGTchain(SGs,[0 pi -pi/4]);


SGplot(SGc);
view(3);

arm_length = input('How long should the lower arm be? ');
SGs{end+1} = SGUnterarm(arm_length,'z','sm40bl');
SGc = SGTchain(SGs,phis);
clf;
SGplot(SGc);
view(3);

yes_no = input('Do you want a different degree of Freedom? y/n ','s');
last_dof = 'z';
if yes_no == 'y'
	last_dof = input('What degree of freedom do you want at the end of your lower arm? x = movement in plane/ y=movement perpendicular to plane of lower arm ','s');
	SGs{end} = SGUnterarm(arm_length,last_dof,'sm40bl');
	SGc = SGTchain(SGs,phis);
	clf;
	SGplot(SGc);
	view(3);
end
last_servo = input('Do you want a different motor for the dof? n for sm40bl no or name of motor e.g. sm85bl','s');
if last_servo ~= 'n'
	SGs{end} = SGUnterarm(arm_length,last_dof,last_servo);
	SGc = SGTchain(SGs,phis);
	clf;
	SGplot(SGc);
	view(3);	
else
	last_servo = 'sm40bl';
end


next_dof = input('Specify next degree of freedom! n = no additional dof/ x/z/y for next dof','s');
next_servo = 'sm40bl';
while next_dof ~= 'n'
	SGs{end+1} = SGdofsLCL('attach_dof',last_dof,'dof',next_dof,'servo',next_servo,'attach_servo',last_servo);
	phis = [phis 0];
	SGc = SGTchain(SGs,phis);
	clf;
	SGplot(SGc);
	view(3);		
	next_servo = input('Do you want a different motor for the dof? n for sm40bl no or name of motor e.g. sm85bl','s');
	if next_servo ~= 'n'
		SGs{end} = SGdofsLCL('attach_dof',last_dof,'dof',next_dof,'servo',next_servo,'attach_servo',last_servo);
		SGc = SGTchain(SGs,phis);
		clf;
		SGplot(SGc);
		view(3);
	else
		next_servo = 'sm40bl'
	end
	last_servo = next_servo;
	last_dof = next_dof;
	next_dof = input('Specify next degree of freedom! n = no additional dof/ x/z/y for next dof','s');
	next_servo = 'sm40bl';
end


SGs{end+1} = SGrotatinglockadapter(1,last_dof,last_servo);
SGc = SGTchain(SGs,[phis 0]);
clf;
SGplot(SGc);
view(3);

SGwriteSTL(SGc);

end