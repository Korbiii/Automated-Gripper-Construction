%  LCLRobot() - Leads the user through the generation of LCL DOFs
%
%	This function leads the user through a process that sets up the LCL
%	robots degree of freedoms
%
%	SG = LCLRobot()
%	=== INPUT PARAMETERS ===
%	=== OUTPUT RESULTS ======
%	SG:	Outputs all generated SGs in a user chosen folder
function [SG] = SGLCLdofsUI(varargin)
close all
fig_pos = [100 100 1280 720];
neutralSGs= [];
[~,~,dof_options] = SGdofsLCL();
servo_options = readtable('Servos.xlsx');
servo_options = table2cell(servo_options(:,1));

load('STL/Base.mat');
load('STL/Shoulder.mat');
load('STL/UpperArm.mat');

phis = [0 pi -pi/4 -pi/4];
SGs = {SG_base,SG_shoulder,SG_upper_Arm};
SGc = SGTchain(SGs,[0 pi -pi/4]);
SGfigure;
set(gcf, 'Position',fig_pos);
refresh(SGc);

success = 0;
while success == 0
	arm_length = str2double(inputdlg('How long should the lower arm be?'));
	try
		SGs{end+1} = SGLCLlowerArm('arm_length',arm_length,'dof','z','servo','sm40bl');
		success = 1;
	catch	
	end
end

SGc = SGTchain(SGs,phis);
refresh(SGc);

success = 0;
while success == 0
	last_dof = dof_options{listdlg('ListString',dof_options,'SelectionMode','single','PromptString','Choose the lower arm degree of freedom')};
	try		
		SGs{end} = SGLCLlowerArm('arm_length',arm_length,'dof',last_dof,'servo','sm40bl');
		success = 1;
	catch
	end
end
SGc = SGTchain(SGs,phis);
refresh(SGc);

success = 0;
while success == 0
	last_servo = servo_options{listdlg('ListString',servo_options,'SelectionMode','single','PromptString','Choose the motor for the dof')};
	try		
		SGs{end} = SGLCLlowerArm('arm_length',arm_length,'dof',last_dof,'servo',last_servo);
		success = 1;
	catch
	end
end
SGc = SGTchain(SGs,phis);
refresh(SGc);


next_dof = listdlg('ListString',dof_options,'SelectionMode','single','PromptString','Choose the next dof');
while ~isempty(next_dof)
	next_dof = dof_options{next_dof};
	success = 0;
	while success == 0
		next_servo = servo_options{listdlg('ListString',servo_options,'SelectionMode','single','PromptString','Choose the motor for the dof')};
		try
			SGs{end+1} = SGdofsLCL('attach_dof',last_dof,'dof',next_dof,'servo',next_servo,'attach_servo',last_servo);
			success = 1;
		catch
		end
	end	
	phis = [phis 0];
	SGc = SGTchain(SGs,phis);
	refresh(SGc);
	
	last_servo = next_servo;
	last_dof = next_dof;
	next_dof = listdlg('ListString',dof_options,'SelectionMode','single','PromptString','Choose the next dof');	
end

SGs{end+1} = SGdofsLCL('attach_dof',last_dof,'dof','rotLock','attach_servo',last_servo,'cable');
SGc = SGTchain(SGs,[phis 0]);

if nargout == 0
	fnames = {};
	for i=4:size(SGs,2)		
		fnames{end+1} = SGwriteSTL(SGs{i},strcat('DoF',num2str(i-2)));		
	end
	SGsSaveToFolder(fnames);
	clf;
	SGplot(SGc);
	view(3);
end

end

function refresh(SG)
	clf;
	SG = SGcat(SG);
	SGplot(SG);
	VLFLplotlight;	
	set(gca,'visible','off')
	view(3);
end