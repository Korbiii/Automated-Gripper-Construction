function [SG,CPL] = SGLCLzdof(servo_name,varargin)
clf;
servo_name = lower(servo_name);
load Servos;
switch servo_name
	case 'sm40bl'
		servo = Servos.sm40;
	case 'sm85bl'
		servo = Servos.sm85;
	otherwise
		error('Only SM40BL and SM85BL implemented');
end

attach_dof = 0; if nargin>=2 && ~isempty(varargin{1}); attach_dof=varargin{1}; end
attach_servo = 0; if nargin>=3 && ~isempty(varargin{2}); attach_servo=varargin{2}; end
tol = 0.5;
screw_length = 14-3;

%% Servomount
distance_axis = servo.length-servo.shaft_offs;
servo.width = servo.width+tol;
servo.length = servo.length+tol;
servo.height = servo.height+tol;
outer_radius = servo.width/2+Servos.cable_gap+3;
start_cable_gap = servo.height/2+min(servo.PL_cable_gap_ver(:,2));

CPL_outside = CPLconvexhull([PLcircle(outer_radius);[-outer_radius,-distance_axis-2;outer_radius,-distance_axis-2]]);
CPL_outside = CPLbool('-',CPL_outside,CPLconvexhull([PLcircle(servo.connect_R);[-servo.connect_R,30;servo.connect_R,30]]));

CPL_outside_w_cable_slots = CPLbool('-',CPL_outside,PLsquare(servo.width+2*Servos.cable_gap,servo.length*2));
CPL_outside_w_servo_slot = CPLbool('-',CPL_outside,PLsquare(servo.width,servo.length*2));
CPL_outside_w_screw_slots = CPL_outside_w_servo_slot;


if(~isnan(servo.screw_mount_x))
	shaft_mid_Dis = servo.length/2-servo.shaft_offs;
	bot = min(servo.screw_mount_x(:,2))-servo.screw_R;
	top = max(servo.screw_mount_x(:,2))+servo.screw_R;
	CPL_outside_w_screw_slots = CPLbool('-',CPL_outside_w_screw_slots,PLtrans(PLsquare(outer_radius*2,top-bot),[0 -shaft_mid_Dis-top/2]));
	
elseif(~isnan(servo.screw_mount_z))
	%%TODO
end

SG_front = SGof2CPLsz(CPL_outside,CPL_outside,3);
SG_1 = SGofCPLz(CPL_outside_w_servo_slot,start_cable_gap);
SG_2 = SGofCPLz(CPL_outside_w_cable_slots,Servos.cable_gap_width);
SG_3 = SGofCPLz(CPL_outside_w_screw_slots,servo.height-Servos.cable_gap_width-start_cable_gap);
SG_back = SGof2CPLsz(CPL_outside,CPL_outside,3);

if(~isnan(servo.screw_mount_x))
	CPL_screws_small = PLsquare(servo.height-Servos.cable_gap_width-start_cable_gap,top-bot);
% 	screw_hols = PLtrans(CPLatPL(PLcircle(servo.screw_R),servo.screw_mount_x),[0 0])
	SG_screws_small = SGofCPLz(CPL_screws_small,screw_length);
	SG_screws_small = SGtransrelSG(SG_screws_small,SG_3,'roty',pi/2,'transy',-shaft_mid_Dis-top/2,'transx',servo.width/2,'aligntop');
	SG_screws_small = SGcat(SG_screws_small,SGmirror(SG_screws_small,'yz'));
    SG_3 = SGcat(SG_3,SG_screws_small);
elseif(~isnan(servo.screw_mount_z))
	%%TODO
end


SG = SGstack('z',SG_front,SG_1,SG_2,SG_3,SG_back);
SG = SGtransrelSG(SG,'','rotx',pi/2,'centery');

CPL = CPLconvexhull(CPLofSGslice(SG,min(SG.VL(:,2))));
CPL = [CPL;NaN NaN;CPLgrow(CPL,4)];
H_f = [rotx(90) [0;0;0]; 0 0 0 1];
SG = SGTset(SG,'F',H_f);
if attach_dof ~= 0
    if attach_dof == 'z'
        [SG_connector,CPL_coonector] = SGrotationdisk(attach_servo);
    elseif attach_dof == 'x'
        [SG_connector,CPL_coonector] = SGbracket(attach_servo);
	end
	SG_connection = SGof2CPLsz(CPL_coonector,CPL,10);	
	SG = SGstack2('z',SG_connector,SG_connection,SG);
end



%% Screw Holes
% screw_rad = 1.5;
% SG_hole = SGofCPLz(PLcircle(screw_rad),servo.width+10);
% SG_HOLE = SGofCPLz(PLcircle(screw_rad*2),servo.width);
% SG_hole = SGstackb('z',SG_HOLE,SG_hole,SG_HOLE);
% SG_hole = SGtransrelSG(SG_hole,SG_hole,'roty',pi/2,'centerx');
% SG_hole_1 = SGtransrelSG(SG_hole,SG_hole,'transy',-6,'transz',-4);
% SG_hole_2 = SGtransrelSG(SG_hole_1,SG_hole_1,'transz',-24);
% SG_servo_cage = SGbool('-',SG_top_cage,SG_hole_1);
% SG_servo_cage = SGbool('-',SG_servo_cage,SG_hole_2);
% 



SG = SGtransrelSG(SG,'','alignbottom');

end

%%  [SG] = SGstack(SGs)
%	=== INPUT PARAMETERS ===
%   dir:        Direction of stacking y,x,z
%	SGs:        Array of SGs

%	=== OUTPUT RESULTS ======
%	SG:         SG of stacked SGs
function [SG] = SGstack(dir,varargin)
SG = varargin{1};
for i=2:nargin-1
    switch dir
        case 'z'
            SG = SGcat(SG,SGontop(varargin{i},SG));            
        case 'y'
            SG = SGcat(SG,SGinfront(varargin{i},SG));       
        case 'x'
            SG = SGcat(SG,SGleft(varargin{i},SG));
    end
end
end

%%  [SG] = SGstack(SGs)
%	=== INPUT PARAMETERS ===
%   dir:        Direction of stacking y,x,z
%	SGs:        Array of SGs

%	=== OUTPUT RESULTS ======
%	SG:         SG of stacked SGs
function [SG] = SGstackb(dir,varargin)
SG = varargin{1};
for i=2:nargin-1
    switch dir
        case 'z'
            SG = SGbool('+',SG,SGontop(varargin{i},SG));            
        case 'y'
            SG = SGbool('+',SGinfront(varargin{i},SG));       
        case 'x'
            SG = SGbool('+',SGleft(varargin{i},SG));
    end
end
end

