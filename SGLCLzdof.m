function [SG,CPL] = SGLCLzdof(servo_name,varargin)
clf;
servo_name = lower(servo_name);
servo = readServoFromTable(servo_name);

attach_dof = 0; if nargin>=2 && ~isempty(varargin{1}); attach_dof=varargin{1}; end
attach_servo = 'sm40bl'; if nargin>=3 && ~isempty(varargin{2}); attach_servo=varargin{2}; end

tol = 0.5;
screw_length = 14-3;

%% Servomount
distance_axis = (servo.length/2)+servo.shaft_offs;
servo.width = servo.width+tol;
servo.length = servo.length+tol;
servo.height = servo.height+tol;
outer_radius = servo.width/2+servo.cable_gap+3;
start_cable_gap = servo.height/2+min(servo.PL_cable_gap_ver(:,2));
cable_gap_length = max(servo.PL_cable_gap_ver(:,2))-min(servo.PL_cable_gap_ver(:,2));

CPL_outside = CPLconvexhull([PLcircle(outer_radius);[-outer_radius,-distance_axis;outer_radius,-distance_axis]]);
CPL_outside = CPLbool('-',CPL_outside,CPLconvexhull([PLcircle(servo.connect_R);[-servo.connect_R,30;servo.connect_R,30]]));

CPL_outside_small = CPLconvexhull([PLcircle(outer_radius-3);[-outer_radius+3,-distance_axis;outer_radius-3,-distance_axis]]);
CPL_outside_small= CPLbool('-',CPL_outside_small,CPLconvexhull([PLcircle(servo.connect_R);[-servo.connect_R,30;servo.connect_R,30]]));

CPL_outside_w_servo_slot = CPLbool('-',CPL_outside,PLsquare(servo.width,servo.length*2));
CPL_outside_w_cable_slots = CPLbool('-',CPL_outside,PLsquare(servo.width+2*servo.cable_gap,servo.length*2));



CPL_outside_w_screw_slots = CPL_outside_w_servo_slot;


if(~isnan(servo.screw_mount_x))
	t_min = min(servo.PL_cable_gap_ver(:,2));
	t_max = max(servo.PL_cable_gap_ver(:,2));
	avail_screw_mounts = (servo.screw_mount_x(:,1) > t_max | servo.screw_mount_x(:,1) < t_min);
	avail_screw_mounts = servo.screw_mount_x(avail_screw_mounts,:);
	shaft_mid_Dis = servo.length/2-servo.shaft_offs;
	bot = min(avail_screw_mounts(:,2))-servo.screw_R;
	top = max(avail_screw_mounts(:,2))+servo.screw_R;
	CPL_outside_w_screw_slots = CPLbool('-',CPL_outside_w_screw_slots,PLtrans(PLsquare(outer_radius*2,top-bot),[0 -servo.shaft_offs+t_max-tol]));
	
elseif(~isnan(servo.screw_mount_z))
	%%TODO
end

SG_front = SGof2CPLsz(CPL_outside_small,CPL_outside,3);
SG_1 = SGofCPLz(CPL_outside_w_screw_slots,start_cable_gap);
SG_2 = SGofCPLz(CPL_outside_w_cable_slots,cable_gap_length);
SG_3 = SGofCPLz(CPL_outside_w_screw_slots,servo.height-cable_gap_length-start_cable_gap);
SG_back = SGof2CPLsz(CPL_outside,CPL_outside_small,3);

if(~isnan(servo.screw_mount_x))
	
	CPL_screw_TH = CPLatPL(PLcircle(servo.attach_screw_R),avail_screw_mounts);
	CPL_servo_side = PLsquare(servo.height,servo.length);
	CPL_servo_side = CPLbool('-',CPL_servo_side,CPL_screw_TH);	
	
	CPL_servo_side = CPLbool('-',CPL_servo_side,[5000 top; 5000 top+5000;-5000 top+5000;-5000 top]);
	CPL_servo_side = CPLbool('-',CPL_servo_side,[5000 bot; 5000 bot-5000;-5000 bot-5000;-5000 bot]);	
	
	CPL_cable_slot = [servo.PL_cable_gap_ver(:,2) servo.PL_cable_gap_ver(:,1)];
	
		
	CPL_screw_HH = CPLatPL(PLcircle(servo.attach_screw_R*2),avail_screw_mounts);
	CPL_servo_side_HH = PLsquare(servo.height,servo.length);
	CPL_servo_side_HH = CPLbool('-',CPL_servo_side_HH,CPL_screw_HH);	
	
	CPL_servo_side_HH = CPLbool('-',CPL_servo_side_HH,[5000 top; 5000 top+5000;-5000 top+5000;-5000 top]);
	CPL_servo_side_HH = CPLbool('-',CPL_servo_side_HH,[5000 bot; 5000 bot-5000;-5000 bot-5000;-5000 bot]);	
	
	left = max(CPL_cable_slot(:,1));
	CPL_servo_side_right = CPLbool('-',CPL_servo_side,[left -5000;left 5000;left-5000 5000;left-5000 -5000]);
	CPL_servo_side_HH_right = CPLbool('-',CPL_servo_side_HH,[left -5000;left 5000;left-5000 5000;left-5000 -5000]);
	
	right = min(CPL_cable_slot(:,1));
	CPL_servo_side_left = CPLbool('-',CPL_servo_side,[right -5000;right 5000;right+5000 5000;right+5000 -5000]);
	CPL_servo_side_HH_left = CPLbool('-',CPL_servo_side_HH,[right -5000;right 5000;right+5000 5000;right+5000 -5000]);
	
	SG_screw_TH = SGofCPLz(CPL_servo_side_right,screw_length-3);
	SG_screw_HH = SGofCPLz(CPL_servo_side_HH_right,3);
	SG_screw_TH = SGstack('z',SG_screw_TH,SG_screw_HH);
	SG_screw_TH = SGtransrelSG(SG_screw_TH,SG_3,'roty',-pi/2,'aligntop','alignleft','transy',-servo.shaft_offs+tol);
	SG_screw_TH = SGcat(SG_screw_TH,SGmirror(SG_screw_TH,'yz'));1
    SG_3 = SGcat(SG_3,SG_screw_TH);
	
	SG_screw_TH = SGofCPLz(CPL_servo_side_left,screw_length-3);
	SG_screw_HH = SGofCPLz(CPL_servo_side_HH_left,3);
	SG_screw_TH = SGstack('z',SG_screw_TH,SG_screw_HH);
	SG_screw_TH = SGtransrelSG(SG_screw_TH,SG_1,'roty',-pi/2,'aligntop','alignleft','transy',-servo.shaft_offs+tol);
	SG_screw_TH = SGcat(SG_screw_TH,SGmirror(SG_screw_TH,'yz'));
    SG_1 = SGcat(SG_1,SG_screw_TH);
	
	
	
	
	
	
	
	
	
	
% 	CPL_screws_small = PLsquare(servo.height-cable_gap_length-start_cable_gap,top-bot);
% 	CPL_screws_small = PLtrans(CPL_screws_small,[0 -(top-bot)+servo.shaft_offs-tol]);
% 	
% % 	screw_hols = PLtrans(CPLatPL(PLcircle(servo.screw_R),servo.screw_mount_x),[0 0])
% 	SG_screws_small = SGofCPLz(CPL_screws_small,screw_length);
% 	SG_screws_small = SGtransrelSG(SG_screws_small,SG_3,'roty',pi/2,'transy',-shaft_mid_Dis-top/2,'transx',servo.width/2,'aligntop');
elseif(~isnan(servo.screw_mount_z))
	%%TODO
end


SG = SGstack('z',SG_front,SG_1,SG_2,SG_3,SG_back);
SG = SGtransrelSG(SG,'','rotx',pi/2,'centery');

CPL_slice = CPLofSGslice(SG,min(SG.VL(:,2)));
CPL_out = CPLconvexhull(CPL_slice);
CPL_inner = CPLbool('+',PLsquare(servo.width,servo.height-10),VLswapY(servo.PL_cable_gap_ver));
CPL = [CPL_out;NaN NaN;CPL_inner];
CPL = CPLaddauxpoints(CPL,0.5);
H_f = [rotx(90) [0;0;0]; 0 0 0 1];
SG = SGTset(SG,'F',H_f);
if attach_dof ~= 0
    if attach_dof == 'z'
        [SG_connector,CPL_connector] = SGrotationdisk(attach_servo);
    elseif attach_dof == 'x'
        [SG_connector,CPL_connector] = SGbracket(attach_servo);
	end	
	SG_connection = SGof2CPLsz(CPL_connector,CPL,10,'center');	
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

SGplot(SG);
SGwriteSTL(SG);

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

