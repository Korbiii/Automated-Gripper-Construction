%%   [SGs] = SGgripper(attach_width,max_grip_R,gripper_number,ang,gripper_angles,conn_servo_name)
%    === INPUT PARAMETERS ===
%    attach_width :
%    max_grip_R :
%    gripper_number :
%    ang :
%    gripper_angles :
%    conn_servo_name :
%    === OUTPUT RESULTS ======
%    SGs :
function [SG] = SGgripper()

%%Inputs
attach_width = 20;
max_grip_R =50;

gripper_number = 4;
ang = 360/gripper_number;
gripper_angles =[];
for i = 0 : gripper_number-1
    gripper_angles = [gripper_angles ang*i];
    
end


% 
% gripper_number = 3;
% 
% max_grip_R =55;
% gripper_angles = [160,200,0];
% % 
% gripper_number = 2;
% max_grip_R =55;
% gripper_angles = [20,160,240,300];


conn_type  = 'xy';
conn_servo_name = "sm40bl";
finger_length_low = 65;

finger_tip_length = 50;

clf;
% servo_name = lower(servo_name);
servo_name = "sm40bl";
load Servos;
switch servo_name
	case 'sm40bl'
		servo = Servos.sm40;
	case 'sm85bl'
		servo = Servos.sm85;
	otherwise
		error('Only SM40BL and SM85BL implemented');
end
tol = 0.5;
screw_length = 14-3;
axle_R = 3;


%%Top
outer_radius = 26.5;
distance_axis = servo.length-servo.shaft_offs;
top_height = (axle_R*2+4)+10;
cable_clearance_start = 10;

if conn_type == 'xy'
	[SG_connector,CPL_connector] = SGbracket(conn_servo_name);
elseif conn_type == 'z'
	[SG_connector, CPL_connector] = SGrotationdisk(conn_servo_name);
end


%% Servocage
CPL_servo_outer = PLtrans(PLsquare(servo.width+2*Servos.cable_gap,servo.length+2*(screw_length)),[0,-(servo.length-distance_axis)]);
CPL_grip_attach = [-(attach_width+5)/2 -max_grip_R;(attach_width+5)/2 -max_grip_R;(attach_width+5)/2 0;-(attach_width+5)/2 0];
CPL_grip_attach_test = [-(attach_width+5)/2 -max_grip_R;(attach_width+5)/2 -max_grip_R];
CPL_grip_attach_test = CPLaddauxpoints(CPL_grip_attach_test,0.5);
CPL_outer = [];

for i=1:gripper_number
	is_inside = insideCPS(CPL_servo_outer,PLtransR(CPL_grip_attach_test,rot(deg2rad(gripper_angles(i)))));
	if sum(is_inside(:) == 1) > 0
		CPLplot(CPL_servo_outer,'b');
		CPLplot(PLtransR(CPL_grip_attach,rot(deg2rad(gripper_angles(i)))));
		error("Radius too small for angle "+ i);
	end
	CPL_outer = CPLbool('+',CPL_outer,PLtransR(CPL_grip_attach,rot(deg2rad(gripper_angles(i)))));
end
CPL_outer = CPLbool('+',CPL_servo_outer,CPL_outer);

radii = [];
size_CPL =size(CPL_outer,1);
for i=0:size_CPL
	dist_fw = pdist2(CPL_outer(mod(i,size_CPL)+1,:),CPL_outer(mod(i+1,size_CPL)+1,:));
	dist_bw = pdist2(CPL_outer(mod(i,size_CPL)+1,:),CPL_outer(mod(i-1,size_CPL)+1,:));
	radii(end+1) = min(min(dist_fw,dist_bw)/2,100);
end

CPL_outer = PLroundcorners(CPL_outer,1:size(CPL_outer,1),radii); %%TODO Real RADIUS


for i=1:gripper_number
	CPL_outer = CPLbool('+',CPL_outer,PLtransR(PLroundcorners(CPL_grip_attach,[1,2],2.5),rot(deg2rad(gripper_angles(i)))));
end


% CPL_outer = CPLconvexhull(CPL_outer);
CPL_inner = PLtrans(PLsquare(servo.width+tol,servo.length+tol),[0 -servo.length+distance_axis]);
CPL_inner = CPLbool('+',CPL_inner,PLroundcorners(servo.PL_cable_gap_hor,[1,2,3,4],5));
CPL_top = CPLbool('-',CPL_outer,CPL_inner);


CPL_cut_mask = [-10 -max_grip_R;10 -max_grip_R;10 0;-10 0];
CPL_top_screw_pos = CPL_top;
for i=1:gripper_number
	CPL_top_screw_pos = CPLbool('-',CPL_top_screw_pos,PLtransR(CPL_cut_mask,rot(deg2rad(gripper_angles(i)))));
end
CPL_top_screw_pos = CPLbool('-',CPL_top_screw_pos,PLcircle(servo.connect_R+2*5.5+2));
num_CPls = separateNaN(CPL_top_screw_pos);
screw_pos = [];
for i = 1 : num_CPls
	CPL_temp = separateNaN(CPL_top_screw_pos,i);
	CPL_temp_shrink = CPLbuffer(CPL_temp,-5);
	temp = polyshape(CPL_temp);
	[x,y] = centroid(temp);
	if ~isempty(CPL_temp_shrink)
		if insideCPS(CPL_temp_shrink,[x y]) ~= -1
			screw_pos = [screw_pos;x y];
		else
			distances = pdist2([x y],CPL_temp_shrink);
			[~,idx] = min(distances);
			screw_pos = [screw_pos;CPL_temp_shrink(idx,:)];
		end
	end
	
end
CPL_screw_holes = CPLatPL(PLcircle(1.5),screw_pos);
CPL_top_w_screws = CPLbool('-',CPL_top,CPL_screw_holes);
SG_top = SGofCPLz(CPL_top_w_screws,top_height);

CPL_servocage_bot = CPLbool('+',PLtrans(PLsquare(servo.width+8,servo.length+8),[0,-(servo.length-distance_axis)]),PLgrow(CPL_inner,-2));
CPL_servocage_bot = CPLconvexhull(CPL_servocage_bot);
CPL_servocage_bot = CPLbool('-',CPL_servocage_bot,CPL_inner);
CPL_servocage_bot = CPLaddauxpoints(CPL_servocage_bot,2);

CPL_top = CPLaddauxpoints(CPL_top,2);
SG_bot = SGof2CPLsz(CPL_servocage_bot,CPL_top,servo.height-20+2,'center');
SG_servocage = SGstack('z',SG_bot,SG_top);

CPL_servo_stop = [CPL_inner;NaN NaN; CPLgrow(CPL_inner,2)] ;

SG_servo_stop = SGofCPLz(CPL_servo_stop,2);
SG_servo_stop = SGtransrelSG(SG_servo_stop,SG_servocage,'alignbottom');

%% ================================================================ 3D-FDM
CPL_print_help = CPLconvexhull(CPL_servo_stop);
SG_print_help = SGofCPLz(CPL_print_help,0.4);
SG_print_help = SGtransrelSG(SG_print_help,SG_servo_stop,'aligntop');
SG_servo_stop = SGcat(SG_servo_stop,SG_print_help);
% ================================================================= 3D-FDM

SG_servocage = SGcat(SG_servo_stop,SG_servocage);

CPL_gripp_attach = CPLconvexhull([PLsquare(axle_R*2+4);(axle_R*2+4)/2 -top_height+(axle_R+2)]);
CPL_gripp_attach = [CPL_gripp_attach;NaN NaN;PLcircle(axle_R)];
SG_gripp_attach = SGofCPLz(CPL_gripp_attach,4);
SG_gripp_attach = SGtransrelSG(SG_gripp_attach,SG_servocage,'rotx',pi/2,'rotz',pi/2,'aligntop','transy',-max_grip_R-(axle_R+2)+0.1,'transx',attach_width/2-6);
SG_gripp_attach = SGcat(SG_gripp_attach,SGmirror(SG_gripp_attach,'yz'));

SG_gripp_attachments = [];
for i=1:gripper_number
	SG_gripp_attachments = SGcat(SG_gripp_attachments,SGtransrelSG(SG_gripp_attach,'','rotz',deg2rad(gripper_angles(i))));
end
SG_servocage = SGcat(SG_servocage,SG_gripp_attachments);
SG_connecting = SGof2CPLsz(CPL_connector,CPL_servocage_bot,10,'center');
SG = SGstack('z',SG_connector,SG_connecting,SG_servocage);


height_SG = max(SG.VL(:,3));
for i=1:gripper_number
	pos_gripper = PLtransR([0 -max_grip_R-5],deg2rad(gripper_angles(i)));
	H_f = [roty(90)*rotz(-90)*roty(-gripper_angles(i))*rotz(0) [pos_gripper';height_SG-5]; 0 0 0 1];
	SG = SGTset(SG,append('F',num2str(i)),H_f);
end



%% CPL lid One
CPL_lid_outer = CPLgrow(CPL_outer,-2);
CPL_lid_outer_in = CPLgrow(CPL_outer,-0.2);
CPL_cut_mask = [-500 -max_grip_R;500 -max_grip_R;500 -500;-500 -500];
for i=1:gripper_number
	CPL_lid_outer = CPLbool('-',CPL_lid_outer,PLtransR(CPL_cut_mask,rot(deg2rad(gripper_angles(i)))));
end
CPL_lid_outer_overhang = CPLbool('-',CPL_lid_outer,CPL_lid_outer_in);
CPL_lid_outer = CPLbool('-',CPL_lid_outer,PLcircle(servo.connect_R));
SG_overhang = SGofCPLz(CPL_lid_outer_overhang,3);

CPL_lid_outer = CPLbool('-',CPL_lid_outer,CPL_screw_holes);
SG_lid = SGofCPLz(CPL_lid_outer,1);
SG_lid = SGstack('z',SG_overhang,SG_lid);

CPL_lid_top = CPLbool('-',CPL_lid_outer,PLcircle(servo.connect_R+2));
CPL_guide = PLtrans(PLsquare(28,max_grip_R-4),[0 -max_grip_R/2+2]);
CPL_guide_cutout_back = PLtrans(PLsquare(18,max_grip_R),[0 -max_grip_R/2-0.1]);

CPL_guide_insert = [-14 0;-14 5;-9 5];
SG_guide_insert = SGofCPLz(CPL_guide_insert,max_grip_R-((servo.connect_R+2*5.5+2))-4);
SG_guide_insert = SGtransrelSG(SG_guide_insert,'','rotx',pi/2,'transy',-(servo.connect_R+2*5.5+2),'aligntop');
SG_guide_insert = SGcat(SG_guide_insert,SGmirror(SG_guide_insert,'yz'));
SG_guide_inserts = [];

for i=1:gripper_number
	CPL_lid_top = CPLbool('-',CPL_lid_top,PLtransR(CPL_guide,rot(deg2rad(gripper_angles(i)))));
	CPL_lid_top = CPLbool('-',CPL_lid_top,PLtransR(CPL_guide_cutout_back,rot(deg2rad(gripper_angles(i)))));
	SG_guide_inserts = SGcat(SG_guide_inserts,SGtransrelSG(SG_guide_insert,'','rotz',deg2rad(gripper_angles(i))));
end


CPL_lid_top = CPLbool('-',CPL_lid_top,CPLatPL(PLcircle(3),screw_pos));
CPL_lid_top = CPLbool('-',CPL_lid_top,PLcircle(servo.connect_R+2*5.5+2));

SG_guide = SGofCPLz(CPL_lid_top,5);
SG_guide_inserts = SGtransrelSG(SG_guide_inserts,SG_guide,'aligntop');
SG_guide = SGcat(SG_guide,SG_guide_inserts);
SG_guides = SGtransrelSG(SG_guide,SG_lid,'ontop');
SG_lid = SGcat(SG_lid,SG_guides);

height_SG = max(SG_lid.VL(:,3));
for i=1:gripper_number
	pos_gripper = PLtransR([0 -max_grip_R+10],deg2rad(gripper_angles(i)));
	H_f = [rotz(gripper_angles(i))*roty(180)*rotz(-90) [pos_gripper';height_SG-5]; 0 0 0 1];
	SG_lid = SGTset(SG_lid,append('F',num2str(i)),H_f);
end
%% servorotator
axle_hole_R = 3;

CPL_mid_rotator = PLcircle(servo.connect_R);
CPL_screw_pattern = CPLcopyradial(PLcircle(servo.screw_R),servo.mount_screw_R,servo.mount_screw_Num);
CPL_guide_hole =  PLtrans(PLcircle(3),[0 -servo.connect_R-2-axle_R]);
CPL_guide_hole_out =  PLtrans(PLcircle(5.5),[0 -servo.connect_R-2-axle_R]);
CPL_mid_rotator_c = CPL_mid_rotator;
for i=1:gripper_number
	CPL_temp = CPLconvexhull([CPL_mid_rotator_c;PLtransR(CPL_guide_hole_out,rot(deg2rad(gripper_angles(i))))]);
	CPL_temp_2 = PLtransR(CPL_guide_hole,rot(deg2rad(gripper_angles(i))));
	CPL_mid_rotator = CPLbool('+',CPL_mid_rotator,CPL_temp);
	CPL_mid_rotator = CPLbool('-',CPL_mid_rotator,CPL_temp_2);
	
end
CPL_mid_rotator = CPLbool('-',CPL_mid_rotator,CPL_screw_pattern);
CPL_mid_rotator = CPLbool('-',CPL_mid_rotator,PLcircle(servo.attach_top_R));
SG_mid_rotator = SGofCPLz(CPL_mid_rotator,4);


CPL_mid_rotator_w_through_holes = CPLbool('-',CPL_mid_rotator_c,CPL_screw_pattern);
SG_mid_rotator_top = SGofCPLz(CPLbool('-',CPL_mid_rotator_w_through_holes,PLsquare(servo.attach_top_R*2,500)),7);

CPL_screw_head_pattern = CPLcopyradial(PLcircle(servo.screw_R*2),servo.mount_screw_R,servo.mount_screw_Num);
CPL_mid_rotator_screw_heads = CPLbool('-',CPL_mid_rotator_c,CPL_screw_head_pattern);
SG_mid_rotator_top_screw_heads = SGofCPLz(CPLbool('-',CPL_mid_rotator_screw_heads,PLsquare(servo.attach_top_R*2,500)),2);
SG_mid_rotator = SGstack('z',SG_mid_rotator,SG_mid_rotator_top,SG_mid_rotator_top_screw_heads);

%% Connecting rod
rod_length = max_grip_R-10-servo.connect_R-5.5;
rod_R = 60;
start_point = [sqrt(rod_R^2-(rod_length/2)^2) -rod_length/2];
end_point = [sqrt(rod_R^2-(rod_length/2)^2) rod_length/2];


angle = 2*asin(rod_length/(2*rod_R));
CPL_rod_contour = PLkidney(rod_R-5,rod_R+5,angle);
SG_rod_contour = SGofCPLz(CPL_rod_contour,4);

CPL_rod_connections = CPLatPL(PLcircle(axle_R-0.25),[start_point;end_point]);
CPL_rod_connections_small = CPLatPL(PLcircle(axle_R-1.25),[start_point;end_point]);
CPL_rod_connections_big = CPLatPL(PLcircle(axle_R+1.25),[start_point;end_point]);

SG_bottom_chamfer = SGof2CPLsz(CPL_rod_connections_small,CPL_rod_connections,1);
SG_connection_body = SGofCPLz(CPL_rod_connections,2);
SG_top_chamfer = SGof2CPLsz(CPL_rod_connections,CPL_rod_connections_big,1);

SG_rod_connections = SGstack('z',SG_bottom_chamfer,SG_connection_body,SG_top_chamfer,SG_rod_contour);

H_b = [rotz(90)*rotx(180) [start_point';-1]; 0 0 0 1];
SG_rod_connections = SGTset(SG_rod_connections,'B',H_b);

H_f = [rotz(90)*rotx(180) [end_point';-1]; 0 0 0 1];
SG_rod_connections = SGTset(SG_rod_connections,'F',H_f);

%% läufer in guide

CPL_runner_vertical = [PLcircle(axle_R+2);NaN NaN;PLcircle(axle_R)];
CPL_runner_ver2hor = CPLbool('-',PLtrans(PLsquare(axle_R+2,4.75),[(axle_R+2)/2 4.75/2-(axle_R+2)]),PLcircle(axle_R+2));
SG_runner_vertical = SGofCPLz(CPL_runner_vertical,7.5);
SG_runner_ver2hor = SGofCPLz(CPL_runner_ver2hor,7.5);
SG_runner_vertical = SGcat(SG_runner_vertical,SG_runner_ver2hor);
CPL_runner_hor = [PLsquare(18,7.5);NaN NaN;PLcircle(axle_R)];
SG_runner_hor = SGofCPLz(CPL_runner_hor,4.75);

CPL_runner_cut = [0 0;0 4.75;4.75 0];
CPL_runner_tri = [0 0;0 0.01;0.01 0];
CPL_front_tri = [0 0;2 0;0 4.75];
SG_runner_1 = SGof2CPLsz(CPL_runner_cut,CPL_runner_tri,2);
SG_runner_2 = SGofCPLz(CPL_runner_cut,7.5/2);
SG_runner_sides = SGstack('z',SG_runner_2,SG_runner_1);
SG_runner_sides = SGcat(SG_runner_sides,SGmirror(SG_runner_sides,'xy'));
SG_runner_sides = SGtransrelSG(SG_runner_sides,SG_runner_hor,'rotx',pi/2,'transx',7.5/2,'right');
SG_runner_sides = SGcat(SG_runner_sides,SGmirror(SG_runner_sides,'yz'));
SG_runner_hor = SGcat(SG_runner_sides,SG_runner_hor);
SG_front_tri = SGofCPLz(CPL_front_tri,18);
SG_back_tri = SGofCPLz(CPL_front_tri,9-3.75);

SG_runner_hor = SGtransrelSG(SG_runner_hor,SG_runner_vertical,'rotx',-pi/2,'roty',-pi/2,'centerz','aligninfront','right',-2);
SG_front_tri = SGtransrelSG(SG_front_tri,SG_runner_hor,'centerz','aligninfront','right',-2);
SG_back_tri = SGtransrelSG(SG_back_tri,SG_runner_hor,'roty',pi,'centery','left',-2);
SG_back_tri_2 = SGtransrelSG(SG_back_tri,SG_runner_hor,'aligntop',-4.75);

SG_runner = SGcat(SG_runner_vertical,SG_runner_ver2hor,SG_runner_hor,SG_front_tri,SG_back_tri,SG_back_tri_2);
SG_runner = SGtransrelSG(SG_runner,'','centerz');

H_b = [rotx(-90) [axle_R+2+7.5/2;-5;0]; 0 0 0 1];
SG_runner = SGTset(SG_runner,'B',H_b);

H_f = [rotz(180) [0;0;0]; 0 0 0 1];
SG_runner = SGTset(SG_runner,'F',H_f);

%% Finger lower part
attachment_point = 27;

CPL_lower_finger = CPLconvexhull([PLcircle(axle_hole_R+2);PLtrans(PLcircle(axle_hole_R+2),[0 finger_length_low])]);
CPL_lower_finger = CPLbool('+',CPL_lower_finger,PLtrans(PLsquare(15+(axle_hole_R+2),2*(axle_hole_R+2)),[(axle_hole_R+2) finger_length_low]));
CPL_lower_finger = PLroundcorners(CPL_lower_finger,20,5);
CPL_lower_finger_chamfer = CPLbool('+',CPL_lower_finger,PLtrans(PLsquare(15+(axle_hole_R+2),2*(axle_hole_R+2)+5),[(axle_hole_R+2) finger_length_low-2.5]));
CPL_lower_finger_chamfer = PLroundcorners(CPL_lower_finger_chamfer,20,5);
CPL_lower_finger = CPLbool('-',CPL_lower_finger,PLcircle(axle_hole_R));
CPL_lower_finger = CPLbool('-',CPL_lower_finger,PLtrans(PLcircle(axle_hole_R),[0 finger_length_low]));

CPL_lower_finger_chamfer = CPLbool('-',CPL_lower_finger_chamfer,PLcircle(axle_hole_R));
CPL_lower_finger_chamfer = CPLbool('-',CPL_lower_finger_chamfer,PLtrans(PLcircle(axle_hole_R),[0 finger_length_low]));

SG_lower_finger = SGof2CPLsz(CPL_lower_finger,CPL_lower_finger_chamfer,5,'center');

CPL_lower_finger_mid = CPLconvexhull([PLtrans(PLcircle(axle_hole_R+2),[0 attachment_point]);PLtrans(PLcircle(axle_hole_R+2),[0 finger_length_low-2*(axle_hole_R+2)])]);
SG_lower_finger_mid = SGofCPLz(CPL_lower_finger_mid,8);

CPL_cover = [-(axle_hole_R+2) 2;-(axle_hole_R+2) finger_length_low-(axle_hole_R+2);15 finger_length_low-(axle_hole_R+2);25 8];
CPL_cover = PLroundcorners(CPL_cover,[1,4],[10 20]);
CPL_cover_small = CPLbuffer(CPL_cover,-4);
SG_cover = SGof2CPLsz(CPL_cover,CPL_cover_small,4);

CPL_spring_attachment = PLtubbing(2,4,'',-pi/2,pi/2);
SG_spring_attachment = SGofCPLz(CPL_spring_attachment,5);
SG_spring_attachment = SGtransrelSG(SG_spring_attachment,'','rotx',pi/2,'transx',axle_hole_R+2,'transy',attachment_point+axle_hole_R+2);

SG_lower_finger = SGstack('z',SG_lower_finger_mid,SG_lower_finger,SG_cover);
SG_lower_finger = SGcat(SG_lower_finger,SGmirror(SG_lower_finger,'xy'),SG_spring_attachment);

H_b = [rotz(190) [0;0;0]; 0 0 0 1];
SG_lower_finger = SGTset(SG_lower_finger,'B',H_b);

H_f = [rotz(0) [0;finger_length_low;0]; 0 0 0 1];
SG_lower_finger = SGTset(SG_lower_finger,'F',H_f);

%% Finger tip

PL_attach_positons = [0 0;-15 -10];

CPL_finger_tip_back_attach = CPLconvexhull([CPLatPL(PLcircle(axle_hole_R+2),PL_attach_positons);axle_hole_R+2 8;-12 8]);
CPL_finger_tip_front_attach = CPLconvexhull([PLcircle(axle_hole_R+2);axle_hole_R+2 8;-12 8]);
CPL_finger_tip_front_attach = CPLbool('-',CPL_finger_tip_front_attach,PLcircle(axle_hole_R));
CPL_finger_tip_back_attach = CPLbool('-',CPL_finger_tip_back_attach,CPLatPL(PLcircle(axle_hole_R),PL_attach_positons));
SG_finger_tip_front_attach = SGofCPLz(CPL_finger_tip_front_attach,2);
SG_finger_tip_back_attach = SGofCPLz(CPL_finger_tip_back_attach,4);

SG_finger_tip_attach = SGstack('z',SG_finger_tip_front_attach,SG_finger_tip_back_attach,SG_finger_tip_front_attach);
SG_finger_tip_attach = SGcat(SG_finger_tip_attach,SGmirror(SG_finger_tip_attach,'xy'));

CPL_finger_tip_1 = PLsquare(16,axle_hole_R+2+12);
CPL_finger_tip_2 = PLtrans(PLsquare(12,12),[0 -((axle_hole_R+2+12)-12)/2]);

alpha_ratio = atan((4)/(finger_tip_length-10));
x_ratio = tan(alpha_ratio)*10;

CPL_finger_tip_3 = PLtrans(PLsquare(12-x_ratio,2),[0 -((axle_hole_R+2+12)-2)/2]);

SG_finger_tip_1 = SGof2CPLz(CPL_finger_tip_1,CPL_finger_tip_2,finger_tip_length-10);
SG_finger_tip_2 = SGof2CPLz(CPL_finger_tip_2,CPL_finger_tip_3,10);
SG_finger_tip = SGstack('z',SG_finger_tip_1,SG_finger_tip_2);
SG_finger_tip = SGtransrelSG(SG_finger_tip,SG_finger_tip_attach,'rotx',-pi/2,'roty',pi/2,'behind','transx',-3.5);

SG_finger_top = SGcat(SG_finger_tip,SG_finger_tip_attach);

H_b = [rotz(0) [0;0;0]; 0 0 0 1];
SG_finger_top = SGTset(SG_finger_top,'B',H_b);

H_f = [rotz(0) [PL_attach_positons(2,:)';0]; 0 0 0 1];
SG_finger_top = SGTset(SG_finger_top,'F',H_f);

%%  Finger2horguide rod
finger2hor_length = 35;

CPL_finger2horguide = CPLconvexhull([PLtrans(PLcircle(axle_R+2),[0 finger2hor_length/2]);PLtrans(PLcircle(axle_R+2),[0 -finger2hor_length/2])]);
CPL_finger2horguide = CPLbool('-',CPL_finger2horguide,PLtrans(PLcircle(axle_R),[0 finger2hor_length/2]));
CPL_finger2horguide = CPLbool('-',CPL_finger2horguide,PLtrans(PLcircle(axle_R),[0 -finger2hor_length/2]));

CPL_finger2horguide_mid = CPLbool('-',CPL_finger2horguide,PLtrans(PLcircle(axle_R+3),[0 -finger2hor_length/2]));
PL_cut = PLtransR(PLsquare(30-axle_R+2,2*(axle_R+2),2*(axle_R+2)),rot(pi/2));
CPL_finger2horguide_mid = CPLbool('-',CPL_finger2horguide_mid,PLtrans(PL_cut,[0 (axle_R+2)+4]));

SG_finger2horguide = SGofCPLz(CPL_finger2horguide,3.75);
SG_finger2horguide_mid = SGofCPLz(CPL_finger2horguide_mid,8);

SG_finger2horguide = SGstack('z',SG_finger2horguide,SG_finger2horguide_mid,SG_finger2horguide);
SG_finger2horguide = SGtransrelSG(SG_finger2horguide,'','centerz');

H_b = [rotz(-90) [0;-finger2hor_length/2;0]; 0 0 0 1];
SG_finger2horguide = SGTset(SG_finger2horguide,'B',H_b);

H_f = [rotz(0) [0;finger2hor_length/2;0]; 0 0 0 1];
SG_finger2horguide = SGTset(SG_finger2horguide,'F',H_f);
%% Finger_back_bone_bottom
bb_bottom_length = 40;
bb_bottom_conn_point = 30;

CPL_bb_bottom = CPLconvexhull([PLcircle(axle_R+2);PLtrans(PLcircle(axle_R+2),[0 bb_bottom_length])]);

CPL_bb_bottom = CPLbool('-',CPL_bb_bottom,PLtrans(PLcircle(axle_R),[0 0]));
CPL_bb_bottom = CPLbool('-',CPL_bb_bottom,PLtrans(PLcircle(axle_R),[0 bb_bottom_length]));
CPL_bb_bottom = CPLbool('-',CPL_bb_bottom,PLtrans(PLcircle(axle_R),[0 bb_bottom_conn_point]));

CPL_bb_bottom_mid = CPLbool('-',CPL_bb_bottom,PLtrans(PLcircle(axle_R+4),[0 bb_bottom_length]));

SG_bb_bottom = SGofCPLz(CPL_bb_bottom,1.875);
SG_bb_bottom_mid = SGofCPLz(CPL_bb_bottom_mid,4.25);
SG_bb_bottom = SGstack('z',SG_bb_bottom,SG_bb_bottom_mid,SG_bb_bottom);
SG_bb_bottom = SGtransrelSG(SG_bb_bottom,'','centerz');
H_b = [rotz(-90) [0;bb_bottom_length;0]; 0 0 0 1];
SG_bb_bottom = SGTset(SG_bb_bottom,'B',H_b);

H_f = [rotz(0) [0;0;0]; 0 0 0 1];
SG_bb_bottom = SGTset(SG_bb_bottom,'F',H_f);
%% Finger_back_bone_top
bb_top_length = 40;

CPL_bb_top = PLtrans(PLsquare(5,bb_top_length),[2.5 0]);
CPL_bb_top = CPLbool('+',CPL_bb_top,PLtrans(PLcircle(axle_R+2),[0 bb_top_length/2]));
CPL_bb_top = CPLbool('+',CPL_bb_top, PLtrans(PLcircle(axle_R+2),[0 -bb_top_length/2]));
CPL_bb_top = CPLbool('+',CPL_bb_top, PLtrans(PLcircle(3),[0 bb_top_length/2-20]));

CPL_bb_top = CPLbool('-',CPL_bb_top,PLtrans(PLcircle(axle_R),[0 bb_top_length/2]));
CPL_bb_top = CPLbool('-',CPL_bb_top, PLtrans(PLcircle(axle_R),[0 -bb_top_length/2]));
CPL_bb_top = CPLbool('-',CPL_bb_top, PLtrans(PLcircle(1.5),[0 bb_top_length/2-20]));
SG_bb_top = SGofCPLz(CPL_bb_top,4);

H_b = [rotz(-30) [0;bb_top_length/2;2]; 0 0 0 1];
SG_bb_top = SGTset(SG_bb_top,'B',H_b);

H_f = [rotz(0) [0;-bb_top_length/2;2]; 0 0 0 1];
SG_bb_top = SGTset(SG_bb_top,'F',H_f);
%% STLWRITE
SGwriteSTL(SG);

%% Plots
fc2 = {};
SG_eles = {SG};
for i = 1 : gripper_number
	fc2{end+1} = 1;
	fc2{end+1} = append('F',num2str(i));
	fc2{end+1} = 2+(4*(i-1));
	fc2{end+1} = 'B';
	fc2{end+1} = 2+(4*(i-1));
	fc2{end+1} = 'F';
	fc2{end+1} = 3+(4*(i-1));
	fc2{end+1} = 'B';	
	fc2{end+1} = 3+(4*(i-1));
	fc2{end+1} = 'F';
	fc2{end+1} = 4+(4*(i-1));
	fc2{end+1} = 'B';
	fc2{end+1} = 4+(4*(i-1));
	fc2{end+1} = 'F';
	fc2{end+1} = 5+(4*(i-1));
	fc2{end+1} = 'B';	
	SG_eles{end+1} = SG_lower_finger;
	SG_eles{end+1} = SG_finger_top;
	SG_eles{end+1} = SG_bb_top;
	SG_eles{end+1} = SG_bb_bottom;
	
end
fc = SGTframeChain(1,fc2);
SGc = SGTchain(SG_eles,[0],'',fc);
SGplot(SGc);


SG_lid = SGtransrelSG(SG_lid,SG,'ontop',-3);
fc2 = {};
SG_eles = {SG_lid};
for i = 1 : gripper_number
	fc2{end+1} = 1;
	fc2{end+1} = append('F',num2str(i));
	fc2{end+1} = 2+(3*(i-1));
	fc2{end+1} = 'B';
	fc2{end+1} = 2+(3*(i-1));
	fc2{end+1} = 'B';
	fc2{end+1} = 3+(3*(i-1));
	fc2{end+1} = 'F';
	fc2{end+1} = 2+(3*(i-1));
	fc2{end+1} = 'F';
	fc2{end+1} = 4+(3*(i-1));
	fc2{end+1} = 'B';	
	SG_eles{end+1} = SG_runner;
	SG_eles{end+1} = SG_rod_connections;
	SG_eles{end+1} = SG_finger2horguide;
end
fc = SGTframeChain(1,fc2);
SGc = SGTchain(SG_eles,[0],'',fc);
SGplot(SGc);

SG_mid_rotator = SGtransrelSG(SG_mid_rotator,SG_lid,'ontop',-4);
SGplot(SG_mid_rotator);
end



