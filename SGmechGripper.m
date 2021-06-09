function SG = SGmechGripper(varargin)
clf;

SG_object = [];
i_idx = 1;
while i_idx<=size(varargin,2)
	if ~ischar(varargin{i_idx})
		i_idx = i_idx+1; 
		continue; 
	end
	switch varargin{i_idx}
		case 'SG_object'
			SG_object = varargin{i_idx+1};
	end
	i_idx = i_idx+1;
end

main_R = 35;
main_H = 40;

servo_name = 'sm40bl';
conn_type = 'rotLock';

path_depth = 10;
spring_R = 9.6/2;
spring_inner_R = spring_R-2;
spring_guide_R = spring_R+3;
spring_length = 50;
spring_compressed_height = 16;

plunger_guide_W = 5;
axle_R = 3;
axle_oR = axle_R+3;
gripper_attach_R = main_R+20;
gripper_attach_H = (main_H+7) + 40;

servo = readServoFromTable(servo_name);
CPL_screw_pattern = CPLcopyradial(PLcircle(servo.screw_R),servo.mount_screw_R,servo.mount_screw_Num);
CPL_screwhead_pattern = CPLcopyradial(PLcircle(servo.screw_R*2),servo.mount_screw_R,servo.mount_screw_Num);


mid_slot = 2*(servo.mount_screw_R+servo.screw_R);

%% MAINBODY


CPL_main = PLcircle(main_R);

[SG_connector,CPL_connection] = SGconnAdaptersLCL('servo',servo_name,'adapter_type',conn_type,'cable',0);

if conn_type == 'z'
	CPL_connection =  CPLbool('-',CPL_connection,CPL_screw_pattern);
	CPL_2main_connection = CPLbool('-',CPL_main,CPL_screwhead_pattern);
	CPL_connection =  CPLbool('-',CPL_connection,CPL_screwhead_pattern);
else
	CPL_2main_connection = CPL_main;
end


CPL_connection = CPLaddauxpoints(CPL_connection,0.5);
CPL_2main_connection = CPLaddauxpoints(CPL_2main_connection,0.5);
SG_2main_connection = SGof2CPLsz(CPL_connection,CPL_2main_connection,10);

CPL_mid_slot = PLsquare(main_R,mid_slot);

alpha = atan(CPL_mid_slot(1,2)/CPL_mid_slot(1,1));
y = sin(alpha)*main_R-2;
x = cos(alpha)*main_R-2;
CPL_plunger_guide = [CPL_mid_slot(3,:);x y;x 0;CPL_mid_slot(3,1) 0];
CPL_plunger_guide = CPLbool('+',CPL_plunger_guide,VLswapY(CPL_plunger_guide));
CPL_plunger_guide = CPLbool('+',CPL_plunger_guide,VLswapX(CPL_plunger_guide));

CPL_spring_guides = PLtrans(PLcircle(spring_guide_R),[(main_R/2)+spring_guide_R 0]);
CPL_spring_guides = CPLbool('+',CPL_spring_guides,VLswapX(CPL_spring_guides));


CPL_path_lock_guide = [-main_R/2 -(mid_slot/2)-path_depth;-(main_R/2)+2 -main_R;(main_R/2)-2 -main_R;main_R/2 -(mid_slot/2)-path_depth];
CPL_path_lock_guide = CPLbool('+',CPL_path_lock_guide,VLswapY(CPL_path_lock_guide));

CPL_path_slots = PLsquare(main_R-5,mid_slot+2*path_depth);

CPL_path_slot_stops = [mid_slot/2 -main_H/2;mid_slot/2 -(main_H/2)+2;(mid_slot/2)+path_depth -main_H/2];
CPL_path_slot_stops = CPLbool('+',CPL_path_slot_stops,VLswapY(CPL_path_slot_stops));
SG_path_slot_stops = SGofCPLz(CPL_path_slot_stops,main_R-5);



CPL_body = CPLbool('-',CPL_main,CPL_mid_slot);
CPL_body = CPLbool('-',CPL_body,CPL_plunger_guide);
CPL_body = CPLbool('-',CPL_body,CPL_spring_guides);
CPL_body = CPLbool('-',CPL_body,CPL_path_lock_guide);

CPL_top = CPL_body;
CPL_body = CPLbool('-',CPL_body,CPL_path_slots);

SG_main_body = SGofCPLz(CPL_body,main_H);
SG_path_slot_stops = SGtransrelSG(SG_path_slot_stops,SG_main_body,'rotx',pi/2,'rotz',pi/2,'centerx','aligntop');
SG_path_slot_stops = SGcat(SG_path_slot_stops,SGmirror(SG_path_slot_stops,'xz'));
SG_main_body = SGcat(SG_main_body,SG_path_slot_stops);
SG_top_main_body = SGofCPLz(CPL_top,7);


CPL_spring_internal_guides = PLcircle(spring_inner_R);
CPL_spring_internal_guides_chamfer_bot = PLcircle(spring_inner_R+1);
CPL_spring_internal_guides_chamfer_top = PLcircle(spring_inner_R/2);

SG_spring_internal_guides = SGofCPLz(CPL_spring_internal_guides,spring_length-1-(spring_inner_R/2)-10);
SG_spring_internal_guides_cham_bot = SGof2CPLsz(CPL_spring_internal_guides_chamfer_bot,CPL_spring_internal_guides,1);
SG_spring_internal_guides_cham_top = SGof2CPLsz(CPL_spring_internal_guides,CPL_spring_internal_guides_chamfer_top,spring_inner_R/2);


SG_spring_internal_guides = SGstack('z',SG_spring_internal_guides_cham_bot,SG_spring_internal_guides,SG_spring_internal_guides_cham_top);
SG_spring_internal_guides = SGtrans(SG_spring_internal_guides,[(main_R/2)+spring_guide_R 0 0]);
SG_spring_internal_guides = SGcat(SG_spring_internal_guides,SGmirror(SG_spring_internal_guides,'yz'));

SG_main_body = SGcat(SG_main_body,SG_spring_internal_guides);

SG_main_body = SGstack('z',SG_connector,SG_2main_connection,SG_main_body,SG_top_main_body);


CPL_gripper_attach_arms = CPLconvexhull([PLtrans(PLcircle(axle_oR),[gripper_attach_R gripper_attach_H]);x (main_H+7);x 0]);
CPL_gripper_attach_arms = CPLbool('-',CPL_gripper_attach_arms,PLtrans(PLcircle(axle_R),[gripper_attach_R gripper_attach_H]));
 
SG_gripper_attach_arm = SGofCPLz(CPL_gripper_attach_arms,5);
SG_gripper_attach_arm = SGtransrelSG(SG_gripper_attach_arm,SG_main_body,'rotx',pi/2,'transy',-10,'transz',19);
SG_gripper_attach_arm = SGcat(SG_gripper_attach_arm,SGmirror(SG_gripper_attach_arm,'xz'));
SG_gripper_attach_arm = SGcat(SG_gripper_attach_arm,SGmirror(SG_gripper_attach_arm,'yz'));

SG_main_body = SGcat(SG_main_body,SG_gripper_attach_arm);

H_b = [rotz(180) [0;0;0]; 0 0 0 1];
SG_main_body = SGTset(SG_main_body,'B',H_b);

[~,~,mZ] = sizeVL(SG_main_body.VL);

H_f = [rotx(90)*rotz(-40) [gripper_attach_R;0;mZ-axle_oR]; 0 0 0 1];
SG_main_body = SGTset(SG_main_body,'F1',H_f);

H_f = [rotx(90)*rotz(40)*roty(180) [-gripper_attach_R;0;mZ-axle_oR]; 0 0 0 1];
SG_main_body = SGTset(SG_main_body,'F2',H_f);

%% PLUNGER

CPL_plunger_guide_male = [CPL_mid_slot(3,:);x y;x 0;CPL_mid_slot(3,1) 0];
CPL_plunger_guide_male = PLroundcorners(CPL_plunger_guide_male,2,2);
CPL_plunger_guide_male = CPLbuffer(CPL_plunger_guide_male,-0.25);
CPL_plunger_guide_male = CPLbool('+',CPL_plunger_guide_male,VLswapY(CPL_plunger_guide_male));
CPL_plunger_guide_male = CPLbool('+',CPL_plunger_guide_male,VLswapX(CPL_plunger_guide_male));


CPL_outer_spring_guides = PLtrans(PLcircle(spring_guide_R-0.5),[(main_R/2)+spring_guide_R  0]);
CPL_outer_spring_guides = CPLbool('+',CPL_outer_spring_guides,VLswapX(CPL_outer_spring_guides));
CPL_plunger_guide_male = CPLbool('+',CPL_plunger_guide_male,CPL_outer_spring_guides);


CPL_outer_inner_guides = PLtrans(PLcircle(spring_inner_R+0.5),[(main_R/2)+spring_guide_R  0]);
CPL_outer_inner_guides = CPLbool('+',CPL_outer_inner_guides,VLswapX(CPL_outer_inner_guides));
CPL_plunger_guide_male = CPLbool('-',CPL_plunger_guide_male,CPL_outer_inner_guides);

CPL_plunger_guide_top = CPLbool('-',CPL_plunger_guide_male,CPLbool('-',PLsquare(5000,10),PLsquare(2*((main_R/2)-spring_guide_R),10)));

SG_top_plunger = SGofCPLz(CPL_plunger_guide_male,main_H-spring_compressed_height+2.5);
SG_top_plunger_w_cutout =SGofCPLz(CPL_plunger_guide_top,7.5);

CPL_inner_spring_guides = PLtrans(PLcircle(spring_R),[(main_R/2)+spring_guide_R  0]);
CPL_inner_spring_guides = CPLbool('+',CPL_inner_spring_guides,VLswapX(CPL_inner_spring_guides));
CPL_plunger_guide_male = CPLbool('-',CPL_plunger_guide_male,CPL_inner_spring_guides);

SG_plunger_bottom =  SGofCPLz(CPL_plunger_guide_male,spring_compressed_height);
SG_plunger = SGstack('z',SG_plunger_bottom,SG_top_plunger,SG_top_plunger_w_cutout);

CPL_bridge_body = PLroundcorners(PLsquare(main_R+1,10),[1,2],15,[1 -2],0);
CPL_middle_bridge = CPLbool('-',CPL_bridge_body,PLcircle(axle_R));
SG_middle_bridge = SGofCPLz(CPL_middle_bridge,10);
SG_middle_bridge = SGtransrelSG(SG_middle_bridge,SG_plunger,'rotx',pi/2,'aligntop','centery');
SG_plunger = SGcat(SG_plunger,SG_middle_bridge);



CPL_connection_bridge = PLroundcorners(PLsquare(2.25*x,19),[3,4],5);

CPL_connection_bridge_chafer = PLroundcorners(PLsquare(2.25*x,19),[1,2],4,'',0);
CPL_connection_bridge = CPLbool('x',CPL_connection_bridge,CPL_connection_bridge_chafer);

CPL_connection_holes = PLtrans(PLcircle(axle_R),[(1.125*x)-axle_R-3 3]);
CPL_connection_holes = CPLbool('+',CPL_connection_holes,VLswapX(CPL_connection_holes));

CPL_connection_holes_cutout = PLtrans(PLroundcorners(PLsquare((axle_oR)*4,(axle_oR)*3.5),[4],[5],4),[(1.25*x)-axle_R-3 0.5]);
CPL_connection_holes_cutout = CPLbool('+',CPL_connection_holes_cutout,VLswapX(CPL_connection_holes_cutout));

CPL_connection_bridge_outside = CPLbool('-',CPL_connection_bridge,CPL_connection_holes);
CPL_connection_bridge_middle = CPLbool('-',CPL_connection_bridge,CPL_connection_holes_cutout);
SG_connectionbridge_mid = SGofCPLz(CPL_connection_bridge_middle,10);
SG_connectionbridge_outside = SGofCPLz(CPL_connection_bridge_outside,5);
SG_connection_bridge = SGstack('z',SG_connectionbridge_outside,SG_connectionbridge_mid,SG_connectionbridge_outside);

SG_middle_bridge = SGtransrelSG(SG_connection_bridge,SG_plunger,'rotx',pi/2,'ontop','centery');

SG_plunger = SGcat(SG_plunger,SG_middle_bridge);
%% pathrod

length_rod = 40;

CPL_outline = CPLconvexhull([PLcircle(2);PLtrans(PLcircle(axle_R+2),[0 length_rod])]);
CPL_outline=CPLbool('-',CPL_outline,PLtrans(PLcircle(axle_R),[0 length_rod]));
SG_outside = SGofCPLz(CPL_outline,2.5);

CPL_outline_mid = CPLbool('x',CPL_outline,PLcircle(length_rod-15));
SG_outside_mid = SGofCPLz(CPL_outline_mid,10.25);

CPL_pin = PLcircle(2);
CPL_pin_chamfer = PLcircle(1.5);
SG_pin =SGofCPLz(CPL_pin,path_depth-1);
SG_pin_chamfer =SGof2CPLsz(CPL_pin,CPL_pin_chamfer,1);
SG_pin_chamfer_2 =SGof2CPLsz(CPL_pin_chamfer,CPL_pin,1);

SG_pathrod = SGstack('z',SG_pin_chamfer_2,SG_pin,SG_outside,SG_outside_mid,SG_outside,SG_pin,SG_pin_chamfer);



%% PATH
CPL_path = [];
pHeight = (main_H-10)/2;
pWidth = (main_R-11)/2;
positions = [.1 1;.1 .5;1 0;1 -.3;.5 -.7;.5 -.4;.1 -.3;.1 -.5;-.4 -.7;-.4 -.5;-1 0];

path_R = 2.5;
for i=0:size(positions,1)
	x_first = positions(mod(i,size(positions,1))+1,1)*pWidth;
	y_first = positions(mod(i,size(positions,1))+1,2)*pHeight;
	p_first = [x_first,y_first];
	
	x_second = positions(mod(i+1,size(positions,1))+1,1)*pWidth;
	y_second = positions(mod(i+1,size(positions,1))+1,2)*pHeight;
	p_second = [x_second,y_second];
	
	CPL_path_add = CPLconvexhull([PLtrans(PLcircle(path_R),p_first);PLtrans(PLcircle(path_R),p_second)]);
	
	CPL_path = CPLbool('+',CPL_path,CPL_path_add);
end

CPL_path = CPLbool('-',PLsquare(main_R-5,main_H-4),CPL_path);
SG_path = SGofCPLz(CPL_path,path_depth-1);
SG_path_bottom = SGofCPLz(PLsquare(main_R-5,main_H-4),1);


CPL_path_slot_stops_anti = [mid_slot/2 -(main_H/2)+2;(mid_slot/2)+path_depth -(main_H/2)+2;(mid_slot/2)+path_depth -(main_H/2)+0.5];
CPL_path_slot_stops_anti = CPLbool('+',CPL_path_slot_stops_anti,VLswapY(CPL_path_slot_stops_anti));
SG_path_slot_stops_anti = SGofCPLz(CPL_path_slot_stops_anti,main_R-5);
SG_path = SGstack('z',SG_path_bottom,SG_path);
SG_path_slot_stops_anti = SGtransrelSG(SG_path_slot_stops_anti,SG_path,'roty',pi/2,'centerx','aligntop');
SG_path = SGcat(SG_path,SG_path_slot_stops_anti);
SGwriteSTL(SG_path);

%% SGconnector 
length_connector = 20;

CPL_connector_PG = CPLconvexhull([PLcircle(axle_oR);PLtrans(PLcircle(axle_oR),[0 length_connector])]);
CPL_connector_PG = CPLbool('-',CPL_connector_PG,PLcircle(axle_R));
CPL_connector_PG = CPLbool('-',CPL_connector_PG,PLtrans(PLcircle(axle_R),[0 length_connector]));

SG_connector_PG = SGofCPLz(CPL_connector_PG,9.5);
SG_connector_PG = SGtransrelSG(SG_connector_PG,'','centerz');
H_b = [rotz(150) [0;0;0]; 0 0 0 1];
SG_connector_PG = SGTset(SG_connector_PG,'B',H_b);

H_f = [rotz(0) [0;length_connector;0]; 0 0 0 1];
SG_connector_PG = SGTset(SG_connector_PG,'F',H_f);

%% SGgripper 
distance_connection = 17;
l_factor = 4;
CPL_connection_tra = [-axle_oR -axle_oR;axle_oR -axle_oR;axle_oR+3 axle_oR;-axle_oR-3 axle_oR];

CPL_gripper = CPLconvexhull([PLcircle(axle_oR);PLtrans(PLcircle(axle_oR),[0 distance_connection])]);
CPL_gripper_top = CPLconvexhull([PLtrans(PLcircle(axle_oR),[0 distance_connection]);PLtrans(PLcircle(axle_oR),[0 distance_connection*l_factor])]);
CPL_gripper_top = CPLbool('+',CPL_gripper_top,PLtrans(CPL_connection_tra,[0 distance_connection*l_factor]));
CPL_gripper_top = CPLbool('+',CPL_gripper_top,PLtrans(VLswapY(CPL_connection_tra),[0 (distance_connection*l_factor)-2*(axle_oR)]));

CPL_gripper_top = PLtransC(CPL_gripper_top,[0 distance_connection],rot(0.835*pi));

CPL_gripper = CPLbool('+',CPL_gripper,CPL_gripper_top);
CPL_gripper = CPLbool('-',CPL_gripper,PLcircle(axle_R));
CPL_gripper = CPLbool('-',CPL_gripper,PLtrans(PLcircle(axle_R),[0 distance_connection]));

CPL_gripper_mid = CPLconvexhull([PLtrans(PLcircle(axle_oR),[0 distance_connection]);PLtrans(PLcircle(axle_oR),[0 distance_connection*l_factor])]);
CPL_gripper_mid = CPLbool('+',CPL_gripper_mid,PLtrans(CPL_connection_tra,[0 distance_connection*l_factor]));
CPL_gripper_mid = CPLbool('+',CPL_gripper_mid,PLtrans(VLswapY(CPL_connection_tra),[0 (distance_connection*l_factor)-2*(axle_oR)]));
CPL_gripper_mid = CPLbool('-',CPL_gripper_mid,PLtrans(PLcircle(axle_R),[0 distance_connection]));
CPL_gripper_mid = PLtransC(CPL_gripper_mid,[0 distance_connection],rot(0.835*pi));

SG_gripper_sides = SGofCPLz(CPL_gripper,4.75);
SG_gripper_mid = SGofCPLz(CPL_gripper_mid,10);

SG_gripper = SGstack('z',SG_gripper_sides,SG_gripper_mid,SG_gripper_sides);
SG_gripper = SGtransrelSG(SG_gripper,'','centerz');

H_b = [rotz(90) [0;0;0]; 0 0 0 1];
SG_gripper = SGTset(SG_gripper,'F',H_b);

H_f = [rotz(10) [0;distance_connection;0]; 0 0 0 1];
SG_gripper = SGTset(SG_gripper,'B',H_f);

SGwriteSTL(SG_gripper);

%%SGgripper attachment


gripper_height = 50;

PL_gripper_blank = [-gripper_attach_R-20 -20;-gripper_attach_R-20 gripper_height-20;0 gripper_height-20;0 -20];

CPL_attachment_cutout = PLtrans(CPL_connection_tra,[-gripper_attach_R axle_oR]);
CPL_attachment_cutout = CPLbool('+',CPL_attachment_cutout,VLswapY(CPL_attachment_cutout));
CPL_attachment_cutout = CPLbool('+',CPL_attachment_cutout,PLtrans(PLsquare(12,50),[-gripper_attach_R -25]));
CPL_attachment_cutout = CPLbuffer(CPL_attachment_cutout,0.2);

CPL_gripper = PLroundcorners(PL_gripper_blank,[1,2],[10,35]);
CPL_gripper = CPLbool('-',CPL_gripper,CPL_attachment_cutout);
SG_gripper_left = SGtransrelSG(SGofCPLz(CPL_gripper,20),'','rotx',pi/2,'transz',151,'centery');
SG_gripper_right = SGmirror(SG_gripper_left,'yz');

if ~isempty(SG_object)
	SG_gripper_left = SGboolh('-',SG_gripper_left,SG_object);
	SG_gripper_right = SGboolh('-',SG_gripper_right,SG_object);
	
	
end

% 
% CPL_gripper_left = CPLbool('-',CPL_gripper,PLtrans(CPL_object,[0 -151 ]));
% CPL_gripper_right = CPLbool('-',VLswapX(CPL_gripper),PLtrans(CPL_object,[0 -151 ]));

SG_gripper_attachment = SGcat(SG_gripper_left,SG_gripper_right);
% SG_gripper_attachment = SGofCPLz(CPL_gripper,20);

SG_gripper_attachment = SG_gripper_attachment;

% SGplot(SG_gripper_attachment);
%% SGTchains

SGs = {SG_main_body,SG_gripper,SG_connector_PG};
fc = SGTframeChain(1,[1 'F1' 2 'B' 2 'F' 3 'B' 1 'F2' 2 'B' 4 'F' 3 'B']);
% 
SGc = SGTchain(SGs,[0],'',fc);
SGc = SGcat(SGc);


%% PLOTS
% % SGplot(SG_main_body);
% SG_path = SGtransrelSG(SG_path,'','rotx',pi/2,'transz',39,'transy',19.5);
% SGplot(SG_path);
% 
SG_plunger = SGtransrelSG(SG_plunger,'','transz',29);
% SGplot(SG_plunger);

% SGwriteSTL(SG_gripper);
% SGwriteSTL(SG_connector_PG);

SG = SGcat(SGc,SG_plunger);

% SGplot(SG);
end