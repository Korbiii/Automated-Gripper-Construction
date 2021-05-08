function SG = SGmechGripper()
clf;
main_R = 30;
main_H = 40;

servo_name = 'sm40bl';

path_depth = 10;
spring_guide_R = 5;
spring_inner_R = 1.6;
spring_R = 2.5;
plunger_guide_W = 5;
spring_length = 30;
axle_R = 3;
gripper_attach_R = main_R+20;
gripper_attach_H = (main_H+7) + 20;

servo = readServoFromTable(servo_name);
CPL_screw_pattern = CPLcopyradial(PLcircle(servo.screw_R),servo.mount_screw_R,servo.mount_screw_Num);
CPL_screwhead_pattern = CPLcopyradial(PLcircle(servo.screw_R*2),servo.mount_screw_R,servo.mount_screw_Num);


mid_slot = 2*(servo.mount_screw_R+servo.screw_R);

%% MAINBODY


CPL_main = PLcircle(main_R);

[SG_connector,CPL_connection] =  SGrotationdisk(servo_name,0);
CPL_connection =  CPLbool('-',CPL_connection,CPL_screw_pattern);

CPL_2main_connection = CPLbool('-',CPL_main,CPL_screwhead_pattern);
CPL_connection =  CPLbool('-',CPL_connection,CPL_screwhead_pattern);
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
CPL_spring_internal_guides_chamfer_top = PLcircle(spring_inner_R-1);

SG_spring_internal_guides = SGofCPLz(CPL_spring_internal_guides,spring_length);
SG_spring_internal_guides_cham_bot = SGof2CPLsz(CPL_spring_internal_guides_chamfer_bot,CPL_spring_internal_guides,1);
SG_spring_internal_guides_cham_top = SGof2CPLsz(CPL_spring_internal_guides,CPL_spring_internal_guides_chamfer_top,1);
SG_spring_internal_guides = SGstack('z',SG_spring_internal_guides_cham_bot,SG_spring_internal_guides,SG_spring_internal_guides_cham_top);
SG_spring_internal_guides = SGtrans(SG_spring_internal_guides,[(main_R/2)+spring_guide_R 0 0]);
SG_spring_internal_guides = SGcat(SG_spring_internal_guides,SGmirror(SG_spring_internal_guides,'yz'));

SG_main_body = SGcat(SG_main_body,SG_spring_internal_guides);

SG_main_body = SGstack('z',SG_connector,SG_2main_connection,SG_main_body,SG_top_main_body);


CPL_gripper_attach_arms = CPLconvexhull([PLtrans(PLcircle(axle_R+2),[gripper_attach_R gripper_attach_H]);x (main_H+7);x 0]);
CPL_gripper_attach_arms = CPLbool('-',CPL_gripper_attach_arms,PLtrans(PLcircle(axle_R),[gripper_attach_R gripper_attach_H]));
 
SG_gripper_attach_arm = SGofCPLz(CPL_gripper_attach_arms,5);
SG_gripper_attach_arm = SGtransrelSG(SG_gripper_attach_arm,SG_main_body,'rotx',pi/2,'transy',-10,'transz',19);
SG_gripper_attach_arm = SGcat(SG_gripper_attach_arm,SGmirror(SG_gripper_attach_arm,'xz'));
SG_gripper_attach_arm = SGcat(SG_gripper_attach_arm,SGmirror(SG_gripper_attach_arm,'yz'));

SG_main_body = SGcat(SG_main_body,SG_gripper_attach_arm);

%% PLUNGER
CPL_plunger_guide_male = CPLbuffer(CPL_plunger_guide,-0.5);
CPL_outer_spring_guides = PLtrans(PLcircle(spring_guide_R-0.5),[(main_R/2)+spring_guide_R  0]);
CPL_outer_spring_guides = CPLbool('+',CPL_outer_spring_guides,VLswapX(CPL_outer_spring_guides));
CPL_plunger_guide_male = CPLbool('+',CPL_plunger_guide_male,CPL_outer_spring_guides);

SG_top_plunger = SGofCPLz(CPL_plunger_guide_male,main_H-spring_length+10);

CPL_inner_spring_guides = PLtrans(PLcircle(spring_R),[(main_R/2)+spring_guide_R  0]);
CPL_inner_spring_guides = CPLbool('+',CPL_inner_spring_guides,VLswapX(CPL_inner_spring_guides));
CPL_plunger_guide_male = CPLbool('-',CPL_plunger_guide_male,CPL_inner_spring_guides);

SG_plunger_bottom =  SGofCPLz(CPL_plunger_guide_male,spring_length);
SG_plunger = SGstack('z',SG_plunger_bottom,SG_top_plunger);

CPL_bridge_body = PLroundcorners(PLsquare(main_R+1,10),[1,2],15,[1 -2]);
CPL_middle_bridge = CPLbool('-',CPL_bridge_body,PLcircle(axle_R));
SG_middle_bridge = SGofCPLz(CPL_middle_bridge,10);
SG_middle_bridge = SGtransrelSG(SG_middle_bridge,SG_plunger,'rotx',pi/2,'aligntop','centery');
SG_plunger = SGcat(SG_plunger,SG_middle_bridge);

CPL_connection_bridge = PLroundcorners(PLsquare(2*x,14),[3,4],5);
CPL_connection_holes = PLtrans(PLcircle(axle_R),[x-axle_R-3 1]);
CPL_connection_holes = CPLbool('+',CPL_connection_holes,VLswapX(CPL_connection_holes));

CPL_connection_holes_cutout = PLtrans(PLroundcorners(PLsquare((axle_R+4)*2),[1,2,4],[5,3,5],[-2,4]),[x-axle_R-3 1]);
CPL_connection_holes_cutout = CPLbool('+',CPL_connection_holes_cutout,VLswapX(CPL_connection_holes_cutout));

CPL_connection_bridge_outside = CPLbool('-',CPL_connection_bridge,CPL_connection_holes);
CPL_connection_bridge_middle = CPLbool('-',CPL_connection_bridge,CPL_connection_holes_cutout);
SG_connectionbridge_mid = SGofCPLz(CPL_connection_bridge_middle,10);
SG_connectionbridge_outside = SGofCPLz(CPL_connection_bridge_outside,5);
SG_connection_bridge = SGstack('z',SG_connectionbridge_outside,SG_connectionbridge_mid,SG_connectionbridge_outside);

SG_middle_bridge = SGtransrelSG(SG_connection_bridge,SG_plunger,'rotx',pi/2,'ontop','centery');

SG_plunger = SGcat(SG_plunger,SG_middle_bridge);

% SGplot(SG_plunger);
% 
%% PATH
CPL_path = [];
pHeight = (main_H-10)/2;
pWidth = (main_R-11)/2;
positions = [.1 1;.1 .5;1 0;1 -.7;.5 -1;.5 -.7;.1 -.5;.1 -.7;-.4 -1;-.4 -.7;-1 0];

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


%% PLOTS
SGplot(SG_main_body);
SG_path = SGtransrelSG(SG_path,'','rotx',pi/2,'transz',39,'transy',19.5);
SGplot(SG_path);

SG_plunger = SGtransrelSG(SG_plunger,'','transz',20);
SGplot(SG_plunger);




end