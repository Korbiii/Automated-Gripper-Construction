function [SG] = SGparrallelGripper()
clf;
tol=0.5;
thread_length = 12;
printhelp =1;
servo_name = 'sm40bl';

gripper_number = 2;
max_grip_R = 47;


gripper_angles = [0,180];

conn_servo_name = 'sm40bl';
conn_type  = 'rotLock';

servo = readServoFromTable(servo_name);

servo.width = servo.width+tol;
servo.height = servo.height+tol;

CPL_servo = PLtrans(PLsquare(servo.width,servo.length),[0 -servo.shaft_offs]);
CPL_cable_slot_hor = PLtrans(servo.PL_cable_gap_hor,[0 -servo.shaft_offs]);
CPL_cable_slot_hor = PLroundcorners(CPL_cable_slot_hor,[1,2,3,4],servo.cable_gap-2);
CPL_servo_w_hor_cablegap = PLtrans(CPLbool('+',PLsquare(servo.width+servo.cable_gap,servo.length),PLroundcorners(servo.PL_cable_gap_hor,[1,2,3,4],servo.cable_gap)),[0 -servo.shaft_offs]);

%% Main Body
main_R = sqrt((servo.width/2)^2+((servo.length/2)+servo.shaft_offs)^2)+2;

CPL_main_bottom_layer = PLcircle(main_R);
CPL_main_bottom_layer = CPLbool('-',CPL_main_bottom_layer,CPL_cable_slot_hor);

CPL_main_bottom_layer_w_slot = CPLbool('-',CPL_main_bottom_layer,CPLconvexhull([PLcircle(5);PLtrans(PLcircle(5),[0 500])]));

SG_bottom_layer = SGofCPLz(CPL_main_bottom_layer,2);
SG_bottom_layer_w_wlot = SGofCPLz(CPL_main_bottom_layer_w_slot,3);
SG_bottom_layer = SGstack('z',SG_bottom_layer,SG_bottom_layer_w_wlot);
CPL_servo_channel = PLtrans(PLsquare(servo.width+servo.cable_gap,5000),[0 2500]);
CPL_main_mid = CPLbool('-',PLcircle(main_R),CPL_servo_w_hor_cablegap);
CPL_main_mid = CPLbool('-',CPL_main_mid,CPL_servo_channel);


out = (servo.width+servo.cable_gap)/2;
CPL_big_dove = [-out main_R;out main_R;out+2 (servo.length/2)-servo.shaft_offs;-out-2 (servo.length/2)-servo.shaft_offs];



CPL_main_mid = CPLbool('-',CPL_main_mid,CPL_big_dove);

min_y_cslot = min(CPL_cable_slot_hor(:,2));
min_y_servo = min(CPL_servo(:,2));

CPL_cable_channel_insert = CPLbool('-',PLsquare(servo.width+servo.cable_gap,servo.height),PLsquare(servo.width,servo.height));
SG_insert_wo_cable_slot = SGofCPLz(CPL_cable_channel_insert,abs(min_y_cslot-min_y_servo));


CPL_hor_servo_slot = PLroundcorners(servo.PL_cable_gap_ver,[1,2,3,4],servo.cable_gap,'',0);
CPL_cable_channel_insert = CPLbool('-',CPL_cable_channel_insert,CPL_hor_servo_slot);

SG_mid_layer = SGofCPLz(CPL_main_mid,servo.height);

max_y_cslot = max(CPL_cable_slot_hor(:,2));
max_y_servo = max(CPL_servo(:,2));

SG_insert = SGofCPLz(CPL_cable_channel_insert,abs(max_y_cslot-max_y_servo));
SG_insert = SGtransrelSG(SG_insert,SG_mid_layer,'rotx',pi/2,'alignbottom','transy',-servo.shaft_offs-servo.length/2+servo.length);

SG_insert_wo_cable_slot = SGtransrelSG(SG_insert_wo_cable_slot,SG_mid_layer,'rotx',pi/2,'alignbottom','transy',(-servo.shaft_offs-servo.length/2)+abs(min_y_cslot-min_y_servo));

SG_mid_layer= SGcat(SG_mid_layer,SG_insert,SG_insert_wo_cable_slot);


CPL_insertslot = CPLconvexhull([PLcircle(servo.connect_R);PLtrans(PLcircle(servo.connect_R),[0 500])]);
CPL_top_layer = CPLbool('-',PLcircle(main_R),CPL_insertslot);
CPL_top_layer_chamfer = CPLbool('-',PLcircle(main_R-2),CPL_insertslot);

CPL_top_layer_chamfer = CPLbool('-',CPL_top_layer_chamfer,CPL_big_dove);
CPL_top_layer = CPLbool('-',CPL_top_layer,CPL_big_dove);


SG_top_layer = SGof2CPLsz(CPL_top_layer,CPL_top_layer_chamfer,2);

% PRINT HELP
if printhelp
	CPL_printhelp = CPLconvexhull(CPL_top_layer_chamfer);	
	CPL_printhelp = CPLbool('-',CPL_printhelp,CPL_big_dove);
	SG_printhelp = SGofCPLz(CPL_printhelp,0.3);
	SG_top_layer = SGcat(SG_top_layer,SGalignbottom(SG_printhelp,SG_printhelp));
end
% PRINTHELP


SG = SGstack('z',SG_bottom_layer,SG_mid_layer,SG_top_layer);



CPL_guide = [7 -5;6 0;-2.5 0;-2.5 5;-6 5;-9.75 -5];
CPL_double_dove = [-1.5 0;-2.5 2.5];
CPL_double_dove = CPLconvexhull([CPL_double_dove;VLswapX(CPL_double_dove)]);
CPL_double_dove = CPLbool('+',CPL_double_dove,VLswapY(CPL_double_dove));
CPL_guide = CPLbool('-',CPL_guide,PLtrans(CPL_double_dove,[0 -2.5]));

SG_guide = SGofCPLzdelaunayGrid(CPL_guide,0.5,0.5);
SG_guide=SGtransrelSG(SG_guide,SG,'rotx',pi/2,'transy',0.5,'transx',-out-9,'ontop');
SG_guide = SGfittoOutsideCPL(SG_guide,PLcircle(main_R-2),'y+');
SG_guide = SGcat(SG_guide,SGmirror(SG_guide,'yz'));
SG_guide = SGcat(SG_guide,SGmirror(SG_guide,'xz'));

SG = SGcat(SG,SG_guide);

[SG_connector,CPL_connection] = SGconnAdaptersLCL('servo',servo_name,'adapter_type',conn_type,'cable',1);
CPL_connection = CPLaddauxpoints(CPL_connection,.5);
CPL_main_bottom_layer = CPLaddauxpoints(CPL_main_bottom_layer,.5);
SG_2main_connection = SGof2CPLsz(CPL_connection,CPL_main_bottom_layer,10);


SG = SGstack('z',SG_connector,SG_2main_connection,SG);
CPL_insert = CPLbuffer(CPL_big_dove,-0.2);
CPL_insert = CPLbool('x',CPL_insert,PLcircle(main_R));
SG_insert = SGofCPLz(CPL_insert,36.5);

%% Gear

CPL_gear = PLgearDIN(1,round(servo.connect_R*2.5));
CPL_gear = CPLbool('-',CPL_gear,PLcircle(servo.connect_R));
CPL_screw_pattern_TH = CPLcopyradial(PLcircle(servo.attach_screw_R),servo.mount_screw_R,servo.mount_screw_Num);
CPL_screw_pattern_HH = CPLcopyradial(PLcircle(servo.attach_screw_R*2),servo.mount_screw_R,servo.mount_screw_Num);
CPL_middle_TH = CPLbool('-',PLcircle(servo.connect_R),CPL_screw_pattern_TH);
CPL_middle_TH = CPLbool('-',CPL_middle_TH,PLcircle(servo.attach_top_R));
CPL_middle_HH = CPLbool('-',PLcircle(servo.connect_R),CPL_screw_pattern_HH);

SG_gear = SGofCPLz(CPL_gear,10);
SG_gear_mid_TH = SGofCPLz(CPL_middle_TH,thread_length-3);
SG_gear_mid_HH = SGofCPLz(CPL_middle_HH,10-(thread_length-3));
SG_gear_mid = SGstack('z',SG_gear_mid_TH,SG_gear_mid_HH);
SG_gear = SGcat(SG_gear,SGalignbottom(SG_gear_mid,SG_gear));
SGwriteSTL(SG_gear);
%% Gripper


CPL_double_dove_small = CPLbuffer(CPL_double_dove,-0.2);
CPL_add_to_doves = [-2.2 2.3;2.2 2.3;2.2 2.7;8 2.7;11 -.5;11 9.5;-2.2 9.5];
CPL_double_dove_small = CPLbool('+',CPL_double_dove_small,CPL_add_to_doves);
CPL_holes = CPLatPL(PLcircle(1.6),[2 6;7 6]);
CPL_double_dove_small = CPLbool('-',CPL_double_dove_small,CPL_holes);
SG_gripper_guide = SGtransrelSG(SGofCPLz(CPL_double_dove_small,60),'','rotx',pi/2,'transx',-out-9);

CPL_gear_rack = PLgearrackDIN(1,60.01);
SG_gear_rack = SGofCPLz(CPL_gear_rack,10);
SG_gear_rack = SGtransrelSG(SG_gear_rack,SG_gripper_guide,'rotz',-pi/2,'aligntop','right','alignback');
SG_gripper_guide = SGcat(SG_gripper_guide,SG_gear_rack);

CPL_gripper_act = [-29.7 0;30 0;30 -10;-29.7 -10];

SG_gripper_act = SGofCPLz(CPL_gripper_act,30);

SG_gripper_guide= SGcat(SG_gripper_guide,SGontop(SG_gripper_act,SG_gripper_guide));
SGwriteSTL(SG_gripper_guide);
%%

 SGplot(SGaligntop(SG_gripper_guide,SG,-2));
 SGplot(SGaligntop(SG_gear,SG,-1));
 SGplot(SG);
% SGwriteSTL(SG);
% SGwriteSTL(SG_insert);


end