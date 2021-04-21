function SG = SGmechGripper()
clf;
main_R = 30;
main_H = 40;

servo_name = 'sm40bl';

mid_slot = 16;
path_depth = 10;
spring_guide_R = 5;
spring_inner_R = 1.6;
plunger_guide_W = 5;
spring_length = 30;
axle_R = 3;
gripper_attach_R = main_R+20;
gripper_attach_H = (main_H+7) + 20;

%% MAINBODY

CPL_main = PLcircle(main_R);

CPL_mid_slot = PLsquare(main_R,mid_slot);

CPL_path_slots = PLsquare(main_R-10,mid_slot+2*path_depth);

CPL_path_lock_guide = [-main_R/2 -(mid_slot/2)-path_depth;-(main_R/2)+2 -main_R;(main_R/2)-2 -main_R;main_R/2 -(mid_slot/2)-path_depth];
CPL_path_lock_guide = CPLbool('+',CPL_path_lock_guide,VLswapY(CPL_path_lock_guide));

CPL_spring_guides = PLtrans(PLcircle(spring_guide_R),[(main_R/2)+spring_guide_R 0]);
CPL_spring_guides = CPLbool('+',CPL_spring_guides,VLswapX(CPL_spring_guides));

CPL_plunger_guide = [(main_R/2) 0;(main_R/2) mid_slot/2;(main_R/2)-4 (mid_slot/2)+6;(main_R/2)+plunger_guide_W+4 (mid_slot/2)+6;(main_R/2)+plunger_guide_W (mid_slot/2);(main_R/2)+plunger_guide_W 0];
CPL_plunger_guide = CPLbool('+',CPL_plunger_guide,VLswapX(CPL_plunger_guide));
CPL_plunger_guide = CPLbool('+',CPL_plunger_guide,VLswapY(CPL_plunger_guide));

CPL_spring_internal_guides = PLcircle(spring_inner_R);
CPL_spring_internal_guides_chamfer_bot = PLcircle(spring_inner_R+1);
CPL_spring_internal_guides_chamfer_top = PLcircle(spring_inner_R-1);


CPL_body = CPLbool('-',CPL_main,CPL_path_lock_guide);
CPL_body = CPLbool('-',CPL_body,CPL_mid_slot);
CPL_body = CPLbool('-',CPL_body,CPL_spring_guides);
CPL_body = CPLbool('-',CPL_body,CPL_plunger_guide);

CPL_top = CPL_body;
CPL_body = CPLbool('-',CPL_body,CPL_path_slots);

CPL_gripper_attach_arms = CPLconvexhull([PLtrans(PLcircle(axle_R+2),[gripper_attach_R gripper_attach_H]);main_R (main_H+7);main_R 0]);
CPL_gripper_attach_arms = CPLbool('-',CPL_gripper_attach_arms,PLtrans(PLcircle(axle_R),[gripper_attach_R gripper_attach_H]));

SG_bottom = SGofCPLz(CPL_main,2);

SG_spring_internal_guides = SGofCPLz(CPL_spring_internal_guides,spring_length);
SG_spring_internal_guides_cham_bot = SGof2CPLsz(CPL_spring_internal_guides_chamfer_bot,CPL_spring_internal_guides,1);
SG_spring_internal_guides_cham_top = SGof2CPLsz(CPL_spring_internal_guides,CPL_spring_internal_guides_chamfer_top,1);
SG_spring_internal_guides = SGstack('z',SG_spring_internal_guides_cham_bot,SG_spring_internal_guides,SG_spring_internal_guides_cham_top);
SG_spring_internal_guides = SGtrans(SG_spring_internal_guides,[(main_R/2)+spring_guide_R 0 0]);
SG_spring_internal_guides = SGcat(SG_spring_internal_guides,SGmirror(SG_spring_internal_guides,'yz'));

SG_body = SGofCPLz(CPL_body,main_H);
SG_body = SGcat(SG_body,SG_spring_internal_guides);
SG_top = SGofCPLz(CPL_top,5);

SG_main_body = SGstack('z',SG_bottom,SG_body,SG_top);

SG_gripper_attach_arm = SGofCPLz(CPL_gripper_attach_arms,5);
SG_gripper_attach_arm = SGtransrelSG(SG_gripper_attach_arm,SG_main_body,'rotx',pi/2,'transy',-10);
SG_gripper_attach_arm = SGcat(SG_gripper_attach_arm,SGmirror(SG_gripper_attach_arm,'xz'));
SG_gripper_attach_arm = SGcat(SG_gripper_attach_arm,SGmirror(SG_gripper_attach_arm,'yz'));

SG_main_body = SGcat(SG_main_body,SG_gripper_attach_arm);

[SG_connector,CPL_connection] = SGrotatingdiskwithoutservo(servo_name);
SG_connection = SGof2CPLsz(CPL_connection,CPL_main,10);
SG_main_body = SGstack('z',SG_connector,SG_connection,SG_main_body);

%% PLUNGER




%% PLOTS
SGplot(SG_main_body);



end