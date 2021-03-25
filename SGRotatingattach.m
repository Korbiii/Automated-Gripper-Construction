function [SG, CPL] = SGRotatingattach()
clf;
servo_name = lower('SM40BL');
% servo_name = lower('SM85BL');
load Servos;
switch servo_name
	case 'sm40bl'
		servo = Servos.sm40;
	case 'sm85bl'
		servo = Servos.sm85;
	otherwise
		error('Only SM40BL and SM85BL implemented');
end
screw_length = 14-3;

tol = 0.5;
servo.width = servo.width+tol;
servo.length = servo.length+tol;
servo.height = servo.height+tol;


screw_R = 1.5;
screw_head_R = 3; 
outer_radius_ser = 26.5;
outer_radius_ser = max(sqrt(servo.shaft_offs^2+(servo.width/2)^2)+3,max(servo.PL_cable_gap_hor(:,1))+3);
servo_shaft_offs = 11.25;
screw_length = 14;

screw_hor_distance = 16;
screw_ver_distance = 12;

CPL_out = CPLconvexhull([PLcircle(outer_radius_ser);PLgrow(PLroundcorners(servo.PL_cable_gap_hor,[1,2,3,4],Servos.cable_gap/4),1.5);PLtrans([-servo.width/2-5 0;servo.width/2+5 0],[0 -servo.length+servo.shaft_offs-screw_length+5])]);
idx = find(CPL_out(:,2) == min(CPL_out(:,2)));
CPL_out = PLroundcorners(CPL_out,idx',3);
CPL_in = PLtrans(PLsquare(servo.width,servo.length),[0 -((servo.length/2)-servo.shaft_offs)+0.5*tol]);
CPL_in = CPLbool('+',CPL_in,servo.PL_cable_gap_hor);
CPL_in = PLroundcorners(CPL_in,[2,3,4,9,10,11],Servos.cable_gap/4);
CPL_in = CPLbool('+',CPL_in,PLtrans(PLsquare(servo.width-4,servo.length+8),[0 -(((servo.length+8)/2)-servo.shaft_offs)+0.5*tol+4]));

CPL_in_ledge = PLtrans(PLsquare(servo.width,servo.length-4),[0 -(((servo.length-4)/2)-servo.shaft_offs)]);
CPL_in_ledge = CPLbool('+',CPL_in_ledge,servo.PL_cable_gap_hor);

CPL_addon = PLcircseg (outer_radius_ser,'',0,pi);

CPL_in_ledge = PLroundcorners(CPL_in_ledge,[2,3,4,9,10,11],Servos.cable_gap_width/4);
CPL_in_ledge = CPLbool('-',CPL_in_ledge,CPL_addon);
CPL_in_ledge = CPLbool('+',CPL_in_ledge,PLcircle(servo.attach_top_R));

CPL_bottom = [CPL_out;NaN NaN;CPL_in];

if(~isnan(servo.screw_mount_y))
	CPL_screw_cuts = CPLbool('-',PLsquare(screw_hor_distance+2*screw_head_R,servo.length*2),PLsquare(screw_hor_distance-2*screw_head_R,servo.length*2));
	CPLout_w_screw_gaps = CPLbool('-',CPL_bottom,CPL_screw_cuts);
	SG_bottom = SGofCPLz(CPLout_w_screw_gaps,servo.height);
	CPL_screw_inserts_screw =  CPLbool('-',PLsquare(screw_hor_distance+2*screw_head_R,servo.height),PLsquare(screw_hor_distance-2*screw_head_R,servo.height));
	CPL_screw_inserts_screw =  CPLbool('-',CPL_screw_inserts_screw,PLtrans(PLcircle(screw_R),[screw_hor_distance/2 screw_ver_distance/2]));
	CPL_screw_inserts_screw =  CPLbool('-',CPL_screw_inserts_screw,PLtrans(PLcircle(screw_R),[-screw_hor_distance/2 screw_ver_distance/2]));
	CPL_screw_inserts_screw_head =  CPLbool('-',PLsquare(screw_hor_distance+2*screw_head_R,servo.height),PLsquare(screw_hor_distance-2*screw_head_R,servo.height));
	CPL_screw_inserts_screw_head =  CPLbool('-',CPL_screw_inserts_screw_head,PLtrans(PLcircle(screw_head_R),[screw_hor_distance/2 screw_ver_distance/2]));
	CPL_screw_inserts_screw_head =  CPLbool('-',CPL_screw_inserts_screw_head,PLtrans(PLcircle(screw_head_R),[-screw_hor_distance/2 screw_ver_distance/2]));
	
	SG_screw_insert_screw = SGofCPLz(CPL_screw_inserts_screw,5);
	SG_screw_inserts_screw_head = SGofCPLz(CPL_screw_inserts_screw_head,0.01);
	
	SG_screw_inserts = SGstack('z',SG_screw_insert_screw,SG_screw_inserts_screw_head);
	SG_screw_inserts_1 = SGtransrelSG(SG_screw_inserts,SG_bottom,'rotx',pi/2,'aligntop','transy',servo_shaft_offs+0.5*tol-servo.length-4);
	SG_screw_inserts_2 = SGtransrelSG(SG_screw_inserts,SG_bottom,'rotx',pi/2,'rotz',pi,'aligntop','transy',servo_shaft_offs+0.5*tol+4);
	SG_screw_inserts_1 = SGfittoOutsideCPL(SG_screw_inserts_1,CPL_out,'y-');
	SG_screw_inserts_2 = SGfittoOutsideCPL(SG_screw_inserts_2,CPL_out,'y+');
	SG_bottom  = SGcat(SG_screw_inserts_1,SG_screw_inserts_2,SG_bottom);
elseif(~isnan(servo.screw_mount_z))
	CPL_screw_holes = CPLatPL(PLcircle(servo.screw_R),servo.screw_mount_z);
	CPL_in_ledge = CPLbool('+',CPL_in_ledge,PLtrans(CPL_screw_holes,[0 -servo.shaft_offs]));
	SG_bottom = SGofCPLz(CPL_bottom,servo.height);	
end

SG_bottom_w_ledge = SGofCPLz([CPL_out;NaN NaN;CPL_in_ledge],5);
SG = SGstack('z',SG_bottom_w_ledge,SG_bottom);

SGplot(SG);

% %%Lid
% attach_radius = 23.5;
% servo_horn_radius = 11;
% 
% 
% CPL_lid_outline = CPL_out;
% CPL_lid_top = [CPL_lid_outline;NaN NaN;PLcircle(attach_radius)];
% CPL_lid_bottom = [CPL_lid_outline;NaN NaN;PLroundcorners(PLsquare(attach_radius*2-1,servo_horn_radius*2),[1,2,3,4],servo_horn_radius)];
% SG_lid_top = SGstack('z',SGofCPLz(CPL_lid_top,6),SGof2CPLsz(CPL_lid_top,[PLgrow(CPL_lid_outline,1);NaN NaN;PLcircle(attach_radius)],1));
% SG_lid = SGstack('z',SGofCPLz(CPL_lid_bottom,3),SG_lid_top);
% 
% CPL_slide_in_attachments = PLroundcorners(PLsquare(servo.width-4,servo.height-5),[1,2],5);
% CPL_slide_in_attachments = CPLbool('-',CPL_slide_in_attachments,PLtrans(PLcircle(screw_radius),[8 (servo.height-5)/2-11]));
% CPL_slide_in_attachments = CPLbool('-',CPL_slide_in_attachments,PLtrans(PLcircle(screw_radius),[-8 (servo.height-5)/2-11]));
% SG_slide_in_attachments = SGofCPLzchamfer(CPL_slide_in_attachments,3,1);
% SG_slide_in_attachment_1 = SGtransrelSG(SG_slide_in_attachments,SG_lid,'rotx',pi/2,'under',-1,'transy',3+servo_shaft_offs+tol+0.5);
% SG_slide_in_attachment_2 = SGtransrelSG(SG_slide_in_attachments,SG_lid,'rotx',pi/2,'under',-1,'transy',-servo.length+servo_shaft_offs-0.5);
% 
% SG_lid = SGcat(SG_lid,SG_slide_in_attachment_1,SG_slide_in_attachment_2);
% 
% H_b = [rotx(0) [0;0;min_z_bott_con]; 0 0 0 1];
% SG = SGTset(SG,'B',H_b);
% H_f = [roty(-90) [min(SG.VL(:,1));0;max(SG.VL(:,3))-outer_radius_ser]; 0 0 0 1];
% SG = SGTset(SG,'F',H_f);
% 
% 
% H_b_lid = [roty(180)*rotz(-90) [0;0;0]; 0 0 0 1];
% SG_lid = SGTset(SG_lid,'B',H_b_lid);
% H_f_lid = [rotx(0) [0;0;3]; 0 0 0 1];
% SG_lid = SGTset(SG_lid,'F',H_f_lid);
% 
% SGs = {SG,SG_lid};
% SGplot(SG);







end
