function [SG, SG_lid, CPL] = SGRotatingattach(servo_name)
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
tol = 0.5;
screw_length = 14-3;

servo.width = servo.width+tol;
servo.length = servo.length+tol;
servo.height = servo.height+tol;

outer_radius_ser = max(sqrt(servo.shaft_offs^2+(servo.width/2)^2)+3,max(servo.PL_cable_gap_hor(:,1))+3);

CPL_out = CPLconvexhull([PLcircle(outer_radius_ser);PLgrow(PLroundcorners(servo.PL_cable_gap_hor,[1,2,3,4],Servos.cable_gap/4),1.5);PLtrans([-servo.width/2-5 0;servo.width/2+5 0],[0 -servo.length+servo.shaft_offs-screw_length+2])]);
idx = find(CPL_out(:,2) == min(CPL_out(:,2)));
CPL_out = PLroundcorners(CPL_out,idx',3);
CPL_in = PLtrans(PLsquare(servo.width,servo.length),[0 -((servo.length/2)-servo.shaft_offs)+0.5*tol]);
CPL_in = CPLbool('+',CPL_in,servo.PL_cable_gap_hor);
CPL_in = PLroundcorners(CPL_in,[2,3,4,9,10,11],Servos.cable_gap/4);
CPL_in = CPLbool('+',CPL_in,PLtrans(PLsquare(servo.width-4,servo.length+8),[0 -(((servo.length+8)/2)-servo.shaft_offs)+0.5*tol+4]));

CPL_in_ledge = PLtrans(PLsquare(servo.width,servo.length-4),[0 -(((servo.length-4)/2)-servo.shaft_offs)]);
CPL_in_ledge = CPLbool('+',CPL_in_ledge,servo.PL_cable_gap_hor);

CPL_addon = PLcircseg (outer_radius_ser,'',0,pi);
CPL_addon = CPLbool('-',CPL_addon,PLroundcorners(servo.PL_cable_gap_hor,[1,2,3,4],Servos.cable_gap/4));

CPL_in_ledge = PLroundcorners(CPL_in_ledge,[2,3,4,9,10,11],Servos.cable_gap/4);
CPL_in_ledge = CPLbool('-',CPL_in_ledge,CPL_addon);
CPL_in_ledge = CPLbool('+',CPL_in_ledge,PLcircle(servo.attach_top_R));
CPL_in_ledge_wo_screws = CPL_in_ledge;

CPL_bottom = [CPL_out;NaN NaN;CPL_in];

if(~isnan(servo.screw_mount_y))
	screw_hor_distance = max(servo.screw_mount_y(2,:))*2;  % TODO: Make it better
	screw_ver_distance = max(servo.screw_mount_y(1,:))*2;
	CPL_screw_cuts = CPLbool('-',PLsquare(screw_hor_distance+2*(servo.attach_screw_R*2),servo.length*2),PLsquare(screw_hor_distance-2*(servo.attach_screw_R*2),servo.length*2));
	CPLout_w_screw_gaps = CPLbool('-',CPL_bottom,CPL_screw_cuts);
	SG_bottom = SGofCPLz(CPLout_w_screw_gaps,servo.height);
	CPL_screw_inserts_screw =  CPLbool('-',PLsquare(screw_hor_distance+2*(servo.attach_screw_R*2),servo.height),PLsquare(screw_hor_distance-2*(servo.attach_screw_R*2),servo.height));
	CPL_screw_inserts_screw =  CPLbool('-',CPL_screw_inserts_screw,PLtrans(PLcircle(servo.attach_screw_R),[screw_hor_distance/2 screw_ver_distance/2]));
	CPL_screw_inserts_screw =  CPLbool('-',CPL_screw_inserts_screw,PLtrans(PLcircle(servo.attach_screw_R),[-screw_hor_distance/2 screw_ver_distance/2]));
	CPL_screw_inserts_screw_head =  CPLbool('-',PLsquare(screw_hor_distance+2*(servo.attach_screw_R*2),servo.height),PLsquare(screw_hor_distance-2*(servo.attach_screw_R*2),servo.height));
	CPL_screw_inserts_screw_head =  CPLbool('-',CPL_screw_inserts_screw_head,PLtrans(PLcircle((servo.attach_screw_R*2)),[screw_hor_distance/2 screw_ver_distance/2]));
	CPL_screw_inserts_screw_head =  CPLbool('-',CPL_screw_inserts_screw_head,PLtrans(PLcircle((servo.attach_screw_R*2)),[-screw_hor_distance/2 screw_ver_distance/2]));
	
	SG_screw_insert_screw = SGofCPLz(CPL_screw_inserts_screw,5);
	SG_screw_inserts_screw_head = SGofCPLz(CPL_screw_inserts_screw_head,0.01);
	
	SG_screw_inserts = SGstack('z',SG_screw_insert_screw,SG_screw_inserts_screw_head);
	SG_screw_inserts_1 = SGtransrelSG(SG_screw_inserts,SG_bottom,'rotx',pi/2,'aligntop','transy',servo.shaft_offs+0.5*tol-servo.length-4);
	SG_screw_inserts_2 = SGtransrelSG(SG_screw_inserts,SG_bottom,'rotx',pi/2,'rotz',pi,'aligntop','transy',servo.shaft_offs+0.5*tol+4);
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
CPL_out_conn = CPLbool('-',CPL_out,PLcircseg(outer_radius_ser+1,'',0,pi));
CPL = CPLconvexhull([CPLgrow(CPL_in_ledge_wo_screws,-2);CPL_out_conn]);
CPL = [CPL;NaN NaN;CPL_in_ledge_wo_screws];

%%Lid
 CPL_lid_outline = CPL_out;
CPL_lid_top = [CPL_lid_outline;NaN NaN;PLcircle(outer_radius_ser-3)];
CPL_lid_top_chamfer = [PLgrow(CPL_lid_outline,1);NaN NaN;PLcircle(outer_radius_ser-3)];
CPL_lid_bottom = [CPL_lid_outline;NaN NaN;PLroundcorners(PLsquare((outer_radius_ser-3)*2-1,servo.connect_R*2),[1,2,3,4],servo.connect_R)];

if(~isnan(servo.screw_mount_z))
	CPL_screw_holes = PLtrans(CPLatPL(PLcircle(servo.screw_R),servo.screw_mount_z),[0 -servo.shaft_offs]);
	CPL_lid_top = CPLbool('-',CPL_lid_top,CPL_screw_holes);
	CPL_lid_top_chamfer= CPLbool('-',CPL_lid_top_chamfer,CPL_screw_holes);
	CPL_lid_bottom= CPLbool('-',CPL_lid_bottom,CPL_screw_holes);
end

SG_lid_top = SGstack('z',SGofCPLz(CPL_lid_top,6),SGof2CPLsz(CPL_lid_top,CPL_lid_top_chamfer,1));
SG_lid = SGstack('z',SGofCPLz(CPL_lid_bottom,3),SG_lid_top);

CPL_slide_in_attachments = PLroundcorners(PLsquare(servo.width-4,servo.height-5),[1,2],5);
CPL_slide_in_attachments_small = PLroundcorners(PLsquare(servo.width-6,servo.height-7),[1,2],5);
CPL_slide_in_attachments_small = PLtrans(CPL_slide_in_attachments_small,[0 1]);

if(~isnan(servo.screw_mount_y))	
	CPL_screw_holes = PLtrans(CPLatPL(PLcircle(servo.attach_screw_R),servo.screw_mount_y),[0 -2.5]);
	CPL_slide_in_attachments = CPLbool('-',CPL_slide_in_attachments,CPL_screw_holes);
	CPL_slide_in_attachments_small = CPLbool('-',CPL_slide_in_attachments_small,CPL_screw_holes);
end
SG_slide_in_attachments = SGof2CPLsz(CPL_slide_in_attachments,CPL_slide_in_attachments_small,1.5);
SG_slide_in_attachments = SGcat(SG_slide_in_attachments,SGmirror(SG_slide_in_attachments,'xy'));
SG_slide_in_attachment_1 = SGtransrelSG(SG_slide_in_attachments,SG_lid,'rotx',pi/2,'under','transy',3+servo.shaft_offs+tol+0.5);
SG_slide_in_attachment_2 = SGtransrelSG(SG_slide_in_attachments,SG_lid,'rotx',pi/2,'under','transy',-servo.length+servo.shaft_offs-0.5);

SG_lid = SGcat(SG_lid,SG_slide_in_attachment_1,SG_slide_in_attachment_2);
H_b_lid = [roty(180)*rotz(-90) [0;0;0]; 0 0 0 1];
SG_lid = SGTset(SG_lid,'B',H_b_lid);
H_f_lid = [rotx(0) [0;0;3]; 0 0 0 1];
SG_lid = SGTset(SG_lid,'F',H_f_lid);

H_f = [roty(-90) [0;0;max(SG.VL(:,3))]; 0 0 0 1];
SG = SGTset(SG,'F',H_f);
end