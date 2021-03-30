function [SG, CPL] = SGrotationdisk(servo_name)
load Servos;
switch servo_name
	case 'sm40bl'
		servo = Servos.sm40;
	case 'sm85bl'
		servo = Servos.sm85;
	case 'sm120bl'
		servo = Servos.sm120;
end


%% Connection
tol = 0.5;
outer_radius_ser = max(sqrt(servo.shaft_offs^2+(servo.width/2)^2)+3,max(servo.PL_cable_gap_hor(:,1))+3);
connection_R = outer_radius_ser-3-tol;
CPL_connection = PLcircle(connection_R);
CPL_screw_holes = CPLcopyradial(PLcircle(servo.screw_R),servo.mount_screw_R,servo.mount_screw_Num);
CPL_cable_slot = PLkidney(servo.mount_screw_R+servo.screw_R+2,servo.mount_screw_R+servo.screw_R+2+6,3/2*pi);
CPL_connection = CPLbool('-',CPL_connection,CPL_screw_holes);
CPL_connection = CPLbool('-',CPL_connection,CPL_cable_slot);
SG = SGofCPLz(CPL_connection,9);

CPL = [PLcircle(connection_R);NaN NaN;PLcircle(servo.mount_screw_R+servo.screw_R+2+6)];
min_z_bott_con = min(SG.VL(:,3));
H_b = [rotx(0) [0;0;min_z_bott_con]; 0 0 0 1];
SG = SGTset(SG,'B',H_b);


end