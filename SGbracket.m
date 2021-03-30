function [SG, CPL] = SGbracket(servo_name)
load Servos;
switch servo_name
	case 'sm40bl'
		servo = Servos.sm40;
	case 'sm85bl'
		servo = Servos.sm85;
	case 'sm120bl'
		servo = Servos.sm120;
end
screw_length = 14-3;

CPL_screw_holes = CPLcopyradial(PLcircle(servo.screw_R),servo.mount_screw_R,servo.mount_screw_Num);
cable_gap_R = sqrt(servo.shaft_offs^2+(servo.width/2)^2);
CPL_bracket = CPLconvexhull([PLcircle(servo.width/2+10);-(servo.width/2+10) cable_gap_R+10;(servo.width/2+10) cable_gap_R+10]);
CPL_bracket_small = CPLconvexhull([PLcircle(servo.width/2+8);-(servo.width/2+8) cable_gap_R+10;(servo.width/2+8) cable_gap_R+10]);;
CPL_bracket = CPLbool('-',CPL_bracket,CPL_screw_holes);
CPL_bracket_small = CPLbool('-',CPL_bracket_small,CPL_screw_holes);

CPL_bracket_w_cut = CPLbool('-',CPL_bracket,PLtrans(PLroundcorners(PLsquare(servo.attach_top_R*2,servo.width+10),[1,2,3,4],servo.attach_top_R/2),[0 -10]));

SG_bracket_w_cut = SGofCPLz(CPL_bracket_w_cut,servo.attach_top_H);
SG_bracket = SGof2CPLsz(CPL_bracket,CPL_bracket_small,2);
missing_screw_length = screw_length -2 - servo.attach_top_H;
SG_screw_length = SGof2CPLsz([PLcircle(servo.mount_screw_R+2*servo.screw_R+missing_screw_length);NaN NaN;CPL_screw_holes],[PLcircle(servo.mount_screw_R+2*servo.screw_R);NaN NaN;CPL_screw_holes],missing_screw_length);

SG = SGstack('z',SG_bracket_w_cut,SG_bracket,SG_screw_length);
SG_bracket_left = SGtransrelSG(SG,SG,'rotx',pi/2,'transy',-servo.connect_dis/2);
SG_bracket_right = SGtransrelSG(SG,SG,'rotx',pi/2,'rotz',pi,'transy',servo.connect_dis/2);
SG = SGcat(SG_bracket_right,SG_bracket_left);

H_b = [rotx(90) [0;0;0]; 0 0 0 1];
SG = SGTset(SG,'B',H_b);

% SGTplot(SG);
CPL = CPLconvexhull(CPLofSGslice(SG,max(SG.VL(:,3))-0.01));
gap_bracket = screw_length-missing_screw_length;
CPL = [CPL;NaN NaN;PLsquare(2*max(CPL(:,1)-5),2*max(CPL(:,2)-gap_bracket-5))];


end