function [SG, CPL] = SGrotatingdiskwithoutservo(servo_name)
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
CPL_connection = CPLbool('-',CPL_connection,CPL_screw_holes);
SG_bottom = SGofCPLz(CPL_connection,9);

minimal_screw_gap = max(servo.mount_screw_R*sin((2*pi)/servo.mount_screw_Num/2)-2*servo.screw_R,1); 
new_angle = asin(minimal_screw_gap/servo.mount_screw_R);
new_widht = connection_R* sin(new_angle);

CPL_triangle = [0 0;-new_widht connection_R;new_widht connection_R];
CPL_triangle = CPLbool('x',CPL_triangle,PLcircle(connection_R));
CPL_triangle = PLtransR(CPL_triangle,rot((1/servo.mount_screw_Num)*(pi)));
CPL_triangles = CPL_triangle;
for i=1:servo.mount_screw_Num
    CPL_triangles = CPLbool('+',CPL_triangles, PLtransR(CPL_triangle,rot(((2*pi)/servo.mount_screw_Num)*i)));    
end
CPL_mid = CPLbool('+',CPL_triangles,PLcircle(servo.mount_screw_R-2*servo.screw_R));
SG_mid = SGofCPLz(CPL_mid,20);

SG_top = SGofCPLz(PLcircle(connection_R),2);

SG = SGstack('z',SG_bottom,SG_mid,SG_top);

min_z_bott_con = min(SG.VL(:,3));
H_b = [rotx(0) [0;0;min_z_bott_con]; 0 0 0 1];
SG = SGTset(SG,'B',H_b);

CPL = PLcircle(connection_R);


end