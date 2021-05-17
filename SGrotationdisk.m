function [SG, CPL] = SGrotationdisk(servo_name,varargin)


cable_slot = 1; if nargin>=2 && ~isempty(varargin{1}); cable_slot=varargin{1}; end
print_help_layer = 0; if nargin>=3 && ~isempty(varargin{2}); print_help_layer=varargin{2}; end
servo = readServoFromTable(servo_name);

%% Connection
tol = 0.5;
outer_radius_ser = max(sqrt(servo.shaft_offs^2+(servo.width/2)^2)+3,max(servo.PL_cable_gap_hor(:,1))+3);
connection_R = outer_radius_ser-1.5-tol;
CPL_connection = PLcircle(connection_R);
CPL_screw_holes = CPLcopyradial(PLcircle(servo.screw_R),servo.mount_screw_R,servo.mount_screw_Num);
if cable_slot
	CPL_cable_slot = PLkidney(servo.mount_screw_R+servo.screw_R+2,servo.mount_screw_R+servo.screw_R+2+6,1.25*pi);
	CPL_connection = CPLbool('-',CPL_connection,CPL_cable_slot);
end

CPL_connection = CPLbool('-',CPL_connection,CPL_screw_holes);
CPL_connection_mid_attach = CPLbool('-',CPL_connection,PLcircle(servo.attach_top_R+0.5));
SG_bottom = SGofCPLz(CPL_connection_mid_attach,servo.attach_top_H+0.5);
SG = SGofCPLz(CPL_connection,9-servo.attach_top_H);
SG= SGstack('z',SG_bottom,SG);

if print_help_layer
	SG_help_layer = SGofCPLz(PLcircle(connection_R),0.3);
	SG = SGcat(SG,SGaligntop(SG_help_layer,SG));
end

if cable_slot
	CPL = [PLcircle(connection_R);NaN NaN;PLcircle(servo.mount_screw_R+servo.screw_R+2+6)];
else
	CPL = PLcircle(connection_R);
end

min_z_bott_con = min(SG.VL(:,3));
H_b = [rotx(0) [0;0;min_z_bott_con]; 0 0 0 1];
SG = SGTset(SG,'B',H_b);


end