function [SG] = SGUnterarm(varargin)


arm_height_increase = 20;
angle = 0;


arm_length = 60; if nargin >=1 && ~isempty(varargin{1}) arm_length = varargin{1}; end
dof = 'z'; if nargin >=2 && ~isempty(varargin{2}) dof = varargin{2}; end
servo_name = 'sm40bl'; if nargin >=3 && ~isempty(varargin{3}) servo_name = varargin{3}; end


%% Parameters
conn_screw_circle_radius = 10.5;


%% Fix Values
inner_run_R = 29.7;
outer_run_R = 37.7;
outer_run_H = 17;
inn_run_H = 6;
legacy_adapter_H = 35;
legacy_inner_R = 22.5;
wall_thick = 2.5;
screw_R = 1.6+0.5;
screw_head_R = 3;
cable_gap = 7;
servo_mid_R = 7.5;
servo_attach_R = 19;
clf;
%% Bottom Connection
outer_R = outer_run_R+wall_thick;
inner_R = conn_screw_circle_radius + screw_head_R + cable_gap + wall_thick; 

CPL_bottom_connection = PLsquare(outer_R-legacy_inner_R,legacy_adapter_H);
CPL_bottom_connection = PLtrans(CPL_bottom_connection,[(legacy_inner_R+outer_R)/2 legacy_adapter_H/2]);

CPL_run_cut = [legacy_inner_R 0;legacy_inner_R inn_run_H;inner_run_R inn_run_H;inner_run_R outer_run_H;outer_run_R outer_run_H;outer_run_R 0];

CPL_bottom_connection = CPLbool('-',CPL_bottom_connection,CPL_run_cut);


CPL_bottom_connection = PLroundcorners(CPL_bottom_connection,[3,4,5,8],[2 2 2 5]);
CPL_bottom_connection_cable_gap = CPLbool('-',CPL_bottom_connection,PLtrans(PLsquare(2*outer_R,cable_gap),[0 legacy_adapter_H-10]));



SG_bottom_conn = SGofCPLrota(CPL_bottom_connection,1.75*pi,false,1.125*pi);
SG_bottom_conn_cable_gap = SGofCPLrota(CPL_bottom_connection_cable_gap,0.25*pi,false,-1.125*pi);
SG_bottom_conn = SGcat(SG_bottom_conn,SG_bottom_conn_cable_gap);



CPL_cable_slot = PLkidney(inner_R+0.5,inner_R-cable_gap,pi/1.5);
CPL_screw_holes = CPLcopyradial(PLcircle(screw_R),conn_screw_circle_radius,8);
PLlayer1 = [PLcircle(servo_attach_R);NaN NaN;PLcircle(servo_mid_R)];
PLlayer1 = CPLbool('-',PLlayer1,CPL_cable_slot);
PLlayer1 = CPLbool('-',PLlayer1,CPL_screw_holes);
SGlayer1 = SGofCPLz(PLlayer1,6.325);

CPL_screw_holes_heads = CPLcopyradial(PLcircle(screw_head_R),conn_screw_circle_radius,8);
PLlayer3 = PLcircle(inner_R);
PLlayer3 = CPLbool('-',PLlayer3,CPL_cable_slot);
PLlayer3 = CPLbool('-',PLlayer3,CPL_screw_holes_heads);
SGlayer3 = SGofCPLz(PLlayer3,13);

SG_mid = SGstack('z',SGlayer1,SGlayer3);
SG_bottom_conn = SGcat(SG_bottom_conn,SGtransrelSG(SG_mid,SG_bottom_conn,'alignbottom'));
SG_bottom_conn = SGtransrelSG(SG_bottom_conn,'','transz',-legacy_adapter_H/2+2);

%% ARM
width_bottom = 50;
width_mid = 40;
width_top = 45;
height_arm = 28;
height_arm_mid = 35;
height_arm_top = 40;
wall_thick = 2;

%%CrossSection1
CPL_crossS_1_out = PLroundcorners(PLsquare(width_bottom,height_arm),[1,2,3,4],2*wall_thick);
CPL_crossS_1_in = PLroundcorners(PLsquare(width_bottom-2*wall_thick,height_arm-2*wall_thick),[1,2,3,4],wall_thick);
CPL_crossS_1 = CPLbool('-',CPL_crossS_1_out,CPL_crossS_1_in);
CPL_crossS_1_conn = CPLaddauxpoints(CPL_crossS_1,1);
%%CrossSection2
CPL_crossS_2_out = PLroundcorners(PLsquare(width_mid,height_arm_mid),[1,2,3,4],2*wall_thick);
CPL_crossS_2_in = PLroundcorners(PLsquare(width_mid-2*wall_thick,height_arm_mid-2*wall_thick),[1,2,3,4],wall_thick);
CPL_crossS_2 = CPLbool('-',CPL_crossS_2_out,CPL_crossS_2_in);
CPL_crossS_2 = PLtrans(CPL_crossS_2,[0 0.66*arm_height_increase+(height_arm_mid-height_arm)/2]);
%%CrossSection3
CPL_crossS_3_out = PLroundcorners(PLsquare(width_top,height_arm_top),[1,2,3,4],2*wall_thick);
CPL_crossS_3_in = PLroundcorners(PLsquare(width_top-2*wall_thick,height_arm_top-2*wall_thick),[1,2,3,4],wall_thick);
CPL_crossS_3 = CPLbool('-',CPL_crossS_3_out,CPL_crossS_3_in);
CPL_crossS_3 = PLtrans(CPL_crossS_3,[0 arm_height_increase+(height_arm_top-height_arm)/2]);

SG_bottom_half = SGof2CPLsz(CPL_crossS_1,CPL_crossS_2,0.66*arm_length,'number','miny');
SG_top_half = SGof2CPLsz(CPL_crossS_2,CPL_crossS_3,0.33*arm_length,'number','miny');

SG_conn_arm_bottom = SGofCPLz(CPL_crossS_1_conn,0.01);
PL_bottom_circ = PLcircle(outer_R,outer_R*10,pi);
VL_bottom_circ = [PL_bottom_circ zeros(size(PL_bottom_circ))];
VL_bottom_circ = VLtransR(VL_bottom_circ,rotx(90));
VL_bottom_circ = VLtrans(VL_bottom_circ,[0 0 -outer_R]);


for i=1:size(SG_conn_arm_bottom.VL,1)
	if(SG_conn_arm_bottom.VL(i,3)==0)
		x = abs(VL_bottom_circ(:,1)-SG_conn_arm_bottom.VL(i,1));
		[idx,~] = find(x == min(x));
		SG_conn_arm_bottom.VL(i,3) = VL_bottom_circ(idx,3);
	end	
end

%% Top Servo mount

[SG_bottom,CPL] = SGdofsLCL('dof',dof,'servo',servo_name);
SG_bottom = SGtrans(SG_bottom,rot(angle));
CPL = PLtransR(CPL,rot(angle));

min_y_servo_mount = min(CPL(:,2));
ind = (SG_top_half.VL(:,3) == max(SG_top_half.VL(:,3)));
min_y_top_arm = min(SG_top_half.VL(ind,2));
SG_bottom = SGtrans(SG_bottom,[0 min_y_top_arm-min_y_servo_mount 0]);
CPL = PLtrans(CPL,[0 min_y_top_arm-min_y_servo_mount]);

CPL_crossS_3 = CPLaddauxpoints(CPL_crossS_3,0.5);
CPL = CPLaddauxpoints(CPL,0.5);
SG_connection = SGof2CPLsz(CPL_crossS_3,CPL,10,'center','miny');


SG_servo_attachment = SGstack2('z',SG_connection,SG_bottom);

SG_arm = SGstack2('z',SG_conn_arm_bottom,SG_bottom_half,SG_top_half,SG_servo_attachment);
SG_arm = SGtransrelSG(SG_arm,SG_bottom_conn,'rotz',-pi/2,'roty',-pi/2,'transx',-outer_R);

SG = SGcat(SG_arm,SG_bottom_conn);

min_z_bott_con = min(SG_conn_arm_bottom.VL(:,3));
H_b = [rotx(0) [0;0;min_z_bott_con]; 0 0 0 1];
SG = SGTset(SG,'B',H_b);

% SGs = {SG,SG_lid};
if nargout== 0
	clf;
    SGplot(SG);
end

% SGwriteSTL(SG);


end

