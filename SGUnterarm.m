function [SG] = SGUnterarm(varargin)

arm_height_increase = 20;
angle = 0;

arm_length = 60; if nargin >=1 && ~isempty(varargin{1}) arm_length = varargin{1}; end
dof = 'z'; if nargin >=2 && ~isempty(varargin{2}) dof = varargin{2}; end
servo_name = 'sm40bl'; if nargin >=3 && ~isempty(varargin{3}) servo_name = varargin{3}; end
attach_servo = 'sm120bl'; if nargin >=4 && ~isempty(varargin{4}) attach_servo = varargin{4}; end
attach_dof = 'legacy'; if nargin >=5 && ~isempty(varargin{5}) attach_dof = varargin{5}; end

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

[SG_connection_bottom, CPL_connection] = SGconnAdaptersLCL('adapter_type',attach_dof,'servo',attach_servo);

CPL_crossS_1 = CPLaddauxpoints(CPL_crossS_1,0.5);
SG_connection_bottom2arm = SGof2CPLsz(CPL_connection,CPL_crossS_1,5);


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


SG_servo_attachment = SGstack('z',SG_connection,SG_bottom);

SG = SGstack('z',SG_connection_bottom,SG_connection_bottom2arm,SG_bottom_half,SG_top_half,SG_servo_attachment);

% SGs = {SG,SG_lid};
if nargout== 0
	clf;
	SGplot(SG);
	SGwriteSTL(SG);
end



end

