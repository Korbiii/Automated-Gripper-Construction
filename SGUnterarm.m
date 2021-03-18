function [SG] = SGUnterarm()
clf;
%% Parameters
conn_screw_circle_radius = 10.5;


%% Fix Values
inner_run = 29.7;
outer_run = 37.7;
run_heigth = 17;
run_heigth_2 = 6;
wall_thick = 2.5;
screw_radius = 1.6;
screw_head_radius = 3;
cable_gap = 7;
servo_mid_radius = 7.5;
servo_attach_radius = 19;

%% Bottom Connection
outer_radius = outer_run+wall_thick;
inner_radius = conn_screw_circle_radius + screw_head_radius + cable_gap + wall_thick; 
CPL_bottom_connection = PLsquare(outer_radius-inner_radius,run_heigth+cable_gap*2);
CPL_bottom_connection = CPLbool('-',CPL_bottom_connection,PLtrans(PLsquare(outer_run-inner_run,run_heigth),[0 -(run_heigth+cable_gap*2-run_heigth)/2]));
CPL_bottom_connection = CPLbool('-',CPL_bottom_connection,PLtrans(PLsquare(inner_run-inner_radius,run_heigth_2),[-(outer_radius-inner_radius-inner_run+inner_radius)/2 -(run_heigth+cable_gap*2-run_heigth_2)/2]));
CPL_bottom_connection = PLroundcorners(CPL_bottom_connection,[1,2,3,4,7,8],2);
CPL_bottom_connection_cable_gap = CPLbool('-',CPL_bottom_connection,PLtrans(PLsquare(outer_radius,cable_gap),[0 10]));
CPL_bottom_connection = PLtrans(CPL_bottom_connection,[(inner_radius+outer_radius)/2 0]);
CPL_bottom_connection_cable_gap = PLtrans(CPL_bottom_connection_cable_gap,[(inner_radius+outer_radius)/2 0]);
SG_bottom_conn = SGofCPLrota(CPL_bottom_connection,1.75*pi,false,1.125*pi);
SG_bottom_conn_cable_gap = SGofCPLrota(CPL_bottom_connection_cable_gap,0.25*pi,false,-1.125*pi);
SG_bottom_conn = SGcat(SG_bottom_conn,SG_bottom_conn_cable_gap);

CPL_cable_slot = PLkidney(inner_radius+0.5,inner_radius-cable_gap,pi/1.5);
CPL_screw_holes = CPLcopyradial(PLcircle(screw_radius),conn_screw_circle_radius,8);
PLlayer1 = [PLcircle(servo_attach_radius);NaN NaN;PLcircle(servo_mid_radius)];
PLlayer1 = CPLbool('-',PLlayer1,CPL_cable_slot);
PLlayer1 = CPLbool('-',PLlayer1,CPL_screw_holes);
SGlayer1 = SGofCPLz(PLlayer1,6.325);

CPL_screw_holes_heads = CPLcopyradial(PLcircle(screw_head_radius),conn_screw_circle_radius,8);
PLlayer3 = PLcircle(inner_radius);
PLlayer3 = CPLbool('-',PLlayer3,CPL_cable_slot);
PLlayer3 = CPLbool('-',PLlayer3,CPL_screw_holes_heads);
SGlayer3 = SGofCPLz(PLlayer3,13);

SG_mid = SGstack('z',SGlayer1,SGlayer3);
SG_bottom_conn = SGcat(SG_bottom_conn,SGtransrelSG(SG_mid,SG_bottom_conn,'alignbottom'));

%% ARM
width_bottom = 55;
width_mid = 35;
width_top = 40;
height_arm = 28;
height_arm_mid = 35;
height_arm_top = 40;
wall_thick = 2;
arm_length = 85;
arm_height_increase = 20;

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
PL_bottom_circ = PLcircle(outer_radius,outer_radius*10,pi);
VL_bottom_circ = [PL_bottom_circ zeros(size(PL_bottom_circ))];
VL_bottom_circ = VLtransR(VL_bottom_circ,rotx(90));
VL_bottom_circ = VLtrans(VL_bottom_circ,[0 0 -outer_radius]);


for i=1:size(SG_conn_arm_bottom.VL,1)
	if(SG_conn_arm_bottom.VL(i,3)==0)
		x = abs(VL_bottom_circ(:,1)-SG_conn_arm_bottom.VL(i,1));
		[idx,~] = find(x == min(x));
		SG_conn_arm_bottom.VL(i,3) = VL_bottom_circ(idx,3);
	end	
end

%% Top Servo mount
tol = 0.5;
screw_rad = 1.5;
servo_width = 28.5+tol;
servo_lenght = 46.5+tol;
servo_height = 34+tol;
outer_radius_ser = 26.5;
servo_shaft_offs = 11.25;
screw_length = 14;
cable_gap = 10;
cable_gap_width = 20;

CPL_out = CPLconvexhull([PLcircle(outer_radius_ser);PLtrans([-servo_width/2-3 0;servo_width/2+5 0],[0 -servo_lenght+servo_shaft_offs-screw_length+5])]);
CPL_out = PLroundcorners(CPL_out,[3,4],3);
CPL_in = PLtrans(PLsquare(servo_width,servo_lenght),[0 -((servo_lenght/2)-servo_shaft_offs)+0.5*tol]);
CPL_in_ledge = PLtrans(PLsquare(servo_width,servo_lenght-4),[0 -(((servo_lenght-4)/2)-servo_shaft_offs)]);
CPL_cable_gap = PLtrans(PLsquare(servo_width+cable_gap,cable_gap_width),[0 0]);
CPL_in = CPLbool('+',CPL_in,CPL_cable_gap);
CPL_in_ledge = CPLbool('+',CPL_in_ledge,CPL_cable_gap);
CPL_in = PLroundcorners(CPL_in,[2,3,4,9,10,11],cable_gap/4);
CPL_in_ledge = PLroundcorners(CPL_in_ledge,[2,3,4,9,10,11],cable_gap/4);
CPL_in = CPLbool('+',CPL_in,PLtrans(PLsquare(servo_width-4,servo_lenght+8),[0 -(((servo_lenght+8)/2)-servo_shaft_offs-4)]));

SG_bottom_w_ledge = SGofCPLz([CPL_out;NaN NaN;CPL_in_ledge],2);
SG_bottom = SGofCPLz([CPL_out;NaN NaN;CPL_in],servo_height);

% Screw Holes
SG_hole = SGofCPLz(PLcircle(screw_rad),servo_lenght+10);
SG_HOLE = SGofCPLz(PLcircle(screw_rad*2),30);
SG_hole = SGstackb('z',SG_HOLE,SG_hole,SG_HOLE);

SG_hole_left = SGtransrelSG(SG_hole,SG_bottom,'roty',pi/2,'rotz',pi/2,'transy',-servo_shaft_offs-(servo_lenght+10+60)/2,'aligntop',-11+screw_rad,'centerx',-8);
SG_bottom = SGbool3('-',SG_bottom,SG_hole_left);
SG_hole_right = SGtransrelSG(SG_hole,SG_bottom,'roty',pi/2,'rotz',pi/2,'transy',-servo_shaft_offs-(servo_lenght+10+60)/2,'aligntop',-11+screw_rad,'centerx',8);
SG_bottom = SGbool3('-',SG_bottom,SG_hole_right);

shift = min(SG_bottom.VL(:,2));
min_z_bott_con = min(SG_conn_arm_bottom.VL(:,2));
servo_attach_offset = -shift+min_z_bott_con+arm_height_increase;
SG_connection = SGof2CPLsz(CPL_crossS_3,PLtrans([CPL_out;NaN NaN;CPL_in_ledge],[0 servo_attach_offset]),10,'center','miny');
SG_servo_attachment = SGstack('z',SG_connection,SGtrans(SG_bottom_w_ledge,[0 servo_attach_offset 0]),SGtrans(SG_bottom,[0 servo_attach_offset 0]));

SG_arm = SGstack('z',SG_conn_arm_bottom,SG_bottom_half,SG_top_half,SG_servo_attachment);
SG_arm = SGtransrelSG(SG_arm,SG_bottom_conn,'rotz',-pi/2,'roty',-pi/2,'transx',-outer_radius);
% SGplot(SG_servo_attachment);
SG = SGcat(SG_arm,SG_bottom_conn);

%%Lid
attach_radius = 23.5;
servo_horn_radius = 11;


CPL_lid_outline = CPL_out;
CPL_lid_top = [CPL_lid_outline;NaN NaN;PLcircle(attach_radius)];
CPL_lid_bottom = [CPL_lid_outline;NaN NaN;PLroundcorners(PLsquare(attach_radius*2-1,servo_horn_radius*2),[1,2,3,4],servo_horn_radius)];
SG_lid_top = SGstack('z',SGofCPLz(CPL_lid_top,6),SGof2CPLsz(CPL_lid_top,[PLgrow(CPL_lid_outline,1);NaN NaN;PLcircle(attach_radius)],1));
SG_lid = SGstack('z',SGofCPLz(CPL_lid_bottom,3),SG_lid_top);

CPL_slide_in_attachments = PLroundcorners(PLsquare(servo_width-4,servo_height-5),[1,2],5);
CPL_slide_in_attachments = CPLbool('-',CPL_slide_in_attachments,PLtrans(PLcircle(screw_radius),[8 (servo_height-5)/2-11]));
CPL_slide_in_attachments = CPLbool('-',CPL_slide_in_attachments,PLtrans(PLcircle(screw_radius),[-8 (servo_height-5)/2-11]));
SG_slide_in_attachments = SGofCPLzchamfer(CPL_slide_in_attachments,3,1);
SG_slide_in_attachment_1 = SGtransrelSG(SG_slide_in_attachments,SG_lid,'rotx',pi/2,'under',-1,'transy',3+servo_shaft_offs+tol+0.5);
SG_slide_in_attachment_2 = SGtransrelSG(SG_slide_in_attachments,SG_lid,'rotx',pi/2,'under',-1,'transy',-servo_lenght+servo_shaft_offs-0.5);

SG_lid = SGcat(SG_lid,SG_slide_in_attachment_1,SG_slide_in_attachment_2);

SGplot(SG_lid);
SGplot(SG);






end

%%  [SG] = SGstack(SGs)
%	=== INPUT PARAMETERS ===
%   dir:        Direction of stacking y,x,z
%	SGs:        Array of SGs

%	=== OUTPUT RESULTS ======
%	SG:         SG of stacked SGs
function [SG] = SGstack(dir,varargin)
SG = varargin{1};
for i=2:nargin-1
    switch dir
        case 'z'
            SG = SGcat(SG,SGontop(varargin{i},SG));            
        case 'y'
            SG = SGcat(SG,SGinfront(varargin{i},SG));       
        case 'x'
            SG = SGcat(SG,SGleft(varargin{i},SG));
    end
end
end

%%  [PL] = PLroundcorners(PL,indices,varargin)
%	=== INPUT PARAMETERS ===
%	PL:     Contour of PL you to search throguh
%	=== OUTPUT RESULTS ======
function [PL] = PLroundcorners(PL,corner_numbers,varargin)
radius = ones(1,size(corner_numbers,2)); if nargin>=3 && ~isempty(varargin{1}); radius=varargin{1}; end
connection = []; if nargin >=4 && ~isempty(varargin{2}); connection = varargin{2}; end
if(size(radius,1)==1)
    radius = repmat(radius,1,size(corner_numbers,2));
end
try
    PL_save = CPLselectinout(PL,1);
catch
    PL_save =[];
end

PL = CPLselectinout(PL,0);

corners = {};
for i=1:size(corner_numbers,2)
    if corner_numbers(i) == 1
        v1 = PL(corner_numbers(i)+1,:)-PL(corner_numbers(i),:);
        v1 = v1/norm(v1);
        v2 = PL(size(PL,1),:)-PL(corner_numbers(i),:);
        v2 = v2/norm(v2);
    elseif corner_numbers(i) == size(PL,1)
        v1 = PL(1,:)-PL(corner_numbers(i),:);
        v1 = v1/norm(v1);
        v2 = PL(corner_numbers(i)-1,:)-PL(corner_numbers(i),:);
        v2 = v2/norm(v2);
    else
        v1 = PL(corner_numbers(i)+1,:)-PL(corner_numbers(i),:);
        v1 = v1/norm(v1);
        v2 = PL(corner_numbers(i)-1,:)-PL(corner_numbers(i),:);
        v2 = v2/norm(v2);
    end
    following_point = PL(corner_numbers(i),:)+(v1*abs(radius(i)));
    trailing_point = PL(corner_numbers(i),:)+(v2*abs(radius(i)));
    corners{end+1} = PLcircarc2([trailing_point;PL(corner_numbers(i),:);following_point]);
    [is_member,pos] = ismember(corner_numbers(i),abs(connection));
    if is_member
        if connection(pos) <0
            corners{end} = PLmirror0(corners{end},[trailing_point;PL(corner_numbers(i),:)],1);
        else
            corners{end} = PLmirror0(corners{end},[following_point;PL(corner_numbers(i),:)],1);
        end
    end
end
for i=1:size(corner_numbers,2)
    if corner_numbers(i) == 1
        PL = [corners{i};PL(corner_numbers(i)+1:end,:)];
    elseif corner_numbers(i)==size(PL,1)
        PL = [PL(1:corner_numbers(i)-1,:);corners{i}];
    else
        PL = [PL(1:corner_numbers(i)-1,:);corners{i};PL(corner_numbers(i)+1:end,:)];
    end
    corner_numbers = corner_numbers + size(corners{i},1)-1;
    
end
if ~isempty(PL_save)
    PL = [PL;NaN NaN;PL_save];
end
end

%%  [PL] = PLcircarc2(PL,radius)
%	=== INPUT PARAMETERS ===

%	=== OUTPUT RESULTS ======
function [PL] = PLcircarc2(PL)
v1 = PL(1,:)-PL(2,:);
v1 = v1/norm(v1);
v2 = PL(3,:)-PL(2,:);
v2 = v2/norm(v2);

v1_n = v1*rot(pi/2);
v2_n = v2*rot(pi/2);

cp = cross2edges(PL(1,:),v1_n,PL(3,:),v2_n);
dist = min([pdist2(PL(1,:),cp),pdist2(PL(3,:),cp)]);

v3 = PL(1,:)-cp;
v4 = PL(3,:)-cp;
v5 = [1 0];

phi_1 = -acos2(v3,v5);
phi_2 = -acos2(v4,v3);

PL_circle = PLtrans(PLcircseg(dist,'',phi_1,phi_1+phi_2),cp);
PL = PL_circle;

end


%%  [SG] = SGstack(SGs)
%	=== INPUT PARAMETERS ===
%   dir:        Direction of stacking y,x,z
%	SGs:        Array of SGs

%	=== OUTPUT RESULTS ======
%	SG:         SG of stacked SGs
function [SG] = SGstackb(dir,varargin)
SG = varargin{1};
for i=2:nargin-1
    switch dir
        case 'z'
            SG = SGbool('+',SG,SGontop(varargin{i},SG,0.005));            
        case 'y'
            SG = SGbool('+',SGinfront(varargin{i},SG));       
        case 'x'
            SG = SGbool('+',SGleft(varargin{i},SG));
    end
end
end
