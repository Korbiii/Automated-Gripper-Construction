function [SG] = SGTrigripper()
clf;
tol = 0.5;

%%Top
outer_radius = 26.5;
connector_radius = 19/2;
distance_axis = 35.25;
servo_height = 34;
servo_length = 46.5;
servo_width = 28.5;
cable_clearance_start = 10; 
%% Servocage
CPL_grip_attach = [-8 -distance_axis-5;8 -distance_axis-5;8 0;-8 0];
CPL_outer = CPLbool('+',PLtrans(PLsquare(servo_width+8,servo_length+8),[0,-(servo_length-distance_axis)]),[CPL_grip_attach;NaN NaN;PLtransR(CPL_grip_attach,rot(2/3*pi));NaN NaN;PLtransR(CPL_grip_attach,rot(-2/3*pi))]);
CPL_outer = PLroundcorners(CPL_outer,[2,5,6,9,11,12,13,14],[22,11,11,22,1,1,1,1]);
CPL_inner = CPLbool('+',PLtrans(PLsquare(servo_width+tol,servo_length+tol),[0 -servo_length+distance_axis]),PLtrans(PLsquare(servo_width+10,15),[0 0]));
CPL_inner = PLroundcorners(CPL_inner,[3,4,9,10],3);
CPL_top = CPLbool('-',CPL_outer,CPL_inner);
SGtop = SGofCPLz(CPL_top,20);

CPL_servocage_bot = CPLbool('+',PLtrans(PLsquare(servo_width+8,servo_length+8),[0,-(servo_length-distance_axis)]),PLgrow(CPL_inner,-2));
CPL_servocage_bot = CPLconvexhull(CPL_servocage_bot);
CPL_servocage_bot = CPLbool('-',CPL_servocage_bot,CPL_inner);
CPL_servocage_bot = CPLaddauxpoints(CPL_servocage_bot,2);
CPL_top = CPLaddauxpoints(CPL_top,2);
SGbot = SGof2CPLsz(CPL_servocage_bot,CPL_top,servo_height-20+2);
SG_servocage = SGstack('z',SGbot,SGtop);

CPL_servo_stop = PLtrans(CPLbool('-',PLsquare(servo_width+tol,servo_length+tol),PLsquare(servo_width-4,servo_length-4)),[0,-(servo_length-distance_axis)]);
CPL_servo_stop = CPLbool('-',CPL_servo_stop,PLtrans(PLsquare(servo_width+10,15),[0 0]));
SG_servo_stop = SGofCPLz(CPL_servo_stop,2);
SG_servo_stop = SGtransrelSG(SG_servo_stop,SG_servocage,'alignbottom');
SG_servocage = SGcat(SG_servo_stop,SG_servocage);

CPL_gripp_attach = CPLconvexhull([PLsquare(10);5 -15]);
CPL_gripp_attach = [CPL_gripp_attach;NaN NaN;PLcircle(3)];
SG_gripp_attach = SGofCPLz(CPL_gripp_attach,4);
SG_gripp_attach = SGtransrelSG(SG_gripp_attach,SG_servocage,'rotx',pi/2,'rotz',pi/2,'transy',-5-distance_axis-5,'transx',3.5,'aligntop');
SG_gripp_attach_d = SGcat(SG_gripp_attach,SGmirror(SG_gripp_attach,'yz'));
SG_gripp_attach = SGcat(SG_gripp_attach_d,SGtransR(SG_gripp_attach_d,rot(0,0,2/3*pi)));
SG_gripp_attach = SGcat(SG_gripp_attach,SGtransR(SG_gripp_attach_d,rot(0,0,-2/3*pi)));
SG_servocage = SGcat(SG_gripp_attach,SG_servocage);
%% Connecting Bracket
diameter_connection = 46.5;
screw_hole_radius = 2;
hole_circle_radius = 7.5;
screw_number = 4;

CPL_screw_holes = CPLcopyradial(PLcircle(screw_hole_radius),hole_circle_radius,screw_number);
CPL_bracket = CPLconvexhull([PLcircle(20);-20 30;20 30]);
CPL_bracket = CPLbool('-',CPL_bracket,CPL_screw_holes);

SG_bracket = SGofCPLz(CPL_bracket,5);

SG_screw_length = SGof2CPLsz([PLcircle(16);NaN NaN;CPL_screw_holes],[PLcircle(12);NaN NaN;CPL_screw_holes],4);

SG_bracket = SGstack('z',SG_bracket,SG_screw_length);

SG_cutout = SGbox([12,30,4]);
SG_cutout = SGtransrelSG(SG_cutout,SG_bracket,'alignbottom',0.01,'alignfront',0.01);
SG_bracket = SGbool('-',SG_bracket,SG_cutout);
SG_bracket = SGtransrelSG(SG_bracket,SG_bracket,'rotx',pi/2,'transy',-20);
SG_bracket = SGcat(SG_bracket,SGmirror(SG_bracket,'xz'));

H_b = [rotx(90) [0;0;0]; 0 0 0 1];

SG_bracket = SGTset(SG_bracket,'B',H_b);

CPL_slice = CPLofSGslice(SG_bracket,max(SG_bracket.VL(:,3))-0.01);
CPL_slice = CPLofPL(CPLconvexhull(CPL_slice));
CPL_slice = [CPL_slice;NaN NaN;PLgrow(CPL_slice,4)];

CPL_slice2 = CPLofSGslice(SG_servocage,min(SG_servocage.VL(:,3))+0.01);
SG_connection_bracket = SGof2CPLsz(CPL_slice,CPL_servocage_bot,10);

SG = SGstack('z',SG_bracket,SG_connection_bracket,SG_servocage);
SGTplot(SG);


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
            SG = SGbool('+',SG,SGontop(varargin{i},SG));            
        case 'y'
            SG = SGbool('+',SGinfront(varargin{i},SG));       
        case 'x'
            SG = SGbool('+',SGleft(varargin{i},SG));
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

%%   [SG] = SGmirror(SG,axis)
%	=== INPUT PARAMETERS ===
%	SG:    SG you want to mirror
%	plane: plane you want to mirror on
%	=== OUTPUT RESULTS ======
%	SG:         Mirrored SG
function [SG] = SGmirror(SG,plane)
switch plane
    case 'yz'
        SG.VL = VLswapX(SG.VL);
        SG.FL(:,[2 3]) = SG.FL(:,[3 2]);
    case 'xz'
        SG.VL = VLswapY(SG.VL);
        SG.FL(:,[2 3]) = SG.FL(:,[3 2]);
    case 'xy'
        SG.VL = VLswapZ(SG.VL);
        SG.FL(:,[1 3]) = SG.FL(:,[3 1]);
    otherwise
        error(plane + " plane doesnt exist");
end
end

