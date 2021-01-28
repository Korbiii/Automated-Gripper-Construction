function [SG] = SGLCLzdof()
clf;
tol = 0.5;

%% Connection
diameter_connection = 46.5;
screw_hole_radius = 2;
hole_circle_radius = 7.5;
screw_number = 4;

CPL_connection = PLcircle(diameter_connection/2);
CPL_screw_holes = CPLcopyradial(PLcircle(screw_hole_radius),hole_circle_radius,screw_number);
CPL_cable_slot = PLkidney(hole_circle_radius+screw_hole_radius+2,hole_circle_radius+screw_hole_radius+2+6,3/2*pi);
CPL_connection = CPLbool('-',CPL_connection,CPL_screw_holes);
CPL_connection = CPLbool('-',CPL_connection,CPL_cable_slot);
SG_connection = SGofCPLz(CPL_connection,9);

CPL_connector_slice = [PLcircle(diameter_connection/2);NaN NaN;PLcircle(hole_circle_radius+screw_hole_radius+2+6)];


%% Servomount
outer_radius = 26.5;
connector_radius = 19/2;
distance_axis = 35.25;
servo_height = 34;
servo_length = 46.5;
servo_width = 28.5;
cable_clearance_start = 3; 


CPL_outside = CPLconvexhull([PLcircle(outer_radius);[-outer_radius,-distance_axis-2;outer_radius,-distance_axis-2]]);
CPL_outside = CPLbool('-',CPL_outside,CPLconvexhull([PLcircle(connector_radius);[-connector_radius,30;connector_radius,30]]));
SG_servo_cage = SGofCPLz(CPL_outside,servo_height+6);
SG_servo_cage = SGtransrelSG(SG_servo_cage,'','rotx',pi/2,'transy',(servo_height+6)/2);



CPL_inside = CPLbool('+',PLsquare(servo_width+tol,servo_height+tol),PLtrans(PLsquare(outer_radius*2-6,15),[0,((servo_height/2)-3)/2]));
SG_inside = SGtrans0(SGofCPLz(CPL_inside,outer_radius*4));
SG_servo_cage = SGbool('-',SG_servo_cage,SG_inside);

CPL_servo_stop_inside = CPLbool('-',PLsquare(servo_width+tol,servo_height+tol),PLsquare(servo_width+tol,servo_height+tol-6));
SG_servo_stop = SGofCPLz(CPL_servo_stop_inside,2);
SG_servo_stop = SGtransrelSG(SG_servo_stop,SG_servo_cage,'alignbottom');
SG_servo_cage = SGcat(SG_servo_stop,SG_servo_cage);


%% Screw Holes
screw_rad = 1.5;
SG_hole = SGofCPLz(PLcircle(screw_rad),servo_width+10);
SG_HOLE = SGofCPLz(PLcircle(screw_rad*2),servo_width);
SG_hole = SGstackb('z',SG_HOLE,SG_hole,SG_HOLE);
SG_hole = SGtransrelSG(SG_hole,SG_hole,'roty',pi/2,'centerx');
SG_hole_1 = SGtransrelSG(SG_hole,SG_hole,'transy',-6,'transz',-4);
SG_hole_2 = SGtransrelSG(SG_hole_1,SG_hole_1,'transz',-24);
SG_servo_cage = SGbool('-',SG_servo_cage,SG_hole_1);
SG_servo_cage = SGbool('-',SG_servo_cage,SG_hole_2);
%%Middle
CPL_servocage_slice = CPLofSGslice(SG_servo_cage,-10);
SG_conn = SGof2CPLsz(CPL_connector_slice,CPL_servocage_slice,10);
SG = SGstack('z',SG_connection,SG_conn,SG_servo_cage);

H_f = [rotx(90) [0;0;60]; 0 0 0 1];

SG = SGTset(SG,'F',H_f);
SGTplot(SG);
% SGwriteSTL(SG);


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

