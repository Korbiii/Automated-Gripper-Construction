%%   [SG_gripper_body,SG_gripper_attachment,SG_final,inputsObject,inputsGripper] = SGparrallelGripper([variable_name, value])
%    === INPUT PARAMETERS ===
%    'jaw_th':				Thickness of gripper jaws
%	 'opening':				Size of gripper opening. Only visual, doesnt change geometry
%	 'servo':				Servo used for gripper
%	 'conn_servo':			Servo in previous degree of freedom	
%    'conn_type':			Type of connection to previous degree of freedom
%    'output':				If set function writes STLs
%	 'c_input':				Cell array of above inputs e.g. {'jaw_th',10;'opening',60}
%    === OUTPUT RESULTS ======
%    SG_gripper_body:		SG of gripper main body
%	 SG_gripper_attachment:	SG of variable gripper parts with reduced alpha value
%	 SG_final:				SG complete gripper
%	 inputsObject:			Input array for object manipulation
%	 inputsGripper:			Input array for gripper manipulation
function [SG_gripper_body,SG_gripper_attachment,SG_final,inputsObject,inputsGripper] = SGparrallelGripper(varargin)

tol=0.5;
thread_length = 12;
printhelp =0;
servo_name = 'sm40bl';
conn_type  = 'rotLock';

jaw_th = 5;
jaw_H = 40;
opening = 40;

inputsObject = {'transz',2,30,31;'transy',2,52,54;'transx',2,50,56;'roty',pi/2,115,119;'rotz',pi/2,97,100;'rotx',pi/2,101,113};
inputsGripper = {'jaw_th',jaw_th,1,43,45;'opening' opening 2 29 28;'jaw_H',jaw_H,1,104,106};

if ~isempty(varargin)
	if strcmp(varargin{1},'c_inputs')
		temp = varargin{2};
		varargin(1) = [];
		varargin(1) = [];
		for k=1:size(temp,1)
			varargin{end+1} = temp{k,1};
			varargin{end+1} = temp{k,2};
		end
	end
end

output = 0;
SG_object = [];
i_idx = 1;
while i_idx<=size(varargin,2)
	if ~ischar(varargin{i_idx})
		i_idx = i_idx+1; 
		continue; 
	end
	switch varargin{i_idx}
		case 'SG_object'
			SG_object = varargin{i_idx+1};
		case 'jaw_th'
			jaw_th = max(jaw_th,varargin{i_idx+1});
		case 'jaw_H'
			jaw_H = max(jaw_H,varargin{i_idx+1});
		case 'opening'
			opening = varargin{i_idx+1};	
		case 'conn_type'
			SG_object = varargin{i_idx+1};			
			i_idx = i_idx+1;	
		case 'conn_servo'
			SG_object = varargin{i_idx+1};			
			i_idx = i_idx+1;	
		case 'output'
			output = 1;
	end
	i_idx = i_idx+1;
end

servo = readServoFromTable(servo_name);

servo.width = servo.width+tol;
servo.height = servo.height+tol;

CPL_servo = PLtrans(PLsquare(servo.width,servo.length),[0 -servo.shaft_offs]);
CPL_cable_slot_hor = PLtrans(servo.PL_cable_gap_hor,[0 -servo.shaft_offs]);
CPL_cable_slot_hor = PLroundcorners(CPL_cable_slot_hor,[1,2,3,4],servo.cable_gap-2);
CPL_servo_w_hor_cablegap = PLtrans(CPLbool('+',PLsquare(servo.width+servo.cable_gap,servo.length),PLroundcorners(servo.PL_cable_gap_hor,[1,2,3,4],servo.cable_gap)),[0 -servo.shaft_offs]);

%% Main Body
main_R = sqrt((servo.width/2)^2+((servo.length/2)+servo.shaft_offs)^2)+2;

CPL_main_bottom_layer = PLcircle(main_R);
CPL_main_bottom_layer = CPLbool('-',CPL_main_bottom_layer,CPL_cable_slot_hor);

CPL_main_bottom_layer_w_slot = CPLbool('-',CPL_main_bottom_layer,CPLconvexhull([PLcircle(5);PLtrans(PLcircle(5),[0 500])]));

SG_bottom_layer = SGofCPLz(CPL_main_bottom_layer,2);
SG_bottom_layer_w_wlot = SGofCPLz(CPL_main_bottom_layer_w_slot,3);
SG_bottom_layer = SGstack('z',SG_bottom_layer,SG_bottom_layer_w_wlot);
CPL_servo_channel = PLtrans(PLsquare(servo.width+servo.cable_gap,5000),[0 2500]);
CPL_main_mid = CPLbool('-',PLcircle(main_R),CPL_servo_w_hor_cablegap);
CPL_main_mid = CPLbool('-',CPL_main_mid,CPL_servo_channel);


out = (servo.width+servo.cable_gap)/2;
CPL_big_dove = [-out main_R;out main_R;out+2 (servo.length/2)-servo.shaft_offs;-out-2 (servo.length/2)-servo.shaft_offs];

CPL_main_mid = CPLbool('-',CPL_main_mid,CPL_big_dove);

min_y_cslot = min(CPL_cable_slot_hor(:,2));
min_y_servo = min(CPL_servo(:,2));

CPL_cable_channel_insert = CPLbool('-',PLsquare(servo.width+servo.cable_gap,servo.height),PLsquare(servo.width,servo.height));
SG_insert_wo_cable_slot = SGofCPLz(CPL_cable_channel_insert,abs(min_y_cslot-min_y_servo));


CPL_hor_servo_slot = PLroundcorners(servo.PL_cable_gap_ver,[1,2,3,4],servo.cable_gap,'',0);
CPL_cable_channel_insert = CPLbool('-',CPL_cable_channel_insert,CPL_hor_servo_slot);

SG_mid_layer = SGofCPLz(CPL_main_mid,servo.height);

max_y_cslot = max(CPL_cable_slot_hor(:,2));
max_y_servo = max(CPL_servo(:,2));

SG_insert = SGofCPLz(CPL_cable_channel_insert,abs(max_y_cslot-max_y_servo));
SG_insert = SGtransrelSG(SG_insert,SG_mid_layer,'rotx',pi/2,'alignbottom','transy',-servo.shaft_offs-servo.length/2+servo.length);

SG_insert_wo_cable_slot = SGtransrelSG(SG_insert_wo_cable_slot,SG_mid_layer,'rotx',pi/2,'alignbottom','transy',(-servo.shaft_offs-servo.length/2)+abs(min_y_cslot-min_y_servo));

SG_mid_layer= SGcat(SG_mid_layer,SG_insert,SG_insert_wo_cable_slot);

CPL_insertslot = CPLconvexhull([PLcircle(servo.connect_R);PLtrans(PLcircle(servo.connect_R),[0 500])]);
CPL_top_layer = CPLbool('-',PLcircle(main_R),CPL_insertslot);
CPL_top_layer_chamfer = CPLbool('-',PLcircle(main_R-2),CPL_insertslot);

CPL_top_layer_chamfer = CPLbool('-',CPL_top_layer_chamfer,CPL_big_dove);
CPL_top_layer = CPLbool('-',CPL_top_layer,CPL_big_dove);

SG_top_layer = SGof2CPLsz(CPL_top_layer,CPL_top_layer_chamfer,2);

% PRINT HELP
if printhelp
	CPL_printhelp = CPLconvexhull(CPL_top_layer_chamfer);	
	CPL_printhelp = CPLbool('-',CPL_printhelp,CPL_big_dove);
	SG_printhelp = SGofCPLz(CPL_printhelp,0.3);
	SG_top_layer = SGcat(SG_top_layer,SGalignbottom(SG_printhelp,SG_printhelp));
end
% PRINTHELP

SG = SGstack('z',SG_bottom_layer,SG_mid_layer,SG_top_layer);

CPL_guide = [7 -5;6 0;-2.5 0;-2.5 5;-6 5;-9.75 -5];
CPL_double_dove = [-1.5 0;-2.5 2.5];
CPL_double_dove = CPLconvexhull([CPL_double_dove;VLswapX(CPL_double_dove)]);
CPL_double_dove = CPLbool('+',CPL_double_dove,VLswapY(CPL_double_dove));
CPL_guide = CPLbool('-',CPL_guide,PLtrans(CPL_double_dove,[0 -2.5]));

SG_guide = SGofCPLzdelaunayGrid(CPL_guide,0.5,0.5,0.5);
SG_guide=SGtransrelSG(SG_guide,SG,'rotx',pi/2,'transy',0.5,'transx',-out-9,'ontop');
SG_guide = SGfittoOutsideCPL(SG_guide,PLcircle(main_R-2,1000),'y+');
SG_guide = SGcat(SG_guide,SGmirror(SG_guide,'yz'));
SG_guide = SGcat(SG_guide,SGmirror(SG_guide,'xz'));

SG = SGcat(SG,SG_guide);

[SG_connector,CPL_connection] = SGconnAdaptersLCL('servo',servo_name,'adapter_type',conn_type,'cable',1);
CPL_connection = CPLaddauxpoints(CPL_connection,.5);
CPL_main_bottom_layer = CPLaddauxpoints(CPL_main_bottom_layer,.5);
SG_2main_connection = SGof2CPLsz(CPL_connection,CPL_main_bottom_layer,10);

SG_main_body = SGstack('z',SG_connector,SG_2main_connection,SG);

height = max(SG_main_body.VL(:,3));
H_Gripper_pos = [rotz(90)*rotx(180) [0;0;height]; 0 0 0 1];
SG_main_body = SGTset(SG_main_body,'GripperT',H_Gripper_pos);



%% Insert to close up servo
CPL_insert = CPLbuffer(CPL_big_dove,-0.2);
CPL_insert_bot = CPLbool('x',CPL_insert,PLcircle(main_R));
CPL_insert_top = CPLbool('x',CPL_insert,PLcircle(main_R-2));
SG_insert = SGofCPLz(CPL_insert_bot,34.5);
SG_insert_top = SGof2CPLsz(CPL_insert_bot,CPL_insert_top,2);
SG_insert = SGstack('z',SG_insert,SG_insert_top);
SG_insert = SGtransrelSG(SG_insert,SG_main_body,'aligntop',-10);
%% Gear to drive gearracks

CPL_gear = PLgearDIN(1,round(servo.connect_R*2.5));
CPL_gear = CPLbool('-',CPL_gear,PLcircle(servo.connect_R));

CPL_screw_pattern_TH = CPLcopyradial(PLcircle(servo.connect_screw_R),servo.connect_screw_circle_R,servo.connect_screw_Num);
CPL_screw_pattern_HH = CPLcopyradial(PLcircle(servo.connect_screw_R*2),servo.connect_screw_circle_R,servo.connect_screw_Num);

CPL_middle_TH = CPLbool('-',PLcircle(servo.connect_R),CPL_screw_pattern_TH);
CPL_middle_TH_bottom = CPLbool('-',CPL_middle_TH,PLcircle(servo.connect_top_R));
CPL_middle_HH = CPLbool('-',PLcircle(servo.connect_R),CPL_screw_pattern_HH);

SG_gear = SGofCPLz(CPL_gear,10);
SG_gear_mid_TH_bottom = SGofCPLz(CPL_middle_TH_bottom,servo.connect_top_H);
SG_gear_mid_TH = SGofCPLz(CPL_middle_TH,thread_length-3-servo.connect_top_H);
SG_gear_mid_HH = SGofCPLz(CPL_middle_HH,10-(thread_length-3));
SG_gear_mid = SGstack('z',SG_gear_mid_TH_bottom,SG_gear_mid_TH,SG_gear_mid_HH);
SG_gear = SGcat(SG_gear,SGalignbottom(SG_gear_mid,SG_gear));
SG_gear = SGtransrelSG(SG_gear,SG_main_body,'ontop',-8);

%% Grippers

CPL_double_dove_small = CPLbuffer(CPL_double_dove,-0.2);
CPL_add_to_doves = [-2.2 2.3;2.2 2.3;2.2 2.7;8 2.7;11 -.5;11 10;-2.2 10];
CPL_double_dove_small = CPLbool('+',CPL_double_dove_small,CPL_add_to_doves);
CPL_holes = CPLatPL(PLcircle(1.6),[1 6;8 6]);
CPL_double_dove_small = CPLbool('-',CPL_double_dove_small,CPL_holes);
SG_gripper_guide = SGtransrelSG(SGofCPLz(CPL_double_dove_small,100),'','rotx',pi/2,'transx',-out-9);

CPL_gear_rack = PLgearrackDIN(1,100.01);
SG_gear_rack = SGofCPLz(CPL_gear_rack,10.5);
SG_gear_rack = SGtransrelSG(SG_gear_rack,SG_gripper_guide,'rotz',-pi/2,'aligntop','right','alignback');
SG_gripper_guide = SGcat(SG_gripper_guide,SG_gear_rack);

CPL_gripper_act = PLsquare(2*(out+2.2+9),10);
CPL_gripper_attach_dov = CPLbool('+',PLsquare(6,6,3),VLswapX(PLsquare(6,6,3)));
CPL_gripper_attach_dov = CPLbool('+',CPL_gripper_attach_dov,PLtrans(PLsquare(6,3),[0 -3.5]));
CPL_gripper_attach_dov = PLtrans(CPL_gripper_attach_dov,[-15 -2]);
CPL_gripper_attach_dov = CPLbool('+',CPL_gripper_attach_dov,VLswapX(CPL_gripper_attach_dov));
CPL_gripper_act_dov = CPLbool('-',CPL_gripper_act,CPL_gripper_attach_dov);
SG_gripper_act = SGofCPLz(CPL_gripper_act,5);
SG_gripper_act_w_dov = SGofCPLz(CPL_gripper_act_dov,25);
SG_gripper_act = SGstack('z',SG_gripper_act,SG_gripper_act_w_dov);

CPL_brace = [0 0;-10 0;0 30];
SG_brace = SGofCPLz(CPL_brace,13);

SG_gripper_act = SGtransrelSG(SG_gripper_act,SG_gripper_guide,'ontop','alignleft','alignback',-10);
SG_brace = SGtransrelSG(SG_brace,SG_gripper_act,'roty',-pi/2,'rotx',pi/2,'aligntop','behind','alignleft');
SG_grippers = SGcat(SG_gripper_guide,SG_gripper_act,SG_brace);

SG_grippers_f_stops = SGtransrelSG(SG_grippers,SG,'aligntop',32.5,'aligninfront');
SG_grippers = SGtransrelSG(SG_grippers,SG,'aligntop',32.5,'transy',20+opening/2+jaw_th+1);

SG_grippers_mir = SGcat(SG_grippers,SGmirror(SGmirror(SG_grippers,'yz'),'xz'));

%stops

CPL_stop_top = [11 10;-2.2 10;-2.2 -3;11 -3];
CPL_stop_top = CPLbool('-',CPL_stop_top,CPL_holes);
SG_stop_top = SGofCPLz(CPL_stop_top,5);
SG_stop_top=SGtransrelSG(SG_stop_top,'','rotx',pi/2);

CPL_stop_bot = [11 10;-2.2 10;-2.2 -20;11 -20];
SG_stop_bot = SGofCPLzdelaunayGrid(CPL_stop_bot,5,0.5,0.5);

SG_stop_bot=SGtransrelSG(SG_stop_bot,SG_stop_top,'rotx',pi/2,'under');


SG_stop_top=SGtransrelSG(SG_stop_top,SG_grippers_f_stops,'aligntop',-30,'infront','alignleft');
SG_stop_bot=SGtransrelSG(SG_stop_bot,SG_stop_top,'under','alignbehind','alignleft');
SG_stop_bot = SGfittoOutsideCPL(SG_stop_bot,PLcircseg(main_R,100,-pi,0),'y+');


SG_stop= SGcat(SG_stop_top,SG_stop_bot);

%% Gripper inserts

CPL_jaws = PLsquare(opening+2*(jaw_th),2*(out+2.2+9));
CPL_jaws = CPLbool('-',CPL_jaws,PLsquare(opening,2*(out+2.2+9)));

SG_jaws_bottom = SGofCPLz(CPL_jaws,5);

CPL_jaws_top = PLsquare(opening+2*(jaw_th+10),2*(out+2.2+9));
CPL_jaws_top = CPLbool('-',CPL_jaws_top,PLsquare(opening,2*(out+2.2+9)));
SG_jaws_top = SGofCPLz(CPL_jaws_top,jaw_H-30);

CPL_jaw_dovs = PLtransR(CPLbuffer(CPL_gripper_attach_dov,-0.35),rot(pi/2));
CPL_jaw_dovs = PLroundcorners(CPL_jaw_dovs,[4,5,11,12],1);


CPL_jaw_dovs = PLtrans(PLtrans0(CPL_jaw_dovs),[-2.5-(opening/2)-jaw_th 0]);
CPL_jaw_dovs = CPLbool('+',CPL_jaw_dovs,VLswapX(CPL_jaw_dovs));

CPL_jaws = CPLbool('+',CPL_jaws,CPL_jaw_dovs);
SG_jaws = SGofCPLz(CPL_jaws,25);

SG_gripper_attachment = SGstack('z',SG_jaws_bottom,SG_jaws,SG_jaws_top);
SG_gripper_attachment = SGtrans0(SG_gripper_attachment);

H_Object = [rotx(0) [0;0;0]; 0 0 0 1];
SG_gripper_attachment = SGTset(SG_gripper_attachment,'ObjectPos',H_Object);
height = min(SG_gripper_attachment.VL(:,3));
H_Gripper_pos = [rotx(0) [0;0;height-3]; 0 0 0 1];
SG_gripper_attachment = SGTset(SG_gripper_attachment,'GripperT',H_Gripper_pos);


SG_gripper_attachment = SGtransrelSG(SG_gripper_attachment,SG_main_body,'alignTz',{'GripperT','GripperT'},'transz',10);

if ~isempty(SG_object)
	SG_object = SGtransrelSG(SG_object,SG_gripper_attachment,'alignTz',{'ObjectPos','ObjectPos'});	
	SG_gripper_attachment = SGslicebool(SG_gripper_attachment,SG_object);
end


% SG_gripper_attachment_ = SGanalyzeGroupParts(SG_gripper_attachment);

%% STLs
if output
	SGwriteSTL(SG_main_body,'Main Body');
	SGwriteSTL(SGtransrelSG(SG_gripper_attachment,'','rotx',pi),'Gripper Jaws');
	SGwriteSTL(SG_gear,'Middle Gear');
	SGwriteSTL(SGtransrelSG(SG_stop,'','rotx',pi/2),'SG_stop');
	SGwriteSTL(SG_grippers,'Grippers');	
	SGwriteSTL(SG_insert,'Insert');
	
end

%% Plots
SG_gripper_body = SGcatF(SG_main_body,SGontop(SG_grippers_mir,SG_main_body,-10));
SG_final = SGcat(SG_gripper_body,SG_insert);
if nargout== 0
	clf;
    SGplot(SG_final);	
	SG_stops = SGtransrelSG(SG_stop,SG_grippers_mir,'alignfront',5,'under',-38);
	SG_stops = SGcat(SG_stops,SGmirror(SGmirror(SG_stops,'yz'),'xz'));
end




end