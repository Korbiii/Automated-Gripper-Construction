%%   [SG, CPL] = SGdofsLCL([variable_name, value])
%    === INPUT PARAMETERS ===
%    'dof':					Type of degree of freedom. Rotation: x,y,z
%	 'servo':				Servo used for degree of freedom
%	 'attach_dof':			Previous degree of freedom
%	 'attach_servo':		Servo used in previous degree of freedom	
%	 'thread_length':		Threadlength of used screw: default = 12	
%    'fdm_help_layers':		Activate help Layers for FDM printing
%    === OUTPUT RESULTS ======
%    CPL:					CPL of crosssection of bottom of degree of freedom
%	 SG:					SG of degree of freedom
function [SG, CPL, dofs] = SGdofsLCL(varargin)

dofs = {'z','x','y','rotLock'};
SG = [];
CPL = [];
tol = 0.5;
servo_name = 'sm40bl';
thread_length = 12;
dof = 'x';

attach_dof = '';
attach_servo = 'sm40bl';

print_help_layer = 0;
cable = 1;
print_help=1;

i_idx = 1;
while i_idx<=size(varargin,2)
	if ~ischar(varargin{i_idx})
		i_idx = i_idx+1;
		continue;
	end
	switch varargin{i_idx}
		case 'dof'
			dof = varargin{i_idx+1};
			i_idx = i_idx+1;
		case 'attach_dof'
			attach_dof = varargin{i_idx+1};
			i_idx = i_idx+1;
		case 'servo'
			servo_name = lower(varargin{i_idx+1});
			i_idx = i_idx+1;
		case 'attach_servo'
			attach_servo = lower(varargin{i_idx+1});
			i_idx = i_idx+1;
		case 'fdm_help_layers'
			print_help_layer = 1;
		case 'thread_length'
			thread_length = varargin{i_idx+1};
		case 'cable'
			cable = 1;
		case 'print_help'
			print_help =1;
		otherwise
			error(varargin{i_idx} + " isn't a valid flag!");
	end
	i_idx = i_idx+1;
end

switch dof
	case 'z'
		servo = readServoFromTable(servo_name);
		servo.width = servo.width+tol;
		servo.length = servo.length+tol;
		servo.height = servo.height+tol;
		
		outer_radius_ser = max(sqrt(servo.shaft_offs^2+(servo.width/2)^2)+3,max(servo.PL_cable_gap_hor(:,1))+3);		
		connection_R = outer_radius_ser-1.5-tol;
		
		CPL_cable_opening_lid = PLroundcorners(PLsquare((outer_radius_ser-3)*2-1,servo.connect_R*2),[1,2,3,4],servo.connect_R);
		
		%% Main Body
		CPL_cable_gap = PLtrans(servo.PL_cable_gap_hor,[0 -((servo.length/2)-servo.shaft_offs)+0.5*tol]);
		CPL_out = CPLconvexhull([PLcircle(outer_radius_ser);PLgrow(PLroundcorners(CPL_cable_gap,[1,2,3,4],servo.cable_gap/4),1.5);PLtrans([-servo.width/2-5 0;servo.width/2+5 0],[0 -servo.length+servo.shaft_offs-thread_length])]);
		idx = find(CPL_out(:,2) == min(CPL_out(:,2)));
		CPL_out = PLroundcorners(CPL_out,idx',3);
		CPL_in = PLtrans(PLsquare(servo.width,servo.length),[0 -((servo.length/2)-servo.shaft_offs)+0.5*tol]);
		CPL_in = CPLbool('+',CPL_in,PLtrans(servo.PL_cable_gap_hor,[0 -((servo.length/2)-servo.shaft_offs)+0.5*tol]));
		CPL_in = PLroundcorners(CPL_in,[2,3,4,9,10,11],servo.cable_gap/4);
		CPL_in = CPLbool('+',CPL_in,PLtrans(PLsquare(servo.width-4,servo.length+8),[0 -(((servo.length+8)/2)-servo.shaft_offs)+0.5*tol+4]));
		
		CPL_in = CPLbool('+',CPL_in,CPL_cable_opening_lid);
		
		CPL_in_ledge = PLtrans(PLsquare(servo.width-8,servo.length-8),[0 -((servo.length/2)-servo.shaft_offs)+0.5*tol]);
		min_cable_gap = min(CPL_cable_gap(2,:));
		CPL_in_ledge = CPLbool('+',CPL_in_ledge,CPL_cable_gap);
		CPL_in_ledge = CPLbool('-',CPL_in_ledge,[-5000 min_cable_gap;5000 min_cable_gap;5000 -5000;-5000 -5000]);
		
		CPL_in_ledge = PLroundcorners(CPL_in_ledge,[1,2,3,6,7,8],servo.cable_gap/4);
		CPL_in_ledge = CPLbool('+',CPL_in_ledge,PLcircle(servo.connect_top_R));
		CPL_in_ledge = CPLbool('+',CPL_in_ledge,CPL_cable_opening_lid);
		CPL_in_ledge_wo_screws = CPL_in_ledge;
		
		CPL_bottom = [CPL_out;NaN NaN;CPL_in];
		
		if(~isnan(servo.screw_mount_y))
			screw_hor_distance = max(servo.screw_mount_y(2,:))*2;  % TODO: Make it better
			screw_ver_distance = max(servo.screw_mount_y(1,:))*2;
			CPL_screw_cuts = CPLbool('-',PLsquare(screw_hor_distance+2*(servo.mounting_screw_R*2),servo.length*2),PLsquare(screw_hor_distance-2*(servo.mounting_screw_R*2),servo.length*2));
			CPLout_w_screw_gaps = CPLbool('-',CPL_bottom,CPL_screw_cuts);
			SG_bottom = SGofCPLz(CPLout_w_screw_gaps,servo.height);
			CPL_screw_inserts_screw =  CPLbool('-',PLsquare(screw_hor_distance+2*(servo.mounting_screw_R*2),servo.height),PLsquare(screw_hor_distance-2*(servo.mounting_screw_R*2),servo.height));
			CPL_screw_inserts_screw =  CPLbool('-',CPL_screw_inserts_screw,PLtrans(PLcircle(servo.mounting_screw_R),[screw_hor_distance/2 screw_ver_distance/2]));
			CPL_screw_inserts_screw =  CPLbool('-',CPL_screw_inserts_screw,PLtrans(PLcircle(servo.mounting_screw_R),[-screw_hor_distance/2 screw_ver_distance/2]));
			CPL_screw_inserts_screw_head =  CPLbool('-',PLsquare(screw_hor_distance+2*(servo.mounting_screw_R*2),servo.height),PLsquare(screw_hor_distance-2*(servo.mounting_screw_R*2),servo.height));
			CPL_screw_inserts_screw_head =  CPLbool('-',CPL_screw_inserts_screw_head,PLtrans(PLcircle((servo.mounting_screw_R*2)),[screw_hor_distance/2 screw_ver_distance/2]));
			CPL_screw_inserts_screw_head =  CPLbool('-',CPL_screw_inserts_screw_head,PLtrans(PLcircle((servo.mounting_screw_R*2)),[-screw_hor_distance/2 screw_ver_distance/2]));
			
			SG_screw_insert_screw = SGofCPLz(CPL_screw_inserts_screw,thread_length-4-3);
			SG_screw_inserts_screw_head = SGofCPLz(CPL_screw_inserts_screw_head,0.01);
			
			SG_screw_inserts = SGstack('z',SG_screw_insert_screw,SG_screw_inserts_screw_head);
			SG_screw_inserts_1 = SGtransrelSG(SG_screw_inserts,SG_bottom,'rotx',pi/2,'aligntop','transy',servo.shaft_offs+0.5*tol-servo.length-4);
			SG_screw_inserts_2 = SGtransrelSG(SG_screw_inserts,SG_bottom,'rotx',pi/2,'rotz',pi,'aligntop','transy',servo.shaft_offs+0.5*tol+4);
			SG_screw_inserts_1 = SGfittoOutsideCPL(SG_screw_inserts_1,CPLaddauxpoints(CPL_out,0.5),'y-');
			SG_screw_inserts_2 = SGfittoOutsideCPL(SG_screw_inserts_2,CPLaddauxpoints(CPL_out,0.5),'y+');
			SG_bottom  = SGcat(SG_screw_inserts_1,SG_screw_inserts_2,SG_bottom);
		elseif(~isnan(servo.screw_mount_z))
			CPL_screw_holes = CPLatPL(PLcircle(servo.connect_screw_R),servo.screw_mount_z);
			CPL_screw_holes_addons = CPLatPL(PLcircle(servo.connect_screw_R+2),servo.screw_mount_z);
			CPL_in_ledge = CPLbool('-',CPL_in_ledge,PLtrans(CPL_screw_holes_addons,[0 -servo.shaft_offs]));
			CPL_in_ledge = CPLbool('+',CPL_in_ledge,PLtrans(CPL_screw_holes,[0 -servo.shaft_offs]));
			SG_bottom = SGofCPLz(CPL_bottom,servo.height);
		end
		
		SG_bottom_w_ledge = SGofCPLz([CPL_out;NaN NaN;CPL_in_ledge],5);
		SG = SGstack('z',SG_bottom_w_ledge,SG_bottom);
		CPL_out_conn = CPLbool('-',CPL_out,PLcircseg(outer_radius_ser+1,'',0,pi));
		
		CPL = CPLconvexhull([PLcircle(outer_radius_ser);CPLbuffer(CPL_in_ledge_wo_screws,2)]);
		CPL = [CPL;NaN NaN;CPL_in_ledge_wo_screws];
		%% Lid
		CPL_lid_outline = CPL_out;
		CPL_lid_top = [CPL_lid_outline;NaN NaN;PLcircle(connection_R+.4)];
		CPL_lid_top_chamfer = [PLgrow(CPL_lid_outline,1);NaN NaN;PLcircle(connection_R+.4)];
		CPL_lid_bottom = [CPL_lid_outline;NaN NaN;PLroundcorners(PLsquare((connection_R+.5)*2-1,servo.connect_R*2),[1,2,3,4],servo.connect_R)];
		
		if(~isnan(servo.screw_mount_z))
			CPL_screw_holes = PLtrans(CPLatPL(PLcircle(servo.connect_screw_R),servo.screw_mount_z),[0 -servo.shaft_offs]);
			CPL_lid_top = CPLbool('-',CPL_lid_top,CPL_screw_holes);
			CPL_lid_top_chamfer= CPLbool('-',CPL_lid_top_chamfer,CPL_screw_holes);
			CPL_lid_bottom= CPLbool('-',CPL_lid_bottom,CPL_screw_holes);
		end
		
		
		
		SG_lid_top = SGstack('z',SGofCPLz(CPL_lid_top,6),SGof2CPLsz(CPL_lid_top,CPL_lid_top_chamfer,1));
		SG_lid_bottom = SGofCPLz(CPL_lid_bottom,3);
		if print_help_layer
			CPL_lid_bottom_help_layer = CPLconvexhull(CPL_lid_bottom);
			SG_lid_print_help_layer = SGofCPLz(CPL_lid_bottom_help_layer,0.3);
			SG_lid_bottom = SGcat(SG_lid_bottom,SGaligntop(SG_lid_print_help_layer,SG_lid_bottom));
		end		
		
		SG_lid = SGstack('z',SG_lid_bottom,SG_lid_top);
		
		
		
		CPL_slide_in_attachments = PLroundcorners(PLsquare(servo.width-4,servo.height-5),[1,2],5);
		CPL_slide_in_attachments_small = PLroundcorners(PLsquare(servo.width-6,servo.height-7),[1,2],5);
		CPL_slide_in_attachments_small = PLtrans(CPL_slide_in_attachments_small,[0 1]);
		
		if(~isnan(servo.screw_mount_y))
			CPL_screw_holes = PLtrans(CPLatPL(PLcircle(servo.mounting_screw_R),servo.screw_mount_y),[0 -2.5]);
			CPL_slide_in_attachments = CPLbool('-',CPL_slide_in_attachments,CPL_screw_holes);
			CPL_slide_in_attachments_small = CPLbool('-',CPL_slide_in_attachments_small,CPL_screw_holes);
		end
		SG_slide_in_attachments = SGof2CPLsz(CPL_slide_in_attachments,CPL_slide_in_attachments_small,1.5);
		SG_slide_in_attachments = SGcat(SG_slide_in_attachments,SGmirror(SG_slide_in_attachments,'xy'));
		SG_slide_in_attachment_1 = SGtransrelSG(SG_slide_in_attachments,SG_lid,'rotx',pi/2,'under','transy',servo.shaft_offs+(tol/2)+1.5);
		SG_slide_in_attachment_2 = SGtransrelSG(SG_slide_in_attachments,SG_lid,'rotx',pi/2,'under','transy',-servo.length+servo.shaft_offs+(tol/2)-1.5);
		
		CPL_lid_centering = CPLbool('-',CPLbuffer(CPL_lid_outline,2),CPLbuffer(CPL_lid_outline,0.5));
		CPL_lid_centering_top = CPLbool('-',CPLbuffer(CPL_lid_outline,2),CPL_lid_outline);
		CPL_lid_centering_small = CPLbool('-',CPLbuffer(CPL_lid_outline,0.75),CPLbuffer(CPL_lid_outline,0.5));
		CPL_lid_centering_top_small = CPLbool('-',CPLbuffer(CPL_lid_outline,0.01),CPL_lid_outline);
		
		
		CPL_lid_centering = CPLaddauxpoints(CPL_lid_centering,0.5);
		CPL_lid_centering_small = CPLaddauxpoints(CPL_lid_centering_small,0.5);
		CPL_lid_centering_top = CPLaddauxpoints(CPL_lid_centering_top,0.5);
		CPL_lid_centering_top_small = CPLaddauxpoints(CPL_lid_centering_top_small,0.5);
		
		SG_lid_centering_bottom = SGof2CPLsz(CPL_lid_centering_small,CPL_lid_centering,2);
		SG_lid_centering_top = SGof2CPLsz(CPL_lid_centering_top,CPL_lid_centering_top_small,9);
		SG_lid_centering = SGtrans(SGstack('z',SG_lid_centering_bottom,SG_lid_centering_top),[0 0 -2]);
		
		SG_lid = SGcat(SG_lid,SG_slide_in_attachment_1,SG_slide_in_attachment_2,SG_lid_centering);
	
				
		
		H_b_lid = [roty(180)*rotz(-90) [0;0;0]; 0 0 0 1];
		SG_lid = SGTset(SG_lid,'B',H_b_lid);
		H_f_lid = [rotx(0) [0;0;3]; 0 0 0 1];
		SG_lid = SGTset(SG_lid,'F',H_f_lid);
		if attach_dof ~= 0
			[SG_connector,CPL_connector] = SGconnAdaptersLCL('servo',attach_servo,'adapter_type',attach_dof);
			CPL_coonector = CPLaddauxpoints(CPL_connector,0.5);
			CPL = CPLaddauxpoints(CPL,0.5);
			SG_connection = SGof2CPLsz(CPL_coonector,CPL,10);
			SG = SGstack('z',SG_connector,SG_connection,SG);
		end
		H_f = [roty(180) [0;0;max(SG.VL(:,3))]; 0 0 0 1];
		SG = SGTset(SG,'F',H_f);
		

	case  {'x','y','xy','yx'}
		servo = readServoFromTable(servo_name);
		distance_axis = (servo.length/2)+servo.shaft_offs;
		servo.width = servo.width+tol;
		servo.length = servo.length+tol;
		servo.height = servo.height+tol;
		outer_radius = servo.width/2+servo.cable_gap+3;
		start_cable_gap = servo.height/2+min(servo.PL_cable_gap_ver(:,2));
		cable_gap_length = max(servo.PL_cable_gap_ver(:,2))-min(servo.PL_cable_gap_ver(:,2));
		
		CPL_outside = CPLconvexhull([PLcircle(outer_radius);[-outer_radius,-distance_axis;outer_radius,-distance_axis]]);
		CPL_outside = CPLbool('-',CPL_outside,CPLconvexhull([PLcircle(servo.connect_R);[-servo.connect_R-tol,30;servo.connect_R+tol,30]]));
		
		CPL_outside_small = CPLconvexhull([PLcircle(outer_radius-3);[-outer_radius+3,-distance_axis;outer_radius-3,-distance_axis]]);
		CPL_outside_small= CPLbool('-',CPL_outside_small,CPLconvexhull([PLcircle(servo.connect_R);[-servo.connect_R-tol,30;servo.connect_R+tol,30]]));
		
		CPL_outside_w_servo_slot = CPLbool('-',CPL_outside,PLsquare(servo.width,servo.length*2));
		CPL_outside_w_cable_slots = CPLbool('-',CPL_outside,PLsquare(servo.width+2*servo.cable_gap,servo.length*2));
		
		
		CPL_outside_w_screw_slots = CPL_outside_w_servo_slot;
		
		if(~isnan(servo.screw_mount_x))
			t_min = min(servo.PL_cable_gap_ver(:,2));
			t_max = max(servo.PL_cable_gap_ver(:,2));
			avail_screw_mounts = (servo.screw_mount_x(:,1) > t_max | servo.screw_mount_x(:,1) < t_min);
			avail_screw_mounts = servo.screw_mount_x(avail_screw_mounts,:);
			bot = min(avail_screw_mounts(:,2))-servo.connect_screw_R;
			top = max(avail_screw_mounts(:,2))+servo.connect_screw_R;
			CPL_outside_w_screw_slots = CPLbool('-',CPL_outside_w_screw_slots,PLtrans(PLsquare(outer_radius*2,top-bot),[0 -servo.shaft_offs+t_max-tol]));
			
		elseif(~isnan(servo.screw_mount_z))
			CPL_screw_holes = CPLatPL(PLcircle(servo.connect_screw_R),servo.screw_mount_z);
			CPL_screw_holes = PLtrans(CPL_screw_holes,[0 -servo.shaft_offs]);
			CPL_outside = CPLbool('-',CPL_outside,CPL_screw_holes);
			CPL_outside_small = CPLbool('-',CPL_outside_small,CPL_screw_holes);
		end
		
		side_panels_thickness = ((servo.connect_dis-servo.height)/2)-tol;
		CPL_outside_small = CPLaddauxpoints(CPL_outside_small,0.5);
		CPL_outside = CPLaddauxpoints(CPL_outside,0.5);
		
		SG_front = SGof2CPLsz(CPL_outside_small,CPL_outside,side_panels_thickness,'angle');
		SG_1 = SGofCPLz(CPL_outside_w_screw_slots,start_cable_gap);
		SG_2 = SGofCPLz(CPL_outside_w_cable_slots,cable_gap_length);
		SG_3 = SGofCPLz(CPL_outside_w_screw_slots,servo.height-cable_gap_length-start_cable_gap);
		SG_back = SGof2CPLsz(CPL_outside,CPL_outside_small,side_panels_thickness,'angle');
		
		if(~isnan(servo.screw_mount_x))
			
			CPL_screw_TH = CPLatPL(PLcircle(servo.mounting_screw_R),avail_screw_mounts);
			CPL_servo_side = PLsquare(servo.height,servo.length);
			CPL_servo_side = CPLbool('-',CPL_servo_side,CPL_screw_TH);
			
			CPL_servo_side = CPLbool('-',CPL_servo_side,[5000 top; 5000 top+5000;-5000 top+5000;-5000 top]);
			CPL_servo_side = CPLbool('-',CPL_servo_side,[5000 bot; 5000 bot-5000;-5000 bot-5000;-5000 bot]);
			
			CPL_cable_slot = [servo.PL_cable_gap_ver(:,2) servo.PL_cable_gap_ver(:,1)];
			
			
			CPL_screw_HH = CPLatPL(PLcircle(servo.mounting_screw_R*2),avail_screw_mounts);
			CPL_servo_side_HH = PLsquare(servo.height,servo.length);
			CPL_servo_side_HH = CPLbool('-',CPL_servo_side_HH,CPL_screw_HH);
			
			CPL_servo_side_HH = CPLbool('-',CPL_servo_side_HH,[5000 top; 5000 top+5000;-5000 top+5000;-5000 top]);
			CPL_servo_side_HH = CPLbool('-',CPL_servo_side_HH,[5000 bot; 5000 bot-5000;-5000 bot-5000;-5000 bot]);
			
			left = max(CPL_cable_slot(:,1));
			CPL_servo_side_right = CPLbool('-',CPL_servo_side,[left -5000;left 5000;left-5000 5000;left-5000 -5000]);
			CPL_servo_side_HH_right = CPLbool('-',CPL_servo_side_HH,[left -5000;left 5000;left-5000 5000;left-5000 -5000]);
			
			right = min(CPL_cable_slot(:,1));
			CPL_servo_side_left = CPLbool('-',CPL_servo_side,[right -5000;right 5000;right+5000 5000;right+5000 -5000]);
			CPL_servo_side_HH_left = CPLbool('-',CPL_servo_side_HH,[right -5000;right 5000;right+5000 5000;right+5000 -5000]);
			
			SG_screw_TH = SGofCPLz(CPL_servo_side_right,thread_length-servo.mounting_screw_depth);
			SG_screw_HH = SGofCPLz(CPL_servo_side_HH_right,outer_radius-(servo.width/2)-thread_length+servo.mounting_screw_depth);
			SG_screw_TH = SGstack('z',SG_screw_TH,SG_screw_HH);
			SG_screw_TH = SGtransrelSG(SG_screw_TH,SG_3,'roty',-pi/2,'aligntop','alignleft','transy',-servo.shaft_offs+tol);
			SG_screw_TH = SGcat(SG_screw_TH,SGmirror(SG_screw_TH,'yz'));
			SG_3 = SGcat(SG_3,SG_screw_TH);
			
			SG_screw_TH = SGofCPLz(CPL_servo_side_left,thread_length-servo.mounting_screw_depth);
			SG_screw_HH = SGofCPLz(CPL_servo_side_HH_left,outer_radius-(servo.width/2)-thread_length+servo.mounting_screw_depth);
			SG_screw_TH = SGstack('z',SG_screw_TH,SG_screw_HH);
			SG_screw_TH = SGtransrelSG(SG_screw_TH,SG_1,'roty',-pi/2,'aligntop','alignleft','transy',-servo.shaft_offs+tol);
			SG_screw_TH = SGcat(SG_screw_TH,SGmirror(SG_screw_TH,'yz'));
			SG_1 = SGcat(SG_1,SG_screw_TH);
		end
		
		SG = SGstack('z',SG_front,SG_1,SG_2,SG_3,SG_back);
		SG = SGtransrelSG(SG,'','rotx',pi/2,'centery');
		
		CPL_slice = CPLofSGslice(SG,min(SG.VL(:,3))+0.1);
		CPL_out = CPLconvexhull(CPL_slice);
		CPL_inner = CPLbool('+',PLsquare(servo.width,servo.height-10),VLswapY(servo.PL_cable_gap_ver));
		CPL = [CPL_out;NaN NaN;CPL_inner];
		CPL = CPLaddauxpoints(CPL,0.5);
		H_f = [rotx(90) [0;0;0]; 0 0 0 1];
		SG = SGTset(SG,'F',H_f);
		if attach_dof ~= 0
			[SG_connector,CPL_connector] = SGconnAdaptersLCL('servo',attach_servo,'adapter_type',attach_dof);
			SG_connection = SGof2CPLsz(CPL_connector,CPL,10,'center');
			SG = SGstack('z',SG_connector,SG_connection,SG);
		end
		
		SG = SGtransrelSG(SG,'','alignbottom');
	case 'rotLock'		
		outer_R = 32.5;
		inner_R = 15.5;
		connect_R = 23.5;
		height = 9.5;
		
		CPL_connection_bottom = [connect_R+.55 height;connect_R+.5 5.5;connect_R-.5 5.5;21 5.5;inner_R+1.05 height];
		CPL_connection_bottom = PLroundcorners(CPL_connection_bottom,5,2);
		CPL_connection_stop = CPLbool('+',CPL_connection_bottom,[connect_R+.55 0;connect_R+.55 height;connect_R-3 height;connect_R-3 0]);
		
		
		SG_stub = SGofCPLrota(CPL_connection_bottom,(1/4)*pi,false);
		SG_stub_stop = SGofCPLrota(CPL_connection_stop,0.2,false);
		SG_stub_stop = SGtransR(SG_stub_stop,rot(0,0,-0.2));
		SG_stub = SGcat(SG_stub,SG_stub_stop);
		SG_stubs = SGcircularpattern(SG_stub,3,2*pi/3);
		
		CPL_main = [outer_R-0.5 height;outer_R-0.5 0;connect_R+.5 0;connect_R+.5 height];
		CPL_main = PLroundcorners(CPL_main,1,2.5,'',0);
		SG_main = SGofCPLrota(CPL_main,((2/3)*pi)-0.4,false);
		
		
		SG_main = SGcircularpattern(SG_main,3,2*pi/3);
		
		
		CPL_main_connections = [outer_R-4 height;outer_R-4 0;connect_R+.5 0;connect_R+.5 height];
		SG_main_connections = SGofCPLrota(CPL_main_connections,0.4,false,-0.4);
		
		SG_main_connections = SGcircularpattern(SG_main_connections,3,2*pi/3);
		SG_main =SGcat(SG_main,SG_main_connections);
		
		
	
		
		SG = SGcat(SG_stubs,SG_main);
		
		servo = readServoFromTable(attach_servo);
		if cable
			CPL_bottom = PLcircle(outer_R-0.5);
			CPL_bottom = CPLbool('-',CPL_bottom,PLcircle(servo.connect_R));
		else
			CPL_bottom = PLcircle(outer_R-0.5);
		end
		
		
		if attach_dof ~= 0
			[SG_connector,CPL_connector] = SGconnAdaptersLCL('servo',attach_servo,'adapter_type',attach_dof,'cable',cable);
			if attach_dof == 'z'
				CPL_bottom = CPLbool('-',CPL_bottom,PLcircle(servo.connect_R));
				CPL_connector = CPLbool('-',CPL_connector,PLcircle(servo.connect_R));
			end
			SG_connection = SGof2CPLsz(CPLaddauxpoints(CPL_connector,0.5),CPL_bottom,10,'center');
			if print_help
				SG_help_layer = SGofCPLz(PLcircle(outer_R-0.5),0.2);
				SG_connection = SGcat(SG_connection,SGaligntop(SG_help_layer,SG_connection));
			end		
			
			SG = SGstack('z',SG_connector,SG_connection,SG);
		end
				
end

if nargout == 0
	clf;
	SGplot(SG);
	SGwriteSTL(SG);
end
end