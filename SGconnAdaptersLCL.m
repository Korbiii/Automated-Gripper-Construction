function [SG, CPL] = SGconnAdaptersLCL(varargin)

tol = 0.5;
servo_name = 'sm40bl';
screw_length = 12-3;
adapter_type = 'rotLock';
cable = 1;
print_help_layer = 0;


i_idx = 1;
while i_idx<=size(varargin,2)
	if ~ischar(varargin{i_idx})
		i_idx = i_idx+1;
		continue;
	end
	switch varargin{i_idx}
		case 'adapter_type'
			adapter_type = varargin{i_idx+1};
			i_idx = i_idx+1;
		case 'thread_length'
			screw_length = varargin{i_idx+1};
		case 'servo'
			servo_name = lower(varargin{i_idx+1});
			i_idx = i_idx+1;
		case 'cable'
			cable = varargin{i_idx+1};
		case 'fdm_help_layers'
			print_help_layer = 1;
		otherwise
			error(varargin{i_idx} + " isn't a valid flag!");
	end
	i_idx = i_idx+1;
end

servo = readServoFromTable(servo_name);

switch adapter_type
	case 'z'
		outer_radius_ser = max(sqrt(servo.shaft_offs^2+(servo.width/2)^2)+3,max(servo.PL_cable_gap_hor(:,1))+3);
		connection_R = outer_radius_ser-1.5-tol;
		CPL_connection = PLcircle(connection_R);
		CPL_screw_holes = CPLcopyradial(PLcircle(servo.screw_R),servo.mount_screw_R,servo.mount_screw_Num);
		if cable
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
		
		if cable
			CPL = [PLcircle(connection_R);NaN NaN;PLcircle(servo.mount_screw_R+servo.screw_R+2+6)];
		else
			CPL = PLcircle(connection_R);
		end
		
		min_z_bott_con = min(SG.VL(:,3));
		H_b = [rotx(0) [0;0;min_z_bott_con]; 0 0 0 1];
		SG = SGTset(SG,'B',H_b);		
		
	case {'x','y','xy','yx'}
		CPL_screw_holes = CPLcopyradial(PLcircle(servo.screw_R),servo.mount_screw_R,servo.mount_screw_Num);
		cable_gap_R = sqrt(servo.shaft_offs^2+(servo.width/2)^2);
		CPL_bracket = CPLconvexhull([PLcircle(servo.width/2+5);-(servo.width/2+5) cable_gap_R+12;(servo.width/2+5) cable_gap_R+12]);
		CPL_bracket_small = CPLconvexhull([PLcircle(servo.width/2+3);-(servo.width/2+3) cable_gap_R+12;(servo.width/2+3) cable_gap_R+12]);
		CPL_bracket = CPLbool('-',CPL_bracket,CPL_screw_holes);
		CPL_bracket_small = CPLbool('-',CPL_bracket_small,CPL_screw_holes);
		
		CPL_bracket_w_cut = CPLbool('-',CPL_bracket,PLtrans(PLroundcorners(PLsquare(servo.attach_top_R*2,servo.width+10),[1,2,3,4],servo.attach_top_R/2),[0 -10]));
		
		SG_bracket_w_cut = SGofCPLz(CPL_bracket_w_cut,servo.attach_top_H);
		SG_bracket = SGof2CPLsz(CPL_bracket,CPL_bracket_small,2);
		missing_screw_length = screw_length -2 - servo.attach_top_H;
		SG_screw_length = SGof2CPLsz([PLcircle(servo.mount_screw_R+2*servo.screw_R+missing_screw_length);NaN NaN;CPL_screw_holes],[PLcircle(servo.mount_screw_R+2*servo.screw_R);NaN NaN;CPL_screw_holes],missing_screw_length);
		
		SG = SGstack('z',SG_bracket_w_cut,SG_bracket,SG_screw_length);
		CPL_chamfer = [0 0;5 0;0 -5];
		SG_chamfer = SGofCPLz(CPL_chamfer,(servo.width/2+5)*2);
		SG_bracket_left = SGtransrelSG(SG,SG,'rotx',pi/2,'transy',-servo.connect_dis/2);
		SG_chamfer_left = SGtransrelSG(SG_chamfer,SG_bracket_left,'roty',pi/2,'rotz',pi,'aligntop','centerx','behind');
		SG_bracket_right = SGtransrelSG(SG,SG,'rotx',pi/2,'rotz',pi,'transy',servo.connect_dis/2);
		SG_chamfer_right = SGtransrelSG(SG_chamfer,SG_bracket_right,'roty',pi/2,'aligntop','centerx','infront');
		
		SG = SGcat(SG_bracket_right,SG_bracket_left,SG_chamfer_left,SG_chamfer_right);
		
		H_b = [rotx(90) [0;0;0]; 0 0 0 1];
		SG = SGTset(SG,'B',H_b);
		
		CPL = CPLconvexhull(CPLofSGslice(SG,max(SG.VL(:,3))-0.1));
		gap_bracket = screw_length-missing_screw_length;
		CPL = [CPL;NaN NaN;PLsquare(2*max(CPL(:,1)-5),2*max(CPL(:,2)-gap_bracket-5))];
		if adapter_type == 'x'
			SG = SGtransR(SG,rot(0,0,pi/2));
			CPL = PLtransR(CPL,rot(pi/2));
		end
	case 'rotLock'
		outer_R = 32.5;
		inner_R = 15.5;
		connect_R = 23.5;
		height = 10;
		
		CPL_connection_bottom = [connect_R+.5 height;connect_R+.5 4;connect_R-.5 5;21 5;inner_R+.5 height];
				
		CPL_main = CPLbool('-',PLcircle(outer_R+2),PLcircle(outer_R));
		CPL_main = CPLbool('+',CPL_main,PLcircle(inner_R));
		
		if cable
			CPL_main = CPLbool('-',CPL_main,PLcircle(inner_R-5));
		end
			
		SG_main = SGofCPLz(CPL_main,height);
		
		CPL_connection_bottom_buf = PLtrans(CPL_connection_bottom,[0 -0.5]);
		CPL_connection_bottom_buf = CPLbuffer(CPL_connection_bottom_buf,0.5);
		CPL_connection_stub = CPLbool('-',[inner_R 0;connect_R 0;connect_R 10;inner_R 10],CPL_connection_bottom_buf);
		
		SG_stub = SGofCPLrota(CPL_connection_stub,(1/3)*pi,false);
		SG_stubs = SGcircularpattern(SG_stub,3,2*pi/3);
		
		SG_main = SGcat(SG_main,SG_stubs);
		
		CPL_bottom_lid = PLcircle(outer_R+2);
		if cable
			CPL_bottom_lid = CPLbool('-',CPL_bottom_lid,PLcircle(inner_R-5));
		end
		
		SG_bottom_lid = SGofCPLz(CPL_bottom_lid,2);
		SG_bottom_gap = SGofCPLz(CPLbool('-',PLcircle(outer_R+2),PLcircle(outer_R)),4);
		
		SG = SGstack('z',SG_bottom_gap,SG_main,SG_bottom_lid);
		
		if cable
			CPL =  CPLbool('-',PLcircle(outer_R+2),PLcircle(inner_R-5));
		else
			CPL =  PLcircle(outer_R+2);
		end
	otherwise
		disp('Connection Type doesnt exist');
end
if cable == 0
	CPL = CPLconvexhull(CPL);
end

end