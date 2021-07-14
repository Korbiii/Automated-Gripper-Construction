%%   [SG, CPL] = SGconnAdaptersLCL([variable_name, value])
%    === INPUT PARAMETERS ===
%    'adapter_type':		Type of adapter. Rotation x,z,rotLock
%	 'servo':				Servo used for adapters that arent rotLock
%	 'cable':				Hole for cable. default = Cable (1)
%	 'thread_length':		Threadlength of used screw: default = 12	
%    'fdm_help_layers':		Activate help Layers for FDM printing
%    === OUTPUT RESULTS ======
%    CPL:					CPL of crosssection of top of adapter
%	 SG:					SG of Adapter
function [SG, CPL, conn_dofs] = SGconnAdaptersLCL(varargin)

conn_dofs = {'z','x','y','rotLock','legacy'};
tol = 0.5;
servo_name = 'sm40bl';
screw_length = 12;
adapter_type = 'x';
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
		CPL_screw_holes = CPLcopyradial(PLcircle(servo.connect_screw_R),servo.connect_screw_circle_R,servo.connect_screw_Num);
		if cable
			CPL_cable_slot = PLkidney(servo.connect_screw_circle_R+servo.connect_screw_R+2,servo.connect_screw_circle_R+servo.connect_screw_R+2+6,1.25*pi);
			CPL_connection = CPLbool('-',CPL_connection,CPL_cable_slot);
		end
		
		CPL_connection = CPLbool('-',CPL_connection,CPL_screw_holes);
		CPL_connection_mid_attach = CPLbool('-',CPL_connection,PLcircle(servo.connect_top_R+0.5));
		SG_bottom = SGofCPLz(CPL_connection_mid_attach,servo.connect_top_H+0.5);
		SG = SGofCPLz(CPL_connection,9-servo.connect_top_H);
		SG= SGstack('z',SG_bottom,SG);
		
		if print_help_layer
			SG_help_layer = SGofCPLz(PLcircle(connection_R),0.3);
			SG = SGcat(SG,SGaligntop(SG_help_layer,SG));
		end
		
		if cable
			CPL = [PLcircle(connection_R);NaN NaN;PLcircle(servo.connect_screw_circle_R+servo.connect_screw_R+2+6)];
		else
			CPL = PLcircle(connection_R);
		end
		
		min_z_bott_con = min(SG.VL(:,3));
		H_b = [rotx(0) [0;0;min_z_bott_con]; 0 0 0 1];
		SG = SGTset(SG,'B',H_b);		
		
	case {'x','y','xy','yx'}
		CPL_screw_holes = CPLcopyradial(PLcircle(servo.connect_screw_R),servo.connect_screw_circle_R,servo.connect_screw_Num);
		cable_gap_R = sqrt(servo.shaft_offs^2+(servo.width/2)^2);
		CPL_bracket = CPLconvexhull([PLcircle(servo.width/2+5);-(servo.width/2+5) cable_gap_R+12;(servo.width/2+5) cable_gap_R+12]);
		CPL_bracket_small = CPLconvexhull([PLcircle(servo.width/2+3);-(servo.width/2+3) cable_gap_R+12;(servo.width/2+3) cable_gap_R+12]);
		CPL_bracket = CPLbool('-',CPL_bracket,CPL_screw_holes);
		CPL_bracket_small = CPLbool('-',CPL_bracket_small,CPL_screw_holes);
		
		CPL_bracket_w_cut = CPLbool('-',CPL_bracket,PLtrans(PLroundcorners(PLsquare(servo.connect_top_R*2,servo.width+10),[1,2,3,4],servo.connect_top_R/2),[0 -10]));
		
		SG_bracket_w_cut = SGofCPLz(CPL_bracket_w_cut,servo.connect_top_H);
		SG_bracket = SGof2CPLsz(CPL_bracket,CPL_bracket_small,2);
		missing_screw_length = screw_length-3 -1 - servo.connect_top_H;
		SG_screw_length = SGof2CPLsz([PLcircle(servo.connect_screw_circle_R+2*servo.connect_screw_R+missing_screw_length);NaN NaN;CPL_screw_holes],[PLcircle(servo.connect_screw_circle_R+2*servo.connect_screw_R);NaN NaN;CPL_screw_holes],missing_screw_length);
 		
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
		
		CPL = CPLconvexhull(CPLofSGslice(SG,max(SG.VL(:,3))-1));
		gap_bracket = screw_length-missing_screw_length-3;
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
		
		CPL_connection_bottom = [connect_R+.55 height;connect_R+.5 4;connect_R-.5 5;21 5;inner_R+.55 height];
				
		CPL_main = CPLbool('-',PLcircle(outer_R+2),PLcircle(outer_R));
		CPL_main_cut = CPLconvexhull([PLcircseg(outer_R+3,'',pi-0.2,pi+1);0 0]);
		CPL_main_cut = VLswapY(VLswapX(CPL_main_cut));
		CPL_click_mask = CPLconvexhull([PLcircseg(outer_R+3,'',pi-.1,pi+1.1);0 0]);
		CPL_clicks = CPLbool('x',CPL_main,CPL_click_mask);
		
		CPL_inner_catch = PLroundcorners([outer_R+3 0;outer_R-3.5 0;outer_R+3 -10],[2,3],[1.5,6]);
		CPL_inner_catch = CPLbool('x',CPL_inner_catch,PLcircle(outer_R+2));		
		CPL_clicks = VLswapY(VLswapX(CPL_clicks));
		CPL_clicks =CPLbool('+',CPL_clicks,CPL_inner_catch);
		
		CPL_thumb_button = PLroundcorners([outer_R 0;outer_R+10 0;outer_R+10 6;+outer_R-10 10],[2,3],2);	
		CPL_thumb_button = PLtransR(CPL_thumb_button,rot(-0.1));
		CPL_thumb_button = CPLbool('-',CPL_thumb_button,PLcircle(outer_R));							
		CPL_clicks =CPLbool('+',CPL_clicks,CPL_thumb_button);
		
		
		SG_clicks = SGofCPLz(CPL_clicks,height-0.6);
		
		CPL_main = CPLbool('-',CPL_main,CPL_main_cut);
		CPL_main = CPLbool('+',CPL_main,PLcircle(inner_R));
		
		if cable
			CPL_main = CPLbool('-',CPL_main,PLcircle(inner_R-5));
		end	
		
		SG_main = SGofCPLz(CPL_main,height);
		SG_main = SGcat(SG_main,SGalignbottom(SG_clicks,SG_main,0.3));
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
	case {'xr','yr'}  %%TODOOOOO
		inner_run_R = 29.7;
		outer_run_R = 37.7;
		outer_run_H = 17;
		inn_run_H = 6;
		legacy_adapter_H = 35;
		legacy_inner_R = 22.5;
		wall_thick = 2.5;
		screw_R = servo.connect_screw_R+0.5;
		screw_head_R = 3;
		cable_gap = 7;
		servo_mid_R = 7.5;
		servo_attach_R = 19;
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
		SG = SGtransrelSG(SG_bottom_conn,'','transz',-legacy_adapter_H/2+2);			
		
		CPL = PLroundcorners(PLsquare(1.25*outer_R,legacy_adapter_H-10),[1,2,3,4],5);
		CPL = [CPL;NaN NaN;CPLbuffer(CPL,-2.5)];
		CPL = CPLaddauxpoints(CPL,.5);
		
		SG_connection = SGofCPLz(CPL,0.1);
		SG_connection = SGtransrelSG(SG_connection,SG,'rotx',pi/2,'rotz',pi/2,'left',1,'centerz');
		CPL_circ_fit = PLcircseg(outer_R,1000,pi/2,3*pi/2);
		SG_connection = SGfittoOutsideCPL(SG_connection,CPL_circ_fit,'x+');
		SG = SGcat(SG,SG_connection);
		SG = SGtransrelSG(SG,'','roty',pi/2,'centerx','rotz',pi/2);
		
	case 'legacy'
		inner_run_R = 29.7;
		outer_run_R = 37.7;
		outer_run_H = 17;
		inn_run_H = 6;
		legacy_adapter_H = 35;
		legacy_inner_R = 22.5;
		wall_thick = 2.5;
		screw_R = servo.connect_screw_R+0.5;
		screw_head_R = 3;
		cable_gap = 7;
		servo_mid_R = 7.5;
		servo_attach_R = 19;		
		conn_screw_circle_radius = 10.5;
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
		SG = SGtransrelSG(SG_bottom_conn,'','transz',-legacy_adapter_H/2+2);	
		
		CPL = PLroundcorners(PLsquare(1.25*outer_R,legacy_adapter_H-10),[1,2,3,4],5);
		CPL = [CPL;NaN NaN;CPLbuffer(CPL,-2.5)];
		CPL = CPLaddauxpoints(CPL,.5);
		
		SG_connection = SGofCPLz(CPL,0.1);
		SG_connection = SGtransrelSG(SG_connection,SG,'rotx',pi/2,'rotz',pi/2,'left',1,'centerz');
		CPL_circ_fit = PLcircseg(outer_R,1000,pi/2,3*pi/2);
		SG_connection = SGfittoOutsideCPL(SG_connection,CPL_circ_fit,'x+');
		SG = SGcat(SG,SG_connection);
		min_z_bott_con = min(SG.VL(:,3));
		H_b = [rotx(0) [0;0;min_z_bott_con]; 0 0 0 1];
		SG = SGTset(SG,'B',H_b);
		SG = SGtransrelSG(SG,'','roty',pi/2,'centerx','rotz',pi/2);
		
		
	otherwise
		disp('Connection Type doesnt exist');
end
if cable == 0
	CPL = CPLconvexhull(CPL);
end
if nargout == 0
	clf;
	SGplot(SG);
	SGwriteSTL(SG);
end

end