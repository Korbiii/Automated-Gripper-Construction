function [SG] = SGLCLtoolholder()
clf;
tool_R = 15;
tool_angle = 45;
handle_length = 50;


CPL_main_body = PLsquare(handle_length);
CPL_main_body_small = PLtrans(PLsquare(handle_length-10,handle_length-5),[0 -2.5]);
CPL_main_body = PLroundcorners(CPL_main_body,[3,4],handle_length/4);
CPL_main_body_small = PLroundcorners(CPL_main_body_small,[3,4],handle_length/4);


CPL_tool_cutout = PLsquare(tool_R*2,2*handle_length);
CPL_tool_cutout = PLtransR(CPL_tool_cutout,rot(pi/2));
CPL_tool_cutout = PLtransC(CPL_tool_cutout,[0 0],deg2rad(tool_angle));
CPL_main_body_w_cutout = CPLbool('-',CPL_main_body,CPL_tool_cutout);

SG_main_body_sides = SGofCPLz(CPL_main_body,(handle_length/2)-(tool_R)-5);
SG_main_body_sides_chamfer = SGof2CPLsz(CPL_main_body,CPL_main_body_small,5);
SG_main_body_w_cutout = SGofCPLz(CPL_main_body_w_cutout,tool_R);

SG_main_body = SGstack('z',SG_main_body_w_cutout,SG_main_body_sides,SG_main_body_sides_chamfer);
SG_main_body = SGcat(SG_main_body,SGmirror(SG_main_body,'xy'));
SG_main_body = SGtransrelSG(SG_main_body,'','rotx',pi/2);

CPL_stop = PLroundcorners(PLsquare(handle_length),[1,2,3,4],5);
SG_stop = SGofCPLz(CPL_stop,4);

insert_length = handle_length/sin(tool_angle);
insert_length = min(insert_length,handle_length)-10;
CPL_insert = CPLbool('-',PLsquare(2*tool_R),PLcircle(tool_R));
CPL_insert_holder = CPLbool('x',PLcircle(tool_R),CPLcopyradial(PLcircle(1),tool_R,10));
CPL_insert = CPLbool('+',CPL_insert,CPL_insert_holder);
SG_insert = SGofCPLz(CPL_insert,insert_length);
SG_insert = SGtransrelSG(SG_insert,'','transz',-insert_length/2,'roty',pi/2-deg2rad(tool_angle));
SG_main_body = SGcat(SG_main_body,SG_insert);
SG_main_body = SGstack('z',SG_stop,SG_main_body);
SG_main_body = SGtransrelSG(SG_main_body,'','centerz',-2);


[SG_connector, CPL_connector] = SGrotatingdiskwithoutservo('sm40bl');
SG_connection = SGof2CPLsz(CPL_connector,CPLaddauxpoints(CPL_stop,1),10);
SG = SGstack('z',SG_connector,SG_connection,SG_main_body);

SGplot(SG);


end