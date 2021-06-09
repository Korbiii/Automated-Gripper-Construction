function [SG] = SGLCLtoolholder(width_holder)
servo_name = 'sm40bl';



CPL_main_body = PLsquare(width_holder);
CPL_main_body_small = PLtrans(PLsquare(width_holder-10,width_holder-5),[0 -2.5]);
CPL_main_body = PLroundcorners(CPL_main_body,[3,4],width_holder/4);
CPL_main_body_small = PLroundcorners(CPL_main_body_small,[3,4],width_holder/4);

SG_main_body_sides = SGofCPLz(CPL_main_body,width_holder/2-5);
SG_main_body_sides_chamfer = SGof2CPLsz(CPL_main_body,CPL_main_body_small,5);

SG_main_body = SGstack('z',SG_main_body_sides,SG_main_body_sides_chamfer);
SG_main_body = SGcat(SG_main_body,SGmirror(SG_main_body,'xy'));
SG_main_body = SGtransrelSG(SG_main_body,'','rotx',pi/2);

CPL_stop = PLroundcorners(PLsquare(width_holder),[1,2,3,4],5,'',0);
SG_stop = SGofCPLz(CPL_stop,4);

SG_main_body = SGstack('z',SG_stop,SG_main_body);
SG_main_body = SGtransrelSG(SG_main_body,'','centerz',-2);


[SG_connector, CPL_connector] = SGconnAdaptersLCL('adapter_type','rotLock','servo',servo_name,'cable',0);
CPL_connector = CPLaddauxpoints(CPL_connector,0.5);
CPL_stop = CPLaddauxpoints(CPL_stop,0.5);
SG_connection = SGof2CPLsz(CPL_connector,CPL_stop,10);
SG = SGstack('z',SG_connector,SG_connection,SG_main_body);

if nargout == 0
	clf;
	SGplot(SG);
	SGwriteSTL(SG);
end



end