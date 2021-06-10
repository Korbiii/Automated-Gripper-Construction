function [SG] = SGLCLtoolholder(varargin)
servo_name = 'sm40bl';
width_holder = 50;
i_idx = 1;
while i_idx<=size(varargin,2)
	if ~ischar(varargin{i_idx})
		i_idx = i_idx+1; 
		continue; 
	end
	switch varargin{i_idx}
		case 'width'
			width_holder = varargin{i_idx+1};
		case 'height'
			height_holder = varargin{i_idx+1};
	end
	i_idx = i_idx+1;
end


CPL_base = PLroundcorners(PLsquare(width_holder),[1,2,3,4],10);
CPL_base_small = PLroundcorners(PLsquare(width_holder-10),[1,2,3,4],10);
SG_base_top = SGof2CPLsz(CPL_base,CPL_base_small,5);
SG_main_body = SGofCPLz(CPL_base,width_holder);
SG_main_body = SGboolh('+',SG_main_body,SGontop(SG_base_top,SG_main_body));



[SG_connector, CPL_connector] = SGconnAdaptersLCL('adapter_type','rotLock','servo',servo_name,'cable',0);
CPL_connector = CPLaddauxpoints(CPL_connector,0.5);
CPL_base = CPLaddauxpoints(CPL_base,0.5);
SG_connection = SGof2CPLsz(CPL_connector,CPL_base,10);
SG = SGstack('z',SG_connector,SG_connection,SG_main_body);

height = max(SG.VL(:,3));
H_Object = [rotx(0) [0;0;height]; 0 0 0 1];
SG = SGTset(SG,'O',H_Object);
if nargout == 0
	clf;
	SGplot(SG);
	SGwriteSTL(SG);
end



end