function [SG_base,SG_main_body,SG_complete] = SGLCLtoolholder(varargin)
servo_name = 'sm40bl';
width_holder = 50;
i_idx = 1;
SG_object = {};
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
		case 'SG_object'
			SG_object = varargin{i_idx+1};
	end
	i_idx = i_idx+1;
end


CPL_base = PLroundcorners(PLsquare(width_holder),[1,2,3,4],10);
CPL_base_small = PLroundcorners(PLsquare(width_holder-10),[1,2,3,4],10);
SG_base_top = SGof2CPLsz(CPL_base,CPL_base_small,5);
SG_main_body = SGofCPLz(CPL_base,width_holder);
SG_main_body = SGboolh('+',SG_main_body,SGontop(SG_base_top,SG_main_body));

height_max = max(SG_main_body.VL(:,3));
T_Object = [rotx(0) [0;0;height_max]; 0 0 0 1];
SG_main_body = SGTset(SG_main_body,'ObjectPos',T_Object);

height_min = min(SG_main_body.VL(:,3));
T_connection_top = [rotx(0) [0;0;height_min]; 0 0 0 1];
SG_main_body = SGTset(SG_main_body,'GripperT',T_connection_top);

if ~isempty(SG_object)
	SG_object = SGtransrelSG(SG_object,SG_main_body,'alignTz',{'ObjectPos','ObjectPos'});	
	SG_main_body = SGbool3('-',SG_main_body,SG_object);
end

[SG_connector, CPL_connector] = SGconnAdaptersLCL('adapter_type','rotLock','servo',servo_name,'cable',0);
CPL_connector = CPLaddauxpoints(CPL_connector,0.5);
CPL_base = CPLaddauxpoints(CPL_base,0.5);
SG_connection = SGof2CPLsz(CPL_connector,CPL_base,10);

SG_base = SGstack('z',SG_connector,SG_connection);

height_max = max(SG_base.VL(:,3));
T_Connection_bot = [rotx(0) [0;0;height_max]; 0 0 0 1];
SG_base = SGTset(SG_base,'GripperT',T_Connection_bot);

SG_complete = SGstack('z',SG_connector,SG_connection,SG_main_body);

height_max = max(SG_complete.VL(:,3));
T_Object = [rotx(0) [0;0;height_max]; 0 0 0 1];
SG_complete = SGTset(SG_complete,'ObjectPos',T_Object);

height_max = min(SG_complete.VL(:,3));
H_GripperPos = [rotx(0) [0;0;height_max+20]; 0 0 0 1];
SG_complete = SGTset(SG_complete,'GripperT',H_GripperPos);

if nargout == 0
	clf;
	SGplot(SG_complete);
	SGwriteSTL(SG_complete);
end
SG_main_body.alpha = 0.45;


end