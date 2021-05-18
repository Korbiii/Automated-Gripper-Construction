function [SG] = SGrotatinglockadapter(varargin)

cable = 0; if nargin >=1 && ~isempty(varargin{1}) cable = varargin{1}; end
attach_dof = 'z'; if nargin >=2 && ~isempty(varargin{2}) attach_dof = varargin{2}; end
attach_servo = 'sm40bl'; if nargin >=3 && ~isempty(varargin{3}) attach_servo = varargin{3}; end
print_help_layer = 0; if nargin >=4 && ~isempty(varargin{4}) print_help_layer = varargin{4}; end

outer_R = 32.5;
inner_R = 15.5;
connect_R = 23.5;
height = 10;

CPL_connection_bottom = [connect_R+.5 height;connect_R+.5 4;connect_R-.5 5;21 5;inner_R+.5 height];
CPL_connection_stop = CPLbool('+',CPL_connection_bottom,[connect_R+.5 0;connect_R+.5 height;connect_R-3 height;connect_R-3 0]);

SG_stub = SGofCPLrota(CPL_connection_bottom,(1/4)*pi,false);
SG_stub_stop = SGofCPLrota(CPL_connection_stop,0.2,false);
SG_stub_stop = SGtransR(SG_stub_stop,rot(0,0,-0.2));
SG_stub = SGcat(SG_stub,SG_stub_stop);
SG_stubs = SGcircularpattern(SG_stub,3,2*pi/3);

CPL_main = CPLbool('-',PLcircle(outer_R-0.5),PLcircle(connect_R+.5));
SG_main = SGofCPLz(CPL_main,height);

CPL_main = [outer_R-0.5 height;outer_R-0.5 0;connect_R+.5 0;connect_R+.5 height];
CPL_main = PLroundcorners(CPL_main,1,2.5,'',0);
SG_main = SGofCPLrot(CPL_main);

if print_help_layer
	SG_help_layer = SGofCPLz(PLcircle(outer_R-0.5),0.2);
	SG_main = SGcat(SG_main,SGalignbottom(SG_help_layer,SG_main));
end

SG_bottom = SGcat(SG_stubs,SG_main);

servo = readServoFromTable(attach_servo);	
if cable 	
	CPL_bottom = PLcircle(outer_R-0.5);
	CPL_bottom = CPLbool('-',CPL_bottom,PLcircle(servo.connect_R));
else
	CPL_bottom = PLcircle(outer_R-0.5);
end

[SG_connector,CPL_connector] = SGconnAdaptersLCL('servo',attach_servo,'adapter_type',attach_dof,'cable',cable);

if attach_dof == 'z'	
	CPL_bottom = CPLbool('-',CPL_bottom,PLcircle(servo.connect_R));
	CPL_connector = CPLbool('-',CPL_connector,PLcircle(servo.connect_R));
end

CPL_connector = CPLaddauxpoints(CPL_connector,0.5);
SG_connections = SGof2CPLsz(CPL_connector,CPL_bottom);

SG = SGstack('z',SG_connector,SG_connections,SG_bottom);
if nargout == 0
	clf;
	SGplot(SG);
end
end