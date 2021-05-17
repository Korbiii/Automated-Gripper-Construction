function [SG,CPL] = SGrotatinglockadapter(varargin)
clf;

cable_circle = 1; if nargin >=1 && ~isempty(varargin{1}) cable_circle = varargin{1}; end
attach_dof = 'z'; if nargin >=2 && ~isempty(varargin{2}) attach_dof = varargin{2}; end
attach_servo = 'sm40bl'; if nargin >=3 && ~isempty(varargin{3}) attach_dof = varargin{3}; end
print_help_layer = 1; if nargin >=4 && ~isempty(varargin{4}) attach_dof = varargin{4}; end

outer_R = 32.5;
inner_R = 15.5;
connect_R = 23.5;
height = 10;

%% Bottom
CPL_connection_bottom = [connect_R+.5 height;connect_R+.5 4;connect_R-.5 5;21 5;inner_R+.5 height];

SG_stub = SGofCPLrota(CPL_connection_bottom,(1/4)*pi,false);
SG_stubs = SGcircularpattern(SG_stub,3,2*pi/3);

CPL_main = CPLbool('-',PLcircle(outer_R-0.5),PLcircle(connect_R+.5));
SG_main = SGofCPLz(CPL_main,height);

if print_help_layer
	SG_help_layer = SGofCPLz(PLcircle(outer_R-0.5),0.2);
	SG_main = SGcat(SG_main,SGalignbottom(SG_help_layer,SG_main));
end

SG_bottom = SGcat(SG_stubs,SG_main);

CPL_bottom = PLcircle(outer_R-0.5);
servo = readServoFromTable(attach_servo);	

if attach_dof == 'z'
	[SG_connector,CPL_connector] = SGrotationdisk(attach_servo,cable_circle,1);
	CPL_bottom = CPLbool('-',CPL_bottom,PLcircle(servo.connect_R));
	if cable_circle == 0
		CPL_connector =CPLbool('-',CPL_connector,PLcircle(servo.connect_R));
	end
elseif attach_dof == 'x'
	[SG_connector,CPL_connector] = SGbracket(attach_servo);	
	if cable_circle == 0
		CPL_connector = CPLconvexhull(CPL_connector);
	else
		CPL_bottom = CPLbool('-',CPL_bottom,PLcircle(servo.connect_R));
	end
end



CPL_connector = CPLaddauxpoints(CPL_connector,0.5);
SG_connections = SGof2CPLsz(CPL_connector,CPL_bottom);

SG_bottom = SGstack('z',SG_connector,SG_connections,SG_bottom);
SGplot(SG_bottom);
%% TOP
CPL_main = CPLbool('-',PLcircle(outer_R+2),PLcircle(outer_R));
CPL_main = CPLbool('+',CPL_main,PLcircle(inner_R));

if cable_circle	
	CPL_main = CPLbool('-',CPL_main,PLcircle(inner_R-5));
end


SG_main_bottom = SGofCPLz(CPL_main,height);

CPL_connection_bottom_buf = PLtrans(CPL_connection_bottom,[0 -0.5]);
CPL_connection_bottom_buf = CPLbuffer(CPL_connection_bottom_buf,0.5);
CPL_connection_stub = CPLbool('-',[inner_R 0;connect_R 0;connect_R 10;inner_R 10],CPL_connection_bottom_buf);

SG_stub = SGofCPLrota(CPL_connection_stub,(1/3)*pi,false);
SG_stubs = SGcircularpattern(SG_stub,3,2*pi/3);

SG_bottom = SGcat(SG_main_bottom,SG_stubs);

CPL_bottom_lid = PLcircle(outer_R+2);
if cable_circle	
	CPL_bottom_lid = CPLbool('-',CPL_bottom_lid,PLcircle(inner_R-5));
end

SG_bottom_lid = SGofCPLz(CPL_bottom_lid,2);

SG_bottom = SGstack('z',SG_bottom,SG_bottom_lid);

if cable_circle	
	CPL =  CPLbool('-',PLcircle(outer_R+2),PLcircle(inner_R-5));
else
	CPL =  PLcircle(outer_R+2);
end
SG = SG_bottom;

end