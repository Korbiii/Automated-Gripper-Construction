function [SG, CPL] = SGrotatinglockbottom(varargin)

cable_circle = 1; if nargin >=1 && ~isempty(varargin{1}) cable_circle = varargin{1}; end


outer_R = 32.5;
inner_R = 15.5;
connect_R = 23.5;
height = 10;

CPL_connection_bottom = [connect_R+.5 height;connect_R+.5 4;connect_R-.5 5;21 5;inner_R+.5 height];


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
SG_bottom_gap = SGofCPLz(CPLbool('-',PLcircle(outer_R+2),PLcircle(outer_R)),4);

SG_bottom = SGstack('z',SG_bottom_gap,SG_bottom,SG_bottom_lid);

if cable_circle	
	CPL =  CPLbool('-',PLcircle(outer_R+2),PLcircle(inner_R-5));
else
	CPL =  PLcircle(outer_R+2);
end
SG = SG_bottom;

% SGplot(SG);

end