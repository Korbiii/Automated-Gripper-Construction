function [CPL] = CPLatPL(CPL_in,PL)

CPL = [];
for i=1:size(PL,1)
	CPL = CPLbool('+',CPL,PLtrans(CPL_in,PL(i,:)));
end

end