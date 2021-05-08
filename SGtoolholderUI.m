function SG = SGtoolholderUI()

SG_object = SGofCPLz(PLcircle(10),30);

SG_object = SGtrans0(SG_object);
[limx,limy,limz] =sizeVL(SG_object);

SG_adapter = SGtrans0(SGofCPLz(PLsquare(30),20));

SGplot(SG_object);
SGplot(SG_adapter);
xlim([-limx limx]);
ylim([-limy limy]);
zlim([-limz limz]);
view(90,0);

while true
	[~,~,in]=ginput(1);
	switch in
		case 31
			SG_object = SGtrans(SG_object,[0 0 -1]);
		case 30
			SG_object = SGtrans(SG_object,[0 0 +1]);
		case 28
			SG_object = SGtrans(SG_object,[0 -1 0]);
		case 29
			SG_object = SGtrans(SG_object,[0 1 0]);
		case 101
			break;
		otherwise		
	end	
	clf;
	SGplot(SG_object);
	SGplot(SG_adapter);
	xlim([-limx limx]);
	ylim([-limy limy]);
	zlim([-limz limz]);
	view(90,0);
end

CPL_outside = PLsquare(20,30);
SG_outside = SGofCPLzdelaunayGrid(CPL_outside,1,1,0.5);
idx = find(SG_outside.VL(:,3)==1);
SG_outside = SGtransrelSG(SG_outside,SG_adapter,'roty',pi/2,'aligntop','alignright');

for i=1:size(idx,1)
	CPL_temp = CPLofSGslice(SG_object,SG_outside.VL(idx(i),3));
	CPL_temp = CPLaddauxpoints(CPL_temp,0.5);
	
	
	
	x=1;
end
SGplot(SG_outside);

end