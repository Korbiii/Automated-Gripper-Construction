function SG = SGaxle(length)
Radius = 3;

SG_back = SGof2CPLsz(PLcircle(Radius-0.1),PLcircle(Radius-0.2),5);
SG_main_part = SGofCPLz(PLcircle(Radius-0.2),length-5);

SG = SGstack('z',SG_back,SG_main_part);

SGwriteSTL(SG,num2str(length));



end