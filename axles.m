length = 12;
CPL_big_circ = PLcircle(3);
CPL_small_circ = PLcircle(2.75);

SG_chamfer = SGof2CPLsz(CPL_big_circ,CPL_small_circ,4);
SG_shaft = SGofCPLz(CPL_small_circ,length-4);

SG = SGstack('z',SG_chamfer,SG_shaft);

SGwriteSTL(SG,num2str(length));