function [SG] = SGLCLcreate()


CPL = [0 0;0 10;3 10;3 3;10 3;10 0;0 0];
HT = TLofCVL(PLsquare(20));
SG = SGofCPLtransT(CPL,HT);
SGplot(SG);
end


function [SG] = SGcreateBaseBox(CPL,width,length)





end