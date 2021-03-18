%%   [SG] = SGmirror(SG,axis)
%	=== INPUT PARAMETERS ===
%	SG:    SG you want to mirror
%	plane: plane you want to mirror on
%	=== OUTPUT RESULTS ======
%	SG:         Mirrored SG
function [SG] = SGmirror(SG,plane)
switch plane
    case 'yz'
        SG.VL = VLswapX(SG.VL);
        SG.FL(:,[2 3]) = SG.FL(:,[3 2]);
    case 'xz'
        SG.VL = VLswapY(SG.VL);
        SG.FL(:,[2 3]) = SG.FL(:,[3 2]);
    case 'xy'
        SG.VL = VLswapZ(SG.VL);
        SG.FL(:,[1 3]) = SG.FL(:,[3 1]);
    otherwise
        error(plane + " plane doesnt exist");
end
end