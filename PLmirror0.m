%%  [PL] = PLmirror0(PL,axis,axis_pos)
%	=== INPUT PARAMETERS ===
%	PL:         PL of contour you want to mirror
%	axis:       Axis you want to mirror your contour on x or y
%	mirror_pos:   position of axis in contour 0 = middle of contour
%	=== OUTPUT RESULTS ======
%	PL:             Mirrored contour
function [PL] = PLmirror0(PL,axis,varargin)
mirror_pos = 0; if nargin >=3 && ~isempty(varargin{1}) mirror_pos = varargin{1}; end;
[sx,sy,~,mx,my,~] = sizeVL(PL);
mirror_pos = mirror_pos+1;  %Shifting input for a more logical user input
if ischar(axis)
    switch axis
        case 'x'
            PL = VLswapX(PL);
            PL = PLtrans(PL,[2*mx+(sx*mirror_pos) 0 ]);
        case 'y'
            PL = VLswapY(PL);
            PL = PLtrans(PL,[0 2*my+(sy*mirror_pos)]);
        otherwise
            error(axis +" doesnt exist");
    end
else
    for i=1:size(PL,1)
        cp = cross2edges(axis(1,:),(axis(2,:)-axis(1,:)),PL(i,:),(axis(2,:)-axis(1,:))*rot(pi/2));
        PL(i,:) = PLtransC(PL(i,:),cp,pi);
     end
end