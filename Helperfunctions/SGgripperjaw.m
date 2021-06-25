% SGgripperjaw(SG,[h_g,l_finger,Add]) - creates a 3D gripper jaw 
%	(by Andreas Schroeffer, Gripper-Lib, 2019-DEZ-19 as class: Construction)
%	
%	SGgripperjaw(SG,h_g,l_finger,Add)
%	=== INPUT PARAMETERS ===
%	SG:	        Object to be grasped (grey)
%	l_finger:	Length of gripper finger
%	h_g:	    depth of gripper jaw into the object
%	Add:	    Vetor with additional border (in mm)
%	=== OUTPUT RESULTS ======
%	SG_Jaw:	    Final Gripper Finger (red) 
%
%	EXAMPLE: 
%	SGgripperjaw(SGsample(19))
%   SGgripperjaw(SGsample(34),7.5)
%   SGgripperjaw(SGsample(42),3)
%	SGgripperjaw(SGsample(9),2)
%	SGgripperjaw(SGsample(9),5,20)
function SGgripperjaw = SGgripperjaw(SG,h_g,l_finger,Add)
%% Fixed Parameter
waka = 0.000001;
dZ   = 0.2;                                                                 % Printer Resolution z (Arburg Freeformer)

%% Parameter Selection 
BB_Load = BBofSG(SG); 
x = BB_Load(2)-BB_Load(1); 
y = BB_Load(4)-BB_Load(3);
z = BB_Load(6)-BB_Load(5); 
switch nargin 
    case 1
        l_finger = z*1.5;
        X_add    = max(1,0.25*x);
        Z_add    = max(1,0.25*z);
        Y_add    = max(1,0.25*y);
        h_g     = (BB_Load(4)-BB_Load(3))/3;
    case 2
        l_finger = z*1.5;
        X_add    = max(1,0.25*x);
        Z_add    = max(1,0.25*z);
        Y_add    = max(1,0.25*y);
    case 3 
        X_add    = max(1,0.25*x);
        Z_add    = max(1,0.25*z);
        Y_add    = max(1,0.25*y);
    otherwise 
        X_add    = Add(1);
        Z_add    = Add(2);
        Y_add    = Add(3);
end

%% Prepare iterative slicing 
CPL0 = [BB_Load(1)-X_add BB_Load(4); BB_Load(2)+X_add BB_Load(4); BB_Load(2)+X_add BB_Load(4)+Y_add;  BB_Load(1)-X_add BB_Load(4)+Y_add];

%% Iteratively construct Jaw
zStart = BB_Load(5)+dZ/2; zEnd = BB_Load(6)-dZ/2; 
zii = zStart:dZ:zEnd;
for ii = 1:length(zii)
    zi = zii(ii);
    CPL_Load = CPLofSGslice(SG,zi);
    CPL_Jaw  = CPLgripperjawAS(CPL_Load,CPL0,h_g);  
    switch ii 
        case 1 
            SG_Jaw = SGofCPLz(CPL_Jaw,dZ+Z_add/2);  
        case length(zii)
            SG_Jaw_i = SGofCPLz(CPL_Jaw,dZ-1+Z_add/2);   
            SG_Jaw_i = SGtransrelSG(SG_Jaw_i,SG_Jaw,'ontop',-ii*waka);
            SG_Jaw = SGcat(SG_Jaw,SG_Jaw_i);      
            try 
                CPL_Jaw2 =  CPLgrow(CPL_Jaw,1); 
                SG_Jaw_i = SGof2CPLz(CPL_Jaw,CPL_Jaw2,1);                
            catch 
                warning('Error with SGof2CPLzheurist');
                SG_Jaw_i = SGofCPLz(CPL_Jaw,1);  
            end 
            SG_Jaw_i = SGtransrelSG(SG_Jaw_i,SG_Jaw,'ontop',-ii*waka);
            SG_Jaw = SGcat(SG_Jaw,SG_Jaw_i);
        otherwise
            SG_Jaw_i = SGofCPLz(CPL_Jaw,dZ);    
            SG_Jaw_i = SGtransrelSG(SG_Jaw_i,SG_Jaw,'ontop',-ii*waka);
            SG_Jaw = SGcat(SG_Jaw,SG_Jaw_i);
    end  
end

%% Construct Finger 
SGfinger = SGofCPLz(CPL0,l_finger);
SGfinger = SGtransrelSG(SGfinger,SG_Jaw,'under', +waka);
SGgripperjaw = SGcat(SGfinger,SG_Jaw);

%% Optional Vizualisation
if nargout<1
    close all;
    SGfigure
    SGplot(SGgripperjaw,'r');
    SG = SGtransP(SG,[0;-10;Z_add/2]);
    SGplot(SG);
end
end


function CPL_Gripper = CPLgripperjawAS(CPL,CPL0,h_in)
%% Construct CPL0
BBofCPL0 = BBofCPL(CPL0);

%% Construct Contact 
CPL1 = [BBofCPL0(1) BBofCPL0(3)+0.1; BBofCPL0(2) BBofCPL0(3)+0.1;  BBofCPL0(2) BBofCPL0(3)-h_in; BBofCPL0(1) BBofCPL0(3)-h_in];

%% Boolean Concernation
pCPL  = polyshape(CPL);
pCPL0 = polyshape(CPL0);
pCPL1 = polyshape(CPL1);
pCPL1 = subtract(pCPL1,pCPL);
pCPL1 = union(pCPL1,pCPL0);
poly2 = CPLfindsmalldist(pCPL1.Vertices,0.5);
poly2 = polyshape(poly2);
pCPL1 = subtract(pCPL1,poly2);


%% Solve Problems with undercats
for i = 0:0.1:h_in 
    pCPL1.Vertices = pCPL1.Vertices + [0 0.1];
    pCPL1 = subtract(pCPL1,pCPL);
end

%% Convert to CPL
pCPL1.Vertices = pCPL1.Vertices - [0 h_in+0.1];
CPL_Gripper = pCPL1.Vertices;


%% Optional Vizualisation
if nargout < 1 
    CPLplot(CPL_Gripper);
end 
end