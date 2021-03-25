function [SG] = SGLCLrobotarm()
SGs = SGUnterarm();
SGs{end+1} = SGLCLzdof();
SGs{end+1} =SGTrigripper();
robotarm = SGTchain(SGs);
SGplot(robotarm);
end
