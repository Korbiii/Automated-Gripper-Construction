% Function that moves saved file to user specified folder
function SGsSaveToFolder(name_array)
	directory = uigetdir;	
	if ~iscell(name_array)
		name_array = {name_array};
	end
	for i=1:size(name_array,2)
		movefile(name_array{i},directory);		
	end	
end
