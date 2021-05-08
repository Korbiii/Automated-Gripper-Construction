function [servo] = readServoFromTable(servo_name)
servo_table = readtable('Servos.xlsx');

servo_table.Properties.RowNames = servo_table.Servoname;
servo_table = removevars(servo_table,{'Servoname'});

servo = servo_table({servo_name},:);

servo = table2struct(servo);

fn = fieldnames(servo);
for k=1:numel(fn)
    if( ischar(servo.(fn{k})) )
       servo.(fn{k}) = str2num(servo.(fn{k}));
    end
end

end