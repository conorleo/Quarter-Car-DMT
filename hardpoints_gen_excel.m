hardpoints = [[pwd '/hardpoints_excel/front_hardpoints.xls']; [pwd '/hardpoints_excel/rear_hardpoints.xls']];

for 
data = readtable(har, 'UseExcel', false);

slices = [0,3,6,8,10,12,16];
cat = {'lwb', 'uwb', 'tr', 'pr', 'wheel', 'inboard'};
col_name = {'front', 'rear', 'outer', 'front', 'rear', 'outer', 'inner', 'outer', 'inner', 'outer', 'tyre_diameter', 'centre', 'rocker_pivot', 'rocker_to_damper', 'roll_damper_left', 'damper_to_chassis'};

for i = 1:length(slices)-1
    data_snip = data(:,slices(i)+1:slices(i+1));
    data_snip.Properties.VariableNames = col_name(slices(i)+1:slices(i+1));
    
    hardpoints_front.(cat{i}) = table2struct(data_snip, 'ToScalar', true);
end

for i = 1:length(slices)-1
    for j = slices(i)+1:slices(i+1)
        hardpoints_front.(cat{i}).(col_name{j}) = transpose(hardpoints_front.(cat{i}).(col_name{j}));
    end
end

save([pwd '/geometries/EV21_rear.mat'], 'hardpoints_front');