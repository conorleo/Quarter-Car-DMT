side = {'front', 'rear'};

vert_max = sims{1, 1}.metrics.BumpTravel;
vert_min = -sims{1, 1}.metrics.DroopTravel;
vert = linspace(vert_min, vert_max, 100);

steer_max = sims{1, 1}.metrics.MaxSteer_inside;
steer_min = sims{1, 1}.metrics.MaxSteer_outside;
steer = linspace(steer_min, steer_max, 100);

for i = 1:length(sims)
    out = sims{i,1};
    params = fieldnames(out.metrics);
    for j = 1:numel(fieldnames(out.metrics))
%         disp(length(out.metrics.(params{j})))
        if length(out.metrics.(params{j})) > 1
            for k = [2,3,11,22] % indices of polynomial coeffs inside sims.metrics which correspond to a sweep of STEERING angle
                if j == k
                    dynamic_response.(side{i}).(params{j}) = polyval(out.metrics.(params{j}), steer);
                else
                    dynamic_response.(side{i}).(params{j}) = polyval(out.metrics.(params{j}), vert);
                end
            end
        end
    end
end