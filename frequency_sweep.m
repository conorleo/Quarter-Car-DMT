clear
close all

% f = [0.5, 5, 10, 20, 25];
f = [1];
for i = 1:length(f)
    sims = run_one_sim(f(i));
    
    Fz = sims{1,1}.SimulationOutput.Fz;
    x_s = sims{1,1}.SimulationOutput.x_s;
    
    mkdir('../results_1', sprintf('%sHz', num2str(f(i))))
    save(sprintf('../results_1/%sHz/%s.mat', num2str(f(i)),'Fz'), 'Fz')
    save(sprintf('../results_1/%sHz/%s.mat', num2str(f(i)),'x_s'), 'x_s')
end