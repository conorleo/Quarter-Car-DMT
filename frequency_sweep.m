clear
close all

f = [1, 4, 5, 10, 15, 20, 25];
%f = [3];
for i = 1:length(f)
    sims = run_one_sim(f(i));
    
    z_s = sims{1,1}.SimulationOutput.z_s;
    vz_s = sims{1,1}.SimulationOutput.vz_s;
    az_s = sims{1,1}.SimulationOutput.az_s;
    Fz_s = sims{1,1}.SimulationOutput.Fz_s;
    Fx_s = sims{1,1}.SimulationOutput.Fxy_s(:,2);
    Fy_s = sims{1,1}.SimulationOutput.Fxy_s(:,3);
    Mx_s = sims{1,1}.SimulationOutput.M_s(:,2);
    My_s = sims{1,1}.SimulationOutput.M_s(:,3);
    Mz_s = sims{1,1}.SimulationOutput.M_s(:,1);
    
    z_a = sims{1,1}.SimulationOutput.z_a;
    vz_a = sims{1,1}.SimulationOutput.vz_s;
    az_a = sims{1,1}.SimulationOutput.az_a;
    Fz_a = sims{1,1}.SimulationOutput.Fz_a;
    
    x_damp = sims{1,1}.SimulationOutput.x_damp;
    
    results_folder = 'results_HS0202B';
%     mkdir(sprintf('../%s', results_folder), sprintf('%sHz', num2str(f(i))))
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'z_s'), 'z_s')
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'vz_s'), 'vz_s')
    save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'az_s'), 'az_s')
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'Fz_s'), 'Fz_s')
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'Fx_s'), 'Fx_s')
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'Fy_s'), 'Fy_s')
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'Mx_s'), 'Mx_s')
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'My_s'), 'My_s')
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'Mz_s'), 'Mz_s')
%     
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'z_a'), 'z_a')
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'vz_a'), 'vz_a')
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'az_a'), 'az_a')
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'Fz_a'), 'Fz_a')
%     
%     save(sprintf('../%s/%sHz/%s.mat', results_folder, num2str(f(i)),'x_damp'), 'x_damp')
end