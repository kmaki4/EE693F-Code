clear all;

figure, hold on;
label_legend = {'N=64,z=2';'N=64,z=1.5';'N=256,z=2';'N=256,z=1.5'};
grid on
title('Ergodic Achievable Rate vs. Transmit Power')
xlabel('Transmit Power (dBm)')
ylabel('Achievable Rate (bits/sec/Hz)')
xlim([0 30])
ylim([0 12])
ax = gca;
ax.TitleFontSizeMultiplier = 0.8;

filenames = {'N_64_H_2.mat','N_64_H_1.5.mat','N_256_H_2.mat', 'N_256_H_1.5.mat'};  
for kk = 1:numel(filenames)
    load(filenames{kk}) % Best to load into an output variable.
    plot(temp,R, 'LineWidth',1, 'DisplayName',sprintf("%s",label_legend{kk}))
    hold on;
    legend('show');
end
