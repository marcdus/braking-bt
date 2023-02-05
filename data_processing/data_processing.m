clear
clearvars
clc
clear all

%% inputs
set(0, 'defaulttextinterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

loc = readtable("_bumblebee_se2_localization_gokart_state.csv");        % localization data (being used)
mcdaq = readtable("_bumblebee_observations_mcdaq.csv");                 % mcdaq data, contains brake pressures (being used)
motors = readtable("_bumblebee_observations_motors_obs.csv");           % motor observation (being used)
tlogs = readtable("logs-temperature-2022-12-12T16_43_25.csv");          % temp readings
% pcmd_logs = readtable("");                                            % pressure commands
cmd_logs = readtable("_bumblebee_autonomous_control_brake_cmd.csv");

% fix time vectors
loc.header_stamp_sec = loc.header_stamp_sec + loc.header_stamp_nanosec / 1e9;       
mcdaq.header_stamp_sec = mcdaq.header_stamp_sec + mcdaq.header_stamp_nanosec / 1e9;
cmd_logs.header_stamp_sec = cmd_logs.header_stamp_sec + cmd_logs.header_stamp_nanosec / 1e9;

tlogs = renamevars(tlogs, 'Var1', 't');
% sampling frequencies of the data
f_loc = 1 / mean(gradient(loc.header_stamp_sec));                              
f_tlogs = 1 / mean(gradient(tlogs.t));
f_mcdaq = 1 / mean(gradient(mcdaq.header_stamp_sec));
% f_pcmd = 1 / mean(gradient(pcmd_logs.Var1));
f_cmd = 1 / mean(gradient(cmd_logs.header_stamp_sec));

% choose resampling frequency
fs= 20;

gray = [0.5 0.5 0.5];
lightgray = [0.75 0.75 0.75];


%% processing
% resampling controller output
[tmp , t_cmd] = resample(cmd_logs.position, cmd_logs.header_stamp_sec, fs);   % create uniform time vector

cmds = zeros(length(t_cmd), 2);                                               % create array with zeros
% t_cmd = t_cmd - (t_cmd(1) - round(t_cmd(1)));                               % making sure t_temp starts with .000
cmds(:, 1) = t_cmd;                                                           % fill first column with time vector

cmds(:, 2) = resample(cmd_logs.position, cmd_logs.header_stamp_sec, fs);
% cmds(:, 2) = lowpass(cmds(:, 2), 5, fs);

cmd = array2table(cmds, 'VariableNames', ["t" "cmd_position"]);
clear tmp cmds t_cmd


% resampling demanded pressure and storing in new table
% [tmp , t_pcmd] = resample(pcmd_logs.(2), pcmd_logs.(1), fs);                % create uniform time vector
% 
% pcmds = zeros(length(t_pcmd), 2);                                           % create array with zeros
% % t_pcmd = t_pcmd - (t_pcmd(1) - round(t_pcmd(1)));                         % making sure t_temp starts with .000
% pcmds(:, 1) = t_pcmd;                                                       % fill first column with time vector
% 
% pcmds(:, 2) = resample(pcmd_logs.(2), pcmd_logs.(1), fs, 'linear');
% % pcmds(:, 2) = lowpass(pcmds(:, 2), 5, fs);
% pcmds(:, 2) = round(pcmds(:,2), 0);
% 
% p_cmd = array2table(pcmds, 'VariableNames', ["t" "brake_pressure_cmd"]);
% clear tmp pcmds t_pcmd


% resampling temperature and storing in new table
[tmp , t_temp] = resample(tlogs.(2), tlogs.(1), fs);                            % create uniform time vector

temps = zeros(length(t_temp), width(tlogs));                                    % create array with zeros
% t_temp = t_temp - (t_temp(1) - round(t_temp(1)));                               % making sure t_temp starts with .000
temps(:, 1) = t_temp;                                                           % fill first column with time vector

for i = [2:width(tlogs)]                                                        % loop over rest of the columns
    temps(:, i) = resample(tlogs.(i), tlogs.t, fs);
    temps(:, i) = lowpass(temps(:, i), 1, fs);
end
temp = array2table(temps, 'VariableNames', tlogs.Properties.VariableNames);
clear tmp temps t_temp

% resampling loc data and storing in new table
[tmp, t] = resample(loc.pose2d_dot_x, loc.header_stamp_sec, fs);
clear tmp
names = {'t', 'x_dot', 'x_dotdot', 'p_left', 'p_right'};
tmp(1:length(names)) = {'double'};
data = table( ...
    'Size', [length(t) length(names)], ...
    'VariableTypes', tmp, ...
    'VariableNames', names);
% t = t - (t(1) - round(t(1)));      % making sure time vector starts with .000
data.t = t;                        % storing time vector

% filtering of loc data and aceleration
data.x_dot = resample(loc.pose2d_dot_x, loc.header_stamp_sec, fs);
data.x_dot = lowpass(data.x_dot, 4, 20);
data.x_dotdot = gradient(data.x_dot, data.t);

% aligning all of the data to start at the same time
t0 = max([data.t(1), cmd.t(1), mcdaq.header_stamp_sec(1)]);

mcdaq.header_stamp_sec = mcdaq.header_stamp_sec - t0;
data.t = data.t - t0;
cmd.t = cmd.t - t0;

if mcdaq.header_stamp_sec(1) ~= 0
    pos = round((-fs) * mcdaq.header_stamp_sec(1));
    mcdaq((1:pos), :) = [];
end

if data.t(1) ~= 0
    pos = round((-fs) * data.t(1));
    data((1:pos), :) = [];
end

if cmd.t(1) ~= 0
    pos = round((-fs) * cmd.t(1));
    cmd((1:pos), :) = [];
end

% resampling, filtering and saving of pressures
tmp = resample(mcdaq.ain_brake_pressure_left, mcdaq.header_stamp_sec, fs);
tmp(height(data)+1 : length(tmp)) = [];
tmp = lowpass(tmp, 5, 20);
data.p_left = tmp;
clear tmp

tmp = resample(mcdaq.ain_brake_pressure_right, mcdaq.header_stamp_sec, fs);
tmp(height(data)+1 : length(tmp)) = [];
tmp = lowpass(tmp, 5, 20);
data.p_right = tmp;
clear tmp

%% plotting std dev
close all
laps = {'70 $^\circ$C', '151 $^\circ$C', '217 $^\circ$C'};
% define the window length in time steps
window = 80;
startpoints = [10349 11232 12189 13113 14128 15950 17664 20605 23691 25694];


p_mean(1:window) = 0;
p_dev(1:window) = 0;

for i = [1:window]
    j = 1;
    for start = startpoints;
        tmp(j) = data.p_right(start + i - 1);
        j = j + 1;
    end
    p_mean(i) = mean(tmp);
    p_dev(i) = std(tmp);
end
% for filling the std dev area
x = 1:window;
x_fill = [x, fliplr(x)];
inBetween = [p_mean-2*p_dev, fliplr(p_mean+2*p_dev)];


% mean and std dev figure
fig_mean_man = figure;
hold on
fill(x_fill, inBetween, lightgray, 'EdgeColor', 'none');
set(gca,'XTick',[0 10 20 30 40]);
set(gca,'XTickLabel',[0.0 0.5 1.0 1.5 2.0]);
plot(p_mean, 'LineWidth', 1, 'Color', [0 0 0])
% initial line
start = startpoints(1);
stop = start + window;
plot(data.p_right(start:stop), ...
    '-', ...
    'LineWidth', 1)

% additional lines
yline(16.5, 'LineWidth', 1, 'LineStyle', '--')
yline(0.9*16.5, 'LineWidth', 1, 'LineStyle', '--', ...
    'Label', '$90 \%$', ...
    'Interpreter', 'latex', ...
    'LabelHorizontalAlignment', 'center', ...
    'LabelVerticalAlignment', 'middle')

% rise time
xline(3, 'LineWidth', 1, 'LineStyle', '--' )
xline(7.47, 'LineWidth', 1, 'LineStyle', '--' )
plot([3 7.47], [8.6 8.6], ...
    'LineStyle', '-', ...
    'LineWidth', 1, ...
    'Marker', '|', ...
    'Color', 'blue')
text(9, 8.6, '$t_{90} = 0.222$ s', 'Color', 'blue')

% overshoot
plot([10 10], [16.5 20.322], ...
    'LineStyle', '-', ...
    'LineWidth', 1, ...
    'Marker', '_', ...
    'Color', 'red')
text(8, 21, '$\hat \epsilon_{max} = 0.232$', 'Color', 'red')

% steady state error
plot([36 36], [16.5 17.81], ...
    'LineStyle', '-', ...
    'LineWidth', 1, ...
    'Marker', '_', ...
    'Color', 'cyan')
text(33, 18.5, '$e_{\infty} = 0.121$', 'Color', 'cyan')

ylim([0 22])
xlim([0 40])
xlabel('time [s]')
ylabel('pressure [bar]')
title('mean step response manually tuned')
legend('$2 \cdot \sigma$ confidence interval', 'mean step response', ...
    'initial step response', ...
    'Location', 'southeast')
hold off

set(fig_mean_man, 'PaperUnits', 'points');
set(fig_mean_man, 'PaperPosition', [0 0 280 280]);
set(fig_mean_man, 'PaperSize', [280 280])
fontsize(fig_mean_man, 10.5, 'points')
saveas(fig_mean_man, 'fig_mean_man.pdf')
