%%
% Import CSV file and plot timestamp vs acce_z
%%
csv_file = csvread('../node_files/A1651795708816.csv');
timestamp = csv_file(:, 1);
acce_z = csv_file(:, 2);
figure;
plot(timestamp, acce_z)

%%
% Delete first element (zero)
%%
acce_z = acce_z(2:end);

%%
% Calculate min and max values to filter data
%%
acce_z_min = min(acce_z)
acce_z_max = max(acce_z)

acce_z = min(acce_z, acce_z_max - 0.01);
acce_z = max(acce_z, acce_z_min + 0.01);

%%
% Histogram with 10 bins and 50 bins
%%
figure;
hist(acce_z, 10)
figure;
hist(acce_z, 50)

%%
% Mean and variance
%%
acce_z_mean = mean(acce_z)
acce_z_var = var(acce_z)