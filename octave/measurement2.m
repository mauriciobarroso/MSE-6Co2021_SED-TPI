pkg load signal

%%
% Import CSV file and plot timestamp vs acce_z
%%
csv_file = csvread('../node_files/A1651795796779.csv');
timestamp = csv_file(:, 1);
acce_z = csv_file(:, 2);
figure;
plot(timestamp, acce_z)

%%
% Delete mean value
%%
%accez = (acce_z - mean(acce_z.*window)/mean(window)).*window;
acce_z_rmean = detrend(acce_z, 'constant');


%%
% Plot spectrogram
%%
fs = 100 % Sample rate in Hz
window = ceil(3 / (1 / fs)) % Window in samples
overlap = ceil(2.8 / (1 / fs)) % Overlap in samples

figure;
specgram(acce_z_rmean, 1024, fs, window, overlap)
colormap('jet')
