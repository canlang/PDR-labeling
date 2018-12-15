%% local functions
function data = load_rawdata(datapath)
acc = csvread(fullfile(datapath,'Accelerometer.csv'),1,0);
gyr = csvread(fullfile(datapath,'Gyroscope.csv'),1,0);
data.acc = acc;
data.gyr = gyr;
data.acc_time = datetime(acc(:,1)/10^3,'convertfrom','posixtime','TimeZone','Asia/Seoul');
data.gyr_time = datetime(gyr(:,1)/10^3,'convertfrom','posixtime','TimeZone','Asia/Seoul');

data.acc_norm = vecnorm(acc(:,3:5),2,2);
data.acc_norm = data.acc_norm - mean(data.acc_norm);
end