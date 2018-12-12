function tests = exampleTest
tests = functiontests(localfunctions);
end

function testFunctionOne(testCase)
% Test specific code
end

function FunctionTwotest(testCase)
% Test specific code
end

% function data = load_rawdata(datapath)
% acc = csvread(fullfile(datapath,'Accelerometer.csv',1,0));
% gyr = csvread(fullfile(datapath,'Gyroscope.csv',1,0));
% data.acc = acc;
% data.gyr = gyr;