clear;

load('testData.mat');

% Uncomment to Remove anomalous results

% indAStar = find(successTimeAStar > 0.0014);
% successTimeAStar = timeAStar;
% successTimeAStar(indAStar) = [];
% successDistAStar = distEuclidean;
% successDistAStar(indAStar) = [];
% successObsDensityRatioMAPA = obsDensityRatioMAP;
% successObsDensityRatioMAPA(indAStar) = [];
% successObsDensityRatioSQA = obsDensityRatioSQ;
% successObsDensityRatioSQA(indAStar) = [];
% successObsAStar = successObsDensityRatioSQA + successObsDensityRatioMAPA;

% indRRT = find(successTimeRRT > 0.035);
% successTimeRRT = timeRRtAvg;
% successTimeRRT(indRRT) = [];
% successDistRRT = distEuclidean;
% successDistRRT(indRRT) = [];
% successObsDensityRatioMAPRRT = obsDensityRatioMAP;
% successObsDensityRatioMAPRRT(indRRT) = [];
% successObsDensityRatioSQRRT = obsDensityRatioSQ;
% successObsDensityRatioSQRRT(indRRT) = [];
% successObsRRT = successObsDensityRatioSQRRT + successObsDensityRatioMAPRRT;


% Test model prediction performance 
tPredictAStar = 0.00005216*successObsAStar + 0.000004956*successDistAStar + 0.001069;
tPredictRRT = 0.009445*successObsRRT + 0.001336*successDistRRT + 0.002401;

figure;
plotregression(successTimeAStar, tPredictAStar, 'Regression');
mseRRT = mse(successTimeAStar-tPredictAStar);
rmseRRT = sqrt(mseRRT);

figure;
plotregression(successTimeRRT, tPredictRRT, 'Regression');
mseAStar = mse(successTimeRRT-tPredictRRT);
rmseAStar = sqrt(mseAStar);