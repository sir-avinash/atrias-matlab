%OSU_STARTUP Oregon State University MATLAB startup script.
    
% Location of current folder on the file system
currentFolder = pwd;

% Add simulation folders to MATLAB path
addpath(...
    [currentFolder '/atrias-simulator/common/'], ...
    [currentFolder '/atrias-simulator/images/'], ...
    [currentFolder '/atrias-simulator/models/']);

% Add real-time folders to MATLAB path
addpath(...
    [currentFolder '/atrias-robot/']);

% Add tools folder to MATLAB path
addpath(...
    [currentFolder '/tools/']);