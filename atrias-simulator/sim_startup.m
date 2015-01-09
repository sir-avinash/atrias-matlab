%SIM_STARTUP ATRIAS simulation MATLAB startup script.

% Location of current folder on the file system
currentFolder = fileparts(mfilename('fullpath'));

% Add simulation folders to MATLAB path
addpath(...
    [currentFolder '/common/'], ...
    [currentFolder '/images/'], ...
    [currentFolder '/models/']);
