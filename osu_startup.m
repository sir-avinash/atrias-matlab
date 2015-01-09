%OSU_STARTUP Oregon State University MATLAB startup script.

% Location of current folder on the file system
currentFolder = fileparts(mfilename('fullpath'));

% Add simulation folders to MATLAB path
addpath([currentFolder '/atrias-simulator/']);
sim_startup;

% Add real-time folders to MATLAB path
addpath([currentFolder '/atrias-robot/']);

% Add tools folder to MATLAB path
addpath([currentFolder '/tools/']);

% Open model to assign parameters
atrias_system;

% Assign ethercat parameters
set_param('atrias_system/EtherCAT Init', 'pci_bus', '2');
set_param('atrias_system/EtherCAT Init', 'pci_slot', '0');

% Set workspace variables
daq_params_osu;

% Bring up the real-time explorer
slrtexplr;