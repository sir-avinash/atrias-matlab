% Initialize 3D Space Mouse object
vrm = vrspacemouse('USB1');

% Get controller name
controllerName = tg.Application;

% Loop forever 
while strcmp(get_param(controllerName, 'SimulationStatus'), 'external')
  % Get mouse input
  v = vrm.speed(1:3);
  
  % Set velocity input block in controller
  set_param([controllerName '/3D Mouse Input'], 'Value', mat2str(v));
  
  % Flush
  drawnow;
end % while