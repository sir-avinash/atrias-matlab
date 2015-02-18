%RUN_3D_MOUSE Sets controller velocity inputs using 3D mouse.

% Initialize 3D Space Mouse object
vrm = vrspacemouse('USB1');

% Get controller name
controllerName = tg.Application;

% Loop while robot is running
while strcmp(get_param(controllerName, 'SimulationStatus'), 'external')
  % Get 3D mouse speed and scale accordingly
  v = [-0.2*vrm.speed(3); 0.1*vrm.speed(1); 0];

  % Set velocity input block in controller
  set_param([controllerName '/Controller Input'], 'Value', mat2str(v));

  % Flush system queue
  drawnow;
end % while
