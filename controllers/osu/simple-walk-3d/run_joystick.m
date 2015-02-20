%RUN_JOYSTICK Sets controller velocity inputs using joystick.

% Initialize gamepad device
vrj = vrjoystick(1);

% Get model name
model = gcs;

% Loop while robot is running
while any(strcmp(get_param(model, 'SimulationStatus'), {'running' 'external'}))
  % Query joystick
  a = axis(vrj);

  % Set controller input block in model
  set_param([model '/Controller Input'], 'Value', mat2str(a(1:4)));
  
  % Flush system queue
  drawnow;
end % while

% Release joystick
clear vrj;
