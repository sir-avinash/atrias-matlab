function run_ps3(isTest)
%RUN_PS3 Run PS3 controller input with graphical display.

  % Check input arguments and set defaults
  if nargin == 0; isTest = false; end % if

  % Initialize while loop exit variable
  isRun = true;

  % Initialize joystick
  vrj = vrjoystick(1);
  
  % Get current simulink model
  model = gcs;

  % Initialize figure
  figure(...
    'Position' , [0 0 1280 320], ...
    'CloseRequestFcn', @onClose);

  % Center figure in screen
  movegui(gcf, 'center');

  % Initialize left stick plot
  subplot(1,3,1);
  hold on; grid on; grid minor; box on; axis square;
  title('Left Joystick');
  h1 = plot(0, 0, '*b');
  xlim([-1 1]);
  ylim([-1 1]);

  % Initialize right stick plot
  subplot(1,3,2);
  hold on; grid on; grid minor; box on; axis square;
  title('Right Joystick');
  h2 = plot(0, 0, '*r');
  xlim([-1 1]);
  ylim([-1 1]);

  % Initialize right stick plot
  ax = subplot(1,3,3);
  hold on; grid on; box on; axis square;
  title('Buttons');
  h3 = barh(1:17, zeros(1,17));
  xlim([0 1]);
  ylim([0 18]);
  ax.XTick = [0 1];
  ax.YTick = 1:17;
  ax.YTickLabel = {'Select' 'LS Click' 'RS Click' 'Start' ...
    'D-Pad Up' 'D-Pad Right' 'D-Pad Down' 'D-Pad Left' ...
    'LL Trigger' 'RL Trigger' 'LU Trigger' 'RU Trigger' ...
    'Triangle' 'Circle' 'Cross' 'Square' 'PS3'};

  % Loop while in run state
  while isRun
    % Query joystick
    a = axis(vrj);
    b = double(button(vrj));

    % Update plots
    set(h1, 'XData', a(1), 'YData', -a(2));
    set(h2, 'XData', a(3), 'YData', -a(4));
    set(h3, 'YData', b);

    % Check test state
    if ~isTest
      % Pass joystick data to Simulink model
      set_param([model '/PS3 Axes'], 'Value', mat2str(a));
      set_param([model '/PS3 Buttons'], 'Value', mat2str(b));
    end % if

    % Flush system queue
    drawnow;
  end % while

  % Release joystick
  clear vrj;

  function onClose(src, ~)
  %ONCLOSE Figure close callback function.

    % Set run state to false
    isRun = false;

    % Delete object
    delete(src);
  end % onClose
end % run_ps3
