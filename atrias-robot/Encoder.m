% This class handles the unwrapping and decoding for an encoder.

classdef Encoder < handle
	methods
		% The constructor. Takes in a couple of values about the encoder.
		%
		% Parameters:
		%     bits          The number of bits this encoder has (i.e. it should take an 2^bits distinct values total)
		%     rads_per_tick The number of radians rotated per tick for this encoder. The sign controls the encoder's direction conventions.
		function this = Encoder(rads_per_tick, bits)
			this.rads_per_tick = rads_per_tick;
            
            if nargin >= 2
                this.bits = int64(bits);
            else
                this.bits = int64(0);
            end
        end % Encoder

		% This sets up the initial calibration for the encoder.  
		%
		% Parameters:
		%     calib_pos   The current position [rad]
        %     calib_tics  The current position [tics]
		function initialize(this, calib_pos, calib_tics)
			this.calib_pos   = calib_pos;
            this.calib_tics  = int64(calib_tics);
			this.initialized = true;
		end

		% Update the class state and process the encoder's data.
		% This will return 0 for position and velocity until initialize() has been called.
		% Since two data samples are needed to compute velocity, the first call after initialize()
		% will return a zero velocity (so as to not rely on the availability of encoder data before initialize() is called).
		%
		% Parameters:
		%     new_ticks     The new encoder tick count.
		%     new_timestamp The new timestamp. Omit if no timestamps are available.
		%
		% Returns: The current position (radians) and velocity (rad/s)
		function [pos, vel] = update(this, new_ticks, new_timestamp)
            % Offset the tics by the calibration position
            new_ticks = int64(new_ticks) - this.calib_tics;

			% Make sure to output zeros until initialize() has been called.
			if ~this.initialized
				pos = 0.0;
				vel = 0.0;
				return
            end
            
            if nargin < 3
                new_timestamp = 0;
            end

			% Update the current position.
			dticks = new_ticks - this.cur_ticks; % Grab the difference -- this is correct mod 2^bits, but takes on the (incorrect) range (-2^bits, 2^bits)
            if this.bits ~= 0 % If we need to wrap
                dticks = mod(dticks + 2^(this.bits-1), 2^this.bits) - 2^(this.bits-1); % This leaves a correct value mod 2^bits, plus now it range (-2^(bits-1), 2^(bits-1))
            end
			this.cur_ticks = this.cur_ticks + dticks;

			% Compute the position (if requested by the caller)
			if nargin >= 1
				pos = this.calib_pos + this.rads_per_tick * double(this.cur_ticks);
			end

			% Update the velocity (if requested by the caller)
			if nargout >= 2
				% Account for the difference in time to sample the sensor
				dtimestamps = new_timestamp - this.timestamp;
                % Time between this sample and the previous one
				dt = this.sample_time + dtimestamps/this.MEDULLA_TIMER_FREQ;

				% Compute the velocity through finite differencing
				vel = this.rads_per_tick * double(dticks) / double(dt);
            end
            
            % Update the stored position and timestamp data
			this.prev_ticks = new_ticks;
			this.timestamp  = new_timestamp;
		end
	end

	properties
		% Basic encoder properties (see the constructor for their meanings)
		bits@int64
		rads_per_tick

		% Calibration position
		calib_pos        = 0; % [rad]
        calib_tics@int64 = int64(0); % [tics]

		% Saved state (used for differentiation in update())
		prev_ticks@int64
		timestamp = 0;

		% Current location (in ticks) -- this is after unwrapping. Relative to the calibration tick count
		cur_ticks@int64 = int64(0);

		% Initialization state
		initialized = false % Whether or not initialize() has been called
		running     = false % Whether or not ticks and timestamp have been properly initialized -- used to delay velocity updating until 2 samples have been done
    end
    
    properties (Access = private, Constant = true)
        sample_time = 0.001;
        MEDULLA_TIMER_FREQ = 32e6;
    end
end % classdef
