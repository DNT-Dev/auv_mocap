function [relative_delays_seconds, alignment_instructions] = CorrelationCalculator(camera_signals, reference_camera_index_in, frame_rate_in)
    %CORRELATIONCALCULATOR Calculates time delays for synchronizing multiple video signals.
    %   Port of Python function calculate_video_sync_delays from correlate.py
    %
    %   [relative_delays_seconds, alignment_instructions] = ...
    %       CorrelationCalculator(camera_signals, reference_camera_index, frame_rate)
    %
    %   Args:
    %       camera_signals (cell array): A cell array where each element is a 1D numeric
    %                                    vector representing the time-series signal for one camera.
    %                                    Each vector should have the same length.
    %       reference_camera_index_in (scalar, optional): The 1-based index of the camera to use
    %                                                 as the reference. Defaults to 1.
    %       frame_rate_in (float, optional): The frame rate of the video files in frames per
    %                                        second (fps). Defaults to 30.0.
    %
    %   Returns:
    %       A tuple containing two structs:
    %       1. relative_delays_seconds: Struct where fields are 'Camera_X' (e.g., 'Camera_0')
    %                                   and values are delays of each camera relative to the
    %                                   reference camera, in seconds (float).
    %       2. alignment_instructions: Struct where fields are 'Camera_X' and values are
    %                                  string instructions for trimming/padding each video
    %                                  to align them to a common start time (t0).
    %
    %   Raises:
    %       Error: If camera_signals is empty, invalid, or reference_camera_index
    %              is out of bounds, or signals have mismatched lengths.

    % Argument handling for defaults
    if nargin < 3 || isempty(frame_rate_in)
        frame_rate = 30.0;
    else
        frame_rate = frame_rate_in;
    end

    if nargin < 2 || isempty(reference_camera_index_in)
        reference_camera_index = 1; % MATLAB 1-based default
    else
        reference_camera_index = reference_camera_index_in; % User provided, 1-based
    end

    if ~iscell(camera_signals) || isempty(camera_signals)
        error('CorrelationCalculator:InvalidInput', 'camera_signals must be a non-empty cell array.');
    end
    
    num_cameras = length(camera_signals);
    if num_cameras == 0
        error('CorrelationCalculator:InvalidInput', 'camera_signals list cannot be empty.');
    end

    if not(isscalar(reference_camera_index) && isnumeric(reference_camera_index) && ...
           reference_camera_index >= 1 && reference_camera_index <= num_cameras && floor(reference_camera_index) == reference_camera_index)
        error('CorrelationCalculator:InvalidInput', ...
              'reference_camera_index %f is out of bounds for %d cameras. Must be an integer between 1 and %d.', ...
              reference_camera_index, num_cameras, num_cameras);
    end

    reference_signal_unverified = camera_signals{reference_camera_index};
    if ~isnumeric(reference_signal_unverified) || ~isvector(reference_signal_unverified)
         error('CorrelationCalculator:InvalidSignal',...
               'Signal for reference camera (Camera %d, 1-based index) must be a numeric vector.', reference_camera_index);
    end
    % Ensure reference signal is a row vector for consistency with xcorr interpretation
    reference_signal = reference_signal_unverified(:)'; 
    sequence_length = length(reference_signal);
    if sequence_length == 0
        error('CorrelationCalculator:InvalidSignal', 'Reference signal (Camera %d) cannot be empty.', reference_camera_index);
    end


    % Validate that all signals have the same length and are numeric vectors
    for i = 1:num_cameras
        signal_unverified = camera_signals{i};
        if ~isnumeric(signal_unverified) || ~isvector(signal_unverified)
            error('CorrelationCalculator:InvalidSignal',...
                  'Signal for Camera %d (1-based index) must be a numeric vector.', i);
        end
        if length(signal_unverified) ~= sequence_length
            error('CorrelationCalculator:MismatchedLengths',...
                  'Signal for Camera %d (1-based index) has length %d, but reference signal (Camera %d) has length %d. All signals must have the same length.', ...
                  i, length(signal_unverified), reference_camera_index, sequence_length);
        end
        % Ensure all signals are row vectors for xcorr
        camera_signals{i} = signal_unverified(:)';
    end

    calculated_delays_frames_struct = struct(); 
    relative_delays_seconds_struct = struct();

    fprintf('--- Calculating Delays relative to Camera %d (1-based index) ---\n', reference_camera_index);

    for i = 1:num_cameras
        camera_name_key = sprintf('Camera_%d', i-1); % For struct field (0-indexed name like 'Camera_0')
        current_camera_signal = camera_signals{i};

        if i == reference_camera_index
            calculated_delays_frames_struct.(camera_name_key) = 0;
            relative_delays_seconds_struct.(camera_name_key) = 0.0;
            fprintf('  %s (Reference - Input Cam Idx %d): Delay = 0 frames (0.00 s)\n', camera_name_key, reference_camera_index);
            continue;
        end
        
        % Perform cross-correlation.
        % [C, LAGS] = xcorr(A, B) returns the cross-correlation sequence C and the vector
        % LAGS containing the lag indices. C(k) is the correlation at LAGS(k).
        % If A is reference and B is current, a peak at a positive lag 'd' means
        % B is 'd' samples "after" A (B lags A by d).
        % A peak at a negative lag 'd' means B is 'd' samples "before" A (B leads A by |d|).
        % This matches the Python code's interpretation of delay_in_frames.
        [correlation_output, lags] = xcorr(reference_signal, current_camera_signal);
        
        [~, max_idx] = max(correlation_output);
        delay_in_frames = lags(max_idx); 
        delay_in_seconds = delay_in_frames / frame_rate;
        
        calculated_delays_frames_struct.(camera_name_key) = delay_in_frames;
        relative_delays_seconds_struct.(camera_name_key) = delay_in_seconds;
        
        fprintf('  %s (Input Cam Idx %d): Delay = %d frames (%.2f s)\n', camera_name_key, i, delay_in_frames, delay_in_seconds);
    end

    fprintf('\n--- Generating Alignment Instructions ---\n');
    
    all_delay_values_cell = struct2cell(calculated_delays_frames_struct);
    if isempty(all_delay_values_cell) % Should not happen if num_cameras > 0
        min_overall_delay_frames = 0;
    else
        all_delay_values_vector = [all_delay_values_cell{:}];
        min_overall_delay_frames = min(all_delay_values_vector);
    end
    

    fprintf('  Overall earliest start (relative to chosen reference''s start time): %d frames\n', min_overall_delay_frames);

    alignment_instructions_struct = struct();
    camera_names = fieldnames(calculated_delays_frames_struct); 
    for k = 1:length(camera_names)
        cam_name_key = camera_names{k};
        delay_frames = calculated_delays_frames_struct.(cam_name_key);
        
        shift_for_sync = delay_frames - min_overall_delay_frames;
        
        if shift_for_sync > 0
            alignment_instructions_struct.(cam_name_key) = sprintf('Trim %d frames from the start', shift_for_sync);
        elseif shift_for_sync < 0
            alignment_instructions_struct.(cam_name_key) = sprintf('Pad %d frames at the start (with black frames/duplicated first frame)', -shift_for_sync);
        else
            alignment_instructions_struct.(cam_name_key) = 'No shift needed (or it''s the earliest-starting camera)';
        end
        fprintf('  %s: %s\n', cam_name_key, alignment_instructions_struct.(cam_name_key));
    end

    % Assign to output arguments
    relative_delays_seconds = relative_delays_seconds_struct;
    alignment_instructions = alignment_instructions_struct;

end

% % --- Example Usage ---
% % To run this example, uncomment it and save CorrelationCalculator.m,
% % then call example_correlation_calculator from the MATLAB command window.
% 
% function example_correlation_calculator()
%     % Simulate pre-extracted time-series signals
%     fprintf('\n\n--- Running Example Usage ---\n');
% 
%     % Example: Simulate a basic movement for 500 frames
%     base_movement_time = linspace(0, 4 * pi, 500);
%     base_movement = sin(base_movement_time) + linspace(0, 1, 500) * 5; % Row vector
% 
%     % Camera 0 (Reference in Python example) - no true delay
%     signal_cam0 = base_movement + randn(1, 500) * 0.1;
% 
%     % Camera 1 - lags by 30 frames
%     signal_cam1 = circshift(base_movement, 30) + randn(1, 500) * 0.1;
% 
%     % Camera 2 - leads by 50 frames
%     signal_cam2 = circshift(base_movement, -50) + randn(1, 500) * 0.1;
% 
%     % Camera 3 - lags by 10 frames
%     signal_cam3 = circshift(base_movement, 10) + randn(1, 500) * 0.1;
% 
%     all_camera_signals_example = {signal_cam0, signal_cam1, signal_cam2, signal_cam3};
% 
%     fprintf('\n--- Running Synchronization Function (Reference: Camera 1 (1-based)) ---\n');
%     [relative_delays, alignment_instr] = CorrelationCalculator(...
%         all_camera_signals_example, ...
%         1, ... % Reference camera index (1-based), so signal_cam0
%         30.0 ...
%     );
% 
%     fprintf('\n--- RESULTS (Reference: Camera 1 (1-based)) ---');
%     fprintf('\nCalculated Delays (relative to reference camera):\n');
%     cam_names_results = fieldnames(relative_delays);
%     for k_res = 1:length(cam_names_results)
%         cam_key = cam_names_results{k_res};
%         fprintf('%s: %.2f seconds\n', cam_key, relative_delays.(cam_key));
%     end
% 
%     fprintf('\nFinal Video Alignment Instructions (to common t0):\n');
%     cam_names_instr = fieldnames(alignment_instr);
%     for k_instr = 1:length(cam_names_instr)
%         cam_key = cam_names_instr{k_instr};
%         fprintf('%s: %s\n', cam_key, alignment_instr.(cam_key));
%     end
% 
%     % Example with a different reference camera
%     % Python example used Camera 1 (0-indexed) as reference, which is signal_cam1
%     % In MATLAB, signal_cam1 is at index 2.
%     fprintf('\n--- Running Synchronization Function with Camera 2 (1-based) as Reference ---\n');
%     [relative_delays_cam2_ref, alignment_instr_cam2_ref] = CorrelationCalculator(...
%         all_camera_signals_example, ...
%         2, ... % Now using Camera 2 (1-based index, which is signal_cam1) as reference
%         30.0 ...
%     );
%     fprintf('\n--- RESULTS (Reference: Camera 2 (1-based)) ---');
%     fprintf('\nCalculated Delays (relative to Camera 2 (1-based)):\n');
%     cam_names_results_c2 = fieldnames(relative_delays_cam2_ref);
%     for k_res_c2 = 1:length(cam_names_results_c2)
%         cam_key_c2 = cam_names_results_c2{k_res_c2};
%         fprintf('%s: %.2f seconds\n', cam_key_c2, relative_delays_cam2_ref.(cam_key_c2));
%     end
% 
%     fprintf('\nFinal Video Alignment Instructions (to common t0):\n');
%     cam_names_instr_c2 = fieldnames(alignment_instr_cam2_ref);
%     for k_instr_c2 = 1:length(cam_names_instr_c2)
%         cam_key_c2 = cam_names_instr_c2{k_instr_c2};
%         fprintf('%s: %s\n', cam_key_c2, alignment_instr_cam2_ref.(cam_key_c2));
%     end
% end
% 
% % To make the example runnable, you could define it as a local function
% % (as above, but without the outer 'function example_correlation_calculator()' line if it's
% % just a script block you copy-paste) or call it:
% % example_correlation_calculator(); 
% % when you want to test it.
