function performCorrelationAndSaveResults(videoSignals, videoFilePaths, allVideoFrameRates, allDetections, submitted_cameras)
    if isempty(videoSignals) || numel(videoSignals) < 2
        disp('Not enough video signals to perform correlation.');
        return;
    end

    disp('--- Normalizing signal lengths ---');
    maxLength = 0;
    for k = 1:length(videoSignals)
        if length(videoSignals{k}) > maxLength
            maxLength = length(videoSignals{k});
        end
    end
    
    for k = 1:length(videoSignals)
        currentLength = length(videoSignals{k});
        if currentLength < maxLength
            videoSignals{k} = [videoSignals{k}, zeros(1, maxLength - currentLength)];
            fprintf('  Padded signal for %s from %d to %d frames.\\\\n', videoFilePaths{k}, currentLength, maxLength);
        end
    end
    
    reference_video_index = 1; 
    
    correlation_frame_rate = 0;
    if ~isempty(allVideoFrameRates)
        uniqueFrameRates = unique(allVideoFrameRates);
        if numel(uniqueFrameRates) > 1
            warning('Correlator:MultipleFrameRates', ...
                    'Multiple frame rates detected among videos: %s. Using the first one (%.2f fps) for correlation. Ensure this is appropriate.', ...
                    mat2str(uniqueFrameRates), uniqueFrameRates(1));
        end
        correlation_frame_rate = uniqueFrameRates(1); % Use the first unique frame rate
    else
        warning('Correlator:NoFrameRateForCorrelation', 'Could not determine frame rate for any video. Assuming default of 30 fps for CorrelationCalculator.');
        correlation_frame_rate = 30.0; % Default if none determined
    end

    fprintf('\\\\n--- Calling CorrelationCalculator ---\\\\n');
    fprintf('Number of signals: %d\\\\n', length(videoSignals));
    if ~isempty(videoFilePaths)
        fprintf('Reference video index (1-based): %d (%s)\\\\n', reference_video_index, videoFilePaths{reference_video_index});
    else
        fprintf('Reference video index (1-based): %d (File path not available)\\\\n', reference_video_index);
    end
    fprintf('Frame rate for correlation: %.2f fps\\\\n', correlation_frame_rate);
    
    try
        [relative_delays_seconds, alignment_instructions] = CorrelationCalculator(videoSignals, reference_video_index, correlation_frame_rate);
        
        disp('\\\\n--- Correlation Results ---');
        disp('Relative Delays (seconds):');
        disp(relative_delays_seconds);
        
        disp('\\\\nAlignment Instructions:');
        disp(alignment_instructions);
        
        assignin('base', 'correlation_delays_seconds', relative_delays_seconds);
        assignin('base', 'correlation_alignment_instructions', alignment_instructions);
        assignin('base', 'correlated_video_paths', videoFilePaths);
        
        disp('Correlation results stored in workspace variables.');
        
        % Save all results to .mat file
        resultsToSave = struct(...
            'all_aruco_detections', allDetections, ...
            'submitted_camera_configs', submitted_cameras, ...
            'correlation_delays_seconds', relative_delays_seconds, ...
            'correlation_alignment_instructions', alignment_instructions, ...
            'correlated_video_paths', {videoFilePaths}, ... % Cell array, ensure it's stored correctly
            'correlation_frame_rate', correlation_frame_rate ...
        );
        
        save('aruco_detection_results.mat', '-struct', 'resultsToSave');
        disp('All detection and correlation results saved to aruco_detection_results.mat');
        
    catch ME_correlate
        fprintf('Error during CorrelationCalculator execution: %s\\\\n', ME_correlate.message);
        fprintf('Stack trace:\\\\n');
        errStack = ME_correlate.stack;
        for k_err=1:length(errStack)
            fprintf('  File: %s, Name: %s, Line: %d\\\\n', errStack(k_err).file, errStack(k_err).name, errStack(k_err).line);
        end
        disp('Correlation could not be completed. Saving detection results only.');
        % Save detection results if correlation failed
        try
            if exist('aruco_detection_results.mat', 'file')
                 save('aruco_detection_results.mat', 'allDetections', 'submitted_cameras', '-append');
            else
                 save('aruco_detection_results.mat', 'allDetections', 'submitted_cameras');
            end
            disp('Detection results saved/appended to aruco_detection_results.mat');
        catch ME_save_fallback
            disp('Could not save detection results to .mat file after correlation error:');
            disp(ME_save_fallback.message);
        end
    end
end
