% Correlator.m

function Correlator()
    % Launch the UI and get the submitted camera configurations
    disp('Launching Camera Configuration UI...');
    submitted_cameras = CorrelationUI(); % Call the UI function and get data

    % Check if any data was returned (e.g., if UI was closed prematurely)
    if isempty(submitted_cameras)
        disp('No camera data submitted or UI was closed before submission.');
        return;
    end

    disp('Camera data received:');
    disp(submitted_cameras);

    % Process each camera to get detections
    allDetections = processAllCameras(submitted_cameras);

    % Check if any detections were made
    if isempty(allDetections) || all(cellfun(@(x) isempty(x) || (isfield(x, 'processed_videos_count') && x.processed_videos_count == 0), allDetections))
        disp('No videos were processed successfully. Cannot proceed with correlation.');
        % Save what we have, if anything
        try
            save('aruco_detection_results.mat', 'allDetections', 'submitted_cameras');
            disp('Partial detection results saved to aruco_detection_results.mat');
        catch ME_save
            disp('Could not save partial results to .mat file:');
            disp(ME_save.message);
        end
        return;
    end
    
    assignin('base', 'all_aruco_detections', allDetections);
    assignin('base', 'submitted_camera_configs', submitted_cameras);
    disp('Detection results stored in workspace variables: all_aruco_detections and submitted_camera_configs.');

    % Extract signals for correlation
    [videoSignals, videoFilePaths, videoFrameRates, validSignalCount] = extractAndPrepareSignals(allDetections);

    % Perform correlation and save results
    if validSignalCount >= 2
        performCorrelationAndSaveResults(videoSignals, videoFilePaths, videoFrameRates, allDetections, submitted_cameras);
    else
        fprintf('Need at least two valid video signals to perform correlation. Found %d.\\n', validSignalCount);
        disp('Correlation step skipped.');
        % Save detection results even if correlation is skipped
        try
            % Check if file exists to append or create new
            if exist('aruco_detection_results.mat', 'file')
                save('aruco_detection_results.mat', 'allDetections', 'submitted_cameras', '-append');
            else
                save('aruco_detection_results.mat', 'allDetections', 'submitted_cameras');
            end
            disp('Detection results saved/appended to aruco_detection_results.mat');
        catch ME_save
            disp('Could not save detection results to .mat file:');
            disp(ME_save.message);
        end
    end
end