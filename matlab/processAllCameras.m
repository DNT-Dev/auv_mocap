function allDetections = processAllCameras(submitted_cameras)
    numCameras = numel(submitted_cameras);
    if numCameras == 0
        disp('No cameras were configured.');
        allDetections = {};
        return;
    end

    allDetections = cell(1, numCameras);

    for i = 1:numCameras
        fprintf('\\nProcessing Camera %d:\\n', i);
        cameraData = submitted_cameras{i};

        if isempty(cameraData.calibration)
            fprintf('  Skipping camera %d: Calibration file not specified.\\n', i);
            allDetections{i} = struct('camera_id', i, 'videos', [], 'error', 'Calibration file not specified', 'processed_videos_count', 0);
            continue;
        elseif ~exist(cameraData.calibration, 'file')
            fprintf('  Skipping camera %d: Calibration file \"%s\" not found.\\n', i, cameraData.calibration);
            allDetections{i} = struct('camera_id', i, 'videos', [], 'error', sprintf('Calibration file not found: %s', cameraData.calibration), 'processed_videos_count', 0);
            continue;
        end

        if isempty(cameraData.videos) || isempty(cameraData.videos{1}) || all(cellfun(@isempty, strtrim(cameraData.videos)))
            fprintf('  Skipping camera %d: No valid video files specified.\\n', i);
            allDetections{i} = struct('camera_id', i, 'videos', [], 'error', 'No valid video files specified', 'processed_videos_count', 0);
            continue;
        end

        cameraVideoResults = cell(1, numel(cameraData.videos));
        processedVideosCount = 0;

        for v = 1:numel(cameraData.videos)
            videoPath = strtrim(cameraData.videos{v});
            if isempty(videoPath)
                fprintf('    Skipping empty video path entry.\\n');
                cameraVideoResults{v} = struct('videoPath', '', 'detections', [], 'numFrames', 0, 'frameRate', 0, 'error', 'Empty video path entry');
                continue;
            end
            if ~exist(videoPath, 'file')
                fprintf('    Skipping video: \"%s\" not found or not specified.\\n', videoPath);
                cameraVideoResults{v} = struct('videoPath', videoPath, 'detections', [], 'numFrames', 0, 'frameRate', 0, 'error', 'Video file not found');
                continue;
            end

            fprintf('  Processing video: %s\\n', videoPath);
            
            try
                detector = ArucoDetector(videoPath);
                try
                    fprintf('    Attempting to load calibration using ArucoDetector: %s\\n', cameraData.calibration);
                    detector = detector.loadCameraParameters(cameraData.calibration);
                catch ME_calib_load
                    fprintf('    Warning: Error loading camera calibration file \"%s\" using ArucoDetector: %s\\n', cameraData.calibration, ME_calib_load.message);
                    fprintf('    ArucoDetector will proceed; pose estimation might be affected or disabled if parameters are not correctly set.\\n');
                end
                
                detector = detector.processVideo();
                
                cameraVideoResults{v} = struct('videoPath', videoPath, ...
                                             'detections', detector.MarkerResults, ...
                                             'numFrames', detector.NumFrames, ...
                                             'frameRate', detector.FrameRate, ...
                                             'error', []);
                fprintf('    Finished processing video. Found %d marker instances. Frames: %d, FPS: %.2f\\n', ...
                        height(detector.MarkerResults), detector.NumFrames, detector.FrameRate);
                processedVideosCount = processedVideosCount + 1;
                
            catch ME_video_proc
                fprintf('    Error processing video \"%s\": %s\\n', videoPath, ME_video_proc.message);
                fprintf('    Stack trace:\\n');
                errStack = ME_video_proc.stack;
                for k=1:length(errStack)
                    fprintf('      File: %s, Name: %s, Line: %d\\n', errStack(k).file, errStack(k).name, errStack(k).line);
                end
                cameraVideoResults{v} = struct('videoPath', videoPath, 'detections', [], 'numFrames', 0, 'frameRate', 0, 'error', ME_video_proc.message);
            end
        end
        allDetections{i} = struct('camera_id', i, 'calibration_file', cameraData.calibration, 'videos', {cameraVideoResults}, 'processed_videos_count', processedVideosCount);
    end
    disp('\\nAll camera processing complete.');
end
