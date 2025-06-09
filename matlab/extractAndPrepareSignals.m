function [videoSignals, videoFilePaths, videoFrameRates, validSignalCount] = extractAndPrepareSignals(allDetections)
    disp('\\\\n--- Preparing signals for CorrelationCalculator ---');
    
    videoSignals = {};
    videoFilePaths = {};
    videoFrameRates = []; % Store all unique frame rates encountered
    validSignalCount = 0;
    numCameras = numel(allDetections);
    
    firstFrameRate = []; % To store the very first frame rate encountered, used for warning

    for i = 1:numCameras
        if isempty(allDetections{i}) || ~isfield(allDetections{i}, 'videos') || isempty(allDetections{i}.videos)
            fprintf('Skipping Camera %d for signal extraction: No video data found.\\\\n', i);
            continue;
        end
        
        cameraData = allDetections{i}; % This is a struct
        if ~isfield(cameraData, 'videos') || isempty(cameraData.videos)
            continue;
        end
        cameraVideoDataArray = cameraData.videos; % This is a cell array of structs
        
        for v = 1:numel(cameraVideoDataArray)
            videoData = cameraVideoDataArray{v};
            
            if isempty(videoData) || ~isfield(videoData, 'detections') || isempty(videoData.detections) || ~isempty(videoData.error)
                if isfield(videoData, 'videoPath') && ~isempty(videoData.videoPath)
                    fprintf('  Skipping video %s (Cam %d, Vid %d): No valid detections or an error occurred during processing.\\\\n', videoData.videoPath, i, v);
                else
                    fprintf('  Skipping video (Cam %d, Vid %d): Video path unknown, no valid detections or an error occurred.\\\\n', i, v);
                end
                continue;
            end
            
            fprintf('  Extracting signal from: %s\\\\n', videoData.videoPath);
            
            try
                if isfield(videoData, 'numFrames') && isfield(videoData, 'frameRate') && ...
                   ~isempty(videoData.numFrames) && videoData.numFrames > 0 && ...
                   ~isempty(videoData.frameRate) && videoData.frameRate > 0
                    numFrames = videoData.numFrames;
                    frameRate = videoData.frameRate;
                else
                    fprintf('    Error: numFrames (%s) or frameRate (%s) missing, zero, or empty in videoData for %s. Skipping signal extraction.\\n', ...
                            mat2str(videoData.numFrames), mat2str(videoData.frameRate), videoData.videoPath);
                    continue;
                end

                if isempty(firstFrameRate)
                    firstFrameRate = frameRate; % Store the first valid frame rate
                elseif firstFrameRate ~= frameRate
                    warning('Correlator:FrameRateMismatch', ...
                            'Video %s has frame rate %.2f fps, but the first processed video had %.2f fps. Correlation will use the frame rate of the first video (%.2f fps). Synchronization accuracy may be affected if rates differ significantly.', ...
                            videoData.videoPath, frameRate, firstFrameRate, firstFrameRate);
                end
                % Store this video's frame rate
                videoFrameRates = [videoFrameRates, frameRate];


                signal = zeros(1, numFrames);
                isFrameSet = false(1, numFrames);
                markerEvents = videoData.detections;
                
                if height(markerEvents) > 0
                    for k_event = 1:height(markerEvents)
                        frameIdx_from_table = markerEvents.FrameIndex(k_event);
                        
                        if frameIdx_from_table < 1 || frameIdx_from_table > numFrames
                            fprintf('    Warning: Invalid FrameIndex %d found in marker data for video %s. Max frames: %d. Skipping event.\\\\n', ...
                                    frameIdx_from_table, videoData.videoPath, numFrames);
                            continue;
                        end
                        
                        if ~isFrameSet(frameIdx_from_table)
                            tvec_cell = markerEvents.TranslationVector{k_event};
                            if ~isempty(tvec_cell) && isnumeric(tvec_cell) && isvector(tvec_cell) && numel(tvec_cell) == 3
                                tvec = tvec_cell(:);
                                signal(frameIdx_from_table) = norm(tvec);
                                isFrameSet(frameIdx_from_table) = true;
                            end
                        end
                    end
                end
                
                videoSignals{end+1} = signal;
                videoFilePaths{end+1} = videoData.videoPath;
                validSignalCount = validSignalCount + 1;
                fprintf('    Signal extracted for %s (Length: %d frames, FPS: %.2f).\\n', videoData.videoPath, numFrames, frameRate);
                
            catch ME_signal
                fprintf('    Error extracting signal for video %s: %s\\\\n', videoData.videoPath, ME_signal.message);
            end
        end
    end
    
    if validSignalCount > 0
        disp('Signal extraction phase complete.');
    else
        disp('No valid signals could be extracted.');
    end
end
