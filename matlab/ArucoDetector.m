% ArUcoDetector.m
%
% This MATLAB class detects ArUco markers in a video file,
% records their IDs, timestamp, and their 3D pose (rotation and translation vectors).
% It supports live visualization of the detected markers and their poses.
%
% Requires:
%   - MATLAB Computer Vision Toolbox (for readArucoMarkers, cameraParameters, worldToImage)
%   - MATLAB Image Processing Toolbox (for insertShape, insertText, imread, imwrite, VideoWriter)

classdef ArucoDetector
    properties
        VideoPath           % Path to the input video file
        MarkerResults       % Table to store detected marker IDs, FrameIndex, and poses
        VideoReaderObj      % VideoReader object for video processing
        ArUcoDictionary     % The ArUco dictionary to use for detection (e.g., '6x6_250')
        DisplayLiveDetection % Boolean flag to enable/disable live visualization (default: false)
        VideoFigure         % Handle to the figure used for live visualization
        CameraParams        % cameraParameters object for pose estimation (optional)
        MarkerSize          % Size of the ArUco marker in meters (e.g., 0.05 for 5cm)
        NumFrames           % Total number of frames in the video
        FrameRate           % Frame rate of the video (fps)
    end

    methods
        function obj = ArucoDetector(videoPath)
            % Constructor: Initializes the ArUcoDetector object.
            %
            % Args:
            %   videoPath: A string specifying the full path to the video file.

            if ~ischar(videoPath) || isempty(videoPath)
                error('ArUcoDetector:InvalidVideoPath', 'Video path must be a non-empty string.');
            end
            if ~exist(videoPath, 'file')
                error('ArUcoDetector:FileNotFound', 'Video file not found at: %s', videoPath);
            end

            obj.VideoPath = videoPath;
            try
                obj.VideoReaderObj = VideoReader(videoPath);
                % Store NumFrames and FrameRate upon successful VideoReader creation
                obj.NumFrames = obj.VideoReaderObj.NumFrames;
                obj.FrameRate = obj.VideoReaderObj.FrameRate;
            catch ME
                error('ArUcoDetector:VideoReaderError', 'Could not open video file: %s\nError: %s', videoPath, ME.message);
            end

            % Initialize MarkerResults as an empty table
            % Columns: 'MarkerID' (numeric), 'FrameIndex' (uint32),
            %          'RotationVector' (cell), 'TranslationVector' (cell)
            obj.MarkerResults = table('Size', [0, 4], ...
                                      'VariableTypes', {'double', 'uint32', 'cell', 'cell'}, ...
                                      'VariableNames', {'MarkerID', 'FrameIndex', 'RotationVector', 'TranslationVector'});
            
            % IMPORTANT: The processVideo method must be updated to store 1-based frame indices
            % in the 'FrameIndex' column instead of timestamps.
            % For example, if 'frameCounter' is your 1-based frame index being looped through:
            %   newRow = table(markerIds(j), uint32(frameCounter), {rvec}, {tvec}, ...
            %                  'VariableNames', {'MarkerID', 'FrameIndex', 'RotationVector', 'TranslationVector'});
            %   obj.MarkerResults = [obj.MarkerResults; newRow];


            % Set default ArUco dictionary and marker size as requested.
            obj.ArUcoDictionary = '4x4_250'; % Changed default to 4x4_250
            obj.MarkerSize = 0.15; % Changed default to 0.15 meters

            obj.DisplayLiveDetection = false; % Default to no live display
            obj.VideoFigure = []; % Initialize figure handle to empty
            obj.CameraParams = []; % Initialize camera parameters to empty

            fprintf('ArUcoDetector initialized for video: %s\n', videoPath);
            fprintf('Using default ArUco dictionary: %s\n', obj.ArUcoDictionary);
            fprintf('Default marker size for pose estimation: %.2f meters\n', obj.MarkerSize);
        end

        function obj = setDictionary(obj, dictionaryName)
            % Sets the ArUco dictionary to be used for detection.
            %
            % Args:
            %   dictionaryName: A string specifying the name of the ArUco dictionary.
            %                   e.g., '4x4_50', '5x5_100', '6x6_250'.
            %                   Refer to readArucoMarkers documentation for supported types.

            if ~ischar(dictionaryName) || isempty(dictionaryName)
                error('ArUcoDetector:InvalidDictionary', 'Dictionary name must be a non-empty string.');
            end
            obj.ArUcoDictionary = dictionaryName;
            fprintf('ArUco dictionary set to: %s\n', obj.ArUcoDictionary);
        end

        function obj = setMarkerSize(obj, markerSize)
            % Sets the physical size of the ArUco marker in meters.
            % This is crucial for accurate pose estimation.
            %
            % Args:
            %   markerSize: A scalar representing the physical size of the marker.

            if ~isnumeric(markerSize) || ~isscalar(markerSize) || markerSize <= 0
                error('ArUcoDetector:InvalidMarkerSize', 'Marker size must be a positive scalar.');
            end
            obj.MarkerSize = markerSize;
            fprintf('Marker size for pose estimation set to: %.2f meters\n', obj.MarkerSize);
        end

        function obj = setLiveDisplay(obj, enableDisplay)
            % setLiveDisplay: Enables or disables live visualization of detected markers.
            %
            % Args:
            %   enableDisplay: A logical (true/false) to enable or disable display.

            if ~islogical(enableDisplay) || isempty(enableDisplay)
                error('ArUcoDetector:InvalidDisplaySetting', 'Display setting must be a logical true or false.');
            end
            obj.DisplayLiveDetection = enableDisplay;
            if enableDisplay
                fprintf('Live detection visualization enabled.\n');
            else
                fprintf('Live detection visualization disabled.\n');
                % Close the figure if it's open and valid
                if ~isempty(obj.VideoFigure) && isvalid(obj.VideoFigure)
                    close(obj.VideoFigure);
                    obj.VideoFigure = [];
                end
            end
        end

        function obj = loadCameraParameters(obj, calibrationFilePath)
            % loadCameraParameters: Loads camera intrinsic parameters from a custom TXT file.
            %
            % Args:
            %   calibrationFilePath: Path to the camera calibration TXT file.
            %
            % The TXT file should have content similar to:
            % [image]
            % width
            % 640
            % height
            % 480
            % [narrow_stereo]
            % camera matrix
            % 624.753385 0.000000 358.394413
            % 0.000000 624.047985 259.912709
            % 0.000000 0.000000 1.000000
            % distortion
            % 0.021105 -0.295123 0.002860 0.012346 0.000000
            % rectification
            % 1.000000 0.000000 0.000000
            % 0.000000 1.000000 0.000000
            % 0.000000 0.000000 1.000000
            % projection
            % 606.312439 0.000000 368.167601 0.000000
            % 0.000000 621.187683 260.762972 0.000000
            % 0.000000 0.000000 1.000000 0.000000

            if ~exist(calibrationFilePath, 'file')
                error('ArUcoDetector:CalibrationFileNotFound', 'Camera calibration file not found at: %s', calibrationFilePath);
            end

            fid = fopen(calibrationFilePath, 'r');
            if fid == -1
                error('ArUcoDetector:FileOpenError', 'Could not open camera calibration file: %s', calibrationFilePath);
            end

            imgWidth = [];
            imgHeight = [];
            cameraMatrix = zeros(3,3);
            distortionCoeffs = zeros(1,5);
            lineNum = 0;
            readState = 'none'; % 'width', 'height', 'cameraMatrix', 'distortion'
            matrixRowIdx = 1;

            try
                while ~feof(fid)
                    lineNum = lineNum + 1;
                    tline = fgetl(fid);
                    tline = strtrim(tline); % Remove leading/trailing whitespace

                    if isempty(tline) || startsWith(tline, '#') % Skip empty lines and comments
                        continue;
                    end

                    if contains(tline, '[image]', 'IgnoreCase', true)
                        readState = 'image_info';
                    elseif contains(tline, 'width', 'IgnoreCase', true) && strcmp(readState, 'image_info')
                        tline = fgetl(fid); lineNum = lineNum + 1; % Read next line for value
                        imgWidth = str2double(strtrim(tline));
                    elseif contains(tline, 'height', 'IgnoreCase', true) && strcmp(readState, 'image_info')
                        tline = fgetl(fid); lineNum = lineNum + 1; % Read next line for value
                        imgHeight = str2double(strtrim(tline));
                    elseif contains(tline, 'camera matrix', 'IgnoreCase', true)
                        readState = 'cameraMatrix';
                        matrixRowIdx = 1; % Reset row index for matrix
                    elseif contains(tline, 'distortion', 'IgnoreCase', true)
                        readState = 'distortion';
                    elseif contains(tline, '[narrow_stereo]', 'IgnoreCase', true) || ...
                           contains(tline, 'rectification', 'IgnoreCase', true) || ...
                           contains(tline, 'projection', 'IgnoreCase', true)
                        % Skip these sections or reset state
                        readState = 'none'; % Or specific state if we needed other data
                    else
                        % Process data based on current read state
                        if strcmp(readState, 'cameraMatrix') && matrixRowIdx <= 3
                            values = sscanf(tline, '%f');
                            if length(values) == 3
                                cameraMatrix(matrixRowIdx, :) = values';
                                matrixRowIdx = matrixRowIdx + 1;
                            else
                                warning('ArUcoDetector:ParseWarning', 'Line %d: Expected 3 values for camera matrix row, got %d. Skipping.', lineNum, length(values));
                            end
                        elseif strcmp(readState, 'distortion')
                            values = sscanf(tline, '%f');
                            if length(values) >= 5
                                distortionCoeffs = values(1:5)'; % Take first 5 for standard k1,k2,p1,p2,k3
                                readState = 'none'; % Done reading distortion
                            else
                                warning('ArUcoDetector:ParseWarning', 'Line %d: Expected at least 5 values for distortion, got %d. Skipping.', lineNum, length(values));
                            end
                        end
                    end
                end
            catch ME
                fclose(fid);
                error('ArUcoDetector:ParseError', 'Error parsing calibration file at line %d: %s\n%s', lineNum, ME.message, ME.getReport);
            end

            fclose(fid);

            % Validate parsed data
            if isempty(imgWidth) || isempty(imgHeight) || any(cameraMatrix(:) == 0) || any(distortionCoeffs(:) == 0)
                error('ArUcoDetector:IncompleteCalibration', 'Failed to parse complete camera parameters from %s. Missing width, height, camera matrix, or distortion coefficients.', calibrationFilePath);
            end

            % Create cameraParameters object
            obj.CameraParams = cameraParameters('IntrinsicMatrix', cameraMatrix', ... % IntrinsicMatrix expects transpose
                                                'ImageSize', [imgHeight, imgWidth], ...
                                                'RadialDistortion', distortionCoeffs(1:2), ... % k1, k2
                                                'TangentialDistortion', distortionCoeffs(3:4), ... % p1, p2
                                                'K3', distortionCoeffs(5)); % k3

            fprintf('Camera parameters loaded successfully from: %s\n', calibrationFilePath);
            fprintf('  Image Size: [%d %d]\n', imgHeight, imgWidth);
            fprintf('  Intrinsic Matrix:\n'); disp(obj.CameraParams.IntrinsicMatrix);
            fprintf('  Radial Distortion: [%f %f]\n', obj.CameraParams.RadialDistortion);
            fprintf('  Tangential Distortion: [%f %f]\n', obj.CameraParams.TangentialDistortion);
            fprintf('  K3: %f\n', obj.CameraParams.K3);
        end


        function obj = detectMarkers(obj)
            % detectMarkers: Processes the video frame by frame to detect ArUco markers.
            % Stores the detected marker IDs, timestamps, and poses in obj.MarkerResults.
            % Optionally visualizes the detection process if DisplayLiveDetection is true.

            fprintf('Starting marker detection for video: %s\n', obj.VideoPath);
            if isempty(obj.CameraParams)
                warning('ArUcoDetector:NoCameraParams', 'No camera parameters loaded. Pose estimation will not be performed.');
                performPoseEstimation = false;
            else
                performPoseEstimation = true;
                fprintf('Pose estimation is enabled using loaded camera parameters and marker size %.2f meters.\n', obj.MarkerSize);
            end

            frameIdx = 0;
            % Reset video reader to the beginning before starting
            obj.VideoReaderObj.CurrentTime = 0;
            totalFrames = obj.VideoReaderObj.NumFrames;

            % Reset MarkerResults for a new detection run
            obj.MarkerResults = table('Size', [0, 4], ...
                                      'VariableTypes', {'double', 'uint32', 'cell', 'cell'}, ...
                                      'VariableNames', {'MarkerID', 'FrameIndex', 'RotationVector', 'TranslationVector'});
            obj.MarkerResults.RotationVector = cell(0,1);
            obj.MarkerResults.TranslationVector = cell(0,1);

            % Initialize figure for live display if enabled
            if obj.DisplayLiveDetection
                % Check if Image Processing Toolbox is available for visualization functions
                if ~license('test', 'Image_Toolbox')
                    warning('ArUcoDetector:MissingToolbox', 'Image Processing Toolbox is required for live visualization (insertShape, insertText) but is not available. Disabling live display.');
                    obj.DisplayLiveDetection = false; % Disable if toolbox is missing
                else
                    if isempty(obj.VideoFigure) || ~isvalid(obj.VideoFigure)
                        obj.VideoFigure = figure('Tag', 'ArUcoLiveDetection', 'Name', 'Live ArUco Detection');
                    else
                        figure(obj.VideoFigure); % Bring existing figure to front
                    end
                    % Clear previous axes contents and get current axes handle
                    clf(obj.VideoFigure);
                    ax = axes('Parent', obj.VideoFigure);
                end
            end

            try
                % Loop through each frame of the video
                while hasFrame(obj.VideoReaderObj)
                    frameIdx = frameIdx + 1;
                    videoFrame = readFrame(obj.VideoReaderObj);
                    timestamp = obj.VideoReaderObj.CurrentTime;

                    % Display progress in the command window
                    if mod(frameIdx, 50) == 0 || frameIdx == totalFrames
                        fprintf('Processing frame %d of %d (%.1f seconds)\n', frameIdx, totalFrames, timestamp);
                    end

                    % Detect ArUco markers in the current frame
                    [markerIds, markerCorners, ~] = readArucoMarkers(videoFrame, obj.ArUcoDictionary);

                    if ~isempty(markerIds)
                        % If markers are detected, estimate their poses
                        if performPoseEstimation
                            [rvecs, tvecs, ~] = estimateArucoPose(markerCorners, markerIds, obj.MarkerSize, obj.CameraParams);
                        else
                            % Create empty cell arrays for rvecs and tvecs if pose estimation is off
                            rvecs = cell(length(markerIds), 1);
                            tvecs = cell(length(markerIds), 1);
                        end

                        % Store results for each detected marker
                        for j = 1:length(markerIds)
                            rvec = []; tvec = []; % Initialize to empty
                            if performPoseEstimation && j <= size(rvecs,1) && j <= size(tvecs,1)
                                rvec = rvecs(j,:); % Store as row vector
                                tvec = tvecs(j,:); % Store as row vector
                            end
                            
                            % Create a new row for the MarkerResults table
                            % Using FrameIndex (1-based) instead of Timestamp
                            newRow = table(markerIds(j), uint32(frameIdx), {rvec}, {tvec}, ...
                                           'VariableNames', {'MarkerID', 'FrameIndex', 'RotationVector', 'TranslationVector'});
                            obj.MarkerResults = [obj.MarkerResults; newRow];
                        end

                        % Live visualization if enabled
                        if obj.DisplayLiveDetection && license('test', 'Image_Toolbox')
                            % Draw marker outlines and IDs on the frame
                            videoFrame = insertShape(videoFrame, 'Polygon', reshape(markerCorners, [], 2), 'LineWidth', 3, 'Color', 'yellow');
                            textPositions = squeeze(markerCorners(1,1,:,:))'; % Use top-left corner for text position
                            videoFrame = insertText(videoFrame, textPositions, arrayfun(@num2str, markerIds, 'UniformOutput', false), 'BoxOpacity', 0.7, 'FontSize', 14);

                            % If pose estimation is enabled, draw axes
                            if performPoseEstimation
                                for k = 1:length(markerIds)
                                    if k <= size(rvecs,1) && k <= size(tvecs,1)
                                        worldPoints = [0 0 0; obj.MarkerSize/2 0 0; 0 obj.MarkerSize/2 0; 0 0 obj.MarkerSize/2];
                                        imagePoints = worldToImage(obj.CameraParams, rvecs(k,:), tvecs(k,:), worldPoints);
                                        % Draw axes lines
                                        videoFrame = insertShape(videoFrame, 'Line', [imagePoints(1,:) imagePoints(2,:); imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
                                                                 'LineWidth', 3, 'Color', {'red', 'green', 'blue'});
                                    end
                                end
                            end
                        end
                    end

                    % Display the frame if live detection is enabled
                    if obj.DisplayLiveDetection && license('test', 'Image_Toolbox')
                        imshow(videoFrame, 'Parent', ax);
                        title(ax, sprintf('Frame: %d, Time: %.2f s, Markers: %d', frameIdx, timestamp, length(markerIds)));
                        drawnow limitrate; % Update figure window, limit to 20 fps to avoid slowdown
                    end
                end % End of while hasFrame
            catch ME
                % Handle potential errors during marker detection
                warning('ArUcoDetector:DetectionError', ...
                        'An error occurred during marker detection in frame %d at timestamp %.2f: %s', ...
                        frameIdx, timestamp, ME.message);
            end

            fprintf('Marker detection complete. Found %d total marker instances.\n', size(obj.MarkerResults, 1));
            % Reset video reader to the beginning for future operations
            obj.VideoReaderObj.CurrentTime = 0;

            % Close the live display figure after detection is complete, if it was opened and is valid
            if obj.DisplayLiveDetection && ~isempty(obj.VideoFigure) && isvalid(obj.VideoFigure)
                close(obj.VideoFigure);
                obj.VideoFigure = [];
            end
        end

        function results = getResults(obj)
            % getResults: Returns the table of detected marker IDs, timestamps, and poses.
            %
            % Returns:
            %   A table containing 'MarkerID', 'Timestamp', 'RotationVector', and 'TranslationVector' columns.
            results = obj.MarkerResults;
        end

        function displayResults(obj)
            % displayResults: Prints the detected marker results to the command window.
            if isempty(obj.MarkerResults)
                fprintf('No ArUco markers were detected.\n');
            else
                fprintf('\n--- Detected ArUco Marker Results ---\n');
                % Display only relevant columns for concise output
                if all(cellfun(@(x) all(isnan(x)), obj.MarkerResults.RotationVector))
                     disp(obj.MarkerResults(:, {'MarkerID', 'FrameIndex'}));
                     fprintf('Pose data not available (no camera parameters or no markers detected with pose).\n');
                else
                    disp(obj.MarkerResults);
                end
                fprintf('-------------------------------------\n');
            end
        end
    end
end

% Example Usage (you can put this in a separate script or run in command window):
%{
% --- Section 1: Create Dummy Video and Camera Calibration File (if you don't have them) ---
% This part requires Image Processing Toolbox and Computer Vision Toolbox.

% Check if dummy video already exists
videoFilePath = 'test_aruco_video.avi';
calibrationFilePath = 'camera_params.txt';

if ~exist(videoFilePath, 'file') || ~exist(calibrationFilePath, 'file')
    fprintf('Creating dummy video and camera calibration file...\n');
    try
        % Create dummy video
        v = VideoWriter(videoFilePath, 'Motion JPEG AVI');
        v.FrameRate = 10;
        open(v);

        markerId1 = 123; markerSizePx = 50;
        markerImage1 = arucoMarker(markerId1, '4x4_250', markerSizePx); % ID 123, using the new default dict
        markerId2 = 45;
        markerImage2 = arucoMarker(markerId2, '4x4_250', markerSizePx); % ID 45, using the new default dict

        imgSize = [480, 640]; % Match camera calibration dimensions
        background = uint8(255 * ones(imgSize(1), imgSize(2), 3)); % White background

        for i = 1:50
            frame = background;

            % Place marker 1
            pos1 = [randi(imgSize(2)-size(markerImage1,2)), randi(imgSize(1)-size(markerImage1,1))];
            frame(pos1(2):pos1(2)+size(markerImage1,1)-1, pos1(1):pos1(1)+size(markerImage1,2)-1, :) = markerImage1;

            % Place marker 2 only after frame 20
            if i > 20
                pos2 = [randi(imgSize(2)-size(markerImage2,2)), randi(imgSize(1)-size(markerImage2,1))];
                frame(pos2(2):pos2(2)+size(markerImage2,1)-1, pos2(1):pos2(1)+size(markerImage2,2)-1, :) = markerImage2;
            end
            writeVideo(v, frame);
        end
        close(v);
        fprintf('Dummy video "%s" created.\n', videoFilePath);

        % Create dummy camera calibration file
        fid_calib = fopen(calibrationFilePath, 'w');
        if fid_calib == -1
            error('Could not create dummy camera calibration file.');
        end
        fprintf(fid_calib, '[image]\n');
        fprintf(fid_calib, 'width\n%d\n', imgSize(2));
        fprintf(fid_calib, 'height\n%d\n', imgSize(1));
        fprintf(fid_calib, '\n[narrow_stereo]\n');
        fprintf(fid_calib, 'camera matrix\n');
        fprintf(fid_calib, '624.753385 0.000000 358.394413\n');
        fprintf(fid_calib, '0.000000 624.047985 259.912709\n');
        fprintf(fid_calib, '0.000000 0.000000 1.000000\n');
        fprintf(fid_calib, 'distortion\n');
        fprintf(fid_calib, '0.021105 -0.295123 0.002860 0.012346 0.000000\n');
        fprintf(fid_calib, 'rectification\n');
        fprintf(fid_calib, '1.000000 0.000000 0.000000\n');
        fprintf(fid_calib, '0.000000 1.000000 0.000000\n');
        fprintf(fid_calib, '0.000000 0.000000 1.000000\n');
        fprintf(fid_calib, 'projection\n');
        fprintf(fid_calib, '606.312439 0.000000 368.167601 0.000000\n');
        fprintf(fid_calib, '0.000000 621.187683 260.762972 0.000000\n');
        fprintf(fid_calib, '0.000000 0.000000 1.000000 0.000000\n');
        fclose(fid_calib);
        fprintf('Dummy camera calibration file "%s" created.\n', calibrationFilePath);

    catch ME_creation
        warning('Failed to create dummy files. Ensure Image Processing Toolbox and Computer Vision Toolbox are installed.\nError: %s', ME_creation.message);
        fprintf('Please provide your own "%s" and "%s" files for the example to run.\n', videoFilePath, calibrationFilePath);
        return; % Exit if creation fails
    end
end


% --- Section 2: Instantiate the class and detect markers ---

% Make sure the paths are correct or dummy files were created.
if ~exist(videoFilePath, 'file')
    fprintf('Video file "%s" not found. Please create it or update the path.\n', videoFilePath);
    return;
end
if ~exist(calibrationFilePath, 'file')
    fprintf('Camera calibration file "%s" not found. Please create it or update the path.\n', calibrationFilePath);
    return;
end

try
    detector = ArUcoDetector(videoFilePath);

    % (Optional) Change the dictionary if needed (defaults are now 4x4_250)
    % detector = detector.setDictionary('6x6_250');

    % (Optional) Set the physical size of your markers in meters.
    % (defaults are now 0.15m)
    % detector = detector.setMarkerSize(0.08); % Example: 8 cm markers

    % Load camera parameters from the calibration file
    detector = detector.loadCameraParameters(calibrationFilePath);

    % Enable live visualization (set to true to see live detection)
    detector = detector.setLiveDisplay(true);

    % Detect markers (this will perform pose estimation if camera parameters are loaded)
    detector = detector.detectMarkers();

    % Get the results
    detectionResults = detector.getResults();

    % Display the results in the command window
    detector.displayResults();

catch ME_example
    fprintf('\nAn error occurred during example execution:\n');
    fprintf('Message: %s\n', ME_example.message);
    fprintf('Please ensure Computer Vision Toolbox (and Image Processing Toolbox for visualization) are installed and all file paths are correct.\n');
    fprintf('Error stack:\n');
    disp(ME_example.stack);
end
%}
