function submitted_cameras = CorrelationUI()
    import uix.*;

    % Main figure
    fig = uifigure('Name', 'Camera Config', 'Position', [100 100 800 600]);

    % Use uigridlayout as a top container for compatibility
    gl = uigridlayout(fig, [1 1]);

    % Root VBox layout
    rootBox = VBox('Parent', gl, 'Padding', 10, 'Spacing', 10);

    %% Top Parameter Section
    paramBox = HBox('Parent', rootBox, 'Spacing', 5);
    % uicontrol('Parent', paramBox, 'Style', 'text', 'String', 'Param1:');
    % param1 = uicontrol('Parent', paramBox, 'Style', 'edit');
    % uicontrol('Parent', paramBox, 'Style', 'text', 'String', 'Param2:');
    % param2 = uicontrol('Parent', paramBox, 'Style', 'edit');
    % set(paramBox, 'Widths', [-1 -2 -1 -2]);  % Relative widths

    %% Scrollable camera section
    scrollBox = uix.ScrollingPanel('Parent', rootBox);
    cameraList = VBox('Parent', scrollBox, 'Spacing', 10, 'Padding', 5);
    scrollBox.Contents = cameraList;

    %% Control Buttons
    buttonBox = HBox('Parent', rootBox, 'Spacing', 10);
    addBtn = uicontrol('Parent', buttonBox, 'Style', 'pushbutton', ...
        'String', 'Add Camera', 'Callback', @add_camera);
    submitBtn = uicontrol('Parent', buttonBox, 'Style', 'pushbutton', ...
        'String', 'Submit', 'Callback', @submit_all);
    set(buttonBox, 'Widths', [-1 -1]);

    %% Resize behavior
    set(rootBox, 'Heights', [40 -1 40]);

    % Store state
    cameraUIs = {};
    submitted_cameras = {}; % Initialize output variable

    %% Callbacks

    function add_camera(~, ~)
        % One camera group
        camPanel = VBox('Parent', cameraList, 'Spacing', 5, 'Padding', 5, ...
            'BackgroundColor', [0.94 0.97 1]);

        % Calibration row
        calRow = HBox('Parent', camPanel, 'Spacing', 5);
        uicontrol('Parent', calRow, 'Style', 'text', 'String', 'Calibration File:');
        calFile = uicontrol('Parent', calRow, 'Style', 'edit', 'Enable', 'off');
        uicontrol('Parent', calRow, 'Style', 'pushbutton', 'String', 'Browse', ...
            'Callback', @(~,~) choose_file(calFile, '*.txt'));
        set(calRow, 'Widths', [-1 -2 -1]);

        % Video row
        vidRow = HBox('Parent', camPanel, 'Spacing', 5);
        uicontrol('Parent', vidRow, 'Style', 'text', 'String', 'Videos:');
        vidList = uicontrol('Parent', vidRow, 'Style', 'edit', ...
            'Enable', 'off', 'Max', 2);  % Multi-line
        uicontrol('Parent', vidRow, 'Style', 'pushbutton', 'String', 'Add Videos', ...
            'Callback', @(~,~) choose_multiple_files(vidList));
        set(vidRow, 'Widths', [-1 -2 -1]);

        cameraUIs{end+1} = struct('calFile', calFile, 'vidList', vidList);

        % Redraw layout
        cameraList.Heights(end+1) = -1;
    end

    function choose_file(field, pattern)
        [file, path] = uigetfile(pattern, 'Select Calibration File');
        if isequal(file, 0), return; end
        field.String = fullfile(path, file);
    end

    function choose_multiple_files(area)
        [files, path] = uigetfile({'*.mp4;*.avi', 'Video Files'}, ...
            'Select Videos', 'MultiSelect', 'on');
        if isequal(files, 0), return; end
        if ischar(files), files = {files}; end
        area.String = strjoin(fullfile(path, files), newline);
    end

    function submit_all(~, ~)
        disp('Submitting configuration...');

        temp_cameras = {};
        for i = 1:numel(cameraUIs)
            cam = cameraUIs{i};
            temp_cameras{i}.calibration = cam.calFile.String;
            temp_cameras{i}.videos = strsplit(cam.vidList.String, newline);
        end

        submitted_cameras = temp_cameras; % Assign to output variable of CorrelationUI

        disp('Cameras:');
        disp(submitted_cameras);

        uialert(fig, 'Configuration submitted. Check console.', 'Done', 'Modal', true);
        % Resume execution for the calling script
        if isvalid(fig)
            uiresume(fig);
        end
    end

    % Wait for uiresume to be called (e.g., from submit_all)
    % This will pause execution of CorrelationUI until uiresume(fig) is called.
    if isvalid(fig)
      uiwait(fig);
    end

    % After uiwait, the figure can be safely deleted if it still exists
    % and the submitted_cameras variable will be returned.
    if isvalid(fig)
        delete(fig);
    end
end
