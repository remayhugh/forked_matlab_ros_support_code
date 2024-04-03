function outStruct = merge_img_structures
% Assumes .mat image structures in ./ros_matlab/code/_vision/data
% Will output a yoloTrainingData_yyyymmdd_hhmmss.mat into a merge folder

    files = dir('*.mat');
    
    % File length
    file_len = length(files);

    % Initialize a cell array to hold the loaded data structures
    loadedData = cell(file_len, 1);
    
    for i = 1:file_len
        % Construct the full path to the file
        filePath = fullfile(files(i).folder, files(i).name);
        
        % Load structures inside cell. Still need to refer to them by internal field name: myImgStruct to access data
        loadedData{i} = load(filePath);
    end    

    % Initialize the new structure
    outStruct = struct();
        
    ctr = 1;
    field_names = cell(1,file_len);
    
    for i = 1:file_len
        str = loadedData{i};
        field_names{i} = fieldnames(str.myImgStruct); % Hold cell array of field names

        % Use field names to set outStruct to the equivalent images
        for j = 1:length(field_names{i})
            field = append('img',num2str(ctr));

            % Copy the image over
            entry = field_names{i}{j};
            outStruct.(field) = str.myImgStruct.(entry);

            % Increase counter
            ctr = ctr + 1;
        end
    end

    %% Save struct to file
    
    % Add timestamp
    formattedDateTimeStr = datetime('now', 'Format', 'yyyyMMdd_HHmmss');
    outputFileName = append('yoloTrainingData',"_", char(formattedDateTimeStr),".mat"); 

    % Save in merge folder
    fullPath = fullfile('merge', outputFileName); % Creates a full file path   
    if ~exist('merge', 'dir')                     % If the folder does not exist, create it 
        mkdir('merge');
    end

    % Final name:   
    save(fullPath,'outStruct');
    
    fprintf('File saved as %s\n', 'yoloTrainingData.mat');
end
