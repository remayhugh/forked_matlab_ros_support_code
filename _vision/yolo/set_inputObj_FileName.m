function outputFileName = set_inputObj_FileName(inputString)

    % Mapping from input strings to output file names
    mapping = containers.Map(...
        {'rCan', 'gCan', 'yCan', 'rBottle', 'bBottle', 'yBottle', 'pouch'}, ...
        {'red_can', 'green_can', 'yellow_can', 'red_bottle', 'blue_bottle', 'yellow_bottle', 'pouch'});

    % Check if the inputString is in the map
    if isKey(mapping, inputString)
        
        % Get the output file name from the map
        base_name = mapping(inputString);

        % Add timestamp
        formattedDateTimeStr = datetime('now', 'Format', 'yyyyMMdd_HHmmss');
        % Final name:

        outputFileName = append(base_name,"_", char(formattedDateTimeStr),".mat"); 

        % Confirmation message
        fprintf('File name will be %s\n', outputFileName);
    
    else
        % Handle the case where the inputString is not recognized
        error('Input string "%s" is not recognized.', inputString);
    end
end
