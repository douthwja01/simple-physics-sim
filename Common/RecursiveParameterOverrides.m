%% Recursive parameter search %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [config] = RecursiveParameterOverrides(defaultConfig,inputParameters)
% This function is designed to recursively search through a configuration
% structure/class for properties provided by a vector of parameter:value
% pairs.

% INPUTS:
% defaultConfig   - (cell/struct) defining original configuration.
% inputParameters - (cell array) of value pairs. 
% OUTPUTS:
% config    - The parameterised structure with 

% Default output state
config = defaultConfig;

% Sanity check #1 - No inputs to apply
if nargin < 2 || isempty(inputParameters)
    return
end

% Sanity check #2 - Check nested cell arrays
assert(iscell(inputParameters),'Entries must be provided as a cell array of parameter:value pairs');

% Sanity check #3 - Unloaded successfully
t = 1;
while numel(inputParameters) == 1 && t < 10
    inputParameters = inputParameters{:};
    t = t + 1;
end
assert(t < 10,'Failed to unload parameters.');

% Sanity check #4 - No inputs
if numel(inputParameters) == 0
    return
end
    
% Sanity check #5 - Valid value/pair list
assert(mod(numel(inputParameters),2) == 0,'Un-even number of parameter:value pairs');

% ////////// BEGIN PARSING THE VECTOR AGAINST THE DATA STRUCTURE //////////
[config] = ScanDataStructure(config,inputParameters);
end

%% Operations on structures 
function [config] = ScanDataStructure(config,inputParameters)
% Move through the properties of "config"
 
% Based one config type, get property names
switch true
    case isstruct(config)
        fieldLabels = fieldnames(config); 
    case isobject(config)
        fieldLabels = properties(config); 
    otherwise
        error('Configuration structure not recognised');
end

% Parse the top level structure
config = parseCellArray(config,inputParameters);

% Scan lower tier structures
for i = numel(fieldLabels)
    % Check if it's a structure or class
    if ~isstruct(config.(fieldLabels{i}))
        continue
    end
    % Conflict between "isobject" and type "sym"
    if isobject(config.(fieldLabels{i})) && ~isa(config.(fieldLabels{i}),'sym')
        continue
    end
    % Recursion
    config.(fieldLabels{i}) = ScanDataStructure(config.(fieldLabels{i}),inputParameters);
end
end
% Parse the cell array against the data structure
function [config] = parseCellArray(config,inputParameters)
% This function checks the field against the parameter vector

for parameterIndex = 1:2:numel(inputParameters)
    % For each user-defined parameter
    givenParameter = inputParameters{parameterIndex};
    
    if isstring(givenParameter)
        givenParameter = char(givenParameter);
    end
    
    if ~ischar(givenParameter)
        continue
    end
    
    % If the data object has the properties
    if isstruct(config) && isfield(config,givenParameter)
        config.(givenParameter) = inputParameters{parameterIndex + 1};  % Make a substitution
        continue
    end
    
    if ~isstruct(config) && isprop(config,givenParameter)
        config.(givenParameter) = inputParameters{parameterIndex + 1};   % Make a substitution
        continue
    end
    warning("Parameter assignment skipped.\n -Parameter '%s' not found on object '%s'.",givenParameter,class(config));
end
end
