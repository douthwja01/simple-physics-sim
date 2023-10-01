
function [tf] = IsClass(candidates,className)
    % This function provides a means to test a single or array of
    % hetrogenous classes and return a matrix where that element is of the
    % target class. 

    % Sanity check
    assert(isstring(className),"Expecting a class name string (i.e 'matlab.ui.Figure').");
    % Execute the check
    tf = arrayfun(@(x)isa(x, className), candidates);
end