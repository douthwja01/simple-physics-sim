
function [tf] = IsClass(className,candidates)
    % This function provides a means to test a single or array of
    % hetrogenous classes and return a matrix where that element is of the
    % target class. 

    % Sanity check
    assert(isa(className,"string"),"Expecting a class name string ('matlab.ui.Figure').");
    % Execute the check
    tf = arrayfun(@(x)isa(x, className), candidates);
end