classdef (Abstract) SimElement < matlab.mixin.Heterogeneous & handle 
    % This is the basis of all component of the 'Simulator' class.

    properties (Abstract,Constant)
        Name;           % The name of the component/approach.
    end
end