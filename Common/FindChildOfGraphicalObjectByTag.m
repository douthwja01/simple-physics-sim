
function [child] = FindChildOfGraphicalObjectByTag(gobj,tag)
% This function finds any (immediate) child with the provided tag string.

% Sanity check
assert(isa(gobj,"matlab.graphics.Graphics"),"Expecting a hierarchical graphical object.");
assert(isstring(tag),"Expecting a valid tag string.");

% Get the set of child objects
children = get(gobj,"Children");
% Select the corresponding children
child = children(strcmp(get(children,"Tag"),tag));
end