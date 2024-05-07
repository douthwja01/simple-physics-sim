classdef (Abstract) TreeElement < Element
    % TREEELEMENT - This class adds the tree behaviour elements to the base 
    % modelling element definition.
   
    properties %(SetAccess = private)
        Parent;
        Children;
    end
    properties (Hidden)
        NumberOfParents;
        NumberOfChildren;
    end
    properties (Constant,Access = private)
        OriginVertexName = "Origin Line";
    end
    % Main
    methods
        % Constructor
        function [this] = TreeElement(entity)
            % This is the base constructor for all tree-element modelling
            % primitives. It handles tree-traversal and element finding
            % utilities.

            if nargin < 1
                entity = Entity.empty;
            end

            % Call the parent
            this = this@Element(entity);
        end
        % Get/sets
        function set.Parent(this,parent)
            % Sanity check, same type
            assert(isa(parent,"TreeElement"),"Expecting valid child element");
            % Collate changes
            prv = this.Parent;
            this.Parent = parent;
            % Remove ourselves as child from previous parent
            if ~isempty(prv)
                if prv.HasImmediateChild(this)
                    prv.RemoveChild(this);
                end
            end
            % Add ourselves as a child
            if ~isempty(parent) 
                if ~parent.HasImmediateChild(this)
                    parent.AddChild(this);
                end
            end
        end
        function set.Children(this,children)
            % Sanity check, same type
            assert(isa(children,"TreeElement"),"Expecting valid child element");
            % Collate changes
            prv = this.Children;
            this.Children = children;
            % Disconnect all previous children (not ideal)
            if ~isempty(prv)
                for n = 1:numel(prv)
                    if prv(n).HasImmediateChild(this)
                        prv(n).RemoveChild(this);
                    end
                end
            end
            % Connect all new children if needed
            if ~isempty(children)
                for n = 1:numel(children)
                    if ~children(n).HasImmediateParent(this)
                        children(n).AddParent(this);
                    end
                end
            end
        end
        function [n] = get.NumberOfParents(this)
            n = numel(this.Parent);
        end
        function [n] = get.NumberOfChildren(this)
            n = numel(this.Children);
        end
    end

    %% Visualisation
    methods
        function [h] = Plot(this)                        
            % This function is used to plot a coordinate frame
            
            % Plot the posed element
            [this.Handle] = Plot@PosedElement(this);

            % Sanity check
            if isa(this.Parent.Handle,"matlab.graphics.GraphicsPlaceholder")
                error("Container for %s is a placeholder.",this.Name);
            end

            % Gizmo properties
            gizmoParams = VisualUtilities.GizmoProperties();
            % Assign the parent container to the posed element
            set(this.Handle,"Parent",this.Parent.Handle);

            % If the parent container is a transform
            if isa(this.Handle,'matlab.graphics.primitive.Transform')
                % Draw line from parent origin to this origin 
                % - (in parent frame coordinates)
                p = transpose(this.Transform.transform)*-[this.Transform.position;0];
                % Concatinate
                points = [p(1:3)';zeros(1,3)];
                % Line plot
                lh = plot3(this.Handle,points(:,1),points(:,2),points(:,3),"k");
                set(lh, ...
                    "LineStyle","--", ...
                    "LineWidth",gizmoParams.lineWidth,...
                    "Tag",this.OriginVertexName);
            end
            % Return handles (for clarity)
            h = this.Handle;
        end
    end
    methods (Hidden)
        function [this] = OnDraw(this)
            % This function updates the graphical handles assocated with
            % this tree element to it's current pose.

            % Update the posed-element draw
            [this] = OnDraw@PosedElement(this);

            % If no transform to modify
            if ~isa(this.Handle,'matlab.graphics.primitive.Transform') 
                return
            end
            % Retrieve the 'OriginLine' to update
            [child] = FindChildOfGraphicalObjectByTag(this.Handle,this.OriginVertexName);

            % The position of the parent in the frame of this element.
            p_parent_in_i = -this.Transform.rotation'*this.Transform.position;

            % Draw line from parent origin to this origin
            points = [p_parent_in_i';zeros(1,3)];
            % Set the line data
            set(child,"XData",points(:,1),"YData",points(:,2),"ZData",points(:,3));
        end
    end
  
    %% Heirarchical (framework) utilities
    methods (Access = public)
        % General helpers
        function [elements] = ListChildrenOfType(this,typeString)          
            % List all tree elements beneath the current tree element.

            % Do a recursive list
            [elements] = this.ChildWiseRecursiveList(this);                 % This function returns all transforms in the transform hierarchy.
            % Check if any selection is required
            if nargin < 2 || isempty(typeString)
                return;                                                     % Return the array
            end
            % Parse the array based on type
            assert(isstring(typeString),"Expecting a string defining 'Element/Frame' etc.");
            logicalSelect = false(size(elements));
            for i = 1:numel(elements)
                if isa(elements(i),typeString)
                    logicalSelect(i) = true;
                end
            end
            elements = elements(logicalSelect);                             % Return the array the type
        end
        function [element,sequence,isParent] = Find(this,eParam)            % Global search for a target frame, from the reference frame
            % This function will search the complete "Component"
            % hierarchy in order to find the Element with the given
            % parameter. The sequence of Entities between the "reference"
            % and "target" is returned and is empty if no path exists.

            % INPUT:
            % eParam : (frame,uid,label)

            % OUTPUT:
            % element   - The "found" target frame.
            % sequence - Element sequence (sequence(end) is always the target).
            % isParent - Indicator if the target is a parent/child of this.

            % Parent-wise search usually (faster)
            [element,sequence] = this.ParentWiseRecursiveSearch(this,eParam);
            if isempty(element)
                [element,sequence] = this.ChildWiseRecursiveSearch(this,eParam);
                isParent = false;
            else
                isParent = true;
            end
        end
        function [element]  = GetRoot(this)
            % Get the maximal parent of this element
            tf = this;
            while tf.NumberOfParents ~= 0
                tf = tf.Parent;
            end
            element = tf;
        end
        % Parentage modification
        function [this]     = AddChild(this,child)
            % Add a child element to this element

            % Sanity check, same type
            assert(isa(child,"TreeElement"),"Expecting valid child element");
            assert(isscalar(child),"Expecting a singular child element.");
            
            % Check: Is same element
            if this.IsSame(child)
                warning("Element '%s' cannot be its own child",child.label);
                return
            end
            % Check: Child is already present
            if this.HasImmediateChild(child)
                warning("Element '%s' is already a child of element '%s'",child.label,this.label);
                return                                                      % If the child already exists, skip assignment
            end
            % Reset the children as the concatinated array
            this.Children = [this.Children,child];

            % Check: Once child is set, reciprocate with parent assignment
            if ~child.HasImmediateParent(this)
                child.AddParent(this);
            end
        end
        function [this]     = AddParent(this,parent)
            % Add a parent to this element

            % Check: Is an element
            assert(isa(parent,"Element"),sprintf("Expecting child of type '%s'",class(this)));
            % Check: Is same element
            if this.IsSame(parent)
                warning("Element '%s' cannot be its own parent",parent.Name);
                return
            end
            % Check: Has parent
            if this.HasImmediateParent(parent)
                warning("Element '%s' is already a parent of element '%s'",parent.Name,this.Name);
                return                                                      % If the parent already exists, skip assignment
            end
            % Reset the parents as the concatinated array
            this.Parent = parent; %[this.Parent,parent];
            % Check: Once parent is set, reciprocate with child assignment
            if ~parent.HasImmediateChild(this)
                parent.AddChild(this);
            end
        end
        function [this]     = RemoveChild(this,cParam)
            % Attempt to find immediate child
            [flag,ind] = this.HasImmediateChild(cParam);
            if ~flag
                warning("Provided element is not a child of '%s'",this.label);
                return;
            end
            % Logically select/remove child
            logicalSelection = true(1,this.NumberOfChildren);
            logicalSelection(ind) = false;
            this.Children = this.Children(logicalSelection);                % Remove the selected child
            % Check for bad child-parent reference
            if cParam.HasImmediateParent(this)
                cParam.RemoveParent(this);                                	% Remove child-parent reference
            end
        end
        function [this]     = RemoveParent(this,pParam)                     % Remove parent frame
            % Remove the parent from the tree element identified by a given
            % parameter.

            if isempty(pParam)
                this.Parent = TreeElement.empty;
                return;
            end
            
            % Attempt to find immediate child
            [flag,ind]  = this.HasImmediateParent(pParam);
            if ~flag
                warning("Provided element is not a parent of '%s'",this.label);
                return;
            end
            % Logically select/remove parent
            logicalSelection = true(1,this.NumberOfParents);
            logicalSelection(ind) = false;
            this.Parent = this.Parent(logicalSelection);                    % Remove the selected parent
            % Check for bad parent-child reference
            if pParam.HasImmediateChild(this)
                pParam.RemoveChild(this);
            end
        end
        function [element] 	= SelectChild(this,cParam)
            % Select child by attribute
            element = [];
            if isnumeric(cParam)                                            % If simple indices
                assert(cParam <= this.NumberOfChildren,"Invalid numeric child selection (out of range).");
                element = this.Children(cParam);
                return
            end
            [flag,ind] = this.HasImmediateChild(id);                        % Do parse of immediate transform
            if flag
                element = this.Children(ind);                               % If found, return component
                return
            end
            warning("No child found using reference %s",string(cParam));
        end
        function [element] 	= SelectParent(this,pParam)                          	
            % Select parent by a given parent parameter

            element = [];
            if isnumeric(pParam)                                            % If simple indices
                assert(pParam <= this.NumberOfParents,"Invalid numeric parent selection (out of range).");
                element = this.Parent(pParam);
                return
            end
            [flag,ind] = this.HasImmediateParent(id);                      	% Do parse of immediate transform
            if flag
                element = this.Parent(ind);                                 % If found, return component
                return
            end
            warning("No parent found using reference %s",string(pParam));
        end
        % (Non-Recursive) Operations
        function [flag,ind] = HasImmediateParent(this,eParam)               % Simple one level parent check
            flag = false;
            ind  = [];
            for i = 1:this.NumberOfParents
                if this.Parent(i).IsSame(eParam)
                    ind = i;
                    flag = true;
                    return
                end
            end
        end
        function [flag,ind]	= HasImmediateChild(this,eParam)                % Simple one level child check
            flag = false;
            ind  = [];
            for i = 1:this.NumberOfChildren
                if this.Children(i).IsSame(eParam)
                    ind = i;
                    flag = true;
                    return
                end
            end
        end
        % Recursive Operations
        function [flag,sequence] = HasParent(this,eParam)                   % Check if frame is parent of this transform
            [~,sequence] = this.ParentWiseRecursiveSearch(this,eParam);
            flag = numel(sequence) > 1;
        end
        function [flag,sequence] = HasChild(this,eParam)                    % Check if frame is child of this transform
            [~,sequence] = this.ChildWiseRecursiveSearch(this,eParam);
            flag = numel(sequence) > 1;
        end
    end

    %% Tree traversal utilities
    methods (Static, Access = protected)
        function [sequence] = ChildWiseRecursiveList(sequence)
            % This function recursively iterates through a hierarchical
            % frame structure in a child-wise manner and appends the
            % objects to a list for later reference.
            
            % Sanity check
            assert(numel(sequence) > 0,"Expecting at least one initial reference element.");
            
            % No more children
            currentElement = sequence(end);
            if currentElement.NumberOfChildren == 0                         % End of child branch
                return
            end
           
            % Handle multiple child-branches
            for i = 1:currentElement.NumberOfChildren                     
                % Create new branch with child
                subSeq = currentElement.ChildWiseRecursiveList(currentElement.Children(i));                                 
                % Concatinate children branch
                sequence = vertcat(sequence,subSeq);                                
            end
        end
        function [sequence] = ParentWiseRecursiveList(sequence)
            % No more parents
            currentElement = sequence(end);
            if currentElement.NumberOfParents == 0                          % End of parent branch
                return
            end
           
            % Handle multiple child-branches
            for i = 1:currentElement.NumberOfParents                              
                subSeq = currentElement.ParentWiseRecursiveList(...         % Create new branch with parent
                    currentElement.Parent(i));                                 
                sequence = vertcat(sequence,subSeq);                        % Concatinate parent branch
            end
        end
        function [element,sequence] = ChildWiseRecursiveSearch(sequence,childParameter)
            % This function recursively moves down an "Element" hierarchy to
            % find the given "target" Element. The transversal returns the
            % element sequence as an array, or is empty if the target is not
            % found.
            
            % INPUT:
            % tParam : (frame,uid,label) (IsSame Comparison)
            
            element = Element.empty();

            % No transform or this is desired transform
            if sequence(end).IsSame(childParameter)
                element = sequence(end);
                return
            end
            % "id" not found before hitting root
            if sequence(end).NumberOfChildren == 0
                sequence = [];                                              % Not found (kill the path)
                return
            end
            % Handle multiple child-branches
            for i = 1:sequence(end).NumberOfChildren
                tc_seq = sequence(end).Children(i);                         % Create new branch with child
                [element,tc_seq] = sequence(end).ChildWiseRecursiveSearch(tc_seq,childParameter);	% Search child-branch      
                % Check if a child-wise path exists
                if ~isempty(tc_seq)                                         % If a path exists from this parent
                    sequence = vertcat(sequence,tc_seq);                    % Add the child-path to the end of the sequence
                    return 
                end
            end
            sequence = [];                                                  % Of children found, no valid paths exist; kill path
        end
        function [element,sequence] = ParentWiseRecursiveSearch(sequence,parentParameter)
            % This function recursively moves up an "Element" hierarchy to a 
            % given target "Element". The transversal returns the
            % transform sequence as a result.
            % INPUT:
            % tParam : (frame,uid,label)

            element = Element.empty();

            % No transform or this is desired transform
            if sequence(end).IsSame(parentParameter)
                element = sequence(end);
                return
            end
            % "id" not found before hitting root
            if sequence(end).NumberOfParents == 0
                sequence = [];                                              % Not found (kill the path)
                return
            end
            % Handle multiple parent-branches
            for i = 1:sequence(end).NumberOfParents
                tp_seq = sequence(end).Parent(i);                           % Create new branch with parent
                [element,tp_seq] = sequence(end).ParentWiseRecursiveSearch(tp_seq,parentParameter);	% Search child-branch      
                % Check if a parent-wise path exists
                if ~isempty(tp_seq)                                         % If a path exists from this parent
                    sequence = vertcat(sequence,tp_seq);                    % Add the parent-path to the end of the sequence
                    return 
                end
            end
            sequence = [];                                                  % Of children found, no valid paths exist; kill path
        end
    end
end