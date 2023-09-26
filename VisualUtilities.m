
classdef VisualUtilities
  	%% Constant properties
    properties (Constant, Access = public)
        colours = 'rgbymc';
    end
    
    %% Figure Generation & Handling
    methods (Static, Access = public)
        function [figureHandle,axesHandle] = FigureTemplate(name)
            if nargin < 1
                figureHandle = figure('Name',"Figure Template");
            else
                if isnumeric(name)
                    figureHandle = figure(name);
                elseif isstring(name)
                    figureHandle = figure('Name',name);
                else
                    error("Please defined the figure name as a string or number");
                end
            end

            % Import the standard figure properties
            figureProperties = VisualUtilities.Properties();

            % Configure the figure frame
            setappdata(...
                figureHandle,'SubplotDefaultAxesLocation',...
                [0.1, 0.1, 0.80, 0.83]);
            set(figureHandle,'Position',figureProperties.position);           % [x y width height]
            set(figureHandle,'Color',figureProperties.figureColor);           % Background colour
            axesHandle = axes(figureHandle);
            grid on; box on; hold on; grid minor;

            % Axes properties
            set(axesHandle,...
                'TickLabelInterpreter',figureProperties.interpreter,...
                'fontname',figureProperties.fontName,...
                'FontSize',figureProperties.axisFontSize,...
                'FontWeight',figureProperties.fontWeight,...
                'Color',figureProperties.axesColor,...
                'FontSmoothing','On',...
                'GridLineStyle',figureProperties.gridLineStyle,...
                'GridAlpha',figureProperties.gridAlpha,...
                'GridColor',figureProperties.gridColor);
            % Refine the title box
            title(axesHandle,"",...
                'Interpreter',figureProperties.interpreter,...
                'fontname',figureProperties.fontName,...
                'fontsize',figureProperties.titleFontSize,...
                'fontweight',figureProperties.fontWeight,...
                'fontsmoothing','On');
            % X-axes
            xlabel(axesHandle,"",...
                'Interpreter',figureProperties.interpreter,...
                'fontname',figureProperties.fontName,...
                'FontSize',figureProperties.axisFontSize,...
                'Fontweight',figureProperties.fontWeight,...
                'FontSmoothing','On');
            % Y-axes
            ylabel(axesHandle,"",...
                'Interpreter',figureProperties.interpreter,...
                'fontname',figureProperties.fontName,...
                'FontSize',figureProperties.axisFontSize,...
                'Fontweight',figureProperties.fontWeight,...
                'FontSmoothing','On');
            % Z-axes
            zlabel(axesHandle,"",...
                'Interpreter',figureProperties.interpreter,...
                'fontname',figureProperties.fontName,...
                'FontSize',figureProperties.axisFontSize,...
                'Fontweight',figureProperties.fontWeight,...
                'FontSmoothing','On');
        end
    end
    
    %% Object Drawing
    methods (Static,Access = public)
%         % Geom
%         patch(ax,'Vertices',objectID1.GEOMETRY.vertices*R_final + finalPosition',...
%             'Faces',objectID1.GEOMETRY.faces,...
%             'FaceColor',SIM.OBJECTS(ID1).colour,...
%             'EdgeColor',DATA.figureProperties.edgeColor,...
%             'EdgeAlpha',DATA.figureProperties.edgeAlpha,...  
%             'FaceLighting',DATA.figureProperties.faceLighting,...
%             'FaceAlpha',DATA.figureProperties.faceAlpha,...
%             'LineWidth',DATA.figureProperties.patchLineWidth);             % Patch properties            % Patch properties
        
        % Gizmos
        function [h,container] = DrawMassMarker(scale,container)
            % This function will plot a 3D "center of mass" marker within
            % the provided axes 'ax' whose position and rotation is
            % defined by homogenous transform 'T'. The vertices are
            % similarly scaled by the scalar or vector parameter 'scale'.
            
            % Credit given to the original author: 
            % https://uk.mathworks.com/matlabcentral/fileexchange/34851-plot-a-center-of-mass
            
            % Sanity check
            if nargin < 2
                container = hggroup("DisplayName","Mass Marker");
            end
            if nargin < 1
                scale = 1;
            end
            k = "smooth";
%             k = "coarse";
            switch k
                case 'coarse'
                    a=.44433292853227944784221559874174;
                    % Spherical vertices
                    v = [ 0 0         1;
                        sin(pi/8) 0 cos(pi/8);0         sin(pi/8) cos(pi/8);
                        sin(pi/4) 0 sin(pi/4);a a sqrt(1-2*a^2);0 sin(pi/4) sin(pi/4);
                        cos(pi/8) 0 sin(pi/8);sqrt(1-2*a^2) a a;a sqrt(1-2*a^2) a;0 cos(pi/8) sin(pi/8);
                        1 0 0;cos(pi/8) sin(pi/8) 0;sin(pi/4) sin(pi/4) 0;sin(pi/8) cos(pi/8) 0;0 1 0]./50;
                    % Generate face connectivity matrix                  
                    f  = [1 2 3;2 4 5;2 5 3;
                        3 5 6;4 7 8;4 8 5;
                        5 8 9;5 9 6;6 9 10;
                        7 11 12;7 12 8;8 12 13;
                        8 13 9;9 13 14;9 14 10;
                        10 14 15];
                    f1 = [f;f+15;f+30;f+45];
                case 'smooth'
                    % smoother com: what is the correct value for b?
                    % b=5275338780780258*2^(-54);
                    b  = 5271341053858645*2^(-54);
                    c  = 5020924469873901*2^(-53);
                    e  = 5590528873889244*2^(-54);
                    a  = [sin(pi/12) cos(pi/12)  sin(pi/6) cos(pi/6) sqrt(.5) sqrt(1-2*b^2) sqrt(1-c^2-e^2)];
                    % Spherical vertices
                    v  = [0 0 1;a(1) 0 a(2);0 a(1) a(2);a(3) 0 a(4);b b a(6);0 a(3) a(4);
                        a(5) 0 a(5);c e a(7);e c a(7);0 a(5) a(5);a(4) 0 a(3) ;a(7) e c;[1 1 1]/sqrt(3);
                        e a(7) c;0 a(4) a(3);a(2) 0 a(1) ;a(6) b b;a(7) c e;c a(7) e;b a(6) b;0 a(2) a(1);
                        1 0 0;a(2) a(1) 0;a(4) a(3) 0;a(5) a(5) 0;a(3) a(4) 0;a(1) a(2) 0;0 1 0];
                    % Generate face connectivity matrix
                    f  = [1 2 3;2 4 5;2 5 3;3 5 6;4 7 8;4 8 5;5 8 9;5 9 6;6 9 10;7 11 12;7 12 8;8 12 13;8 13 9;9 13 14;
                        9 14 10;10 14 15;11 16 17;11 17 12;12 17 18;12 18 13;13 18 19;13 19 14;14 19 20;14 20 15;15 20 21;
                        16 22 23;16 23 17;17 23 24;17 24 18;18 24 25;18 25 19;19 25 26;19 26 20;20 26 27;20 27 21;21 27 28];
                    f1 = [f;f+28;f+56;f+84];
            end
            % Align vertex permutations
            v0 = [v;[-v(:,1) -v(:,2) v(:,3)];[ v(:,1) -v(:,2) -v(:,3)];[-v(:,1) v(:,2) -v(:,3)]];
            % Transform vertex pair
            Ts = Transform.Scale(scale);
            padding = ones(size(v0,1),1);
            v_major = [ v0,padding]*Ts;
            v_minor = [-v0,padding]*Ts;
            
            % Generate patch            
            h(1) = patch(container,'Vertices',v_major(:,1:3),'Faces',f1);
            set(h(1),...
                'FaceColor',[0 0 0],...
                'EdgeAlpha',0,...
                'FaceLighting','phong',...
                'DiffuseStrength',0.2,...
                'FaceAlpha',0.8);
            h(2) = patch(container,'Vertices',v_minor(:,1:3),'Faces',f1);
            set(h(2),...
                'FaceColor',[1 1 0],...
                'EdgeAlpha',0,...
                'FaceLighting','phong',...
                'DiffuseStrength',0.2,...
                'FaceAlpha',0.8);  
        end      
        function [h,container] = DrawTriad(scale,container)
            % Draw a vector triad with the provided transform and scale to
            % the current axes.
            
            % Sanity check
            assert(numel(scale) == 1,"Expecting a numeric scaling variable.");
            
            % Create a triad group
            h = hggroup("DisplayName","Triad","Tag","Triad");
            
            % Assign parent if one is provided
            if nargin > 1
                set(h,"Parent",container);
            end

            % Standardised properties
            figureProperties = VisualUtilities.Properties();
            % Transform the triad vectors
            frameOrigin     = zeros(3,1);
            triadVectors    = eye(3)*scale;
            % Plot the frame origin in the current axes
            plot3(h, ...
                frameOrigin(1),frameOrigin(2),frameOrigin(3), ...
                "ko", ...
                "Tag","Origin");
            % Axis vectors
            for n = 1:3                                          
                temp = [frameOrigin';triadVectors(n,:)];
                % Draw axis unit vectors
                plot3(h,temp(:,1),temp(:,2),temp(:,3),...
                    "Color",VisualUtilities.colours(n),...
                    "LineWidth",figureProperties.lineWidth,...
                    "LineStyle",figureProperties.lineStyle);
            end
        end
        function [h,container] = DrawAxis(axis,container)
            % Sanity check
            assert(IsColumn(axis,3),"Expecting a reference vector axis.");
            if nargin < 2
                container = hggroup("DisplayName","Axis");
            end

            % Gizmo properties
            gizmo = VisualUtilities.GizmoProperties();
            % Create graphics container
            axisNorm = norm(axis);
            axisUnit = axis/axisNorm;
            pA =  gizmo.scale*(axisNorm/2)*axisUnit;
            pB = -gizmo.scale*(axisNorm/2)*axisUnit;
            axisPoints = [pA,pB]';
            % Plot the frame origin in the current axes
            h = plot3(container,axisPoints(:,1),axisPoints(:,2),axisPoints(:,3),...
                "Color","k",...
                "LineWidth",gizmo.lineWidth,...
                "LineStyle","-.");
        end
        % Graphical primitives
        function [hArrow,container] = DrawArrow(point0,point1,container,varargin)
            
            % Sanity check
            assert(IsColumn(point0),"Expecting a valid Cartesian point [3x1]");
            assert(IsColumn(point1),"Expecting a valid Cartesian point [3x1]");
            
            % Ensure valid axes
            if nargin < 3
                container = hggroup("DisplayName","Cylinder");
            end

            % Default inputs
            params = struct( ...
                "bodyOrInertial","body", ...
                "name","velocity", ...
                "container",container, ...
                "colour","r", ...
                "stemWidth",0.02,...
                "tipWidth",0.03,...
                "faceAlpha",0.5,...
                "scale",1);
            
            % Check for user overrides
            params = RecursiveParameterOverrides(params,varargin);
            
            % Draw line from parent origin to this origin
            hArrow = mArrow3(point0,point1,...
                'facealpha',params.faceAlpha,...
                'color',params.colour,...
                'stemWidth',params.stemWidth,...
                'tipWidth',params.tipWidth);
        end
        function [h,container] = DrawCylinder(p0,p,radius,container)
            % This function assumes the origin of the cylinder is half way
            % along the cylinder's length.
            
            % Sanity check
            assert(IsColumn(p0,3) && IsColumn(p,3),"Expecting two Cartesian points defining the axis [3x1]");
            assert(numel(radius) == 1,"Expecting a scalar radius.");
            
            % Ensure valid axes
            if nargin < 4
                container = hggroup("DisplayName","Cylinder");
            end
            % Assemble the cylinder geometry
            axis = p - p0;
            unitAxis = axis/norm(axis);
            perpVector = cross(unitAxis,[1;0;0]);
            if sum(perpVector) == 0
                perpVector = cross(unitAxis,[0;1;0]);
            end
            perpVector = radius*perpVector;               
            points = 10;
            nodalAngle = pi/2;
            increment = 2*pi/points;
            coordinates = zeros(2*points,3);
            for i = 1:points
                % Calculate the vector as a result of the perpendicular
                % rotated through "nodal angle"
                radialVector = RodriguesRotation(perpVector,unitAxis,nodalAngle);
                coordinates(i,:) = (p0 + radialVector)';       % Ring defining base
                coordinates(i+10,:) = (p + radialVector)';     % Ring defining top
                % Iterate
                nodalAngle = nodalAngle - increment;
            end
            % Triangulate rotated volume
            [faces,~] = convhull(coordinates(:,1),coordinates(:,2),coordinates(:,3),"simplify",true);
            % Generate patch    
            h = patch(container,'Vertices',coordinates,'Faces',faces);
        end
        function [h,container] = DrawSphere(radius,container,faceNumber)                             % Draw sphere
            % This function returns a sphere coordinate cloud at a given
            % position in space, of a given radius. 'figureHandle' is used
            % to provide the function context.
            
            % Ensure valid axes            
            if nargin < 3
                faceNumber = 10;
            end
            if nargin < 2
                container = hggroup("DisplayName","Sphere");
            end

            % Triangulate a sphere
            [X,Y,Z] = sphere(faceNumber);
            X = X.*radius;
            Y = Y.*radius;
            Z = Z.*radius;
            % Generate patch object
            [faces,vertices,~] = surf2patch(X,Y,Z,'triangles');
            % Generate patch            
            h = patch(container,'Vertices',vertices,'Faces',faces);
        end
        function [h,container] = DrawCuboidFromRadius(radius,container)                   % Draw cuboid (from an enclosing sphere)
            % This function creates a set of vertices for a cube
            % encapsuated in a sphere of a given radius.
            assert(numel(radius) == 1,'The radius of the sphere must be a scalar and non-zero.');
            
            % Ensure valid axes
            if nargin < 2
                container = hggroup("DisplayName","Cuboid");
            end

            % Calculate extents from radius
            v = ones(3,1)*radius/1.7321;                                 	% Rate of dimensional expansion
            [h,container] = VisualUtilities.DrawCuboid(-v,v,container);                   	% Pass to extent calculator
        end
        function [h,container] = DrawCuboid(minExtents,maxExtents,container)              % Draw cuboid (from min-max in each dimension)
            % Return a matrix of point defining a cuboid scaled to that of
            % a dimensions provided.
            
            % Sanity check
            assert(IsColumn(minExtents,3) && IsColumn(maxExtents,3),...
                "Expecting two vectors defining min:max in each dimension");
            % Ensure valid axes
            if nargin < 3
                container = hggroup("DisplayName","Cuboid");
            end

            % Define vertex data from limits
            vertices = zeros(8,3);
            vertices(1,:) = [maxExtents(1),maxExtents(2),maxExtents(3)];
            vertices(2,:) = [maxExtents(1),maxExtents(2),minExtents(3)];
            vertices(3,:) = [maxExtents(1),minExtents(2),minExtents(3)];
            vertices(4,:) = [maxExtents(1),minExtents(2),maxExtents(3)];
            vertices(5,:) = [minExtents(1),minExtents(2),minExtents(3)];
            vertices(6,:) = [minExtents(1),minExtents(2),maxExtents(3)];
            vertices(7,:) = [minExtents(1),maxExtents(2),maxExtents(3)];
            vertices(8,:) = [minExtents(1),maxExtents(2),minExtents(3)];
            % Define face connectivity matrix
            faces =  [ 1,2,7; 1,4,2; 1,7,4; 2,3,8; 2,4,3; 2,8,7; 3,4,6;
                       3,5,8; 3,6,5; 4,7,6; 5,6,8; 6,7,8];              
            % Generate patch  
            h = patch(container,'Vertices',vertices,'Faces',faces);
        end
    end
    
    %% Property Standardisation
    methods (Static, Access = public)
        % Figure properties container 
        function [figureProperties] = Properties()                          % Global figure properties 
            figureProperties = struct();
            % Define window size in pixels
            set(0,'units','pixels')
            figureProperties.screensize = get(0,'ScreenSize');
            figureProperties.position = [...
                10; 50;
                0.6*figureProperties.screensize(3);
                0.8*figureProperties.screensize(4)];                        % [x y width height] 
            % General settings
            figureProperties.figureColor = 'w';
            figureProperties.axesColor = 'w';                               % Universal background colour grey: [0.9 0.9 0.9]
            figureProperties.gridLineStyle = "--";
            figureProperties.gridAlpha = 0.25;
            figureProperties.gridColor = "k";

            % Line settings
            figureProperties.lineWidth = 2;                                 % Applies to both the marker and trails
            figureProperties.lineStyle = ':';
            figureProperties.markerSize = 10;
            figureProperties.markerEdgeColor = 'k';

            % Text settings
            figureProperties.titleFontSize = 18;%22;
            figureProperties.axisFontSize = 16;%18;
            figureProperties.fontWeight = 'bold';
            figureProperties.fontName = 'Helvetica';
            figureProperties.interpreter = 'latex';

            % Component patch settings
            figureProperties.edgeColor = 'k';
            figureProperties.edgeAlpha = 0.5;
            figureProperties.patchLineWidth = 1;                            % Patch properties
            figureProperties.faceColor = "b";
            figureProperties.faceLighting = 'gouraud';
            figureProperties.faceAlpha = 0.25;
        end
        function [gizmo] = GizmoProperties()                                % Global gizmo settings
            gizmo = struct();       
            gizmo.edgeColor = 'k';
            gizmo.edgeAlpha = 0.5;
            gizmo.patchLineWidth = 1;                            
            gizmo.faceColor = "b";
            gizmo.faceLighting = 'gouraud';
            gizmo.faceAlpha = 0.8;
            gizmo.diffuseStrength = 1;  
            gizmo.scale = 0.2;         
            gizmo.lineWidth = 1.2;
        end
    end
end