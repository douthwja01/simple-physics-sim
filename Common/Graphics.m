
classdef Graphics
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
            figureProperties = Graphics.Properties();

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
                
            % Generate the mass-marker meshes
            [majorMesh,minorMesh] = MeshGenerator.MassMarker(zeros(3,1),scale);
            
            % Generate (Major) patch            
            h(1) = patch(container,'Vertices',majorMesh.Vertices,'Faces',majorMesh.Faces);
            set(h(1),...
                'FaceColor',[0 0 0],...
                'EdgeAlpha',0,...
                'FaceLighting','phong',...
                'DiffuseStrength',0.2,...
                'FaceAlpha',0.8);
            % Generate (Minor) patch
            h(2) = patch(container,'Vertices',minorMesh.Vertices,'Faces',minorMesh.Faces);
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
            figureProperties = Graphics.Properties();
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
                    "Color",Graphics.colours(n),...
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
            gizmo = Graphics.GizmoProperties();
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
            
            % Ensure valid axes
            if nargin < 4
                container = hggroup("DisplayName","Cylinder");
            end
            % Generate the mesh
            mesh = MeshGenerator.CylinderFromAxis(p0,p,radius);
            % Generate patch    
            h = patch(container,'Vertices',mesh.Vertices,'Faces',mesh.Faces);
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
            
            % Create a sphere
            mesh = MeshGenerator.Sphere(zeros(3,1),radius,faceNumber);

            % Generate patch            
            h = patch(container,'Vertices',mesh.vertices,'Faces',mesh.faces);
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
            [h,container] = Graphics.DrawCuboid(-v,v,container);            % Pass to extent calculator
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
            % Generate a cuboid mesh
            mesh = MeshGenerator.CuboidFromExtents(minExtents,maxExtents);
            % Generate patch  
            h = patch(container,'Vertices',mesh.Vertices,'Faces',mesh.Faces);
        end
        function [h,container] = DrawPlane(center,normal,width,height,container)
            % This function will draw a plane to a given graphical
            % container.

            % Ensure valid axes
            if nargin < 5
                container = hggroup("DisplayName","Plane");
            end
            % Generate a planar mesh
            mesh = MeshGenerator.Plane(center,normal,width,height);
            % Generate patch  
            h = patch(container,'Vertices',mesh.Vertices,'Faces',mesh.Faces);
            set(h,"FaceColor","g");
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