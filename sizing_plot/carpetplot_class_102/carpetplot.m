classdef carpetplot < handle
    
    % CARPETPLOT is a class for creating carpet plots and cheater plots.
    %   Carpet plots are a way to illustrate three (Fig. 2) or four (Fig. 1)
    %   variables in an easy to read two dimensional plot. The carpet plot
    %   class offers the possiblity to handle different input data, add labels,
    %   show interpolated points inside the plot, and many more features.
    %
    %  |           _____________________ 4    |     .  .  .__.__.__.__.__.__.40 
    %  |          /      /      /      /      |     .  . /. /. /. /. /. /. /.   
    %  |         /      /      /      /       |     .  ./ ./ ./ ./ ./ ./ ./ . 
    %  |        /______/______/_____ / 3      |     .  .__.__.__. _.__.__.30. 
    %  |       /      /      /      /         |     . /. /. /. /. /. /. /.  .
    % Y|      /      /      /      /    A    Y|     ./ ./ ./ ./ ./ ./ ./ .  .       
    %  |     /______/______/______/ 2         |     .__.__.__.__. _.__.20.  .
    %  |    /      /      /      /            |    /. /. /. /. /. /. /.  .  .
    %  |   /      /      /      /             |   / ./ ./ ./ ./ ./ ./ .  .  .
    %  |  /______/______/______/ 1            |  /__.__.__.__.__.__. 10  .  .
    %  | 0.1    0.2    0.3    0.4             | 1   2  3  4  5  6  7  .  .  .
    %  |            B                         |     .  .  .  .  .  .  .  .  .
    %  |____________________________________  |________________________________
    %                     X
    %   Fig 1: A four variable carpet plot.    Fig 2: A cheater plot. The
    %                                          intersections line up vertically
    %                                          and there is no x-axis.
    %
    %   OBJ = CARPETPLOT(a,b,x) plots a cheater plot and returns a carpetplot
    %   object, OBJ.
    %
    %   OBJ = CARPETPLOT(a,b,x,z), where z is a scalar, specifies a
    %   z-coordinate for the carpetplot object useful for interpolating between
    %   multiple carpets in a single plot.
    %
    %   OBJ = CARPETPLOT(a,b,x,y) plots a four variable carpet plot.
    %
    %   OBJ = CARPETPLOT(a,b,x,y,z) adds a z-coordinate for plot interpolation.
    %
    %   OBJ = CARPETPLOT(...,lineSpec) plots the carpet plot with the given
    %   lineSpec.
    % 
    %   OBJ = CARPETPLOT(...,'PropertyName',PropertyValue,...) changes the
    %   appareance of the carpet by changing the line properties.
    % 
    %   Input data may be either scattered data or matrices; the size of the
    %   input vectors and/or matrices must match.
    %
    %   The appereance as well as the plot's input data can be changed using
    %   different methods and properties that are part of the created object.
    %   The changes will update the plot automatically if they are done using
    %   the set() and get() methods.
    %
    %   Example 1: Create a simple cheater plot with matrix input data.
    %       a = 1:0.25:2; b=1:20:100;
    %   	[A B] = meshgrid(a, b);
    %       Y = A.*B;
    %
    %       o = carpetplot(A, B, Y); % carpetplot(a, b, Y) is also acceptable.
    %       label(o, 'A-Axis', 'B-Axis')
    %    
    %   Example 2: Create a four variable carpet plot using scattered inputs.
    %       a = [1 1 1 2 2 2 3 3 3];
    %       b = [10 20 30 10 20 30 10 20 30];
    %       X = a.^3+b/5;
    %       Y = a-b;
    %
    %       o = carpetplot(a, b, X, Y, 'LineWidth', 2, 'Color', 'black');
    %       label(o, 'A-Axis', 'B-Axis')
    %
    %   <a href="matlab:showdemo('carpetplot')">Many more examples</a>.
    % 
    % 
    %   CARPETPLOT properties:
    %   The properties should be accessed using get() and set() methods.
    %
    %   Properties for data handling 
    % 
    %     k0             - X translation of the whole plot (cheater plots only)
    %     k1             - X translation of a values (cheater plots only)
    %     k2             - X translation of b values (cheater plots only)
    %     curvefitting   - Curve fitting method
    %     atick          - Interval of the a values
    %     btick          - Interval of the b values
    %     zvalue         - Z coordinate of the carpet plot
    %
    %   Properties for visualization
    %   These values influence the vizualisation of labels, lines and arrows. 
    %   They will automatically refresh the plot or its labels. If you want 
    %   to change the appereance of the plot fast, check out style.
    % 
    %     style          - Set a pre defined style. Changes most of the
    %                      properties beneath                                
    %
    %     aarrowflipped  - Flip the position of the a-axis arrow
    %     barrowflipped  - Flip the position of the b-axis arrow
    %     aarrowspacing  - Space between the a-axis arrow and the plot
    %     barrowspacing  - Space between the b-axis arrow and the plot
    %
    %     alabelflipped  - Flip the position of the a-axis label
    %     blabelflipped  - Flip the position of the b-axis label 
    %     alabelspacing  - Space between the a-axis label and the plot
    %     blabelspacing  - Space between the b-axis label and the plot
    %
    %     asuffix        - Suffix for all a-axis values
    %     bsuffix        - Suffix for all b-axis values 
    %     aprefix        - Prefix for all a-axis values 
    %     bprefix        - Prefix for all b-axis values 
    % 
    %     atextspacing   - White spaces before or after the a-axis values
    %     btextspacing   - White spaces before or after the b-axis values 
    %     atextrotation  - Defines if the a-axis values will be rotated
    %     btextrotation  - Defines if the b-axis values will be rotated
    %     atextflipped   - Flip the position of the a-axis values
    %     btextflipped   - Flip the position of the b-axis values
    %     atextspacing   - Space between the a-axis text and the carpet
    %     btextspacing   - Space between the b-axis text and the carpet
    %
    %     zalignement    - Position of the plot's z-label     
    %
    %   Handles
    %   In order to further customize the carpet plot, it is possible
    %   to access all graphic handles directly. Changes applied to the handles
    %   may be lost after replotting the carpet.
    %         
    %     alines         - Handles of a-value lines
    %     blines         - Handles of b-value lines
    %     lines          - Handles of all lines
    %
    %     aextraplines   - Handles of a-value extrapolated lines
    %     bextraplines   - Handles of b-value extrapolated lines
    %     extraplines    - Handles of all extrapolated lines
    %
    %     amarkers       - Handles of a-value markers
    %     bmarkers       - Handles of b-value markers
    %     markers        - Handles of all markers
    %
    %     alabeltext     - Handle of the a-axis caption  
    %     blabeltext     - Handle of the b-axis caption
    %     labels         - Handles of both axes captions
    %
    %     atext          - Handles of the a-axis labels
    %     btext          - Handles of the b-axis labels
    %     text           - Handles of the labels
    %
    %     aarrow         - Handle of the a-arrow
    %     barrow         - Handle of the b-arrow
    %     arrows         - Handle of the arrows
    %
    %     zlabeltext     - Handle of the z-label         
    %
    % 
    %   CARPETPLOT methods:
    % 
    %     inputdata       - Change the input data of the carpet plot
    %     reset           - Reset all changes that were made manually
    %     refresh         - refresh the carpet plot and/or its labels
    %
    %     alabel          - Add labels to the carpet plot's a-axis
    %     blabel          - Add labels to the carpet plot's b-axis
    %     labels          - Label both axes at once
    %     zlabel          - Add a label to the carpet plot (z-axis)
    %
    %     legend          - Add a legend to a carpetplot
    %     cheaterlegend   - Adds legend indicating x-axis intervals
    %
    %     set             - Set method for carpet plot properties
    %     get             - Get method for carpet plot properties
    %
    %     contourf        - Insert a filled contour plot to the carpet plot
    %     plot            - Insert a plot into the carpet plot
    %     hatchedline     - Insert a hatchedline into the carpet plot
    %     constraint      - Add a constraint to the carpet plot
    %     showpoint       - Show a point in the carpet plot
    %     interpolateplot - Interpolate a carpet plot
    %     lattice         - Creates a lattice plot from multiple cheater plots
    %
    %     abtoxy          - Transform to the carpet plot's coordinate system
    %
    %   See also LINE, PLOT, CONTOUR, SURF, MESH, HOLD.
    %
    % Matthias Oberhauser
    % matthias.oberhauser(at)tum.de
    %
    %
    % Revision History:
    % 22 April 2013 v. 1.0
    % 01 May 2013 v. 1.01
    %       - Added lattice() method + bug fixes for lattice plots
    %       - zlabels will update automatically
    %       - Added documentation for refresh method
    %       - Updated documentation
    % 30 May 2013 v.1.02
    %       - Reverse axis support
    %       - Fixed automated label rotation (on resize, reverse axis, and log)
    %       - Updated help block (Suggestions by Sky Sartorious)
    
     
    
    
properties (Access = public)

% Variable for debug mode
debugging

%Plot's axis
ca

%Plot's Figure handle
cf

end  

properties(Dependent, SetAccess = private, GetAccess = public)    
    
        % ALINES provides the line handles of the a-axis.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.blines, carpetplot.style
        %
        alines
        
        % BLINES provides the line handles of the b-axis.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        %
        blines
        
        % AEXTRAOLINES provides the line handles of the b-axis extrapolated
        % lines. These lines only appear if there are NaNs or infs in your
        % data.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        %
        aextraplines
        
        % EXTRAPLINES provides the line handles of the extrapolated lines.
        % These lines only appear if there are NaNs or infs in your data.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        %
        extraplines
        
        % BEXTRAPLINES provides the line handles of the b-axis extrapolated
        % lines. These lines only appear if there are NaNs or inf's in your
        % data.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.alines, carpetplot.style
        %
        bextraplines
        
        % LINES provides the line handles.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.alines, carpetplot.blines
        %
        lines
        
        % AMARKERS provides the line handles of the markers of the a-axis.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.bmarkers, carpetplot.style
        %
        amarkers
        
        % BMARKERS provides the line handles of the markers of the b-axis.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.amarkers, carpetplot.style
        %
        bmarkers
        
        % MARKERS provides the line handles of the markers.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.amarkers, carpetplot.bmarkers
        %
        markers
        
        % ALABELTEXT provides the handle of the a-axis label.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.blabel, carpetplot.style
        %
        alabeltext
        
        % BLABELTEXT provides the handle of the b-axis label.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.alabel, carpetplot.style
        %
        blabeltext
        
        % LABELS provides the handle of the axis labels.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.alabel, carpetplot.blabel
        %
        labels
        
        % AARROW provides the handle of the a-arrow.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.barrow, carpetplot.style
        %
        aarrow
        
        % BARROW provides the handle of the b-arrow.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.barrow, carpetplot.style
        %
        barrow
        
        % ARROWS provides the handle of the arrows.
        %
        % Use these handles to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.aarrow, carpetplot.barrow
        %
        arrows
        
        % ATEXT provides the handle of the a-labels.
        %
        % Use this handle to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.btext, carpetplot.style
        %
        atext
        
        % BTEXT provides the handle of the b-labels.
        %
        % Use this handle to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.atext, carpetplot.style
        %
        btext
        
        % TEXT provides the handle of the labels.
        %
        % Use this handle to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.atext, carpetplot.btext
        %
        text        
        
        % ZLABELTEXT provides the handle of the z-label.
        %
        % Use this handle to further customize the plot. But note that the
        % changes might be lost if the plot gets updated by other methods.
        %
        % See also: carpetplot.label
        %
        zlabeltext
        
        
        
        
end

properties(Dependent, SetAccess = public, GetAccess = public)

        % ATEXTFLIPPED defines if the a-axis values will be drawn on the 
        % other side of the plot.
        %
        %   0     -     standard
        %   1     -     flipped
        %
        % See also: carpetplot.blabel, carpetplot.btextflipped
        %
        atextflipped
        
        % BTEXTFLIPPED defines if the b-axis values will be drawn on the 
        % other side of the plot.
        %
        %   0     -     standard
        %   1     -     flipped
        %
        % See also: carpetplot.blabel, carpetplot.atextflipped
        %
        btextflipped
        
        % AARROWFLIPPED defines if the a-axis arrow will be drawn on the 
        % other side of the plot.
        %
        %   0     -     standard
        %   1     -     flipped
        %
        % See also: carpetplot.alabel, carpetplot.aarrowflipped 
        %
        aarrowflipped
        
        % BARROWFLIPPED defines if the b-axis arrow will be drawn on the 
        % other side of the plot.
        %
        %   0     -     standard
        %   1     -     flipped
        %
        % See also: carpetplot.blabel, carpetplot.barrowflipped
        %
        barrowflipped
        
        % ALABELFLIPPED defines if the a-axis label will be drawn on the 
        % other side of the plot.
        %
        %   0     -     standard
        %   1     -     flipped
        %
        % See also: carpetplot.alabel, carpetplot.blabelflipped 
        %
        alabelflipped
        
        % BLABELFLIPPED defines if the b-axis label will be drawn on the 
        % other side of the plot.
        %
        %   0     -     standard
        %   1     -     flipped
        %
        % See also: carpetplot.blabel, carpetplot.blabelflipped
        %
        blabelflipped
 
        % ASUFFIX defines a prefix for the a-axis values.
        %
        %
        % See also: carpetplot.alabel, carpetplot.prefixA carpetplot.suffixA
        %
        asuffix
        
        % BSUFFIX defines a prefix for the b-axis values.
        %
        %
        % See also: carpetplot.aprefix carpetplot.asuffix
        %
        bsuffix
        
        % APREFIX defines a prefix for the a-axis values.
        %
        %
        % See also: carpetplot.alabel, carpetplot.bprefix carpetplot.bsuffix
        %
        aprefix
        
        % BPREFIX defines a prefix for the b-axis values.
        %
        %
        % See also: carpetplot.blabel, carpetplot.aprefix carpetplot.bsuffix
        %
        bprefix
                
        % ATEXTROTATION defines if the a-axis values are being rotated.
        %
        %   0     -     no rotation
        %   1     -     rotation according to the lines of the carpet plot  
        %
        % See also: carpetplot.alabel, carpetplot.btextRotation
        %
        atextrotation
        
        % BTEXTROTATION defines if the b-axis values are being rotated.
        %
        %   0     -     no rotation
        %   1     -     rotation according to the lines of the carpet plot 
        %
        % See also: carpetplot.blabel, carpetplot.atextRotation
        %
        btextrotation
                
        % ATEXTSPACING inserts white spaces between the values of the a-axis
        % of the carpet plot and the plot itself. 
        %
        % Negative values will add white spaces on the other side
        % of the text. 
        %
        % See also: carpetplot.btextSpacing, carpetplot.alabel
        %
        atextspacing
        
        % BTEXTSPACING inserts white spaces between the values of the b-axis
        % of the carpet plot and the plot itself. 
        %
        % Negative values will add white spaces on the other side
        % of the text. 
        %
        % See also: carpetplot.atextspacing, carpetplot.blabel
        %
        btextspacing
                
        % ALABELSPACING controls the distance between the a-label arrow and
        % the carpetplot.
        %
        %   0     -     no spacing at all
        %   1     -     base for spacing is the complete width of the carpet plot 
        %
        % See also: carpetplot.blabelspacing, carpetplot.alabel
        %
        alabelspacing
        
        % BLABELSPACING controls the distance between the a-label arrow and
        % the carpet plot.
        %
        %   0     -     no spacing at all
        %   1     -     base for spacing is the complete width of the carpet plot 
        %
        % See also: carpetplot.alabelspacing, carpetplot.alabel
        %
        blabelspacing
        
        % AARROWSPACING controls the distance between the a-arrow and
        % the carpet plot.
        %
        %   0     -     no spacing at all
        %   1     -     base for spacing is the complete width of the carpet plot 
        %
        % See also: carpetplot.blabelspacing, carpetplot.alabel
        %
        aarrowspacing
        
        % BARROWSPACING controls the space between the a-arrow and
        % the carpet plot.
        %
        %   0     -     no spacing at all
        %   1     -     base for spacing is the complete width of the carpet plot
        %
        % See also: carpetplot.aarrowspacing, carpetplot.blabel
        %
        barrowspacing
        
        
        % AFLIPPED changes the side of all a-axis labeling elemtents like arrows,
        % labels etc.
        %
        %   0     -     standard
        %   1     -     flipped
        %
        % See also: BFLIPPED
        
        aflipped
        
        % BFLIPPED changes the side of all b-axis labeling elemtents like arrows,
        % labels etc.
        %
        %   0     -     standard
        %   1     -     flipped
        %
        % See also: BFLIPPED
        
        bflipped
        
        % CURVEFITTING controls the interpolation method of the data.
        %
        % There are different styles that can be used. 
        %
        %   'linear'    -   Linear interpolation. Default method for less
        %                   than 15 Data Points 
        %   'spline'    -   Spline interpolation.
        %   'pchip'     -   Piecewise cubic interpolation.
        %   'epchip'    -   Exact piecewise cubic interpolation. 
        %   'espline'   -   Exact spline interpolation
        %   'elinear'   -   Exact linear interpolation. Default method for 
        %                   more than 15 Data Points 
        %
        %   'linear', 'spline', and 'pchip' use the specified method to
        %   find carpet intersections based on the input data. The input
        %   data is then discarded and only the calculated intersections
        %   are used to interpolate carpet lines using the specified
        %   method. 'elinear', 'espline', and 'epchip' use the original
        %   input data for interpolating the carpet lines.
        %
        %   See also: spline, pchip
        curvefitting
        
        % ATICK controls the intervals of the a-axis.
        % The property a tick controls the interval of the a-axis.
        %
        % An array with the respective values is the required input. Extrapolation
        % is not possible. In consequence, there will be an error message 
        % if the ticks are out of range of the input data.
        %
        % See also carpetplot.bTick
        % 
        atick
        
        % BTICK controls the intervals of the b-axis.
        % The property b tick controls the interval of the b-axis.
        %
        % An array with the respective values is the required input. Extrapolation
        % is not possible. In consequence, there will be an error message 
        % if the ticks are out of range of the input data.
        %
        % See also carpetplot.aTick  
        %
        btick
        
        % K0 controls the calculated x-value of the a-tick.
        % In a cheater plot (a carpet plot with 3 variables) it is possible to
        % control the plotting direction and position of the a- and the
        % b-axis.
        %
        % The x values are calculated by the following equation:
        % 
        %   x = K0 + a*K1 + b*K2
        %
        % See also: carpetplot.k2 carpetplot.k1
        %
        k0
        
        % K1 controls the calculated x-value of the a-tick.
        % In a cheater plot (a carpet plot with 3 variables) it is possible to
        % controll the plotting direction and position of the a- and the
        % b-axis.
        %
        % The x values are calculated by the following equation:
        % 
        %   x = K0 + a*K1 + b*K2
        %
        % See also: carpetplot.k2 carpetplot.k0
        %
        k1
        
        % K2 controlls the calculated x-value of the a-tick.
        % In a cheater plot (a carpet plot with 3 variables) it is possible to
        % controll the plotting direction and position of the a- and the
        % b-axis.
        %
        % The x values are calculated by the following equation:
        % 
        %   x = K0 + a*K1 + b*K2
        %
        % See also: carpetplot.k1 carpetplot.k2
        %
        k2
        
        % STYLE changes the appereance of the plot.
        % 
        % The style of the plot can be changed individually or a predefined style
        % can be used. 
        %
        % Style argument options:
        %
        %   'default'  -   Similar to the matlab plot line style.
        %   'standard' -   Rotated labels and an arrow to indicate the axis.
        %   'clean'    -   Rotated labels but no arrows.
        %   'basic'    -   A style with no labels for the a- or b-axis but a prefix
        %                  for the labels. 
        %   'minimal'  -   Minimal style.
        % 
        style
        
        % ZALIGNEMENT controls the position of the z-label.
        %
        %   'bottom'    -     On the bottom of the plot.
        %   'top'       -     On top of the plot.
        %
        % See also: carpetplot.alabel, carpetplot.blabel
        %
        zalignement
        
        % ZVALUE saves the information on the z position of the plot.
        %
        % This has no effect on the visualisazion of the plot but on the
        % interpolation of complete plots.
        %
        % See also: carpetplot.interpplot
        %
        zValue
    
end
properties (Access = private)

%Constants

CONTOUR_RESOLUTION
MAX_POINTS

%Resize Listener
listener
listenerX
listenerLogX
listenerY
listenerLogY

% Styles for the interpolation lines
interpLineStyle
interpMarkerStyle
interpTextStyle

% Name of the instance object used for the figure resizefcn
instanceName

% Debugging only
debugPointsA
debugPointsB

% Plot needs a refresh or not.    
needPlotRefresh
needRelabel 
needTextRefresh
needTextStyleRefresh

% Sets the z value of the plot
z

% Private variables of dependend variables

pzlabelandle
pZAlignement
pStyle
pZ
pK1
pK2
pK0
pCurveFitting

% How the data is going to be interpolated
dataFitting

% The data to plot (in the right intervals)
plotDataX
plotDataY

% Matrix of input data
inputMatrixA
inputMatrixB
inputMatrixX
inputMatrixY

% Variables to keep hold functionality
plotholding
holding

% Struct with axis infos. A lot of stuff
axis

% fix the axis limits
keepTicks

% 3... Cheater Plot 4... Four Variable plot
type

recentXLimits;
recentYLimits;

end
    
methods
        
    function obj = carpetplot(varargin)
                
        % Set the Style to 'standard'. This fills the axis variables with
        % certain style parameters
        obj.style = 'default';
        
        % Seperate the linespec from the data parameters
        style = 0;
        for n=1:nargin
            if ischar(varargin{n})
                style = 1;
                break;
            end
        end
        
        if style
            linespec = varargin(n:end);
            varargin = varargin(1:n-1);
            obj.axis{1}.lineSpec = linespec;
            obj.axis{2}.lineSpec = linespec;
            obj.axis{1}.markerSpec = {'marker','none'};
            obj.axis{2}.markerSpec = {'marker','none'};
        end
        
        % supress a warning if the data contains NaNs
        warning('off','MATLAB:interp1:NaNstrip');
        warning('off','MATLAB:chckxy:nan');
        
        % Set 1 for debugging
        obj.debugging = 0;
                        
        obj.recentXLimits = [0 0];
        obj.recentYLimits = [0 0];
        
        % Constants
        obj.CONTOUR_RESOLUTION = 250;
        obj.MAX_POINTS = 15;
        
        % Set default labels: The variable input names
        obj.needRelabel = 0;
        if isempty(inputname(1))
            obj.axis{1}.label = 'a axis';
        else
            obj.axis{1}.label = inputname(1);      
        end
        if isempty(inputname(2))
            obj.axis{2}.label = 'b axis';
        else
            obj.axis{2}.label = inputname(2);      
        end
        
        obj.axis{1}.labelHandle = [];       obj.axis{2}.labelHandle = [];
        obj.axis{1}.arrowHandle = [];       obj.axis{2}.arrowHandle = [];
        obj.axis{1}.textHandles = [];       obj.axis{2}.textHandles = [];
        obj.axis{1}.extrapLineHandles = []; obj.axis{2}.extrapLineHandles = [];
        
        % Set the style for the interpolations
        obj.interpLineStyle =   {       'LineWidth'         , 1.5           , ...
                                        'LineStyle'         , '--'          , ...
                                        'Color'             , [0 0 1]       };
        obj.interpMarkerStyle = {       'Marker'            , 'o'           , ...
                                        'MarkerSize'        , 7             , ...
                                        'MarkerEdgeColor'   , [.2 .2 .2]    , ...
                                        'MarkerFaceColor'   , [0 0 1]       };
        obj.interpTextStyle =   {    'FontSize'          , 9                , ...
                                     'FontWeight'        ,'normal'          , ...
                                     'VerticalAlignment' ,'bottom'          };
        
        % Set a default K0 value (cheater plots only)
        obj.pK0 = 0;
        
        % Default z alignement
        obj.pZAlignement = 'top';
        
        % Set the default data - and curveFitting
        obj.dataFitting = 'linear';
        obj.pCurveFitting = 'linear';
        
        
        
              
        % Initial value for KeepTicks-->Allows Cplot to change axis limits
        obj.keepTicks = 0;
        
        % Set a standard interval 
        if ~isvector(varargin{3}) % If it is no scattered Data
            obj.axis{1}.interval = unique(varargin{1});
            obj.axis{2}.interval = unique(varargin{2});
        else % If it is scattered use 6 Lines for A and B
            obj.axis{1}.interval = min(varargin{1}(:)):(max(varargin{1}(:))-min(varargin{1}(:)))/5:max(varargin{1}(:));
            obj.axis{2}.interval = min(varargin{2}(:)):(max(varargin{2}(:))-min(varargin{2}(:)))/5:max(varargin{2}(:));
        end
        
        % Limit the intervals to MAX_POINTS elements.
        for n=1:2 
            if size(obj.axis{n}.interval(:),1) > obj.MAX_POINTS
                obj.axis{n}.interval = linspace(min(obj.axis{n}.interval(:)),max(obj.axis{n}.interval(:)),obj.MAX_POINTS);
                obj.pCurveFitting = 'elinear';
                warning('Data of Axis %d contains more than %d DataPoints. The plot will be limited to %d Lines]. \nUse Atick and BTick to set more Lines if necessary. \n If the lines don''t line up use another curve fitting method',n,obj.MAX_POINTS);
            end
        end
        
        % Create the inputData matrix using inputdata
        obj.inputdata(varargin{:});
        
        % Plot it
        obj.cplot
        
    end
    
    %% Set and Get Functions
    % These functions do some checks before saving the input values.
    % In addition, they change needPlotrefresh and needTextRefresh to
    % indicate if the change needs a redraw.
    
    function ret = get.alines(obj)
        ret = obj.axis{1}.lineHandles;
    end
    function ret = get.blines(obj)
        ret = obj.axis{2}.lineHandles;
    end
    function ret = get.lines(obj)
        ret = [obj.axis{1}.lineHandles(:); obj.axis{2}.lineHandles(:)];
    end
       
    function ret = get.aextraplines(obj)
        ret = obj.axis{1}.extrapLineHandles;
    end
    function ret = get.bextraplines(obj)
        ret = obj.axis{2}.extrapLineHandles;
    end
    function ret = get.extraplines(obj)
        ret = [obj.axis{1}.extrapLineHandles(:); obj.axis{2}.extrapLineHandles(:)];
    end
        
    function ret = get.amarkers(obj)
        ret = obj.axis{1}.MarkerHandles;
    end
    function ret = get.bmarkers(obj)
        ret = obj.axis{2}.MarkerHandles;
    end
    function ret = get.markers(obj)
        ret = [obj.axis{1}.MarkerHandles(:); obj.axis{2}.MarkerHandles(:)];
    end
    function ret = get.alabeltext(obj)
        ret = obj.axis{1}.labelHandle;
    end
    function ret = get.blabeltext(obj)
        ret = obj.axis{2}.labelHandle;
    end
    function ret = get.labels(obj)
        ret = [obj.axis{1}.labelHandle(:) ; obj.axis{2}.labelHandle(:)];
    end
    function ret = get.aarrow(obj)
        ret = obj.axis{1}.arrowHandle;
    end
    function ret = get.barrow(obj)
        ret = obj.axis{2}.arrowHandle;
    end
    function ret = get.arrows(obj)
        ret = [obj.axis{1}.arrowHandle(:) ; obj.axis{2}.arrowHandle(:)];
    end
    
    function ret = get.atext(obj)
        ret = obj.axis{1}.textHandles;
    end
    function ret = get.btext(obj)
        ret = obj.axis{2}.textHandles;
    end
    function ret = get.text(obj)
        ret = [obj.axis{1}.textHandles(:) ; obj.axis{2}.textHandles(:)];
    end
    
    function ret = get.zlabeltext(obj)
        ret = obj.pzlabelandle;
    end
    
    function set.style(obj,style)
    % Just determine which style is chosen and assign the values to the 
    % axis struct. Feel free to add your own style.
     
        switch style
            case 'default'
                obj.pStyle = style;
                for n=1:2
                    obj.axis{n}.preText = [];
                    obj.axis{n}.postText = [];
                    obj.axis{n}.labelSpacing = 0.3;
                    obj.axis{n}.arrowSpacing = 0.3;
                    obj.axis{n}.lineSpec = {    'LineWidth'         , 0.5                   , ...
                                                'Color'             , [0 0 1]               , ...
                                                'LineStyle'         , '-'                   };
                    obj.axis{n}.extrapLineSpec = {'LineWidth'       , 0.5                   , ...
                                                'Color'             , [1 0 0]               ,...
                                                'LineStyle'         , '--'                  };
                    obj.axis{n}.markerSpec = {  'Marker'            , 'none'                };                      
                    obj.axis{n}.arrowFlipped = 1;
                    obj.axis{n}.labelFlipped = 1;
                    obj.axis{n}.textFlipped = 1;
                    obj.axis{n}.textSpacing = 2;
                    obj.axis{n}.textSpec = {    'FontSize'          ,10         , ...
                                                'FontWeight'        ,'normal'     , ...
                                                'VerticalAlignment' ,'middle'   };
                    obj.axis{n}.textRotation = 1;
                    obj.axis{n}.labelSpec = {   'FontSize'          ,10         , ...
                                                'HorizontalAlignment','center'  , ...
                                                'FontWeight'        ,'normal'   , ...
                                                'visible'           ,'on'       , ...
                                                'VerticalAlignment' ,'bottom'   };
                    obj.axis{n}.arrowSpec = {   'EdgeColor'         ,[0 0 0]    };
                end
            case 'standard'
                obj.pStyle = style;
                for n=1:2
                    obj.axis{n}.preText = [];
                    obj.axis{n}.postText = [];
                    obj.axis{n}.labelSpacing = 0.3;
                    obj.axis{n}.arrowSpacing = 0.3;
                    obj.axis{n}.lineSpec = {    'LineWidth'         , 1.5           , ...
                                                'Color'             , [0 0 0]       , ...
                                                'LineStyle'         , '-'            };
                    obj.axis{n}.extrapLineSpec = {'LineWidth'       , 1.5                   , ...
                                                'Color'             , [1 0 0]               ,...
                                                'LineStyle'         , '--'                  };                        
                    obj.axis{n}.markerSpec = {  'Marker'            , 'none'        };                      
                    obj.axis{n}.arrowFlipped = 1;
                    obj.axis{n}.labelFlipped = 1;
                    obj.axis{n}.textFlipped = 1;
                    obj.axis{n}.textSpacing = 2;
                    obj.axis{n}.textSpec = {    'FontSize'          ,10         , ...
                                                'FontWeight'        ,'normal'     , ...
                                                'Color'             , [0 0 0]               ,...
                                                'VerticalAlignment' ,'middle'   };
                    obj.axis{n}.textRotation = 1;
                    obj.axis{n}.labelSpec = {   'FontSize'          ,10        , ...
                                                'HorizontalAlignment','center'  , ...
                                                'visible'           ,'on'       , ...
                                                'FontWeight'        ,'bold'     , ...
                                                'VerticalAlignment' ,'bottom'   };
                    obj.axis{n}.arrowSpec = {   'EdgeColor'         ,[0 0 0]    };
                end
            case 'basic'
                obj.pStyle = style;
                for n=1:2
                    obj.axis{n}.preText = [];
                    obj.axis{n}.postText = [];
                    obj.axis{n}.labelSpacing = 0.3;
                    obj.axis{n}.arrowSpacing = 0.3;
                    obj.axis{n}.lineSpec = {    'LineWidth'         , 1                     ,...
                                                'Color'             , [0 0 1]               ,...
                                                'LineStyle'         , '-'                   };
                    obj.axis{n}.extrapLineSpec = {'LineWidth'       , 1                     , ...
                                                'Color'             , [1 0 0]               ,...
                                                'LineStyle'         , '--'                  }; 
                    obj.axis{n}.arrowFlipped = 0;
                    obj.axis{n}.labelFlipped = 0;
                    obj.axis{n}.textFlipped = 0;
                    obj.axis{n}.textSpec = {    'FontSize'          ,10             , ...
                                                'FontWeight'        ,'normal'       , ...
                                                'VerticalAlignment' ,'middle'     };
                    obj.axis{n}.textRotation = 0;
                    obj.axis{n}.labelSpec = {   'FontSize'          ,15             , ...
                                                'FontWeight'        ,'bold'         , ...
                                                'VerticalAlignment' ,'middle'       , ...   
                                                'visible'           ,'off'          };
                    obj.axis{n}.arrowSpec = {   'EdgeColor'         ,'none'         , ...
                                                'FaceColor'         ,'none'         };
                    obj.axis{n}.markerSpec = {  'Marker'          , 'o'         , ...
                                                'MarkerSize'      , 7           , ...
                                                'MarkerEdgeColor' , [.2 .2 .2]  , ...
                                                'MarkerFaceColor' , [.7 .7 .7]  };
                end
                obj.axis{2}.textSpacing = 4;
                obj.axis{1}.textSpacing = -4;

            case 'minimal'
                obj.pStyle = style;
                for n=1:2
                    obj.pStyle = style;
                    obj.axis{n}.preText = [];
                    obj.axis{n}.postText = [];
                    obj.axis{n}.labelSpacing = 0.3;
                    obj.axis{n}.arrowSpacing = 0.3;
                    obj.axis{n}.lineSpec = {    'LineWidth'         , 1             , ...
                                                'Color'             , [0 0 0]       , ...
                                                'LineStyle'         , '-'           };
                    obj.axis{n}.extrapLineSpec = {'LineWidth'       , 1                     , ...
                                                'Color'             , [1 0 0]               ,...
                                                'LineStyle'         , '--'                  };                         
                    obj.axis{n}.arrowFlipped = 0;
                    obj.axis{n}.textFlipped = 1;
                    obj.axis{n}.labelFlipped = 0;
                    obj.axis{n}.textSpacing = -5;
                    obj.axis{n}.textSpec = {    'FontSize'          ,10             ,...
                                                'FontWeight'        ,'normal'       ,...
                                                'VerticalAlignment' ,'bottom'       };
                    obj.axis{n}.textRotation = 1;
                    obj.axis{n}.labelSpec = {   'visible'           ,'off'          };
                    obj.axis{n}.arrowSpec = {   'EdgeColor'         ,'none'         , ...
                                                'FaceColor'         ,'none'         };
                    obj.axis{n}.markerSpec = {  'Marker'            , 'none'        };
                end
                case 'clean'
                obj.pStyle = style;
                for n=1:2
                    obj.pStyle = style;
                    obj.axis{n}.preText = [];
                    obj.axis{n}.postText = [];
                    obj.axis{n}.labelSpacing = 0.25;
                    obj.axis{n}.arrowSpacing = 0.3;
                    obj.axis{n}.lineSpec = {    'LineWidth'         , 1.5           , ...
                                                'Color'             , [0 0 0]       , ...
                                                'LineStyle'         , '-'           };
                    obj.axis{n}.extrapLineSpec = {'LineWidth'       , 1.5                   , ...
                                                'Color'             , [1 0 0]               ,...
                                                'LineStyle'         , '--'                  };                         
                    obj.axis{n}.arrowFlipped = 0;
                    obj.axis{n}.textFlipped = 0;
                    obj.axis{n}.labelFlipped = 0;
                    obj.axis{n}.textSpacing = 3;
                    obj.axis{n}.textSpec = {    'FontSize'          ,10             ,...
                                                'FontWeight'        ,'normal'       ,...
                                                'VerticalAlignment' ,'middle'       };
                    obj.axis{n}.textRotation = 1;
                    obj.axis{n}.labelSpec = {   'FontSize'          ,10             , ...
                                                'FontWeight'        ,'bold'         , ...
                                                'VerticalAlignment' ,'middle'       , ...
                                                'HorizontalAlignment' ,'center'     };
                    obj.axis{n}.arrowSpec = {   'EdgeColor'         ,'none'         , ...
                                                'FaceColor'         ,'none'         };
                    obj.axis{n}.markerSpec = {  'Marker'            , 'none'        };
                end
            otherwise
                warning('No Valid style. Keeping the current style.')
        end 
        % Refresh of Plot AND Text is needed
        obj.needPlotRefresh  = 1;
%         obj.label;
         obj.needTextStyleRefresh  = 1;
         obj.needTextRefresh  = 1;
    end
    
    function ret = get.style(obj)
        ret = obj.pStyle;
    end
    
    function set.atick(obj,value)
        %Input checks
        if ~isnumeric(value) || isempty(value)
            warning('ATick Input is not a number')
        elseif min(value) < min(obj.inputMatrixA(:)) || max(value) > max(obj.inputMatrixA(:))
            warning('Out of Range: A should be between %d and %d',min(obj.inputMatrixA(:)),max(obj.inputMatrixA(:)))               
        else
            % Clean input from dublicate values
            value = unique(value(:));
            % Change the ticks
            obj.settick(value,1);
        end    
    end
    function ret = get.atick(obj)
        ret = obj.axis{1}.interval;
    end
    function set.btick(obj,value)
        %Input checks
        if ~isnumeric(value) || isempty(value)
            warning('BTick Input is not a number')
        elseif min(value) < min(obj.inputMatrixB(:)) || max(value) > max(obj.inputMatrixB(:))
            warning('Out of Range: B should be between %d and %d',min(obj.inputMatrixB(:)),max(obj.inputMatrixB(:)))               
        else
            % Clean input from dublicate values
            value = unique(value(:));
            % Change the ticks
            obj.settick(value,2);
        end    
    end
    function ret = get.btick(obj)
        ret = obj.axis{2}.interval;
    end
        
        function set.alabelspacing(obj,value) 
            if isnumeric(value)
                obj.axis{1}.labelSpacing = value;
                obj.needTextRefresh = 1;
            else
                error('aLabelSpacing must be numeric')
            end
        end
        function ret = get.alabelspacing(obj)         
            ret = obj.axis{1}.labelSpacing;
        end
        
        function set.blabelspacing(obj,value) 
            if isnumeric(value)
                obj.axis{2}.labelSpacing = value;
                obj.needTextRefresh = 1;
            else
                error('aLabelSpacing must be numeric')
            end
        end
        function ret = get.blabelspacing(obj)         
            ret = obj.axis{2}.labelSpacing;
        end
        
        function set.aarrowspacing(obj,value) 
            if isnumeric(value)
                obj.axis{1}.arrowSpacing = value;
                obj.needTextRefresh = 1;
            else
                error('aArrowSpacing must be numeric')
            end
        end
        function ret = get.aarrowspacing(obj)         
            ret = obj.axis{1}.arrowSpacing;
        end
        
        function set.barrowspacing(obj,value) 
            if isnumeric(value)
                obj.axis{2}.arrowSpacing = value;
                obj.needTextRefresh = 1;
            else
                error('aArrowSpacing must be numeric')
            end
        end
        function ret = get.barrowspacing(obj)         
            ret = obj.axis{2}.arrowSpacing;
        end
        
        function set.aflipped(obj,value) 
            if (value == 0) || (value == 1)
                obj.axis{1}.textFlipped = value;
                obj.axis{1}.labelFlipped = value;
                obj.axis{1}.arrowFlipped = value;
                obj.needTextRefresh = 1;
            else
                error('aFlipped value must be 0 or 1')
            end
        end
        
        function set.bflipped(obj,value) 
            if (value == 0) || (value == 1)
                obj.axis{2}.textFlipped = value;
                obj.axis{2}.labelFlipped = value;
                obj.axis{2}.arrowFlipped = value;
                obj.needTextRefresh = 1;
            else
                error('bFlipped value must be 0 or 1')
            end
        end
        
        function set.atextflipped(obj,value) 
            if (value == 0) || (value == 1)
                obj.axis{1}.textFlipped = value;
                obj.needTextRefresh = 1;
            else
                error('ATextFlipped value must be 0 or 1')
            end
        end
        function ret = get.atextflipped(obj)         
            ret = obj.axis{1}.textFlipped;
        end
        
        function set.btextflipped(obj,value) 
            if (value == 0) || (value == 1)
                obj.axis{2}.textFlipped = value;
                obj.needTextRefresh = 1;
            else
                error('BTextFlipped value must be 0 or 1')
            end
        end
        function ret = get.btextflipped(obj)         
            ret = obj.axis{2}.textFlipped;
        end
        
        function set.aarrowflipped(obj,value) 
            if (value == 0) || (value == 1)
                obj.axis{1}.arrowFlipped = value;
                obj.needTextRefresh = 1;
            else
                error('AArrowFlipped value must be 0 or 1')
            end
        end
        function ret = get.aarrowflipped(obj)         
            ret = obj.axis{1}.arrowFlipped;
        end
        
        function set.barrowflipped(obj,value) 
            if (value == 0) || (value == 1)
                obj.axis{2}.arrowFlipped = value;
                obj.needTextRefresh = 1;
            else
                error('BArrowFlipped value must be 0 or 1')
            end
        end
        function ret = get.barrowflipped(obj)         
            ret = obj.axis{2}.arrowFlipped;
        end
        
        function set.alabelflipped(obj,value) 
            if (value == 0) || (value == 1)
                obj.axis{1}.labelFlipped = value;
                obj.needTextRefresh = 1;
            else
                error('ALabelFlipped value must be 0 or 1')
            end
        end
        function ret = get.alabelflipped(obj)         
            ret = obj.axis{1}.labelFlipped;
        end
        
        function set.blabelflipped(obj,value) 
            if (value == 0) || (value == 1)
                obj.axis{2}.labelFlipped = value;
                obj.needTextRefresh = 1;
            else
                error('BLabelFlipped value must be 0 or 1')
            end
        end
        function ret = get.blabelflipped(obj)         
            ret = obj.axis{2}.arrowFlipped;
        end
        
        function set.aprefix(obj,value) 
           obj.axis{1}.preText = num2str(value);
           obj.needRelabel = 1;
        end
        function ret = get.aprefix(obj)         
            ret = obj.axis{1}.prefText;
        end
        
        function set.bprefix(obj,value) 
           obj.axis{2}.preText = num2str(value);
           obj.needRelabel = 1;
        end
        function ret = get.bprefix(obj)         
            ret = obj.axis{2}.preText;
        end
        
        function set.asuffix(obj,value) 
           obj.axis{1}.postText = num2str(value);
           obj.needRelabel = 1;
        end
        function ret = get.asuffix(obj)         
            ret = obj.axis{1}.postText;
        end
        
        function set.bsuffix(obj,value) 
           obj.axis{2}.postText = num2str(value);
           obj.needRelabel = 1;
        end
        function ret = get.bsuffix(obj)         
            ret = obj.axis{2}.postText;
        end
        
        function set.atextrotation(obj,value) 
            if (value == 0) || (value == 1)
                obj.axis{1}.textRotation = value;
                obj.needTextRefresh = 1;
            else
                error('ATextRotation value must be 0 or 1')
            end
        end
        function ret = get.atextrotation(obj)   
            ret = obj.axis{1}.textRotation;
        end
        
        function set.btextrotation(obj,value) 
            if (value == 0) || (value == 1)
                obj.axis{2}.textRotation = value;
                obj.needTextRefresh = 1;
            else
                error('BTextRotation value must be 0 or 1')
            end
        end
        function ret = get.btextrotation(obj)   
            ret = obj.axis{2}.textRotation;
        end
        
        function set.atextspacing(obj,value) 
            if isnumeric(value)
                obj.axis{1}.textSpacing = value;
                obj.needTextRefresh = 1;
            else
                error('ATextSpacing must be a number')
            end
        end
        function ret = get.atextspacing(obj)         
            ret = obj.axis{1}.textSpacing;
        end
        
        function set.btextspacing(obj,value) 
            if isnumeric(value)
                obj.axis{2}.textSpacing = value;
                obj.needTextRefresh = 1;
            else
                error('BTextSpacing must be a number')
            end
        end
        function ret = get.btextspacing(obj)         
            ret = obj.axis{2}.textSpacing;
            obj.needTextRefresh = 1;
        end
        
        %Set the K0, K1 and the K2 values
        function set.k0(obj,value)
               if (obj.type==3)
                   obj.pK0 = value;
                   obj.inputMatrixX = obj.pK0 + obj.pK1.*obj.inputMatrixA+obj.pK2.*obj.inputMatrixB;
                   obj.needTextRefresh = 1;
                   obj.needPlotRefresh = 1;
               else
                   warning('The K0 value only affects plots with 3 variables')
               end
        end
        
        function set.k1(obj,value)
               if (value==0)
                   error('K1 Value cannot be zero')
               elseif (obj.type==3)
                   obj.pK1 = value;
                   obj.inputMatrixX = obj.pK0 + obj.pK1.*obj.inputMatrixA+obj.pK2.*obj.inputMatrixB;
                   obj.needTextRefresh = 1;
                   obj.needPlotRefresh = 1;
               else
                   warning('The K1 value only affects plots with 3 variables')
               end
        end
        
        function set.k2(obj,value)
               if (value==0)
                   error('K2 value cannot be zero')
               elseif (obj.type==3)
                   obj.pK2 = value;
                   obj.inputMatrixX = obj.pK0 + obj.pK1.*obj.inputMatrixA+obj.pK2.*obj.inputMatrixB;
                   obj.needTextRefresh = 1;
                   obj.needPlotRefresh = 1;
               else
                   warning('This value only affects plots with 3 variables')
               end
        end
        
        function ret = get.k0(obj)
               ret = obj.pK0;
        end
        
        function ret = get.k2(obj)
               ret = obj.pK2;
        end
        
        function ret = get.k1(obj)
               ret = obj.pK1;
        end
        
        function set.curvefitting(obj,value)
                
            if strcmp('pchip',value)  || strcmp('epchip',value) || ...
               strcmp('spline',value) || strcmp('espline',value) || strcmp('elinear',value)
                % Using linear Data Fitting because otherwise NaN Values
                % will be extrapolated
                obj.dataFitting = 'linear';
                obj.pCurveFitting = value;
            else
               if strcmp('linear',value)
                    obj.dataFitting = 'linear';
                    obj.pCurveFitting = value;
               else
                   warning([value ' is not a supported curve fitting method. Using linear'])
               end            
            end            
            obj.needPlotRefresh = 1;
         end
        
         function ret = get.curvefitting(obj)
            ret = obj.pCurveFitting;
         end
         
         function set.zalignement(obj,value) 
            if strcmpi(value,'bottom') || strcmpi(value,'top')
                obj.pZAlignement = value;
                if ~isempty(obj.pzlabelandle) && ishandle(obj.pzlabelandle)
                    txt = get(obj.pzlabelandle,'string');
                    delete(obj.pzlabelandle);
                    obj.zlabel(txt);
                end
            else
                error('zalignement musst be ''top'' or ''bottom''')
            end
            
        end
        function ret = get.zalignement(obj)         
            ret = obj.pZAlignement;
        end
        
        function set.zValue(obj,value) 
            if isnumeric(value)
                obj.z = value;
            else
                error('zValue must be numeric')
            end
            
        end
        function ret = get.zValue(obj)         
            ret = obj.z;
        end
         
        
    function inputdata(obj,varargin)
        % INPUTDATA changes the input of the carpet plot without changing
        % properties that have been set before.
        % 
        %   INPUTDATA(obj,a,b,x,y) sets the input of four variable carpet plots.
        %
        %   INPUTDATA(obj,a,b,x,y,z) sets the input of four variable carpet plots
        %   including the z coordinate.
        %
        %   INPUTDATA(obj,a,b,y) sets the input of three variable carpet plots.
        %
        %   INPUTDATA(obj,a,b,y,z) sets the input of three variable carpet plots
        %   including the z coordinate.
        %
        
        if nargin < 4
            error('Wrong number of input arguments');
        end
        
        % Assign a,b,x and/or y with same orientation
        a = varargin{1};
        b = varargin{2};        
        if isvector(a)
            a = a(:);
        end
        if isvector(b)
            b = b(:);
        end
        if isvector(varargin{3})
                varargin{3} = varargin{3}(:);
        end
        if (nargin > 4) && isvector(varargin{4})
                varargin{4} = varargin{4}(:);
        end
        
        % If no scattered data --> convert a and b to a meshgrid
        if ~isvector(varargin{3}) && (isvector(a) ==1 && isvector(b) ==1)
                [a,b] = meshgrid(a,b);
        end
        
        
        % Argument Check: What kind of Plot? - Assign Z Value
        if nargin < 5               % Cheater Plot no Z value
            obj.type = 3;
            obj.z = 0;
        elseif nargin < 6           % Cheater or Real?
            if isscalar(varargin{4})
                obj.type = 3;       % Cheater with Z Coordinate
                obj.z = varargin{4};
            else
                obj.type = 4;       % Real Plot
                obj.z = 0;
            end
        elseif nargin < 7           % Real Plot with Z value
            obj.type = 4;
            obj.z = varargin{5};
        end
        
        
        if obj.type == 3;    % Cheater Plot: Calculate the X-Axis
                stepA = (max(obj.axis{1}.interval(:))-min(obj.axis{1}.interval(:)))/(size(obj.axis{1}.interval(:),1)-1);  
                stepB = (max(obj.axis{2}.interval(:))-min(obj.axis{2}.interval(:)))/(size(obj.axis{2}.interval(:),1)-1);
                obj.pK2 = stepA/stepB;  
                obj.pK1 = 1; 
                x = obj.pK0 + obj.pK1.*a+obj.pK2.*b;
                y = varargin{3};
        else                % Real carpet plot: Assign x and y values
                x = varargin{3};
                y = varargin{4};
        end
        

        if ~isvector(x) % Input is a Matrix
            obj.inputMatrixA = a;
            obj.inputMatrixB = b;
            obj.inputMatrixX = x;
            obj.inputMatrixY = y;
        else % Input is scattered data: Interpolate X and Y with Griddedinterpolant
            [obj.inputMatrixA,obj.inputMatrixB] = meshgrid(unique(a),unique(b));
            scatteredX = TriScatteredInterp(a,b,x);
            scatteredY = TriScatteredInterp(a,b,y);
            obj.inputMatrixX = scatteredX(obj.inputMatrixA,obj.inputMatrixB);
            obj.inputMatrixY = scatteredY(obj.inputMatrixA,obj.inputMatrixB);
        end
        
        % Test for a better visualization and change the AB directions of
        % the cheater plot. There must be another way doing this?!
            if (obj.type==3)
          
                % Interpolate input data if there are Nans. Otherwise I
                % cannot calculate the area.
                if sum(isnan(obj.inputMatrixY(:))) > 0
                   iDataY = obj.inputMatrixY;
                   iDataY( :, all(isnan(iDataY), 1)) = [];
                   iDataY(all(isnan(iDataY), 2), :) = [];

                   reSized=interp1(iDataY,linspace(1,size(iDataY,1),size(iDataY,1)),'spline');
                   reSized=interp1(reSized.',linspace(1,size(iDataY,2),size(iDataY,2)),'spline').';
                   iDataY = reSized;
                else
                   iDataY = obj.inputMatrixY;
                end
                
                
                area = zeros(3,2);

                area(2,1) = obj.pK1;
                area(3,1) = obj.pK2;
                
                area(1,1) = polyarea([obj.inputMatrixX(1,1) obj.inputMatrixX(1,end) obj.inputMatrixX(end,end) obj.inputMatrixX(end,1)],[iDataY(1,1) iDataY(1,end) iDataY(end,end) iDataY(end,1)]);
                obj.pK1 = -obj.pK1;
                
                obj.inputMatrixX = obj.pK0 + obj.pK1.*obj.inputMatrixA+obj.pK2.*obj.inputMatrixB;
                
                area(2,2) = obj.pK1;
                area(3,2) = obj.pK2;
                area(1,2) = polyarea([obj.inputMatrixX(1,1) obj.inputMatrixX(1,end) obj.inputMatrixX(end,end) obj.inputMatrixX(end,1)],[iDataY(1,1) iDataY(1,end) iDataY(end,end) iDataY(end,1)]);
                
                area = sortrows(area.',1).';

                obj.pK1 = area(2,2);
                obj.pK2 = area(3,2);
                
                if (obj.pK1 <0) && (obj.pK2 <0)
                    obj.pK1 = abs(obj.pK1);
                    obj.pK2 = abs(obj.pK2);
                end
                
                obj.inputMatrixX = obj.pK0 + obj.pK1.*obj.inputMatrixA+obj.pK2.*obj.inputMatrixB;
            end
            
            % All infs must be converted to Nans
            obj.inputMatrixX(obj.inputMatrixX == inf) = NaN;
            obj.inputMatrixY(obj.inputMatrixY == inf) = NaN;
            
            % If the carpet was ploted before --> refresh the plot
            if ~isempty(obj.plotDataY)
               obj.refreshplot;
               obj.refreshlabels;
            end
           
            
    end
    
    function cplot(obj)
        % CPLOT as a public method is obsolet. It was used in previous versions
        
        % Debugging only - Delete input data points
        if obj.debugging
            delete(obj.debugPointsA);
            delete(obj.debugPointsB);
        end
        
        % Keep hold functionality
        newaxis = obj.holdon;
        % Get the current axis and figures. Check if there was no axis
        % before.
        obj.ca = gca;
        obj.cf = gcf;
                
        % Interpolate the x y position of the intersections according to
        % the axis intervals.   
        [a,b] = meshgrid(obj.axis{1}.interval,obj.axis{2}.interval);
        [obj.plotDataX,obj.plotDataY] = obj.transformtoxy(a,b);
        
        maskX = obj.plotDataX;
        maskY = obj.plotDataY;
        maskX(isfinite(maskX)) = 1;
        maskY(isfinite(maskY)) = 1;
                
        obj.plotDataX = obj.plotDataX .* maskY;
        obj.plotDataY = obj.plotDataY .* maskX;
        
        % Matrix with ones. Plot will be in the foreground.
        plotDataZ = ones(size(obj.plotDataX));
        
        if (sum(isnan(obj.plotDataX(:))) > 0) || (sum(isnan(obj.plotDataY(:))) > 0)
            [extraX,extraY] = obj.getpData;
            nPlots = 4;
        else
            extraX = 0;
            extraY = 0;
            nPlots = 2;
        end
             
        switch obj.pCurveFitting % Do the plotting using a curveFitting method
            
            case {'spline','pchip'}
                
                % Spline, Pchip, z-coordinate
                pointsZ = ones(1,100) * 1;
                
                %Save plotData to temp variables
                tPlotDataX = obj.plotDataX';
                tPlotDataY = obj.plotDataY';
                
                for i = 1:nPlots % A Axis and B Axis and extrapolations if Data contains NaN's
                                        
                    % Clear the linehandles of the axis
                    obj.axis{i}.lineHandles = [];
                    for n = 1:size(tPlotDataX,1) % Every Row of the Plot Data
                        
                        % Plot the carpet
                        if obj.checkXYPoints(tPlotDataX(n,:),tPlotDataY(n,:)) % If there are at least two data Points
                        
                            pointsX = linspace(min(tPlotDataX(n,:)),max(tPlotDataX(n,:)),100);
                            if length(unique(tPlotDataX(n,:))) ==  length(tPlotDataX(n,:))
                                if strcmp(obj.pCurveFitting,'spline')
                                    curvePlot = spline(tPlotDataX(n,:),tPlotDataY(n,:));
                                else
                                	curvePlot = pchip(tPlotDataX(n,:),tPlotDataY(n,:));
                                end
                                if i < 3
                                    obj.axis{i}.lineHandles(n) = plot3(pointsX(:),ppval(curvePlot,pointsX(:)),pointsZ(:),obj.axis{i}.lineSpec{:});
                                else
                                    obj.axis{i-2}.extrapLineHandles(n) = plot3(pointsX(:),ppval(curvePlot,pointsX(:)),pointsZ(:)-0.1,obj.axis{i-2}.extrapLineSpec{:});
                                end
                            else
                                if i < 3
                                    obj.axis{i}.lineHandles(n) = plot3(tPlotDataX(n,:),tPlotDataY(n,:),plotDataZ(n,:),obj.axis{i}.lineSpec{:});
                                else
                                    obj.axis{i-2}.extrapLineHandles(n) = plot3(tPlotDataX(n,:),tPlotDataY(n,:),plotDataZ(n,:)-0.1,obj.axis{i-2}.extrapLineSpec{:});
                                end
                            end
                        end
                 
                    end
                    
                    if i == 1
                        % Flip matrix for the b-axis
                        tPlotDataX = obj.plotDataX;
                        tPlotDataY = obj.plotDataY;
                    elseif i == 2;
                        tPlotDataX = extraX;
                        tPlotDataY = extraY;
                    elseif i == 3;
                        tPlotDataX = extraX';
                        tPlotDataY = extraY';
                    end
                end
                          
            case {'espline','epchip','elinear'}

                % Spline, Pchip, z-coordinate
                pointsZ = ones(1,100) * 1;
                
                % Create matrix for exact espline and exact epchip
                
                interva = unique([min(obj.axis{2}.interval(:)); obj.inputMatrixB(:,1); max(obj.axis{2}.interval(:))]);
                
                aE = repmat(obj.axis{1}.interval(:)',size(interva,1),1);
                bE = repmat(interva,1,size(obj.axis{1}.interval(:),1));
                
                % Crop to give interval
                bE(bE<min(obj.axis{2}.interval)) = nan; bE(bE>max(obj.axis{2}.interval)) = nan;
                aE(aE<min(obj.axis{1}.interval)) = nan; aE(aE>max(obj.axis{1}.interval)) = nan;
                                
                % Transform the a b matrix to the xy coordinate system
                [tPlotDataX,tPlotDataY] = obj.transformtoxy(aE,bE);
                
                % Flip the matrix
                tPlotDataX = tPlotDataX';
                tPlotDataY = tPlotDataY';
              
                for i = 1:nPlots % a-axis and b-axis
                    
                    maskX = tPlotDataX;
                    maskY = tPlotDataY;
                    maskX(isfinite(maskX)) = 1;
                    maskY(isfinite(maskY)) = 1;
                
                    tPlotDataX = tPlotDataX .* maskY;
                    tPlotDataY = tPlotDataY .* maskX;
                    
                    % Clear the linehandles of the axis
                    obj.axis{i}.lineHandles = [];
                    for n = 1:size(tPlotDataX,1) % Every row of the plot data
                        
                        if strcmp(obj.pCurveFitting,'elinear')
                            if i < 3
                                obj.axis{i}.lineHandles(n) = plot3(tPlotDataX(n,:),tPlotDataY(n,:),tPlotDataY(n,:).*0 + 1,obj.axis{i}.lineSpec{:});
                            else
                                obj.axis{i-2}.extrapLineHandles(n) = plot3(tPlotDataX(n,:),tPlotDataY(n,:),tPlotDataY(n,:).*0 - 0.1,obj.axis{i-2}.extrapLineSpec{:});
                            end
                            
                        elseif obj.checkXYPoints(tPlotDataX(n,:),tPlotDataY(n,:)) % If there are at least two data Points
                        
                            pointsX = linspace(min(tPlotDataX(n,:)),max(tPlotDataX(n,:)),100);

                            if strcmp(obj.pCurveFitting,'espline')
                                curvePlot = spline(tPlotDataX(n,:),tPlotDataY(n,:));
                            else
                                curvePlot = pchip(tPlotDataX(n,:),tPlotDataY(n,:));
                            end
                                                        
%                           obj.axis{i}.lineHandles(n) = plot3(pointsX(:),ppval(curvePlot,pointsX(:)),pointsZ(:),obj.axis{i}.lineSpec{:});
                            if i < 3
                                obj.axis{i}.lineHandles(n) = plot3(pointsX(:),ppval(curvePlot,pointsX(:)),pointsZ(:),obj.axis{i}.lineSpec{:});
                            else
                                obj.axis{i-2}.extrapLineHandles(n) = plot3(pointsX(:),ppval(curvePlot,pointsX(:)),pointsZ(:)-0.1,obj.axis{i-2}.extrapLineSpec{:});
                            end    
                        
                        
                        end
                    end
                    
                    % Prepare plot data for the second run
                    if i == 1
                        interva = unique([min(obj.axis{1}.interval(:)) obj.inputMatrixA(1,:) max(obj.axis{1}.interval(:))]);
                        bE = repmat(obj.axis{2}.interval(:),1,size(interva,2));
                        aE = repmat(interva,size(obj.axis{2}.interval(:),1),1);
                        bE(bE<min(obj.axis{2}.interval)) = nan; bE(bE>max(obj.axis{2}.interval)) = nan;
                        aE(aE<min(obj.axis{1}.interval)) = nan; aE(aE>max(obj.axis{1}.interval)) = nan;
                        [tPlotDataX,tPlotDataY] = obj.transformtoxy(aE,bE);
                        maskX = tPlotDataX;
                        maskY = tPlotDataY;
                        maskX(isfinite(maskX)) = 1;
                        maskY(isfinite(maskY)) = 1;
                
                        tPlotDataX = tPlotDataX .* maskY;
                        tPlotDataY = tPlotDataY .* maskX;
                    
                    elseif i == 2;
                        tPlotDataX = extraX;
                        tPlotDataY = extraY;
                    elseif i == 3;
                        tPlotDataX = extraX';
                        tPlotDataY = extraY';
                    end
       
                end
                
        
            otherwise % Linear plot
                obj.axis{1}.lineHandles =  plot3(obj.plotDataX,obj.plotDataY,plotDataZ,obj.axis{1}.lineSpec{:});
                obj.axis{2}.lineHandles =  plot3(obj.plotDataX',obj.plotDataY',plotDataZ',obj.axis{2}.lineSpec{:});
                
        end
        
        % Plot the markers at the intersections. Not nevessary for linear style but for 
        % pchip or spline etc. it is necessary to split line and marker.
        obj.axis{1}.intersectionHandles =  plot3(obj.plotDataX,obj.plotDataY,plotDataZ,obj.axis{1}.markerSpec{:});
        obj.axis{2}.intersectionHandles =  plot3(obj.plotDataX',obj.plotDataY',plotDataZ',obj.axis{2}.markerSpec{:});
        set(obj.axis{1}.intersectionHandles,'LineStyle','none');
        set(obj.axis{2}.intersectionHandles,'LineStyle','none');
        
        if obj.debugging
            obj.debugPointsA =  plot3(obj.inputMatrixX,obj.inputMatrixY,(obj.inputMatrixY.*0)-10,'bo');
            obj.debugPointsB =  plot3(obj.inputMatrixX',obj.inputMatrixY',(obj.inputMatrixY'.*0)-10,'bo');
            set(obj.debugPointsA,'LineStyle','none');
            set(obj.debugPointsA,'LineStyle','none');
        end
        
% %         Extrapolate Nans and draw it.
%         if (sum(isnan(obj.plotDataX(:))) > 0) || (sum(isnan(obj.plotDataY(:))) > 0)
%             [extraX,extraY] = obj.getpData;
%             
%     %         pDataX = obj.plotDataX;
%     %         pDataY = obj.plotDataY;
%     %         pDataX( :, all(isnan(pDataX), 1)) = []; pDataY( :, all(isnan(pDataY), 1)) = [];
%     %         pDataX(all(isnan(pDataX), 2), :) = []; pDataY(all(isnan(pDataY), 2), :) = [];
%     %         pDataX(~isnan(pDataX)) = inf; pDataY(~isnan(pDataY)) = inf;
%     %         pDataX(isnan(pDataX)) = 1; pDataY(isnan(pDataY)) = 1;
%     %         pDataX(isinf(pDataX)) = NaN; pDataY(isinf(pDataY)) = NaN;
%     %         extraX = extraX .* pDataX; extraY = extraY .* pDataY;
% 
%             obj.axis{1}.extrapLineHandles =  plot(extraX,extraY,'r-');
%             obj.axis{2}.extrapLineHandles =  plot(extraX',extraY','r-');
%         end
        
        % Keep hold functionality
        obj.holdoff;
        
        % Change the x and y limits of the axis. Don't do it if keepTicks
        % ==1. see refreshplot
        if obj.keepTicks == 0;
                [pDataX,pDataY] = obj.getpData;
                
                % Get the plot's limits and the current limits
                      
                xlimits = [min(pDataX(:)) max(pDataX(:))];
                ylimits = [min(pDataY(:)) max(pDataY(:))];
                xAbs = abs(xlimits(2) - xlimits(1));
                yAbs = abs(ylimits(2) - ylimits(1));
                
                if newaxis
                    xlim(xlimits);
                    ylim(ylimits);
                    obj.recentXLimits = xlim;
                    obj.recentYLimits = ylim;
                end                   
                    xLimPlotOld = xlim;
                    yLimPlotOld = ylim;
                
                
                xLimPlotNew = xLimPlotOld;
                yLimPlotNew = yLimPlotOld;

            % Check if the limits of the plot are out of bound plus a 20%
            % margin
            if isequal(obj.recentXLimits,xlim) ...
                    && isequal(obj.recentYLimits,ylim)
                xlim([xlimits(1) - 0.2 * xAbs xlimits(2) + 0.2 * xAbs]);
                ylim([ylimits(1) - 0.2 * yAbs ylimits(2) + 0.2 * yAbs]);
                obj.recentXLimits = xlim;
                obj.recentYLimits = ylim;
            else
               
                if xlimits(1) <= (xLimPlotOld(1) + 0.2 * xAbs)
                    xLimPlotNew(1) = xlimits(1) - 0.2 * xAbs;
                end
                if xlimits(2) >= (xLimPlotOld(2) - 0.2 * xAbs)
                    xLimPlotNew(2) = xlimits(2) + 0.2 * xAbs;
                end
                if ylimits(1) <= (yLimPlotOld(1) + 0.2 * yAbs)
                    yLimPlotNew(1) = ylimits(1) - 0.2 * yAbs;
                end
                if ylimits(2) >= (yLimPlotOld(2) - 0.2 * yAbs)
                    yLimPlotNew(2) = ylimits(2) + 0.2 * yAbs;
                end
            
            
            % Assign the new limits
            xlim([xLimPlotNew(1) xLimPlotNew(2)]);
            ylim([yLimPlotNew(1) yLimPlotNew(2)]);
            
            end
            
            % Set a vertical grid if it is a cheater plot            
            if obj.type == 3
                ticks = sort(obj.plotDataX(:));
                ticks(diff([ticks(1)-1 ticks'])<1e-6)=[];
                xlimits = xlim;
                dist = (max(ticks)-min(ticks))/(size(ticks(:),1)-1);
                ticks = [min(ticks(:))-10*dist:dist:min(ticks(:))-dist ticks(:)' max(ticks(:))+dist:dist:xlimits(2)];
                
                
                set(obj.ca,'xtick',ticks);
                
                
                set(obj.ca,'XGrid','off');
                set(obj.ca,'xticklabel',[]);
                %set(obj.ca,'XColor',get(obj.ca,'Color'))
            elseif obj.type == 4
                set(obj.ca,'XTickLabelMode','auto');
                set(obj.ca,'Xtickmode','auto');
            end
        
        end
                
    end
    
    function set(obj,varargin)
    % SET calls set functions for the given properties.
    %
    %  set(obj,'PropertyName',PropertyValue,...)
    %
    % Set works similar to matlab's built in set function.
    % Call HELP CARPETPLOT to see a list of accessible properties.
    %
    % See Also: carpetplot, carpetplot.get
    % 
    
        % Size of varargin must be even. Parameter/Value Pairs.
        if rem(nargin-1,2) ~= 0;
            error('Every argument needs a parameter/value')
        end
        
        % Evaluate the varargin parameter
        for n=1:2:nargin-1
            
            if ischar(varargin{n}) && isprop(obj,lower(varargin{n}))
                mp = findprop(obj,lower(varargin{n}));
                if strcmp(mp.GetAccess,'public')
                    if ischar(varargin{n+1})
                        evalStr = ['obj.' lower(varargin{n}) '=''' num2str(varargin{n+1}) ''';'];
                    else
                        evalStr = ['obj.' lower(varargin{n}) '=[' num2str(varargin{n+1}) '];'];
                    end
                    % Use eval to call set functions
                    eval(evalStr)
                else
                    error('Invalid parameter/value pair arguments')
                end
            else
                error('Invalid parameter/value pair arguments')
            end
        
        end
        
        % Check if any of the set functions need a redraw.
        
        if obj.needPlotRefresh
            obj.refreshplot;
        end
        if obj.needRelabel
            obj.plabel(1);
            obj.plabel(2);
        end
        if obj.needTextRefresh
            obj.refreshlabels('position');
        end 
        if obj.needTextStyleRefresh
            obj.refreshlabels('style');
        end
           
    end
    
    function ret = get(obj,param)
    % GET calls get functions for the given properties.
    %
    %  get(obj,'PropertyName')
    %
    % get works similar to matlab's built in get function.
    % Call HELP CARPETPLOT to see a list of accessible properties.
    %
    % See Also: carpetplot, carpetplot.set
    % 
        
        % Make a string to evaluate and run it via eval.
        if ischar(param) && isprop(obj,lower(param))
            evalStr = ['obj.' lower(param)];
            ret = eval(evalStr);
        else
           error([num2str(param) ' is not a carpet plot property']) 
        end
    end
    
    function refresh(varargin)
    % REFRESH redraws the labels and/or the carpet plot
    %
    % refresh(obj1,obj2,obj3,...) redraw the labels and the carpet plot
    %
    % refresh(...,'textrotation') updates the label rotation
    %
    % refresh(...,'plot') redraws the carpet plot
    %
    % See also: carpetplot.reset
    
    if ischar(varargin{end})    
        switch varargin{end}
            case 'textrotation'
                for n = 1:size(varargin{1}(:),1)
                    varargin{1}(n).refreshlabels(1)
                    varargin{1}(n).refreshlabels(2)
                end
            case 'plot'
                for n = 1:size(varargin{1}(:),1)
                    varargin{1}(n).refreshplot;
                end
            otherwise
                for n = 1:size(varargin{1}(:),1)
                    varargin{1}(n).refreshlabels(1)
                    varargin{1}(n).refreshlabels(2)
                    varargin{1}(n).refreshplot;
                end
        end
        
    else
        for n = 1:size(varargin{1}(:),1)
            varargin{1}(n).refreshlabels(1)
            varargin{1}(n).refreshlabels(2)
            varargin{1}(n).refreshplot;
        end
    end
    
    end
    
    function alabel(obj,text)
    % ALABEL adds labels to the a-axis.
    %
    %  ALABEL(obj) adds the label with a default string. This string is
    %  usually the variable name given to the contructor of the class.
    %
    %  ALABEL(obj,str) adds the label with the defined str value.
    %
    % See Also: carpetplot.blabel, carpetplot.label
    %    
        obj.instanceName = inputname(1);
        
        if nargin > 1
            obj.axis{1}.label = text;
        end
        
        obj.plabel(1)
        
        obj.needRelabel = 0;
    end
    
    function blabel(obj,text)
    % BLABEL adds labels to the b-axis.
    %
    %  BLABEL(obj) adds the label with a default string. This string is
    %  usually the variable name given to the contructor of the class.
    %
    %  BLABEL(obj,str) adds the label with the defined str value.
    %
    % See Also: carpetplot.alabel, carpetplot.label
    %    
        obj.instanceName = inputname(1);
        
        if nargin > 1
            obj.axis{2}.label = text;
        end
        
        obj.plabel(2)
        
        obj.needRelabel = 0;
    end
    
    function label(obj,varargin)
    % LABEL adds labels to the a- and b-axis.
    %
    %  LABEL(obj) adds the labels with a default string.
    %
    %  LABEL(obj,str1) adds the a-label with the defined str1 value.
    %
    %  LABEL(obj,str1,str2) adds the a-label with the defined str1 parameter
    %                       and the b-label with the str2 parameter.
    %
    % See Also: carpetplot.alabel, carpetplot.blabel, carpetplot.label
    %     
        
        obj.instanceName = inputname(1);
        
        if nargin > 2
            obj.axis{1}.label = varargin{1};
            obj.axis{2}.label = varargin{2};
        elseif nargin > 1
            obj.axis{1}.label = varargin{1};
        end
        
        obj.plabel(1);
        obj.plabel(2);
        
        obj.needRelabel = 0;
    end
    
    function zlabel(obj,varargin)
    % ZLABEL adds a z label to the carpet plot.            
        
        if (sum(isnan(obj.plotDataX(:))) > 0) || (sum(isnan(obj.plotDataY(:))) > 0)
            [a,b] = meshgrid(obj.axis{1}.interval,obj.axis{2}.interval);
            [pDataX,pDataY] = obj.transformtoxy(a,b,'spline');
        else
            pDataX = obj.plotDataX;
            pDataY = obj.plotDataY;
        end
        
        mX = min(pDataX(:))+((max(pDataX(:))-min(pDataX(:)))/2);
        
        if nargin > 1
            txt = varargin{1};
        else
            txt = ['z=' num2str(obj.z)];
        end
        if nargin > 2
        obj.pZAlignement = varargin{end};
        end
        
        switch obj.pZAlignement
            case 'top'                
                mY = max(pDataY(:)) + (max(pDataY(:))-min(pDataY(:))).*0.3;                
            case 'bottom'                    
                mY = min(pDataY(:)) - (max(pDataY(:))-min(pDataY(:))).*0.3;
            otherwise
                mY = max(pDataY(:)) + (max(pDataY(:))-min(pDataY(:))).*0.3;  
        end
        obj.deleteHandle(obj.pzlabelandle);
        obj.pzlabelandle = text(mX,mY,txt);
        set(obj.pzlabelandle,'HorizontalAlignment','center');
    end
    
     function [c cont] = contourf(obj,vectorA,vectorB,data,varargin)
        % CONTOURF, used with a carpet plot object, will transform the contour
        % to the a/b coordinate system. 
        % 
        % CONTOURF(obj,a,b,z,v) draws a filled contour using a and b to 
        % determine the a and b limits. All further variables will be
        % handed to the contourf function.
        %  
        % Example:
        %
        %   a =[1;2;3;1;2;3];
        %   b =[10;10;10;30;30;30];
        %   x = b-a.^3;
        %   y = a.*b;
        %
        %   plotObject = carpetplot(a,b,x,y);
        %
        %   contourf(plotObject,1:0.1:3,10:1:30,peaks(21));
        %
        % See also: carpetplot.plot, carpetplot.hatchedline
            
            % Keep hold functionality
            if ishold == 0
                obj.holding = 0;
                hold on
            else
                obj.holding = 1;
            end
        
            % Check input
            if ~isvector(vectorA) || ~isvector(vectorB)
                error('Input must be vectors')
            else
                vectorA = vectorA(:)';
                vectorB = vectorB(:)';
            end
            
            % Get the mask to cut the edges of the contour
            [~,maskPlot] = getConstrMask(obj,[]);
            maskPlot(maskPlot==0) = NaN;
           
            %Extend matrix to contour resolution
            [inputDataX,inputDataY] = meshgrid(vectorA,vectorB);
            vectorA = interp1(1:size(obj.axis{1}.interval(:),1),obj.axis{1}.interval,1:(size(obj.axis{1}.interval(:),1)-1)/obj.CONTOUR_RESOLUTION:size(obj.axis{1}.interval(:),1));
            vectorB = interp1(1:size(obj.axis{2}.interval(:),1),obj.axis{2}.interval,1:(size(obj.axis{2}.interval(:),1)-1)/obj.CONTOUR_RESOLUTION:size(obj.axis{2}.interval(:),1));
            [aaa,bbb] = meshgrid(vectorA,vectorB);
            %Interpolate the data points to the contour resolution
            [dataX,dataY] = meshgrid(vectorA,vectorB);
            data = interp2(inputDataX,inputDataY,data,dataX,dataY,obj.dataFitting);
            xxx = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixX,aaa,bbb,obj.dataFitting);
            yyy = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixY,aaa,bbb,obj.dataFitting);
           
            nxqq = min(xxx(:)):(max(xxx(:))-min(xxx(:)))/obj.CONTOUR_RESOLUTION:(max(xxx(:)));
            nyqq = min(yyy(:)):(max(yyy(:))-min(yyy(:)))/obj.CONTOUR_RESOLUTION:(max(yyy(:)));
            [xqq,yqq] = meshgrid(nxqq,nyqq);
            
          
            a1 = griddata(xxx,yyy,data,xqq,yqq);
            
            % Multiply the matrix with the contour mask
            a1 = a1.*maskPlot;
            %Plot Contourf
            if ~isempty(varargin)
                [c,cont] = contourf(nxqq,nyqq,a1,varargin);
            else
                [c,cont] = contourf(nxqq,nyqq,a1);
            end
            
            set(cont,'edgecolor','none');
            
            % Restore Hold Functionality        
            if obj.holding == 0
                hold off
            end
           
     end
    
    function [hLines] = lattice(varargin)  
    % LATTICE transforms multiple cheater plots into one lattice plot.
    % 
    % hlines = LATTICE(obj1,obj2,obj3,...) changes the x-axis shifting of
    % the plot according to its z value and connects the intersections of
    % the plots with lines. As an output the handles of the lines are
    % available in an array.
    % 
    % hlines = LATTICE(...,'lines') only draws the lines of the cheater
    % plots
    % 
    % LATTICE(...,'position') only changes the k0 values according
    % to the z-values

        
        
%     % Heep Hold functionality
%     if ishold == 0
%       obj.holding = 0;
%       hold on
%     else
%       obj.holding = 1;
%     end
    
    if ischar(varargin{end})
        nPlots = nargin-1;
        string = 1;
    else
        nPlots = nargin;
        string = 0;
    end
    
    
    
    if (ischar(varargin{end}) && ~strcmp(varargin{end},'lines')) || ~ischar(varargin{end})
        intervals = zeros(1,nPlots);
        zValues = zeros(1,nPlots);
        zInterv = zeros(1,nPlots-1);
        o = 1;
        for n = 1:nPlots
            
            for i = 1:size(varargin{n}(:),1)
                intervals(o) = max(varargin{n}(i).plotDataX(:))-min(varargin{n}(i).plotDataX(:));
                zValues(o) = varargin{n}(i).z;
                o=o+1;
            end
        end
            zValues = unique(zValues);
        for n = 2:size(zValues(:),1)
            zInterv(n-1) = abs(zValues(n) - zValues(n-1));
        end
            zMin = min(zInterv(:));
            xMin = max(intervals(:));
        for n = 1:nPlots
            for i = 1:size(varargin{n}(:),1)
                set(varargin{n}(i),'k0',varargin{n}(i).z/zMin*xMin);
            end
        end
        hLines = [];
        
        ticksAll = [];
        for n = 1:nPlots
            
            for i = 1:size(varargin{n}(:),1)
                ticks = sort(varargin{n}(i).plotDataX(:));
                ticks(diff([ticks(1)-1 ticks'])<1e-6)=[];
                xlimits = xlim;
                dist = (max(ticks)-min(ticks))/(size(ticks(:),1)-1);
                ticks = [min(ticks(:))-10*dist:dist:min(ticks(:))-dist ticks(:)' max(ticks(:))+dist:dist:xlimits(2)];
                ticksAll = [ticksAll ticks];
            end
            
        end

        set(varargin{1}(1).ca,'xtick',unique(ticksAll));


        set(varargin{1}(1).ca,'XGrid','off');
        set(varargin{1}(1).ca,'xticklabel',[]);
        
        
    end
        
    if (ischar(varargin{end}) && ~strcmp(varargin{end},'position')) || ~ischar(varargin{end})
        hLines = zeros(1,size(varargin{1}(1).atick(:),1)*size(varargin{1}(1).btick(:),1));
        i = 1;
        for na = 1: size(varargin{1}(1).atick(:),1)
            for nb = 1: size(varargin{1}(1).btick(:),1)
            [hl, hm, ht] = showpoint(varargin{1:end-string},varargin{1}(1).atick(na),varargin{1}(1).btick(nb));
            delete(hm); delete(ht);
            delete(hl(1:end-1));
            hl(1:end-1) = [];
            set(hl,varargin{1}(1).axis{1}.lineSpec{:})
            hLines(i) = hl;
            i = i+1;
            end
        end
    end
    
%     % Hold functionality
%     if obj.holding == 0
%         hold off
%     end
        
    end
        
    function [hLines,hMarkers,hText] = showpoint(varargin)
    % SHOWPOINT shows a point in the carpet plot.
    %
    % After the input of the a- and b-values the position in the carpet plot will
    % be plotted. This also works with multiple plots. The interpolation of the
    % point in z-direction will also be plotted. 
    %
    % The interpolated point will not update if the input data or the
    % intervals of the plot change.
    % 
    %  [lineHandles, MarkerHandles, TextHandles] = SHOWPOINT(obj,a,b,lineSpec) 
    %                   show a point in one carpet plot. lineSpec is
    %                   optional and is handed over to the plot() function.
    %  [lineHandles, MarkerHandles, TextHandles] = SHOWPOINT(obj1,obj2,obj3,...,a,b,style) 
    %                   show the points in several carpet plots.
    %  
    %
    % See Also: carpetplot.interpolateplot
    %       
        
    % Heep Hold functionality
    if ishold == 0
      obj.holding = 0;
      hold on
    else
      obj.holding = 1;
    end

    % Check if there is a given line style.
    lineStyle = varargin{1}.interpLineStyle;
    style = 0;
    for n=1:nargin

        if ischar(varargin{n})
            lineStyle =  varargin(n:end);
            style = 1;
            break;
        end

    end
    
    % Set the input variables.
    
    
    if style
        aSize = (n-3);
        a = varargin{n-2};
        b = varargin{n-1};
    else
        aSize = (n-2);
        a = varargin{n-1};
        b = varargin{n};
    end

    
    % Check if a or b is out of bound.
    if a > max(varargin{1}(1).axis{1}.interval) || a < min(varargin{1}(1).axis{1}.interval)
        error('a is out of Bound')
    end
    if b > max(varargin{1}(1).axis{2}.interval) || b < min(varargin{1}(1).axis{2}.interval)
        error('b is out of Bound')
    end


    interpDataRowX = zeros(aSize,1);
    interpDataRowY = zeros(aSize,1);

%         interpolations = [];

     hLines = []; hMarkers = []; hText = [];

    % Loop through multiple plots and plot the inpterpolations.
    o = 1;
    for n = 1 : aSize
        for i = 1:size(varargin{n}(:),1)
            [X,Y,dataX,dataY] = varargin{n}(i).interpAB(a,b);
            [hl, hm, ht] = varargin{n}(i).drawinterpolation(X,Y,dataX,dataY,lineStyle);
            hLines = [hLines(:);hl(:)]; hMarkers = [hMarkers(:); hm(:)]; hText = [hText(:); ht(:)];
            interpDataRowX(o) = X;
            interpDataRowY(o) = Y;
            o = o+1;
        end
    end
    
    % Plot the line for multiple carpets. I thought of making different
    % curve fitting options for the line but I just sticked with pchip.
    if o > 2
%             if strcmp(interpStyle,'spline')
%                 DataSpline = spline(interpDataRowX,interpDataRowY);
%                 xx = linspace(min(interpDataRowX),max(interpDataRowX),201);
%                 zz = ones(1,size(xx(:),1)) * 0.1;
%                 line = plot3(xx,ppval(DataSpline,xx),zz,'--','LineSmoothing','on');
%             elseif strcmp(interpStyle,'spline')
            dataPchip = pchip(interpDataRowX,interpDataRowY);
            xx = linspace(min(interpDataRowX),max(interpDataRowX),201);
            
            hLines(end+1) = plot3(xx,ppval(dataPchip,xx),xx.*0+2,lineStyle{:});
%             else
%                 zz = ones(1,size(interpDataRowX(:),1)) * 0.1;
%                 line = plot3(interpDataRowX,interpDataRowY,zz,'--');
%              end
    end
    
    % Hold functionality
    if obj.holding == 0
        hold off
    end
        
    end
    
    function outObj = interpolateplot(varargin)
    % INTERPOLATEPLOT interpolates a complete carpet plot.
	%
    % With INTERPOLATEPLOT it is possible to interpolate a complete carpet plot
    % in the z-direction. It is important that the input data of the plots have
    % the same size and that all plots have a unique z-value.
    %
    %  obj = INTERPOLATEPLOT(obj1,obj2,obj3,...,z) returns the interpolated
    %  plot. Z defines the z-coordinate of the new plot.
    %
    %  obj = INTERPOLATEPLOT(obj1,obj2,obj3,...,z,method) returns the interpolated
    %  plot. Z defines the z coordinate of the new plot; method defines the
    %  interpolation method.
    %
    % The default interpolation method is 'linear' but with the argument
    % method other interpolation methods can be defined.
    %
    %   'linear'    -   Linear interpolation. Default method.
    %   'spline'    -   Spline interpolation.
    %   'pchip'     -   Piecewise cubic interpolation.
    %
    % See Also: carpetplot.showpoint
    %
            
    %Check if input matrix is the same.
            
        if ischar(varargin{end})
            nPlots = nargin-2;
            zv=varargin{end-1};
        else
            nPlots = nargin-1;
            zv=varargin{end};
        end

        xxx = zeros(size(varargin{1}.inputMatrixX,1),size(varargin{1}.inputMatrixX,2),nPlots);
        yyy = zeros(size(varargin{1}.inputMatrixY,1),size(varargin{1}.inputMatrixY,2),nPlots);
        zzzi = zeros(1,1,nPlots);

        for n= 1: nPlots
            xxx(:,:,n) = varargin{n}.inputMatrixX;
            yyy(:,:,n) = varargin{n}.inputMatrixY;
            zzzi(:,:,n) = varargin{n}.z;
         end

        F=griddedInterpolant({1:size(xxx,1),1:size(xxx,2),zzzi},xxx);
        xxxi= F({1:size(xxx,1),1:size(xxx,2),zv});
        F=griddedInterpolant({1:size(yyy,1),1:size(yyy,2),zzzi},yyy);
        yyyi= F({1:size(yyy,1),1:size(yyy,2),zv});

        outObj = carpetplot(varargin{1}.inputMatrixA,varargin{1}.inputMatrixB,xxxi,yyyi,zv);
        set(outObj,'aTick',get(varargin{1},'atick'),'bTick',get(varargin{1},'btick'));
                        
    end
    
    function out = constraint(obj,constraint,style,varargin)
    % CONSTRAINT adds a constraint.
    % 
    % constraint(obj,const,style) adds a x- or y-constraint to the
    % carpet plot. Const should be an equation containing 'x' and/or 'y'.
    % constraint must be a string. (e.g. 'x > 5*y')
    %
    % For the style argument there are three different options.
    %
    %   'fill'          -   The restricted area will be filled in grey color.
    %   'hatchedfill'   -   The restricted area will be filled with hatched lines.
    %   'hatchedline'   -   A hatched line will be drawn. The orientation of the 
    %                       hatched lines has to be set manually so far.
    %
    % Example:
    %
    %   hold on;
    %   a =[1;2;3;1;2;3];
    %   b =[10;10;10;30;30;30];
    %   x = b-a.^3;
    %   y = a.*b;
    %
    %   plotObject = carpetplot(a,b,x,y);
    %   constraint(plotObject,'y<60 ','hatchedline');
    %
    %
% 
% % % Change the curve Fitting and style
% % set(plotObject,'curvefitting','pchip','style','standard','blabelspacing',0.2,'barrowspacing',0.2);
% 
% % Add the contourf
% hold on;
% contourf(plotObject,1:0.1:3,10:1:30,peaks(21));
% 
% % Add some Constraints
% const = constraint(plotObject,'y<60 ','fill',[0.3 0.3 0.3]);
    %
    %
    
        if ishold == 0
          obj.holding = 0;
          hold on
        else
          obj.holding = 1;
        end
    
        if nargin < 3
            style = 'hatchedline';
            varargin = {'r-',45};
        end
    
        
   
        switch style
            case 'fill'
                
                [~,maskPlot] = getConstrMask(obj,constraint);
        
                [aaa,bbb] = meshgrid(obj.axis{1}.interval,obj.axis{2}.interval);
                xxx = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixX,aaa,bbb,obj.dataFitting);
                yyy = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixY,aaa,bbb,obj.dataFitting);
                nxqq = min(xxx(:)):(max(xxx(:))-min(xxx(:)))/obj.CONTOUR_RESOLUTION:(max(xxx(:)));
                nyqq = min(yyy(:)):(max(yyy(:))-min(yyy(:)))/obj.CONTOUR_RESOLUTION:(max(yyy(:)));
                
                if all(all(maskPlot == 0))
                    warning('Constraint is out of bound')
                else
                    if isempty(varargin)
                        varargin = {[0.5 0.5 0.5]};
                    end
                    
                    cont = contourc(nxqq,nyqq,maskPlot,[1 1]);
                    out = patch(cont(1,2:end),cont(2,2:end),varargin{:});
                    
                    set(out,'zData',get(out,'yData')*0+0.5);
                    set(out,'LineStyle','none');                    
                end
            case 'hatchedline'
               
                
                
                x = linspace(min(obj.plotDataX(:)),max(obj.plotDataX(:)),50);
                
                constraint = strrep(constraint,'>','==');
                constraint = strrep(constraint,'<','==');
                
                eq = solve(constraint,'y');
                y = eval(eq);
                if isscalar(y)
                    y = ones(1,50).*y;
                end
                
                out = hatchedline(x,y,varargin{:});
                
                for n=1:size(out(:))
                    set(out(n),'zData',get(out(n),'xData').*0+2.1)
                    
                end
                
                % Alternative use of hatchedcontours
%                 if all(all(maskFull == 0)) || all(all(maskFull == 1))
%                     warning('Constraint is out of bound')
%                 else
%                     c=contour(nxqq,nyqq,maskFull,[1 1]);
%                     if ~isempty(varargin)
%                         h = hatchedcontours(c,'b-',degtorad(varargin{1}));
%                         set(h(1),'LineWidth',1.5);
%                     else
%                         h = hatchedcontours(c);
%                     end
%                     out = h;
%                 end    
            case 'hatchedfill'
                
                [~,maskPlot] = getConstrMask(obj,constraint);
        
                [aaa,bbb] = meshgrid(obj.axis{1}.interval,obj.axis{2}.interval);
                xxx = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixX,aaa,bbb,obj.dataFitting);
                yyy = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixY,aaa,bbb,obj.dataFitting);
                nxqq = min(xxx(:)):(max(xxx(:))-min(xxx(:)))/obj.CONTOUR_RESOLUTION:(max(xxx(:)));
                nyqq = min(yyy(:)):(max(yyy(:))-min(yyy(:)))/obj.CONTOUR_RESOLUTION:(max(yyy(:)));
                
                if all(all(maskPlot == 0))
                    warning('Constraint is out of bound')
                else
                    [~,h2] = contourf(nxqq,nyqq,maskPlot,[1 1]); 
                    set(h2,'linestyle','none');
                    hp = findobj(h2,'type','patch');
                    h = hatchfill(hp(end),varargin{:});
                end
                out = h;
%             case 'fade'
%                 if all(all(maskPlot == 0))
%                     warning('Constraint is out of bound')
%                 else
%                     h = fadeline(nxqq,nyqq,maskFull);
%                 end
%                 out = h;
%                 obj.constraints{end+1} = h(:)';    
            otherwise
                error('Unknown Style: Try fill, hatchedfill or hatchedline')
    
        end
        
        % Restore Hold Functionality        
            if obj.holding == 0
                hold off
            end
        
    end
    
    function [x y] = abtoxy(obj,a,b)
    % XYTOAB Transforms XY coordinates into the coordinate system of the
    % carpet. 
    %
    % XYTOAB(obj,A,B) converts the given a and b values to the 
    % coordinate system of the carpet plot. Note that values that are out of the range of the
    % carpet plot will not be calculated.
    %


             %Clear Out of Range Values
             a(a>max(obj.axis{1}.interval)) = NaN;
             a(a<min(obj.axis{1}.interval)) = NaN;
             b(b>max(obj.axis{2}.interval)) = NaN;
             b(b<min(obj.axis{2}.interval)) = NaN;
            
            %Transformate
            [x, y] = obj.transformtoxy(a,b,'linear');
    end
    
    function ret = plot(obj,a,b,varargin)
    % PLOT plots a line into the carpet plot.
    % Plot a 2d-line into the carpet plot.
    %
    % PLOT(obj,X,Y,varargin) the plot method is overloaded so it is possible 
    % to plot into the a/b axis by using the matlab plot command. Varargin 
    % will be passed to the matab plot function.
    %
    % See also: carpetplot.hatchedline, carpetplot.contourf
    %

            [x,y] = obj.abtoxy(a,b);
            ret = plot(x,y,varargin{:});
    end
    
    function ret = hatchedline(obj,a,b,varargin)
    % HATCHEDLINE plots a hatched line into the carpet plot.
    % This function is called the HATCHEDLINE function by Rob McDonald; see his
    % documentation for further details on how to use it.
    %
    % HATCHEDLINE(obj,X,Y,varargin) the HATCHEDLINE method is overloaded so it is possible 
    % to plot into the a/b axis. Varargin will be passed to the HATCHEDLINE
    % function.
    %
    % See also: hatchedline, carpetplot.contourf, carpetplot.plot
    %
            a = a(:);
            b = b(:);
            
            [x,y] = obj.abtoxy(a,b);
            
            x = x(~any(isnan(x),2),:);
            y = y(~any(isnan(y),2),:);
            
            %Plot
            if ~isempty(x) && ~isempty(y)
                ret = hatchedline(x',y',varargin{:});
            else
                warning('Data is out of Range')
                ret = []; 
            end            
    end
    
    
   

    function legend(varargin)
    % LEGEND plots legends for one or multiple carpet plots.
    %  
    % Legend([obj1 obj2 ...],'Name1','Name2',...) The syntax is similar to
    % matlab's legend() function. Altough it is not possibly to make a
    % legend of carpet plots and other graphic objects so far.
    %    
        
    %hb = [obj.alinehandles(:) ; obj.blinehandles(:)];
        
        if nargin > 1
            labels = cell(1,size(varargin{1}(:),1));
            for n=1:size(varargin{1}(:),1)
                labels{n} = 'carpetplot';
            end
            for n=2:nargin
                labels{n-1} = varargin{n};
            end
        end
        
        handles = zeros(1,size(varargin{1}(:),1));
        for n=1:size(varargin{1}(:),1)
            if ishandle(varargin{n})
                handles(n) = varargin{n};
                set(handles(n),'Displayname',labels{n});
            else
                hg = hggroup;
                hb = [varargin{1}(n).lines(:); varargin{1}(n).lines(:)];
                set(hb,'Parent',hg);
                set(hg,'Displayname',labels{n})
                handles(n) = hg;
            end
        end
        
        legend(handles);
    
    end
    
    
    
    function reset(obj)
    % RESET manual changes made with the plot tool box. 
    % 
    % reset(obj) redraws the plot and updates the positions and rotations
    % of the labels. All changes made manually or trough handles will be
    % lost and the current style will be restored.
    %
    % See also: carpetplot.refresh
        obj.refreshplot;
        obj.plabel(1);
        obj.plabel(2);  
    end
    
    
    
     function [outArrow, outText] = cheaterlegend(obj,varargin)
    % CHEATERLEGEND Add arrows to a cheater plot to indicate the x-axis
    % plotting intervals. This only works with uniformely spaced data.
    % 
    % [hA hT] = cheaterlegend(obj) places the arrows on the top left of the
    % figure. The function hA returns the arrow handles, hT the text
    % handles.
    %
    % [hA hT] = cheaterlegend(obj,position) with the positon argument the
    % legend can be positioned. The positions can be:
    %
    %   'NorthWest'     top left (default)
    %   'NorthEast'     top right
    %   'SouthWest'     bottom left
    %   'SouthEast'     bottom right
    %
        if obj.type == 3
            
            spaceA = abs((min(obj.axis{1}.interval(:)) - max(obj.axis{1}.interval(:)))  / (size(obj.axis{1}.interval(:),1)-1));
            spaceB = abs((min(obj.axis{2}.interval(:)) - max(obj.axis{2}.interval(:)))  / (size(obj.axis{2}.interval(:),1)-1));
            arrowLengthA = spaceA * obj.k1;
            arrowLengthB = spaceB * obj.k2;
            xlimits = xlim; ylimits = ylim;
            
            if abs(arrowLengthA) > abs(arrowLengthB)
                length = abs(arrowLengthA);
            else
                length = abs(arrowLengthB);
            end
            
            ticks = get(obj.ca,'xtick');
            
            if nargin > 1
                if ischar(varargin{1})
                    in = lower(varargin{1});
                    
                    switch in
                        case 'northeast'
                            startPos(1) = xlimits(2) - length * 1.3;
                            startPos(2) = ylimits(2) - ((ylimits(2) - ylimits(1)) .* 0.1);
                        case 'southeast'
                            startPos(1) = xlimits(2) - length * 1.3;
                            startPos(2) = ylimits(1) + ((ylimits(2) - ylimits(1)) .* 0.2);
                        case 'southwest'
                            startPos(1) = xlimits(1) + length * 1.3;
                            startPos(2) = ylimits(1) + ((ylimits(2) - ylimits(1)) .* 0.2);
                        otherwise
                            startPos(1) = xlimits(1) + length * 1.3;
                            startPos(2) = ylimits(2) - ((ylimits(2) - ylimits(1)) .* 0.1);
                    end
                    
                else
                    error('Input must be a string.')
                end
            else
                startPos(1) = xlimits(1) + length * 1.3;
                startPos(2) = ylimits(2) - ((ylimits(2) - ylimits(1)) .* 0.1);
            end
                   
            ticks(ticks>startPos(1)) = [];
            startPos(1) = ticks(end);
            
            distance = (ylimits(2) - ylimits(1)) / 10;
            
            if arrowLengthA > 0 
                startA(1) = startPos(1); startA(2) = startPos(2); 
                endA(1) = startPos(1) + abs(arrowLengthA); endA(2) = startPos(2);
            else
                startA(1) = startPos(1) + abs(arrowLengthA); startA(2) = startPos(2); 
                endA(1) = startPos(1); endA(2) = startPos(2);
            end
            
            if arrowLengthB > 0 
                startB(1) = startPos(1); startB(2) = startPos(2)-distance; 
                endB(1) = startPos(1) + abs(arrowLengthB); endB(2) = startPos(2)-distance;
            else
                startB(1) = startPos(1) + abs(arrowLengthB); startB(2) = startPos(2)-distance; 
                endB(1) = startPos(1); endB(2) = startPos(2)-distance;
            end
            
            textPos = (startB + endB)/2 + [0 distance*0.1];
            
            outText(2) = text(textPos(1),textPos(2),[obj.axis{2}.label  '=' num2str(spaceB)]);
            set(outText(2),'VerticalAlignment','bottom','HorizontalAlignment','center','fontWeight','bold');
            
            textPos = (startA + endA)/2 + [0 distance*0.1];
            
            outText(1) = text(textPos(1),textPos(2),[obj.axis{1}.label '=' num2str(spaceA)]);
            set(outText(1),'VerticalAlignment','bottom','HorizontalAlignment','center','fontWeight','bold');
                        
            outArrow(1) = arrow(startA,endA,'BaseAngle',20,'TipAngle',15,'Length',10);
            outArrow(2) = arrow(startB,endB,'BaseAngle',20,'TipAngle',15,'Length',10);
            
        else
            warning('A cheater legend can only be applied to cheater plots')
        end
    end
    
end

methods (Access = private)
    
    function plabel(obj,nAxis,varargin)
        %obj.lhLabelsFigure.Enabled = 'off';
        %obj.lhLabelsAxis.Enabled = 'off';
       
        % Keep Hold Functionality
        if ishold == 0
          obj.holding = 0;
          hold on
        else
          obj.holding = 1;
        end
        
        [a,b] = meshgrid(obj.axis{1}.interval,obj.axis{2}.interval);
        % If matrix contains NaNs --> interpolate
        [pDataX,pDataY] = obj.getpData;
        % Flip the matrix for the second axis
        if nAxis == 2
            pDataX = pDataX';
            pDataY = pDataY';
            pData = b';
        else
            pData = a;
        end

        
        % Delete the arrow and the label
        obj.deleteHandle(obj.axis{nAxis}.labelHandle)
        obj.deleteHandle(obj.axis{nAxis}.arrowHandle)
        
        % Add the new arrow (temporary position)
        obj.axis{nAxis}.arrowHandle = arrow([pDataX(1,1) pDataY(1,1)],[pDataX(1,end) pDataY(1,end)],obj.axis{nAxis}.arrowSpec{:});
        
        % Add the new label (temporary position)
        obj.axis{nAxis}.labelHandle = text(pDataX(1,1),pDataY(1,1),obj.axis{nAxis}.label);
        set(obj.axis{nAxis}.labelHandle,obj.axis{nAxis}.labelSpec{:});
        
        for n = 1 : size(obj.axis{nAxis}.textHandles(:),1)
            obj.deleteHandle(obj.axis{nAxis}.textHandles(n));
        end
        obj.axis{nAxis}.textHandles = [];
        
        % Draw the values
        for n = 1 : size(pDataX,2)
            obj.axis{nAxis}.textHandles(n) = text(pDataX(1,n),pDataY(1,n),[obj.axis{nAxis}.preText num2str(pData(1,n)) obj.axis{nAxis}.postText]);
            set(obj.axis{nAxis}.textHandles(n),obj.axis{nAxis}.textSpec{:});
        end
    
        % Restore hold functionality        
        if obj.holding == 0
            hold off
        end
        
        obj.refreshlabels('position')
        
        % Set a resizeFcn but keep previosly plotted carpets
%         ResizeFcnStr = get(obj.cf,'ResizeFcn');
%         if isempty(obj.instanceName)
%             warning('The label rotation will not refresh automatically when resizing the figure window. Use obj.refresh to do it manually or don''t use an expression for the object''s name like o(1) or varargin{3} etc...')
%         elseif isempty(ResizeFcnStr)
%             set(obj.cf,'ResizeFcn',['carpetplot.refreshmultiplelabels(' obj.instanceName ')']);
%         elseif isempty(strfind(ResizeFcnStr,obj.instanceName))            
%             newResizeFcnStr = strrep(ResizeFcnStr,'carpetplot.refreshmultiplelabels(','');
%             newResizeFcnStr = strrep(newResizeFcnStr,')','');
%             set(obj.cf,'ResizeFcn',['carpetplot.refreshmultiplelabels(' newResizeFcnStr ',' obj.instanceName ')']);
%         end
        if isempty(obj.listener)
            obj.listener = addlistener(obj.ca,'TightInset','PostSet',@obj.refreshlabels);
            obj.listenerX = addlistener(obj.ca,'XDir','PostSet',@obj.refreshlabels);
            obj.listenerY = addlistener(obj.ca,'YDir','PostSet',@obj.refreshlabels);
            obj.listenerLogX = addlistener(obj.ca,'XScale','PostSet',@obj.refreshlabels);
            obj.listenerLogY = addlistener(obj.ca,'YScale','PostSet',@obj.refreshlabels);
        end
        %addlistener(obj.cf,'Position','PostSet',@obj.refreshlabels);
        %addlistener(obj.ca,'OuterPosition','PostSet',@obj.refreshlabels);
        
    end
    
    function refreshlabels(obj,varargin)
        
        % If figure handle exists --> Activate figure
        if ~isempty(obj.cf) && ishandle(obj.cf)
                figure(obj.cf);
        else
            return;
        end
        
        % If axes exists --> Activate (make current axes)
        if ~isempty(obj.ca) && ishandle(obj.ca)
            axes(obj.ca);
        else
            return;
        end
        
        if ~isempty(obj.pzlabelandle) && ishandle(obj.pzlabelandle)
            obj.zlabel(get(obj.pzlabelandle,'string'),obj.pZAlignement)
        end
        
        % Loop a- and b-axis
        for nAxis=1:2

            % Ratio of the axis
            ratio = daspect;

            % Get the size of the figure and multiply with the size of the
            % subplot. Important for the calculation of the text rotation.
            SizeFigure = get(obj.cf, 'Position');
            ratioFigure = get(obj.ca, 'Position');
            SizeFigure = SizeFigure.*ratioFigure;
            ratio = ratio(1)/ratio(2)*(SizeFigure(4)/SizeFigure(3));
            
            % Check for reversed Axes
            reverseX = get(obj.ca,'XDir');
            reverseY = get(obj.ca,'YDir');
            if strcmp(reverseX,'reverse')
                rotErrorX = -1;
            else
                rotErrorX = 1;
            end
            if strcmp(reverseY,'reverse')
                rotErrorY = -1;
            else
                rotErrorY = 1;
            end
            
            % If the plot contains Nans it is important to extrapolate a
            % carpet.
            [pDataX,pDataY] = obj.getpData;
            
            % Flip the data for the b-axis.
            if nAxis == 2
                pDataX = pDataX';
                pDataY = pDataY';
    %             pDataB = b';
            else
    %             pDataA = a;
            end

            % Start end end index of plot matrix for the arrow.
            if ~obj.axis{nAxis}.arrowFlipped
                pStart = 1; pEnd = size(pDataX,1);
            else
                pStart = size(pDataX,1); pEnd = 1;
            end
            
            % Assign the new coordinates to the arrow .
            if ~isempty(obj.axis{nAxis}.arrowHandle) && ishandle(obj.axis{nAxis}.arrowHandle)
                % Get the position vector. Distance arrow to plot.
                middle1 = ([pDataX(pStart,1) pDataY(pStart,1)]+[pDataX(pStart,end) pDataY(pStart,end)])/2;
                middle2 = ([pDataX(pEnd,1) pDataY(pEnd,1)]+[pDataX(pEnd,end) pDataY(pEnd,end)])/2;
                posVector = (middle1-middle2)*obj.axis{nAxis}.arrowSpacing;
                aStart = [pDataX(pStart,1)+posVector(1) pDataY(pStart,1)+posVector(2)];
                aEnd =  [pDataX(pStart,end)+posVector(1) pDataY(pStart,end)+posVector(2)];                
                if nargin > 1 && ischar(varargin{1}) && strcmp(varargin{1},'position') 
                    obj.axis{nAxis}.arrowHandle = arrow(obj.axis{nAxis}.arrowHandle,'start',aStart,'stop',aEnd);
                end
            end

            % Start end end index of plot matrix for the arrow.
            if ~obj.axis{nAxis}.labelFlipped
                pStart = 1; pEnd = size(pDataX,1);
            else
                pStart = size(pDataX,1); pEnd = 1;
            end
            
            % Set position and rotation of the axis label.
            if ~isempty(obj.axis{nAxis}.labelHandle) && ishandle(obj.axis{nAxis}.labelHandle)
                middle1 = ([pDataX(pStart,1) pDataY(pStart,1)]+[pDataX(pStart,end) pDataY(pStart,end)])/2;
                middle2 = ([pDataX(pEnd,1) pDataY(pEnd,1)]+[pDataX(pEnd,end) pDataY(pEnd,end)])/2;
                posVector = (middle1-middle2)*obj.axis{nAxis}.labelSpacing;
                rotation = atan((pDataY(pStart,end)-pDataY(pStart,1))*rotErrorY*ratio/(rotErrorX*(pDataX(pStart,end)-pDataX(pStart,1))))*180/pi;
                set(obj.axis{nAxis}.labelHandle,'rotation',rotation);
                if nargin > 1 && ischar(varargin{1}) && strcmp(varargin{1},'position')
                    set(obj.axis{nAxis}.labelHandle,'position',[middle1(1)+posVector(1) middle1(2)+posVector(2) 0]);
                end
            end
            


            
            % Start end end index of plot matrix for the arrow.
            if ~obj.axis{nAxis}.textFlipped
                pStart = 2; pEnd = 1; 
            else
                pStart = size(pDataX,1)-1; pEnd = size(pDataX,1); 
            end
            
            % Loop the text labels.
            for n = 1 : size(pDataX,2)

                if ~isempty(obj.axis{nAxis}.textHandles) && ~isempty(obj.axis{nAxis}.textHandles(n)) && ishandle(obj.axis{nAxis}.textHandles(n))

                    % Set position and rotation .              
                    posVector = [pDataX(pEnd,n)-pDataX(pStart,n) pDataY(pEnd,n)-pDataY(pStart,n)];
                    posVector(1) = posVector(1)*rotErrorX;
                    posVector(2) = posVector(2)*rotErrorY;
                    rotation = atan(((pDataY(pStart,n)-pDataY(pEnd,n))*ratio*rotErrorY)/(rotErrorX*(pDataX(pStart,n)-pDataX(pEnd,n))))*180/pi;
                    if nargin > 1 && ischar(varargin{1}) && strcmp(varargin{1},'position')
                        set(obj.axis{nAxis}.textHandles(n),'position',[pDataX(pEnd,n) pDataY(pEnd,n) 2]);
                    end
                    
                    % Get text string from label.
                    text = get(obj.axis{nAxis}.textHandles(n),'string');
                    text = strtrim(text);
                    
                    % Assign blanks to the text string according to the
                    % definded textSpacing. Also consider if the text is
                    % flipped or not.
                    
                    
                    if (posVector(1) < 0)                        
                        if (obj.axis{nAxis}.textSpacing > 0)
                            alignment = 'right';
                            outtext = [text blanks(abs(obj.axis{nAxis}.textSpacing))];
                        else
                            alignment = 'left';
                            outtext = [blanks(abs(obj.axis{nAxis}.textSpacing)) text];
                        end                    
                    else
                        if (obj.axis{nAxis}.textSpacing > 0)
                            alignment = 'left';
                            outtext = [blanks(abs(obj.axis{nAxis}.textSpacing)) text];
                        else         
                            alignment = 'right';
                            outtext = [text blanks(abs(obj.axis{nAxis}.textSpacing))];
                        end 
                    end
                    
                    
                    % Set the alignement.
                    set(obj.axis{nAxis}.textHandles(n),'HorizontalAlignment',alignment);
                    
                    % Set the text string.
                    set(obj.axis{nAxis}.textHandles(n),'string',outtext);


                    % Set rotation, text should be readable - not
                    % up-side-down.
                    if obj.axis{nAxis}.textRotation
                        if (rotation >= 90) || (rotation <= -90)
                            rotation = rotation+180;
           
                        end
                        set(obj.axis{nAxis}.textHandles(n),'rotation',rotation);
                    else
                        set(obj.axis{nAxis}.textHandles(n),'rotation',0);
                    end
                    
                    % Update style if neccessary.
                    if nargin > 1 && ischar(varargin{1}) && strcmp(varargin{1},'style')                 
                        if ~isempty(obj.axis{nAxis}.arrowHandle) && ishandle(obj.axis{nAxis}.arrowHandle)
                            obj.axis{nAxis}.arrowHandle = arrow(obj.axis{nAxis}.arrowHandle,obj.axis{nAxis}.arrowSpec{:});
                        end
                        if ~isempty(obj.axis{nAxis}.labelHandle) && ishandle(obj.axis{nAxis}.labelHandle)
                            set(obj.axis{nAxis}.labelHandle,obj.axis{nAxis}.labelSpec{:});
                        end
                        if ~isempty(obj.axis{nAxis}.textHandles(n)) && ishandle(obj.axis{nAxis}.textHandles(n))
                            set(obj.axis{nAxis}.textHandles(n),obj.axis{nAxis}.textSpec{:});
                        end
                        obj.needTextStyleRefresh = 0;
                    end                
                end            
            end
        end
        
    end
    
    function settick(obj,value,axis)
       
        
        
        % Asign the new interval.
        obj.axis{axis}.interval = value;
        obj.needPlotRefresh = 1;           
        % Calculate the x matrix. For cheater plots only.
        if (obj.type == 3)
            stepA = (max(obj.axis{1}.interval(:))-min(obj.axis{1}.interval(:)))/(size(obj.axis{1}.interval(:),1)-1);  
            stepB = (max(obj.axis{2}.interval(:))-min(obj.axis{2}.interval(:)))/(size(obj.axis{2}.interval(:),1)-1);
            if obj.pK2 < 0
                obj.pK2 = -stepA/stepB;
            else
                obj.pK2 = stepA/stepB;
            end
            if obj.pK1 < 0
                obj.pK1 = -1;
            else
                obj.pK1 = 1;
            end
            obj.inputMatrixX = obj.pK0 + obj.pK1.*obj.inputMatrixA+obj.pK2.*obj.inputMatrixB;
            %obj.refreshplot;
        end
            
            if ~isempty(obj.axis{axis}.textHandles)
                % Delete all text handles.
                for n = 1 : size(obj.axis{axis}.textHandles(:),1)
                    obj.deleteHandle(obj.axis{axis}.textHandles(n));
                end
                obj.axis{axis}.textHandles = [];
                obj.refreshplot;
                if axis == 1
                    obj.plabel(1);
                else
                    obj.plabel(2);
                end
            end
     
    end
    
    function refreshplot(obj)   
        % Delete the old line handles.
        for i = 1:2
            for n = 1:size(obj.axis{i}.lineHandles(:),1)
                obj.deleteHandle(obj.axis{i}.lineHandles(n));                
            end
            if ~isempty(obj.axis{i}.extrapLineHandles)
                for n = 1:size(obj.axis{i}.extrapLineHandles(:),1)
                    obj.deleteHandle(obj.axis{i}.extrapLineHandles(n));                
                end
            end
            
            for n = 1:size(obj.axis{i}.intersectionHandles(:),1)
                obj.deleteHandle(obj.axis{i}.intersectionHandles(n));
            end
            obj.axis{i}.lineHandles = [];
            obj.axis{i}.intersectionHandles = [];
        end
        
        % Independent of the hold status --> hold on
        if ishold == 0          
          obj.holding = 0;
          hold on
        else
          obj.holding = 1;
        end
        
        obj.keepTicks = 0;
        
        % Replot the carpet
        obj.cplot()
        obj.keepTicks = 0;
        
        % Restore hold functionality.
        if obj.holding == 0
            hold off
        end
        
        %obj.needPlotRefresh
        obj.needPlotRefresh = 0;
    end
    
    
    
    function newax = holdon(obj)
        % Keep Hold On Hold Off funcionality            
        if ishold == 0
          clf(gcf);
          obj.plotholding = 0;
          newax = 1;
          hold on;
        else
          obj.plotholding = 1;
          if isempty(get(gcf,'CurrentAxes'))
            newax = 1;  
          else
            newax=0;
          end          
        end
        
    end
    
    function deleteHandle(~,handle)
        if ~isempty(handle) && ishandle(handle) && handle ~= 0;
            delete(handle);
        end
    end
    
    function holdoff(obj)        
        if obj.plotholding == 0
            hold off
        end       
    end
    
    function [x y] = transformtoxy(obj,A,B,force)
        if nargin == 4
            fitting = force;
        else
            fitting = obj.dataFitting;
        end
        x = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixX,A,B,fitting);
        y = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixY,A,B,fitting);
    end
    
    function ret = checkXYPoints(~,X,Y) 
        checkedPoints = X .* Y;
        checkedPoints(~isfinite(checkedPoints)) = [];
        if size(checkedPoints(:),1) > 1 
            ret = 1;
        else
            ret = 0;
        end
    end
    
    function [maskFull maskPlot] = getConstrMask(obj,constFunc)
        % This function returns a boolean matrix that represents the given
        % function or just the edges of the carpet plot.
        [aaa,bbb] = meshgrid(obj.axis{1}.interval,obj.axis{2}.interval);
        
        [xxx,yyy] = obj.transformtoxy(aaa,bbb);
      
        nxqq = min(xxx(:)):(max(xxx(:))-min(xxx(:)))/obj.CONTOUR_RESOLUTION:(max(xxx(:)));
        nyqq = min(yyy(:)):(max(yyy(:))-min(yyy(:)))/obj.CONTOUR_RESOLUTION:(max(yyy(:)));
        

        [xqq,yqq] = meshgrid(nxqq,nyqq);
        
        a = griddata(xxx,yyy,aaa,xqq,yqq);
        b = griddata(xxx,yyy,bbb,xqq,yqq);
        
            
         a(abs(a-max(obj.axis{1}.interval))<(max(obj.axis{1}.interval)-min(obj.axis{1}.interval))/obj.CONTOUR_RESOLUTION) = NaN;
         a(abs(a-min(obj.axis{1}.interval))<(max(obj.axis{1}.interval)-min(obj.axis{1}.interval))/obj.CONTOUR_RESOLUTION) = NaN;
         b(abs(b-max(obj.axis{2}.interval))<(max(obj.axis{2}.interval)-min(obj.axis{2}.interval))/obj.CONTOUR_RESOLUTION) = NaN;
         b(abs(b-min(obj.axis{2}.interval))<(max(obj.axis{2}.interval)-min(obj.axis{2}.interval))/obj.CONTOUR_RESOLUTION) = NaN;
         
        aMask = double(isfinite(a));
        aMask(aMask==0) = NaN;
        bMask = double(isfinite(b));
        bMask(bMask==0) = NaN;
        
        a = a.*bMask;
        %b = b.*aMask;
        
        if ischar(constFunc)
            try
                constFunc = str2func(['@(x,y)',constFunc]); 
            catch m1
                error('ERROR: %s is no valid function. Try something like x>3*y',constFunc);
            end 
            ineq2 = ~arrayfun(constFunc,xqq,yqq);
            ineq1 = ineq2 & (isfinite(a));
            hold on;
            ineq1 = double(ineq1);
            %ineq1(ineq1 == 1) = inf;
            maskPlot = ineq1;
            maskFull = ineq2;
        
        else
            maskPlot = aMask.*bMask;
            maskFull = 0;
        end    
        
    end
      
    
    function [X,Y,dataX,dataY] = interpAB(obj,inA,inB)
        if strcmp(obj.curvefitting,'elinear') || ...
                strcmp(obj.curvefitting,'epchip') || ...
                strcmp(obj.curvefitting,'espline') %%% 2013-05-21/Sartorius: added || and extra line break
            [aaa,bbb] = meshgrid(inA,linspace(min(obj.axis{2}.interval),max(obj.axis{2}.interval),100));
        else
            [aaa,bbb] = meshgrid(inA,[obj.axis{2}.interval]);
        end
        
        X = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixX,inA,inB,obj.dataFitting);
        Y = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixY,inA,inB,obj.dataFitting);
        
        dataX = [{} {}];
        dataY = [{} {}];
        
        for n = 1:2
            xxx = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixX,aaa,bbb,obj.dataFitting);
            yyy = interp2(obj.inputMatrixA,obj.inputMatrixB,obj.inputMatrixY,aaa,bbb,obj.dataFitting);
            dataX{n} = xxx;
            dataY{n} = yyy;
            
            if strcmp(obj.curvefitting,'elinear') || ...
                    strcmp(obj.curvefitting,'epchip') || ...
                    strcmp(obj.curvefitting,'espline') %%% 2013-05-21/Sartorius: added || and extra line break
                [aaa,bbb] = meshgrid(linspace(min(obj.axis{1}.interval),max(obj.axis{1}.interval),100),inB);
            else
                [aaa,bbb] = meshgrid(obj.axis{1}.interval,inB);
            end
        end     
    end
      
    function [hLine,hPoint,hText] = drawinterpolation(obj,X, Y, dataX, dataY,lineStyle)
          
          markerStyle = obj.interpMarkerStyle;
          textStyle = obj.interpTextStyle;
        
          hPoint = plot3(X,Y,2,'X',markerStyle{:});
          
          if (obj.type == 3)
            hLine = zeros(3,1);
          else
            hLine = zeros(4,1);
          end
          
            switch obj.pCurveFitting
                    
                 case 'spline'
                     
                     for n = 1:2
                         interpSpline = spline(dataX{n},dataY{n});
                         xx = linspace(min(dataX{n}),max(dataX{n}),30);
                         hLine(n) = plot3(xx,ppval(interpSpline,xx),xx.*0+2,lineStyle{:});
                     end
                    
                 case 'pchip' 
                     for n = 1:2
                         interpPchip = spline(dataX{n},dataY{n});
                         xx = linspace(min(dataX{n}),max(dataX{n}),30);
                         hLine(n) = plot3(xx,ppval(interpPchip,xx),xx.*0+2,lineStyle{:});
                     end
                     
                otherwise
                     for n = 1:2
                        hLine(n) = plot3(dataX{n},dataY{n},dataX{n}.*0+2,lineStyle{:});
                     end
             end

        yLimits = ylim;
        xLimits = xlim;
        
        hText = text(xLimits(1),Y,[' ' num2str(Y)],textStyle{:});
        hLine(3) = plot3([min(xLimits(1)) X],[Y Y],[2 2],lineStyle{:});
        
        if ~(obj.type == 3)
            hText(2) = text(X,yLimits(1),[' ' num2str(X)],textStyle{:});
            hLine(4) = plot3([X X],[Y min(yLimits(1))],[2 2],lineStyle{:});
            set(hText(2),'rotation',90);
        end          
    end
    
    function [pDataX pDataY] = getpData(obj)
        % Get plot data. Interpolate if it contains Nans.
        
        if (sum(isnan(obj.plotDataX(:))) > 0) || (sum(isnan(obj.plotDataY(:))) > 0)
%          [a,b] = meshgrid(obj.axis{1}.interval,obj.axis{2}.interval);
%          [pDataX,pDataY] = obj.transformtoxy(a,b,'spline');
           
           pDataX = obj.plotDataX;
           pDataY = obj.plotDataY;
          
           
           pDataX( :, all(isnan(pDataX), 1)) = []; pDataY( :, all(isnan(pDataY), 1)) = [];
           pDataX(all(isnan(pDataX), 2), :) = []; pDataY(all(isnan(pDataY), 2), :) = [];

           reSized=interp1(pDataX,linspace(1,size(pDataX,1),size(pDataX,1)),'spline');
           reSized=interp1(reSized.',linspace(1,size(pDataX,2),size(pDataX,2)),'spline').';
           pDataX = reSized;

           reSized=interp1(pDataY,linspace(1,size(pDataY,1),size(pDataY,1)),'spline');
           reSized=interp1(reSized.',linspace(1,size(pDataY,2),size(pDataY,2)),'spline').';
           pDataY = reSized;

        else
            pDataX = obj.plotDataX;
            pDataY = obj.plotDataY;
        end
    end
    

end
     
methods(Static)
    
    function refreshmultiplelabels(varargin)
        
        for n = 1 : size(varargin,2)
                if isobject(varargin{n})
                   varargin{n}.refresh('textrotation');
                end
        end
        
    end
    
end

end
