% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef DriveBotSLAMSystem < minislam.slam.SLAMSystem
    
    properties(Access = public, Constant)
        % Platform state dimension
        NP = 3;
        
        % Landmark dimension
        NL = 2;
        
        % Initial cache size; might help a bit with performance
        INITIAL_CACHE_SIZE = 10000;
      
        
    end
    
    properties(Access = protected)
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId;
        
        % The set of all prediction edges. These are removed from the graph
        % afterwards if we don't use prediction
        processModelEdges;
        numProcessModelEdges;
        
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarkIDStateVectorMap;
        
        % How often we recommend running the optimization
        recommendOptimizationPeriod;
        
        % Flag to show if we should prune the edges. This is needed for
        % question Q3a
        removePredictionEdgesFromGraph;
        keepFirstPredictionEdge;
        
        % Q3b -- attribute that when set to be true calls for graph pruning
        % to take place 
        graphPrune = false;
        
    end
    
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = DriveBotSLAMSystem(configuration)
            
            % Call the base class constructor
            this = this@minislam.slam.SLAMSystem(configuration);
            
            % Preallocate for convenience
            this.vehicleVertices = cell(1, this.INITIAL_CACHE_SIZE);
            
            % No vehicle vertices initally set
            this.vehicleVertexId = 0;
            
            % The set of prediction edges, initially empty
            this.processModelEdges = cell(1, this.INITIAL_CACHE_SIZE);
            this.numProcessModelEdges = 0;
            
            % Allocate the landmark map
            this.landmarkIDStateVectorMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
            % By default, run very infrequently
            this.recommendOptimizationPeriod = inf;
            
            this.removePredictionEdgesFromGraph = false;
            this.keepFirstPredictionEdge = false;
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. The logic we have here just recommends
        % an optimization if a fixed number of steps have been completed.
        
        function recommendation = recommendOptimization(this)
            
            % This is how to do it after every 100 steps
            recommendation = rem(this.stepNumber, ...
                this.recommendOptimizationPeriod) == 0;
        end
        
        % Set the value of how often recommend optimization should return
        % true
        function setRecommendOptimizationPeriod(this, newRecommendOptimizationPeriod)
            this.recommendOptimizationPeriod = newRecommendOptimizationPeriod;
        end
        
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = platformEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x = full(xS);
            P = full(PS);
        end
        
        % Returns the entire history of the platform estimates. Suppose
        % there are n vehicle vertices. T is a 1 by N dimensional vector of
        % timesteps. X is a 3 by N dimensional vector of vehicle state (x,
        % y, theta). P is a 3 by N dimensional vector where the nth column
        % are the diagonals from the covariance matrix.
        function [T, X, P] = platformEstimateHistory(this)
            
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            
            % Create the output array
            X = zeros(this.NP, this.vehicleVertexId);
            P = zeros(this.NP, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
                
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this)
            
            landmarkVertices = values(this.landmarkIDStateVectorMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(this.NL, numberOfLandmarks);
            P = NaN(this.NL, this.NL, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
        
        % We overload the optimize method so that you can add additional
        % logic here
        function chi2 = optimize(this, maximumNumberOfOptimizationSteps)
            
            % Remove the prediction edges if requested.
            if (this.removePredictionEdgesFromGraph == true)
                this.deleteVehiclePredictionEdges();
            end
            
            % Now call the actual optimizer. Let it handle the default if
            % no steps are specified.
            if (nargin > 1)
                chi2 = optimize@minislam.slam.SLAMSystem(this, ...
                    maximumNumberOfOptimizationSteps);
            else
                chi2 = optimize@minislam.slam.SLAMSystem(this);
            end
        end
        
        function setRemovePredictionEdges(this, removeEdges, keepFirst)
            this.removePredictionEdgesFromGraph = removeEdges;
            this.keepFirstPredictionEdge = keepFirst;
            
        end
        
        % Q3b -- function to set the attribute of graph pruning 
        function setGraphPruning(this, toPruneOrNot)
            this.graphPrune = toPruneOrNot;
        end 
    end
    
    % These are the methods you will need to overload
    methods(Access = protected)
        
        % Handle the initial condition
        
        function handleInitialConditionEvent(this, event)
            
            % Create the first vertex, set its estimate to the initial
            % value and add it to the graph.
            this.currentVehicleVertex = drivebot.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data);
            this.graph.addVertex(this.currentVehicleVertex);
            
            % Set the book keeping for this initial vertex.
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
            
            % If the covariance is 0, the vertex is known redictiuonerfectly and so
            % we set it as fixed. If the covariance is non-zero, add a
            % unary initial prior condition edge instead. This adds a soft
            % constraint on where the state can be.
            if (det(event.covariance) < 1e-6)
                this.currentVehicleVertex.setFixed(true);
            else
                initialPriorEdge = drivebot.InitialPriorEdge();
                initialPriorEdge.setMeasurement(event.data);
                initialPriorEdge.setInformation(inv(event.covariance));
                initialPriorEdge.setVertex(this.currentVehicleVertex);
                this.graph.addEdge(initialPriorEdge);
            end
        end
        
        function handleNoPrediction(~)
            % Nothing to do
        end
      
        function handleHeartbeatEvent(this, ~)
            % Nothing to do
        end
        
        function handlePredictToTime(this, time, dT)
            
            % Create the next vehicle vertex and add it to the graph

            this.currentVehicleVertex = drivebot.VehicleStateVertex(time);

            %%%%%%%%%% Q1b start %%%%%%%%%%%%:

            % Create the edge
            e = drivebot.VehicleKinematicsEdge(dT);
            e.setVertex(1, this.vehicleVertices{this.vehicleVertexId});
            e.setVertex(2, this.currentVehicleVertex); 
            e.setMeasurement(this.u);
            e.setInformation(inv(this.uCov));
            e.initialize();
            
            % adding edge to graph
            this.graph.addEdge(e);

            % Add the prediciton to the new vertex
            this.currentVehicleVertex.setEstimate(e.vertex(2).estimate());

            % adding the current vehicle vertex to the graph
            this.graph.addVertex(this.currentVehicleVertex);

            this.numProcessModelEdges = this.numProcessModelEdges + 1;
            this.processModelEdges{this.numProcessModelEdges} = e;

            %%%%%%%%%% Q1b ends %%%%%%%%%%%%
            
            %warning('drivebotslam:handlepredicttotime:unimplemented', ...
            %    'Implement the rest of this method for Q1b.');
            
            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end
        
        function handleGPSObservationEvent(this, event)
            
            % Create a GPS measurement edge
            
            %%%%%%%%%% Q1c start %%%%%%%%%%%%
            % Implement prediction code here %%%%%%
            
            % Q1c: %% our code
            omegaR = inv(event.covariance);
            k = this.vehicleVertexId;
            
            % Create the measurement edge
            e = drivebot.GPSMeasurementEdge();

            % Link it so that it connects to the vertex we want to estimate
            e.setVertex(1, this.vehicleVertices{k});

            % Set the measurement value and the measurement covariance
            e.setMeasurement(event.data);
            e.setInformation(omegaR);
  
            % Add the edge to the graph
            this.graph.addEdge(e);
             
            %%%%%%%%%% Q1c end %%%%%%%%%%%%
            %warning('drivebotslam:handlegpsobservationevent:unimplemented', ...
            %    'Implement the rest of this method for Q1c.');
            
        end
        
        function handleLandmarkObservationEvent(this, event)
            
            %Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                
                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);
    
                %%%%%%%%%% Q2b start %%%%%%%%%%%

                % Add the measurement edge
                landmarkRangeBearingEdge = drivebot.LandmarkRangeBearingEdge();
                landmarkRangeBearingEdge.setVertex(1, this.vehicleVertices{this.vehicleVertexId});
                landmarkRangeBearingEdge.setVertex(2, landmarkVertex);
                landmarkRangeBearingEdge.setMeasurement(z);
                landmarkRangeBearingEdge.setInformation(inv(event.covariance));

                %%%%%%%%%% Q2b ends %%%%%%%%%%%%
                
                %warning('drivebotslamsystem:handlelandmarkobservationevent:unimplemented', ...
                %    'Implement the rest of this method for Q2b.');
                
                if (newVertexCreated == true)
                    landmarkRangeBearingEdge.initialize();
                end
                
                this.graph.addEdge(landmarkRangeBearingEdge);
                
            end
        
        end
        function deleteVehiclePredictionEdges(this)
            
            %%%%%%%%%% Q3a start %%%%%%%%%%%
            
            if this.graphPrune == true
                
                % if we have stated for graph pruning to take place, run
                % the function to do so 
                this.graphPruning()
                
                return  
            end 
                        
            % find the number of edges 
            edges = this.graph.edges();
            numOfEdges = length(edges);

            % initialise the edge count 
            count = 0;
            deletedEdges = 0;

            % loop through all the edges and delete them all 
            for i = 1:numOfEdges
                
                % check if it's a prediction edge 
                if class(edges{i}) == "drivebot.VehicleKinematicsEdge"
                    count = count + 1;

                    % check if we need to keep the first edge 
                    if (this.keepFirstPredictionEdge == true && count == 1)

                        % if we need to keep the first edge, move onto the
                        % next one without deleting the first                         
                        continue 
                        
                    else 
                       % otherwise, remove the current edges 
                        this.graph.removeEdge(edges{i});
                        
                        deletedEdges = deletedEdges + 1;
                        
                    end 
                        
                end  
                
            end 
                  
%             warning('drivebotslam:deletevehiclepredictionedges:unimplemented', ...
%                 'Implement the rest of this method for Q3a.');

            %%%%%%%%%% Q3a ends %%%%%%%%%%%%
            
        end
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)
            
            % If the landmark exists already, return it
            if (isKey(this.landmarkIDStateVectorMap, landmarkId) == true)
                landmarkVertex = this.landmarkIDStateVectorMap(landmarkId);
                newVertexCreated = false;
                return
            end
            
            fprintf('Creating landmark %d\n', landmarkId);
            
            % Create the new landmark add it to the graph
            landmarkVertex = drivebot.LandmarkStateVertex(landmarkId);
            this.landmarkIDStateVectorMap(landmarkId) = landmarkVertex;
            
            this.graph.addVertex(landmarkVertex);
            
            newVertexCreated = true;
        end
        
        function storeStepResults(this)
            % Nothing
        end
        
    
        
        %%%%%%%% Q3b begins %%%%%%%%%%%%%%
        
        % Below we can see the implementation of Graph pruning for SLAM 
        function graphPruning(this)            
            edges = this.graph.edges();
            
            numEdges = length(edges);
            
            vertices = this.graph.vertices();
            
            numVertices = length(vertices);            
            
            obsEdgeCount = 0; 
            
            actualNumVertices = 0; 
            removedEdges = 0;
            
            for i = 1:numVertices % loop through all vertices 
                
                vertex = this.vehicleVertices{i};         
                
                if class(vertex) == "drivebot.VehicleStateVertex"
                    
                    actualNumVertices = actualNumVertices + 1;
                    
                    vertexEdges = vertex.edges();
                    
                    numVertexEdges = length(vertexEdges);
                    
                    for j = 1:numVertexEdges 
                        
                        if class(vertexEdges{j}) == "drivebot.LandmarkRangeBearingEdge"
                            obsEdgeCount = obsEdgeCount + 1;
                            
                        end 
                    end
                end 
                                   
            end
            
            actualNumVertices;
            
            avgObsEdges = round(obsEdgeCount / actualNumVertices); 
         
            for i = 1:numVertices 
                obsEdgeCount = 0;      

                vertex = this.vehicleVertices{i};
                if class(vertex) == "drivebot.VehicleStateVertex"
                    
                    % find cell of all edges for this vertex 
                    vertexEdges = vertex.edges();
                    numVertexEdges = length(vertexEdges);
                        
                    % loop through all edges and count the number of
                    % observation edges 
                     for j = 1:numVertexEdges 
                        thisEdge = vertexEdges{j};

                        if class(thisEdge) == "drivebot.LandmarkRangeBearingEdge"
                            
                            obsEdgeCount = obsEdgeCount + 1; 
                       
                        end 
                     end
                     
                     if obsEdgeCount < avgObsEdges && (i ~= 1) && (i ~=2)
                         
                         for k = 1:numVertexEdges 
                            thisEdge = vertexEdges{k};
                            this.graph.removeEdge(thisEdge);

                            % increment number of removed edges 
                            removedEdges = removedEdges + 1;

                        end 
                         
                         
                     end 
                     
                end 
            end
            
        removedEdges 
        length(this.graph.edges())
            
            
        end 
             
        
        function graphPruningOld(this)
            
            
            % get all the edges in the graph 
            edges = this.graph.edges();
            numEdges = length(edges);
            disp('Initial number of edges: ')
            numEdges

%             numEdges = this.graph.numedges(this);
            
            % get all the vertices in the graph
            vertices = this.graph.vertices();
            numVertices = length(vertices);
            
            % initialise count for total nmber of observation edges 
            obsEdgeCount = 0; 
            
            actualNumVertices = 0;
            
            % loop through all the vertices 
            for i = 1:numVertices 
                
                % get current vertex 
                vertex = this.vehicleVertices{i};
                
                if class(vertex) == "drivebot.VehicleStateVertex"
                    
                    actualNumVertices = actualNumVertices + 1;

                    % find out how many observation edges this vertex has 
                    vertexEdges = vertex.edges();
                    numVertexEdges = length(vertexEdges);

                    % loop through all edges and find the number of observation
                    % edges for this vertex
                    for j = 1:numVertexEdges 
                        if class(vertexEdges{j}) == "drivebot.LandmarkRangeBearingEdge"

                            % increment number of observation edges count 
                            obsEdgeCount = obsEdgeCount + 1;
                        end 
                    end
                end 
                                   
            end
            
            disp('Initial number of vertices: ')
            actualNumVertices
            
            % find the average number of observation edges per vertex
            avgObsEdges = round(obsEdgeCount / actualNumVertices); 
            
            % now, loop through the graph and prune any vertices that are
            % attached to fewer than the average number of observation
            % edges 
            
            removedVertices = 0;
            removedEdges = 0; 
            

                    
            for i = 1:numVertices 
                % initialise the number of observation edges count
                obsEdgeCount = 0;      
                % select current vertex -- first check it is a vehicle
                % state vertex 
                vertex = this.vehicleVertices{i};
                if class(vertex) == "drivebot.VehicleStateVertex"
                    
                    % find cell of all edges for this vertex 
                    vertexEdges = vertex.edges();
                    numVertexEdges = length(vertexEdges);
                        
                    % loop through all edges and count the number of
                    % observation edges 
                     for j = 1:numVertexEdges 
                        
                        if class(vertexEdges{j}) == "drivebot.LandmarkRangeBearingEdge"
                            obsEdgeCount = obsEdgeCount + 1;
                         
                        end 
                     end 
                        
                     
                        % check if the number of observation edges in this
                        % vertex is less than the average 
                        % extra condition checks that we don't remove the
                        % first prediction edge (first and second vertices)
                    if (obsEdgeCount < avgObsEdges) && obsEdgeCount ~=0 && (i ~= 1) && (i ~=2)
                        

                        if class(vertexEdges{1}) == "drivebot.LandmarkRangeBearingEdge"
                            this.graph.removeEdge(vertex.edges);

                        end 

                        myEdges = vertex.edges();
                        numMyEdges = length(myEdges);
                        for k = 1:numMyEdges 
                            thisEdge = myEdges{k};
                            this.graph.removeEdge(thisEdge);

                            % increment number of removed edges 
                            removedEdges = removedEdges + 1;

                        end 

                        % increment number of vertices removed 
                        removedVertices = removedVertices + 1;
                        
                        this.vehicleVertices(i) = [];
                        
                        % decrement vehicle vertex ID 
                        this.vehicleVertexId = this.vehicleVertexId - 1;
                        
                        i = i - 1;
                        
                        % grab previous vertex 
                        prevVertex = vertices{i};
                        
                        % grab next vertex
                        nextVertex = vertices{i+1};
                        
                        % create the prediction edge
                        e = drivebot.VehicleKinematicsEdge();

                        % link it so that it connects to the next vertex 
                        e.setVertex(2, nextVertex);
                        
                        % link it so that it connects to the previous
                        % vertex
                        e.setVertex(1, prevVertex);

                        % Set the measurement value and the measurement covariance
                        e.setMeasurement(z);
                        e.setInformation(omega);

                        % Add the edge to the graph
                        this.graph.addEdge(e);

                    end 
                      
                end
            
            end
            finalEdges = length(this.graph.edges())
            
            finalVertices = length(this.graph.vertices())
            
            removedVertices 
            removedEdges 
            
            end 
        
        end 
  
end

