package com.graphhopper.routing.phast;

import com.graphhopper.routing.EdgeToEdgeRoutingAlgorithm;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ch.CHRoutingAlgorithmFactory;
import com.graphhopper.routing.phast.RPHASTAlgorithm.DistTime;
import com.graphhopper.storage.RoutingCHGraph;
import com.graphhopper.util.PMap;

/**
 * {@link RPHASTAlgorithm.PathMetricsProvider} implementation that falls back to classic CH queries to
 * reconstruct path metrics (distance/time) for specific source/target pairs.
 */
public class CHPathMetricsProvider implements RPHASTAlgorithm.PathMetricsProvider {
    private final CHRoutingAlgorithmFactory algorithmFactory;
    private final PMap baseOptions;

    public CHPathMetricsProvider(RoutingCHGraph chGraph, PMap options) {
        this.algorithmFactory = new CHRoutingAlgorithmFactory(chGraph);
        this.baseOptions = options == null ? new PMap() : new PMap(options);
    }

    @Override
    public DistTime unpack(int sourceNode, int targetNode) {
        EdgeToEdgeRoutingAlgorithm algorithm = algorithmFactory.createAlgo(new PMap(baseOptions));
        Path path = algorithm.calcPath(sourceNode, targetNode);
        if (!path.isFound())
            return null;
        long distanceMeters = Math.round(path.getDistance());
        long timeMillis = path.getTime();
        return new DistTime(distanceMeters, timeMillis);
    }
}
