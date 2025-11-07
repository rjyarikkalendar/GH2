package com.graphhopper.routing.phast;

import com.graphhopper.routing.EdgeToEdgeRoutingAlgorithm;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ch.CHRoutingAlgorithmFactory;
import com.graphhopper.routing.phast.RPHASTAlgorithm.DistTime;
import com.graphhopper.storage.RoutingCHGraph;
import com.graphhopper.util.PMap;
import com.graphhopper.util.Parameters;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.Objects;

/**
 * {@link RPHASTAlgorithm.PathMetricsProvider} implementation that falls back to classic CH queries to
 * reconstruct path metrics (distance/time) for specific source/target pairs.
 */
public class CHPathMetricsProvider implements RPHASTAlgorithm.PathMetricsProvider {
    private static final org.slf4j.Logger LOGGER = org.slf4j.LoggerFactory.getLogger(CHPathMetricsProvider.class);
    private final CHRoutingAlgorithmFactory algorithmFactory;
    private final PMap baseOptions;
    private final RoutingCHGraph chGraph;

    // simple cache: key = ((long)source << 32) | (target & 0xFFFFFFFFL)
    private final Map<Long, DistTime> cache = new ConcurrentHashMap<>();

    public CHPathMetricsProvider(RoutingCHGraph chGraph, PMap options) {
        this.algorithmFactory = new CHRoutingAlgorithmFactory(chGraph);
        this.baseOptions = options == null ? new PMap() : new PMap(options);
        this.chGraph = Objects.requireNonNull(chGraph);
    }

    @Override
    public DistTime unpack(int sourceNode, int targetNode) {
        long key = (((long) sourceNode) << 32) | (targetNode & 0xFFFFFFFFL);
        DistTime cached = cache.get(key);
        if (cached != null) {
            LOGGER.debug("CHPathMetricsProvider cache HIT {}/{} -> {}m {}ms", sourceNode, targetNode, cached.distanceMeters, cached.timeMillis);
            return cached;
        }
        LOGGER.debug("CHPathMetricsProvider cache MISS for {}/{}", sourceNode, targetNode);

        // Use CH-based algorithm (prefer A* on CH). Ensure options request an A* CH algorithm
        try {
            PMap algoOpts = new PMap(baseOptions);
            // Force usage of CH + A* regardless of caller hints to avoid falling back to non-CH algorithms
            algoOpts.putObject(Parameters.Routing.ALGORITHM, Parameters.Algorithms.ASTAR_BI);
            EdgeToEdgeRoutingAlgorithm algorithm = algorithmFactory.createAlgo(algoOpts);
            LOGGER.debug("CHPathMetricsProvider using CH algo {} for {}/{}", algoOpts.getString(Parameters.Routing.ALGORITHM, "<none>"), sourceNode, targetNode);
            Path path = algorithm.calcPath(sourceNode, targetNode);
            if (path != null && path.isFound()) {
                long distanceMeters = Math.round(path.getDistance());
                long timeMillis = path.getTime();
                DistTime dt = new DistTime(distanceMeters, timeMillis);
                cache.put(key, dt);
                LOGGER.debug("CHPathMetricsProvider CH path found {}/{} -> {}m {}ms", sourceNode, targetNode, distanceMeters, timeMillis);
                return dt;
            } else {
                LOGGER.debug("CHPathMetricsProvider CH path NOT found for {}/{}", sourceNode, targetNode);
            }
        } catch (Exception ex) {
            // do not fallback to Dijkstra per user's requirement; just return null
        }

        // nothing found via CH
        return null;
    }
}
