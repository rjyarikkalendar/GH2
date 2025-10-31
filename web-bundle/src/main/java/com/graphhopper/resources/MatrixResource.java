package com.graphhopper.resources;

import com.graphhopper.GraphHopper;
import com.graphhopper.http.ProfileResolver;
import com.graphhopper.routing.phast.CHPathMetricsProvider;
import com.graphhopper.routing.phast.RPHASTAlgorithm;
import com.graphhopper.routing.util.DefaultSnapFilter;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.RoutingCHGraph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.Helper;
import com.graphhopper.util.PMap;
import com.graphhopper.util.StopWatch;
import com.graphhopper.util.shapes.GHPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import jakarta.inject.Inject;
import jakarta.validation.constraints.NotNull;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import jakarta.ws.rs.core.Response;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Path("matrix")
@Produces(MediaType.APPLICATION_JSON)
public class MatrixResource {

    private static final Logger LOGGER = LoggerFactory.getLogger(MatrixResource.class);

    private final GraphHopper graphHopper;
    private final ProfileResolver profileResolver;
    private final LocationIndex locationIndex;

    @Inject
    public MatrixResource(GraphHopper graphHopper, ProfileResolver profileResolver, LocationIndex locationIndex) {
        this.graphHopper = graphHopper;
        this.profileResolver = profileResolver;
        this.locationIndex = locationIndex;
    }

    @POST
    @Consumes(MediaType.APPLICATION_JSON)
    public Response calculateMatrix(@NotNull MatrixRequest request) {
        request.validate();

        StopWatch sw = new StopWatch().start();
        String profileName = resolveProfile(request.profile);
        RoutingCHGraph chGraph = graphHopper.getCHGraphs().get(profileName);
        if (chGraph == null)
            throw new IllegalArgumentException("CH profile '" + profileName + "' is not available. Available: " + graphHopper.getCHGraphs().keySet());
        if (chGraph.isEdgeBased())
            throw new IllegalArgumentException("Matrix endpoint currently supports node-based CH profiles only");
        if (chGraph.getWeighting().hasTurnCosts())
            throw new IllegalArgumentException("Matrix endpoint requires CH profiles without turn-costs");

        Weighting weighting = chGraph.getWeighting();
        BooleanEncodedValue inSubnetworkEnc = graphHopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(profileName));
        DefaultSnapFilter snapFilter = new DefaultSnapFilter(weighting, inSubnetworkEnc);

        int[] sourceNodes = snapPoints(request.sources, snapFilter);
        int[] targetNodes = snapPoints(request.targets, snapFilter);

        PMap baseOptions = new PMap(request.algorithmOptions);
        boolean includePathMetrics = request.includePathMetrics;
        RPHASTAlgorithm.PathMetricsProvider metricsProvider = includePathMetrics
                ? new CHPathMetricsProvider(chGraph, baseOptions)
                : null;
        RPHASTAlgorithm algorithm = new RPHASTAlgorithm(chGraph, metricsProvider, includePathMetrics);

        MatrixResponse response;
        if (sourceNodes.length == 1) {
            RPHASTAlgorithm.OneToManyResult result = algorithm.calcOneToMany(sourceNodes[0], targetNodes);
            response = MatrixResponse.fromOneToMany(result, sourceNodes, targetNodes, includePathMetrics);
        } else {
            RPHASTAlgorithm.ManyToManyResult result = algorithm.calcManyToMany(sourceNodes, targetNodes);
            response = MatrixResponse.fromManyToMany(result, sourceNodes, targetNodes, includePathMetrics);
        }

        double tookMillis = sw.stop().getMillisDouble();
        LOGGER.info("matrix profile={} sources={} targets={} took={} ms", profileName, sourceNodes.length, targetNodes.length, String.format("%.1f", tookMillis));

        return Response.ok(response).build();
    }

    private String resolveProfile(String requestedProfile) {
        PMap hints = new PMap();
        hints.putObject("profile", requestedProfile);
        return profileResolver.resolveProfile(hints);
    }

    private int[] snapPoints(List<MatrixPoint> points, DefaultSnapFilter snapFilter) {
        int[] nodes = new int[points.size()];
        for (int i = 0; i < points.size(); i++) {
            MatrixPoint pt = points.get(i);
            Snap snap = locationIndex.findClosest(pt.lat, pt.lon, snapFilter);
            if (!snap.isValid()) {
                throw new IllegalArgumentException("Point " + pt + " could not be snapped to the road network");
            }
            nodes[i] = snap.getClosestNode();
        }
        return nodes;
    }

    public static class MatrixRequest {
        public String profile;
        public List<MatrixPoint> sources = List.of();
        public List<MatrixPoint> targets = List.of();
        public boolean includePathMetrics = true;
        public Map<String, Object> algorithmOptions = Map.of();

        public void validate() {
            if (sources == null || sources.isEmpty())
                throw new IllegalArgumentException("The request must contain at least one source");
            if (targets == null || targets.isEmpty())
                throw new IllegalArgumentException("The request must contain at least one target");
            if (algorithmOptions == null)
                algorithmOptions = Map.of();
        }
    }

    public static class MatrixPoint {
        public double lat;
        public double lon;

        @Override
        public String toString() {
            return new GHPoint(lat, lon).toString();
        }
    }

    public static class MatrixResponse {
        public final List<Integer> sourceNodes;
        public final List<Integer> targetNodes;
        public final List<List<Double>> weights;
        public final List<List<Long>> distances;
        public final List<List<Long>> times;
        public final Metrics metrics;

        private MatrixResponse(List<Integer> sourceNodes, List<Integer> targetNodes,
                               List<List<Double>> weights, List<List<Long>> distances,
                               List<List<Long>> times, Metrics metrics) {
            this.sourceNodes = sourceNodes;
            this.targetNodes = targetNodes;
            this.weights = weights;
            this.distances = distances;
            this.times = times;
            this.metrics = metrics;
        }

        static MatrixResponse fromOneToMany(RPHASTAlgorithm.OneToManyResult result, int[] sources, int[] targets, boolean includePathMetrics) {
            List<Integer> sourceNodes = toList(sources);
            List<Integer> targetNodes = toList(targets);
            List<List<Double>> weights = toMatrix(result.weight);
            List<List<Long>> distances = includePathMetrics ? toMatrix(result.distanceMeters) : null;
            List<List<Long>> times = includePathMetrics ? toMatrix(result.timeMillis) : null;
            Metrics metrics = Metrics.from(result.metrics);
            return new MatrixResponse(sourceNodes, targetNodes, weights, distances, times, metrics);
        }

        static MatrixResponse fromManyToMany(RPHASTAlgorithm.ManyToManyResult result, int[] sources, int[] targets, boolean includePathMetrics) {
            List<Integer> sourceNodes = toList(sources);
            List<Integer> targetNodes = toList(targets);
            List<List<Double>> weights = toMatrix(result.weight);
            List<List<Long>> distances = includePathMetrics ? toMatrix(result.distanceMeters) : null;
            List<List<Long>> times = includePathMetrics ? toMatrix(result.timeMillis) : null;
            Metrics metrics = Metrics.from(result.metrics);
            return new MatrixResponse(sourceNodes, targetNodes, weights, distances, times, metrics);
        }

        private static List<Integer> toList(int[] values) {
            List<Integer> list = new ArrayList<>(values.length);
            for (int value : values)
                list.add(value);
            return list;
        }

        private static List<List<Double>> toMatrix(float[] row) {
            List<List<Double>> matrix = new ArrayList<>(1);
            matrix.add(toRow(row));
            return matrix;
        }

        private static List<List<Double>> toMatrix(float[][] values) {
            List<List<Double>> matrix = new ArrayList<>(values.length);
            for (float[] row : values)
                matrix.add(toRow(row));
            return matrix;
        }

        private static List<List<Long>> toMatrix(long[] row) {
            if (row == null)
                return null;
            List<List<Long>> matrix = new ArrayList<>(1);
            matrix.add(toRow(row));
            return matrix;
        }

        private static List<List<Long>> toMatrix(long[][] values) {
            if (values == null)
                return null;
            List<List<Long>> matrix = new ArrayList<>(values.length);
            for (long[] row : values)
                matrix.add(toRow(row));
            return matrix;
        }

        private static List<Double> toRow(float[] row) {
            List<Double> list = new ArrayList<>(row.length);
            for (float value : row)
                list.add(Float.isInfinite(value) ? null : (double) value);
            return list;
        }

        private static List<Long> toRow(long[] row) {
            List<Long> list = new ArrayList<>(row.length);
            for (long value : row)
                list.add(value < 0 ? null : value);
            return list;
        }
    }

    public static class Metrics {
        public long tSnapSourceMs;
        public long tSnapTargetsMs;
        public long tForwardUpMs;
        public long tPhastScanMs;
        public long tPackResultsMs;
        public long tUnpackTargetsMs;
        public long visitedNodes;
        public long pqPushes;
        public long targetsResolved;
        public long shortcutUnpackedTargets;

        static Metrics from(RPHASTAlgorithm.Metrics metrics) {
            Metrics m = new Metrics();
            m.tSnapSourceMs = metrics.tSnapSourceMs;
            m.tSnapTargetsMs = metrics.tSnapTargetsMs;
            m.tForwardUpMs = metrics.tForwardUpMs;
            m.tPhastScanMs = metrics.tPhastScanMs;
            m.tPackResultsMs = metrics.tPackResultsMs;
            m.tUnpackTargetsMs = metrics.tUnpackTargetsMs;
            m.visitedNodes = metrics.visitedNodes;
            m.pqPushes = metrics.pqPushes;
            m.targetsResolved = metrics.targetsResolved;
            m.shortcutUnpackedTargets = metrics.shortcutUnpackedTargets;
            return m;
        }
    }
}
