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
import java.util.Arrays;
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

        boolean includePathMetrics = true;

        int[] validSourceNodes = extractValidNodes(sourceNodes);
        int[] validSourceIndices = extractValidIndices(sourceNodes);
        int[] validTargetNodes = extractValidNodes(targetNodes);
        int[] validTargetIndices = extractValidIndices(targetNodes);

        MatrixAccumulator accumulator = new MatrixAccumulator(sourceNodes.length, targetNodes.length, includePathMetrics);
        RPHASTAlgorithm.Metrics baseMetrics = new RPHASTAlgorithm.Metrics();

        if (validSourceNodes.length > 0 && validTargetNodes.length > 0) {
            PMap baseOptions = new PMap(request.algorithmOptions);
            RPHASTAlgorithm.PathMetricsProvider metricsProvider = includePathMetrics
                    ? new CHPathMetricsProvider(chGraph, baseOptions)
                    : null;
            RPHASTAlgorithm algorithm = new RPHASTAlgorithm(chGraph, metricsProvider, includePathMetrics);

            if (validSourceNodes.length == 1) {
                RPHASTAlgorithm.OneToManyResult result = algorithm.calcOneToMany(validSourceNodes[0], validTargetNodes);
                accumulator.insertOneToMany(validSourceIndices[0], validTargetIndices, result);
                baseMetrics = result.metrics;
            } else {
                RPHASTAlgorithm.ManyToManyResult result = algorithm.calcManyToMany(validSourceNodes, validTargetNodes);
                accumulator.insertManyToMany(validSourceIndices, validTargetIndices, result);
                baseMetrics = result.metrics;
            }
        }

        MatrixResponse response = MatrixResponse.fromFullMatrices(
                accumulator.distances,
                accumulator.times,
                baseMetrics
        );

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
                LOGGER.warn("Point {} could not be snapped to the road network", pt);
                nodes[i] = -1;
                continue;
            }
            nodes[i] = snap.getClosestNode();
        }
        return nodes;
    }

    private static int[] extractValidNodes(int[] nodes) {
        int count = 0;
        for (int node : nodes) {
            if (node >= 0)
                count++;
        }
        int[] result = new int[count];
        int idx = 0;
        for (int node : nodes) {
            if (node >= 0)
                result[idx++] = node;
        }
        return result;
    }

    private static int[] extractValidIndices(int[] nodes) {
        int count = 0;
        for (int node : nodes) {
            if (node >= 0)
                count++;
        }
        int[] result = new int[count];
        int idx = 0;
        for (int i = 0; i < nodes.length; i++) {
            if (nodes[i] >= 0)
                result[idx++] = i;
        }
        return result;
    }

    private static final class MatrixAccumulator {
        final long[][] distances;
        final long[][] times;

        MatrixAccumulator(int sources, int targets, boolean includePathMetrics) {
            distances = includePathMetrics ? new long[sources][targets] : null;
            times = includePathMetrics ? new long[sources][targets] : null;
            if (includePathMetrics) {
                for (int i = 0; i < sources; i++) {
                    Arrays.fill(distances[i], -1);
                    Arrays.fill(times[i], -1);
                }
            }
        }

        void insertOneToMany(int sourceIndex, int[] targetIndices, RPHASTAlgorithm.OneToManyResult result) {
            if (distances == null || result.distanceMeters == null)
                return;
            long[] distanceRow = distances[sourceIndex];
            long[] timeRow = times != null ? times[sourceIndex] : null;
            for (int i = 0; i < targetIndices.length; i++) {
                int targetIdx = targetIndices[i];
                distanceRow[targetIdx] = result.distanceMeters[i];
                if (timeRow != null && result.timeMillis != null)
                    timeRow[targetIdx] = result.timeMillis[i];
            }
        }

        void insertManyToMany(int[] sourceIndices, int[] targetIndices, RPHASTAlgorithm.ManyToManyResult result) {
            if (distances == null || result.distanceMeters == null)
                return;
            for (int i = 0; i < sourceIndices.length; i++) {
                int sourceIdx = sourceIndices[i];
                long[] distanceRow = distances[sourceIdx];
                long[] distanceResultRow = result.distanceMeters[i];
                long[] timeRow = times != null ? times[sourceIdx] : null;
                long[] timeResultRow = result.timeMillis != null ? result.timeMillis[i] : null;
                for (int j = 0; j < targetIndices.length; j++) {
                    int targetIdx = targetIndices[j];
                    distanceRow[targetIdx] = distanceResultRow[j];
                    if (timeRow != null && timeResultRow != null)
                        timeRow[targetIdx] = timeResultRow[j];
                }
            }
        }
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
        public final List<List<Long>> distances;
        public final List<List<Long>> times;
        public final Metrics metrics;

        private MatrixResponse(List<List<Long>> distances,
                               List<List<Long>> times, Metrics metrics) {
            this.distances = distances;
            this.times = times;
            this.metrics = metrics;
        }

        static MatrixResponse fromFullMatrices(long[][] distances,
                                               long[][] times,
                                               RPHASTAlgorithm.Metrics metrics) {
            List<List<Long>> distanceMatrix = distances != null ? toMatrix(distances) : null;
            List<List<Long>> timeMatrix = times != null ? toMatrix(times) : null;
            Metrics matrixMetrics = Metrics.from(metrics);
            return new MatrixResponse(distanceMatrix, timeMatrix, matrixMetrics);
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

        private static List<Long> toRow(long[] row) {
            List<Long> list = new ArrayList<>(row.length);
            for (long value : row)
                list.add(value < 0 ? -1 : value);
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
            if (metrics == null)
                return m;
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
