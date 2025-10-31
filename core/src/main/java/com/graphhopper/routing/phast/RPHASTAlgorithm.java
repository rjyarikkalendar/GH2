package com.graphhopper.routing.phast;

import com.graphhopper.storage.RoutingCHEdgeExplorer;
import com.graphhopper.storage.RoutingCHEdgeIterator;
import com.graphhopper.storage.RoutingCHGraph;

import java.util.*;
import java.util.concurrent.TimeUnit;

/**
 * Restriction-based PHAST (RPHAST) implementation for Contraction Hierarchies.
 * <p>
 * This implementation follows the high-level structure described in
 * <a href="https://www.microsoft.com/en-us/research/wp-content/uploads/2011/01/faster_batched_shortest_paths_road_networks.pdf">RPHAST</a>:
 * <ol>
 *   <li>Execute a forward upward search that is restricted to increasing CH levels.</li>
 *   <li>Run a single downward scan in decreasing level order to propagate labels.</li>
 *   <li>Optionally unpack paths for the resulting target nodes.</li>
 * </ol>
 * The code is thread-safe per instance if each thread uses its own {@link RPHASTAlgorithm}.
 */
public class RPHASTAlgorithm {
    public static final float INF = Float.POSITIVE_INFINITY;
    private final RoutingCHGraph chGraph;
    private final PathMetricsProvider pathMetricsProvider;
    private final boolean unpackPathsForTargets;
    private final int[] descendingLevelOrder;

    /**
     * @param chGraph               contracted hierarchy used for the queries
     * @param pathMetricsProvider   optional hook used to derive distance/time metrics per target
     * @param unpackPathsForTargets controls if {@link PathMetricsProvider} is consulted for targets with finite weight
     */
    public RPHASTAlgorithm(RoutingCHGraph chGraph,
                           PathMetricsProvider pathMetricsProvider,
                           boolean unpackPathsForTargets) {
        if (chGraph.isEdgeBased())
            throw new IllegalArgumentException("RPHASTAlgorithm currently supports node-based CH graphs only");
        if (chGraph.hasTurnCosts())
            throw new IllegalArgumentException("RPHASTAlgorithm requires CH weightings without turn costs");
        this.chGraph = chGraph;
        this.pathMetricsProvider = pathMetricsProvider;
        this.unpackPathsForTargets = unpackPathsForTargets && pathMetricsProvider != null;
        this.descendingLevelOrder = buildDescendingLevelOrder(chGraph);
    }

    /**
     * Calculates labels for a single source node and multiple targets.
     */
    public OneToManyResult calcOneToMany(int sourceNodeId, int[] targetNodeIds) {
        Metrics metrics = new Metrics();
        if (targetNodeIds == null)
            throw new IllegalArgumentException("targetNodeIds must not be null");

        long t0 = metrics.tic();
        Dedup dedup = dedupTargets(targetNodeIds);
        long dedupMs = metrics.ms(t0);

        return calcOneToManyInternal(sourceNodeId, targetNodeIds, dedup, dedupMs, metrics);
    }

    /**
     * Calculates a dense matrix for multiple sources and targets. Each row in the resulting matrix matches
     * the order of {@code sourceNodeIds}, each column matches {@code targetNodeIds}.
     */
    public ManyToManyResult calcManyToMany(int[] sourceNodeIds, int[] targetNodeIds) {
        if (sourceNodeIds == null || targetNodeIds == null)
            throw new IllegalArgumentException("sourceNodeIds and targetNodeIds must not be null");
        Metrics aggregated = new Metrics();
        float[][] weights = new float[sourceNodeIds.length][targetNodeIds.length];
        long[][] distances = unpackPathsForTargets ? new long[sourceNodeIds.length][targetNodeIds.length] : null;
        long[][] times = unpackPathsForTargets ? new long[sourceNodeIds.length][targetNodeIds.length] : null;

        long t0 = aggregated.tic();
        Dedup dedup = dedupTargets(targetNodeIds);
        long dedupMs = aggregated.ms(t0);
        aggregated.tSnapTargetsMs += dedupMs;

        for (int srcIdx = 0; srcIdx < sourceNodeIds.length; srcIdx++) {
            OneToManyResult rowResult = calcOneToManyInternal(sourceNodeIds[srcIdx], targetNodeIds, dedup, 0, null);
            weights[srcIdx] = rowResult.weight;
            if (unpackPathsForTargets) {
                distances[srcIdx] = rowResult.distanceMeters;
                times[srcIdx] = rowResult.timeMillis;
            }
            aggregated.add(rowResult.metrics);
        }

        return new ManyToManyResult(weights, distances, times, aggregated);
    }

    private OneToManyResult calcOneToManyInternal(int sourceNodeId, int[] targetNodeIds, Dedup reusedDedup,
                                                  long dedupMs, Metrics externalMetrics) {
        Metrics metrics = externalMetrics != null ? externalMetrics : new Metrics();
        metrics.tSnapTargetsMs += dedupMs;
        int nodeCount = chGraph.getNodes();
        if (sourceNodeId < 0 || sourceNodeId >= nodeCount) {
            float[] weightsOut = new float[targetNodeIds.length];
            Arrays.fill(weightsOut, INF);
            long[] distancesOut = null;
            long[] timesOut = null;
            if (unpackPathsForTargets) {
                distancesOut = new long[targetNodeIds.length];
                timesOut = new long[targetNodeIds.length];
                Arrays.fill(distancesOut, -1);
                Arrays.fill(timesOut, -1);
            }
            return new OneToManyResult(weightsOut, distancesOut, timesOut, metrics);
        }

        long t0 = metrics.tic();
        float[] upDistances = forwardUpSearch(sourceNodeId, metrics);
        metrics.tForwardUpMs += metrics.ms(t0);

        t0 = metrics.tic();
        float[] bestLabels = phastScan(upDistances, metrics);
        metrics.tPhastScanMs += metrics.ms(t0);

        t0 = metrics.tic();
        float[] weightsOut = new float[targetNodeIds.length];
        Arrays.fill(weightsOut, INF);
        for (int i = 0; i < reusedDedup.uniqueTargets.length; i++) {
            int node = reusedDedup.uniqueTargets[i];
            if (node < 0 || node >= bestLabels.length)
                continue;
            float label = bestLabels[node];
            for (int originalIndex : reusedDedup.reverse[i]) {
                weightsOut[originalIndex] = label;
                metrics.targetsResolved++;
            }
        }
        metrics.tPackResultsMs += metrics.ms(t0);

        long[] distances = null;
        long[] times = null;
        if (unpackPathsForTargets) {
            t0 = metrics.tic();
            distances = new long[targetNodeIds.length];
            times = new long[targetNodeIds.length];
            Arrays.fill(distances, -1);
            Arrays.fill(times, -1);
            for (int i = 0; i < targetNodeIds.length; i++) {
                if (weightsOut[i] == INF || targetNodeIds[i] < 0)
                    continue;
                DistTime dt = pathMetricsProvider.unpack(sourceNodeId, targetNodeIds[i]);
                if (dt != null) {
                    distances[i] = dt.distanceMeters;
                    times[i] = dt.timeMillis;
                    metrics.shortcutUnpackedTargets++;
                }
            }
            metrics.tUnpackTargetsMs += metrics.ms(t0);
        }

        return new OneToManyResult(weightsOut, distances, times, metrics);
    }

    private float[] forwardUpSearch(int sourceNode, Metrics metrics) {
        int nodes = chGraph.getNodes();
        float[] distances = new float[nodes];
        Arrays.fill(distances, INF);

        PriorityQueue<NodeWeight> pq = new PriorityQueue<>(Comparator.comparingDouble(a -> a.weight));
        distances[sourceNode] = 0;
        pq.add(new NodeWeight(sourceNode, 0));
        metrics.pqPushes++;

        RoutingCHEdgeExplorer outExplorer = chGraph.createOutEdgeExplorer();

        while (!pq.isEmpty()) {
            NodeWeight current = pq.poll();
            metrics.visitedNodes++;
            if (current.weight > distances[current.node])
                continue;
            RoutingCHEdgeIterator iter = outExplorer.setBaseNode(current.node);
            while (iter.next()) {
                int adj = iter.getAdjNode();
                if (chGraph.getLevel(adj) < chGraph.getLevel(current.node))
                    continue;
                double edgeWeight = iter.getWeight(false);
                if (Double.isInfinite(edgeWeight))
                    continue;
                float newWeight = (float) (current.weight + edgeWeight);
                if (newWeight < distances[adj]) {
                    distances[adj] = newWeight;
                    pq.add(new NodeWeight(adj, newWeight));
                    metrics.pqPushes++;
                }
            }
        }
        return distances;
    }

    private float[] phastScan(float[] upDistances, Metrics metrics) {
        float[] labels = Arrays.copyOf(upDistances, upDistances.length);
        RoutingCHEdgeExplorer outExplorer = chGraph.createOutEdgeExplorer();

        for (int node : descendingLevelOrder) {
            float distance = labels[node];
            if (distance == INF)
                continue;
            RoutingCHEdgeIterator iter = outExplorer.setBaseNode(node);
            while (iter.next()) {
                int adj = iter.getAdjNode();
                if (chGraph.getLevel(adj) >= chGraph.getLevel(node))
                    continue;
                double edgeWeight = iter.getWeight(false);
                if (Double.isInfinite(edgeWeight))
                    continue;
                float candidate = (float) (distance + edgeWeight);
                if (candidate < labels[adj])
                    labels[adj] = candidate;
            }
        }
        return labels;
    }

    private static Dedup dedupTargets(int[] targetNodeIds) {
        Map<Integer, List<Integer>> reverse = new LinkedHashMap<>(targetNodeIds.length * 2);
        for (int i = 0; i < targetNodeIds.length; i++) {
            int node = targetNodeIds[i];
            if (node < 0)
                continue;
            reverse.computeIfAbsent(node, k -> new ArrayList<>()).add(i);
        }
        int[] unique = new int[reverse.size()];
        @SuppressWarnings("unchecked")
        List<Integer>[] back = new List[reverse.size()];
        int index = 0;
        for (Map.Entry<Integer, List<Integer>> entry : reverse.entrySet()) {
            unique[index] = entry.getKey();
            back[index] = entry.getValue();
            index++;
        }
        return new Dedup(unique, back);
    }

    private static int[] buildDescendingLevelOrder(RoutingCHGraph chGraph) {
        int nodes = chGraph.getNodes();
        Integer[] boxed = new Integer[nodes];
        for (int i = 0; i < nodes; i++)
            boxed[i] = i;
        Arrays.sort(boxed, (a, b) -> Integer.compare(chGraph.getLevel(b), chGraph.getLevel(a)));
        int[] order = new int[nodes];
        for (int i = 0; i < nodes; i++)
            order[i] = boxed[i];
        return order;
    }

    private static final class NodeWeight {
        final int node;
        final float weight;

        NodeWeight(int node, float weight) {
            this.node = node;
            this.weight = weight;
        }
    }

    private static final class Dedup {
        final int[] uniqueTargets;
        final List<Integer>[] reverse;

        Dedup(int[] uniqueTargets, List<Integer>[] reverse) {
            this.uniqueTargets = uniqueTargets;
            this.reverse = reverse;
        }
    }

    public interface PathMetricsProvider {
        DistTime unpack(int sourceNode, int targetNode);
    }

    public static final class DistTime {
        public final long distanceMeters;
        public final long timeMillis;

        public DistTime(long distanceMeters, long timeMillis) {
            this.distanceMeters = distanceMeters;
            this.timeMillis = timeMillis;
        }
    }

    public static final class OneToManyResult {
        public final float[] weight;
        public final long[] distanceMeters;
        public final long[] timeMillis;
        public final Metrics metrics;

        public OneToManyResult(float[] weight, long[] distanceMeters, long[] timeMillis, Metrics metrics) {
            this.weight = weight;
            this.distanceMeters = distanceMeters;
            this.timeMillis = timeMillis;
            this.metrics = metrics;
        }
    }

    public static final class ManyToManyResult {
        public final float[][] weight;
        public final long[][] distanceMeters;
        public final long[][] timeMillis;
        public final Metrics metrics;

        public ManyToManyResult(float[][] weight, long[][] distanceMeters, long[][] timeMillis, Metrics metrics) {
            this.weight = weight;
            this.distanceMeters = distanceMeters;
            this.timeMillis = timeMillis;
            this.metrics = metrics;
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

        long tic() {
            return System.nanoTime();
        }

        long ms(long t0) {
            return TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - t0);
        }

        void add(Metrics other) {
            this.tSnapSourceMs += other.tSnapSourceMs;
            this.tSnapTargetsMs += other.tSnapTargetsMs;
            this.tForwardUpMs += other.tForwardUpMs;
            this.tPhastScanMs += other.tPhastScanMs;
            this.tPackResultsMs += other.tPackResultsMs;
            this.tUnpackTargetsMs += other.tUnpackTargetsMs;
            this.visitedNodes += other.visitedNodes;
            this.pqPushes += other.pqPushes;
            this.targetsResolved += other.targetsResolved;
            this.shortcutUnpackedTargets += other.shortcutUnpackedTargets;
        }

        @Override
        public String toString() {
            return String.format(Locale.ROOT,
                    "snapSrc=%dms snapTgt=%dms up=%dms phast=%dms pack=%dms unpack=%dms | visited=%d pushes=%d targets=%d unpackTargets=%d",
                    tSnapSourceMs, tSnapTargetsMs, tForwardUpMs, tPhastScanMs, tPackResultsMs, tUnpackTargetsMs,
                    visitedNodes, pqPushes, targetsResolved, shortcutUnpackedTargets);
        }
    }
}
