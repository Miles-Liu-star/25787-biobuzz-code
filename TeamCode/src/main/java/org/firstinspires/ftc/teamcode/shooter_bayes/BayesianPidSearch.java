package org.firstinspires.ftc.teamcode.shooter_bayes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.function.BooleanSupplier;

/** 用两个独立高斯过程分别约束“代价”和“超调裕量”的 PID 搜索。 */
final class BayesianPidSearch {
    static final int MIN_ROUNDS = 20, MAX_ROUNDS = 32;
    static final int HISTORY_STRIDE = 6, HISTORY_ROWS = 40;
    static final int VERIFICATION_ROUNDS = 2;
    private static final int FINALISTS = 3, RECORD_PATIENCE = 8;
    private final double[] max, scale;
    private final Random random = new Random(27270);
    private final List<Observation> data = new ArrayList<>();
    private final List<Review> reviews = new ArrayList<>();
    private int historyCount;

    interface Trial {
        Result test(double[] gains, boolean verification, int pattern) throws InterruptedException;
    }

    static final class Result {
        final double cost, margin;
        final boolean settled;
        final String reason;

        Result(double cost, double margin, boolean settled) {
            this(cost, margin, settled, "");
        }

        Result(double cost, double margin, boolean settled, String reason) {
            this.cost = cost;
            this.margin = margin;
            this.settled = settled;
            this.reason = reason;
        }

        static Result unavailable(String reason) {
            return new Result(Double.NaN, Double.NaN, false, reason);
        }
    }

    static final class Review {
        final double[] gains;
        final int passed;
        final double score;
        final String firstFailure;

        Review(double[] gains, int passed, double score, String firstFailure) {
            this.gains = gains;
            this.passed = passed;
            this.score = score;
            this.firstFailure = firstFailure;
        }
    }

    static final class Selection {
        final double[] gains;
        final double score;

        Selection(double[] gains, double score) {
            this.gains = gains;
            this.score = score;
        }
    }

    BayesianPidSearch(double[] max, double[] scale) {
        this.max = max.clone();
        this.scale = scale.clone();
    }

    List<Review> reviews() {
        return new ArrayList<>(reviews);
    }

    int searchCount() {
        return data.size() - historyCount;
    }

    int historyCount() {
        return historyCount;
    }

    double bestSearchCost() {
        return data.stream().skip(historyCount).mapToDouble(o -> o.result.cost).min().orElse(Double.NaN);
    }

    double bestQualifiedSearchCost() {
        return data.stream().skip(historyCount).filter(o -> o.result.settled && o.result.margin >= 0)
                .mapToDouble(o -> o.result.cost).min().orElse(Double.NaN);
    }

    Selection tune(double[] existing, Trial trial, BooleanSupplier cancelled, double[] history)
            throws InterruptedException {
        loadHistory(history);
        double[] baseline = normalize(existing), center = baseline;
        double radius = 0.30, best = Double.POSITIVE_INFINITY, record = Double.POSITIVE_INFINITY;
        for (Observation row : data) {
            record = Math.min(record, row.result.cost);
            if (row.result.settled && row.result.margin >= 0 && row.result.cost < best) {
                best = row.result.cost;
                center = row.x;
            }
        }
        int success = 0, stagnant = 0, lastRecord = 0;
        for (int round = 0, attempts = 0; round < MAX_ROUNDS && attempts < MAX_ROUNDS + 12
                && !cancelled.getAsBoolean(); attempts++) {
            double[] x = round == 0 ? baseline : propose(center, radius, round % 5 == 0, cancelled);
            if (cancelled.getAsBoolean()) return null;
            Result score = trial.test(gains(x), false, round % 6);
            if (!measured(score)) continue;
            data.add(new Observation(x, score));
            round++;
            if (score.cost < record) {
                record = score.cost;
                lastRecord = round;
            }
            if (score.margin >= 0 && score.cost < best - 0.015) {
                best = score.cost;
                center = x;
                stagnant = 0;
                if (++success == 3) {
                    radius = Math.min(0.55, radius * 1.4);
                    success = 0;
                }
            } else if (++stagnant == 7) {
                radius = radius < 0.13 ? 0.32 : radius * 0.65;
                stagnant = success = 0;
            }
            // 代价记录与“是否可用于 PID”分开判断。
            if (round >= MIN_ROUNDS && round - lastRecord >= RECORD_PATIENCE) break;
        }
        if (cancelled.getAsBoolean() || searchCount() < MIN_ROUNDS) return null;

        List<double[]> finalists = new ArrayList<>();
        finalists.add(baseline);
        List<Observation> ranked = new ArrayList<>(data);
        ranked.sort(Comparator.comparingDouble(o -> o.result.cost));
        for (Observation o : ranked) {
            if (!o.result.settled || o.result.margin < 0) continue;
            if (finalists.stream().noneMatch(x -> distance(x, o.x) <= 0.035)) finalists.add(o.x);
            if (finalists.size() == FINALISTS) break;
        }
        double[][] costs = new double[finalists.size()][VERIFICATION_ROUNDS];
        int[] passed = new int[finalists.size()];
        String[] firstFailure = new String[finalists.size()];
        for (int repeat = 0; repeat < VERIFICATION_ROUNDS; repeat++) {
            for (int i = 0; i < finalists.size(); i++) {
                if (cancelled.getAsBoolean()) return null;
                int j = (i + repeat) % finalists.size();
                if (firstFailure[j] != null) continue;
                double[] candidate = gains(finalists.get(j));
                Result result = trial.test(candidate, true, repeat);
                if (!measured(result) && !cancelled.getAsBoolean()) {
                    Result retry = trial.test(candidate, true, repeat);
                    if (retry != null) result = retry;
                }
                boolean pass = measured(result) && result.settled && result.margin >= 0;
                costs[j][repeat] = pass ? result.cost : Double.POSITIVE_INFINITY;
                if (pass) passed[j]++;
                else if (firstFailure[j] == null) {
                    String reason = result != null && !result.reason.isEmpty() ? result.reason
                            : !measured(result) ? "无有效读数"
                            : result.margin < 0 ? "超调超限" : "未停稳";
                    firstFailure[j] = "第 " + (repeat + 1) + " 组 " + reason;
                }
            }
        }
        int winner = 0;
        double verifiedBest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < costs.length; i++) {
            double value = passed[i] == VERIFICATION_ROUNDS
                    ? validationScore(costs[i]) : Double.POSITIVE_INFINITY;
            reviews.add(new Review(gains(finalists.get(i)), passed[i], value, firstFailure[i]));
            if (value < 0.98 * verifiedBest) {
                verifiedBest = value;
                winner = i;
            }
        }
        return !cancelled.getAsBoolean() && Double.isFinite(verifiedBest)
                ? new Selection(gains(finalists.get(winner)), verifiedBest) : null;
    }

    private void loadHistory(double[] history) {
        for (int i = 0; i + HISTORY_STRIDE <= history.length; i += HISTORY_STRIDE) {
            double[] old = {history[i], history[i + 1], history[i + 2]};
            boolean inBounds = true;
            for (int axis = 0; axis < 3; axis++) inBounds &= old[axis] >= 0 && old[axis] <= max[axis];
            Result remembered = new Result(history[i + 3], history[i + 4], history[i + 5] == 1);
            if (inBounds && measured(remembered)) data.add(new Observation(normalize(old), remembered));
        }
        historyCount = data.size();
    }

    double[] history() {
        int rows = Math.min(HISTORY_ROWS, data.size());
        double[] result = new double[rows * HISTORY_STRIDE];
        for (int i = 0; i < rows; i++) {
            Observation row = data.get(data.size() - rows + i);
            int j = i * HISTORY_STRIDE;
            System.arraycopy(gains(row.x), 0, result, j, 3);
            result[j + 3] = row.result.cost;
            result[j + 4] = row.result.margin;
            result[j + 5] = row.result.settled ? 1 : 0;
        }
        return result;
    }

    private static boolean measured(Result result) {
        return result != null && Maths.finite(result.cost, result.margin);
    }

    private double[] propose(double[] center, double radius, boolean explore, BooleanSupplier cancelled) {
        if (data.size() < 8) {
            double[] x = center.clone();
            int direction = data.size() - 1;
            x[direction % 3] = Maths.clip(x[direction % 3] + (direction < 3 ? 0.15 : -0.15), 0, 1);
            return distance(x, center) > 0.03 && direction < 6 ? x : sample(center, 0.18);
        }
        if (data.stream().noneMatch(o -> o.result.margin >= 0)) {
            return sample(new double[]{0.24, 0.06, 0.25}, 0.13);
        }
        GaussianProcess objective = GaussianProcess.fit(data, false);
        GaussianProcess safety = GaussianProcess.fit(data, true);
        double[] winner = center;
        double bestAcquisition = Double.POSITIVE_INFINITY;
        double[] work = new double[data.size()];
        for (int i = 0; i < 4096; i++) {
            if (i % 64 == 0 && cancelled.getAsBoolean()) break;
            double[] x = sample(center, i < 192 ? Math.min(radius, 0.12) : radius);
            double nearest = Double.POSITIVE_INFINITY;
            boolean nearViolation = false;
            for (Observation o : data) {
                double separation = distance(x, o.x);
                nearest = Math.min(nearest, separation);
                nearViolation |= o.result.margin < 0 && separation < 0.07;
            }
            if (nearest < 0.028 || nearViolation) continue;
            double[] limit = safety.predict(x, work);
            if (limit[0] - (explore ? 0.65 : 0.95) * limit[1] < 0) continue;
            double[] cost = objective.predict(x, work);
            double acquisition = cost[0] - (explore ? 1.9 : 0.85) * cost[1];
            if (acquisition < bestAcquisition) {
                bestAcquisition = acquisition;
                winner = x;
            }
        }
        return winner;
    }

    private double[] sample(double[] center, double radius) {
        double[] x = new double[3];
        for (int i = 0; i < x.length; i++) {
            double lo = Math.max(0, center[i] - radius), hi = Math.min(1, center[i] + radius);
            x[i] = lo + random.nextDouble() * (hi - lo);
        }
        return x;
    }

    private static double validationScore(double[] scores) {
        return 0.65 * Maths.median(scores, scores.length)
                + 0.35 * Arrays.stream(scores).max().orElse(Double.POSITIVE_INFINITY);
    }

    private double[] normalize(double[] gains) {
        double[] x = new double[3];
        for (int i = 0; i < x.length; i++) {
            x[i] = Math.log1p(Maths.clip(gains[i], 0, max[i]) / scale[i]) / Math.log1p(max[i] / scale[i]);
        }
        return x;
    }

    private double[] gains(double[] x) {
        double[] gains = new double[3];
        for (int i = 0; i < x.length; i++) {
            gains[i] = scale[i] * Math.expm1(x[i] * Math.log1p(max[i] / scale[i]));
        }
        return gains;
    }

    private static double distance(double[] a, double[] b) {
        return Math.sqrt(square(a[0] - b[0]) + square(a[1] - b[1]) + square(a[2] - b[2]));
    }

    private static double square(double value) {
        return value * value;
    }

    private static final class Observation {
        final double[] x;
        final Result result;

        Observation(double[] x, Result result) {
            this.x = x;
            this.result = result;
        }
    }

    private static final class GaussianProcess {
        final List<Observation> data;
        final double mean, deviation, length, noise, loss;
        final double[][] lower;
        final double[] alpha;

        static GaussianProcess fit(List<Observation> data, boolean constraint) {
            double[] y = data.stream().mapToDouble(o -> constraint ? o.result.margin : o.result.cost).toArray();
            double mean = Arrays.stream(y).average().orElse(0);
            double deviation = Math.max(constraint ? 0.12 : 0.10,
                    Math.sqrt(Arrays.stream(y).map(v -> square(v - mean)).average().orElse(0)));
            for (int i = 0; i < y.length; i++) y[i] = (y[i] - mean) / deviation;
            GaussianProcess best = null;
            for (double length : new double[]{0.10, 0.18, 0.30, 0.48, 0.75}) {
                for (double noise : new double[]{0.04, 0.15}) {
                    GaussianProcess model = new GaussianProcess(data, y, mean, deviation, length, noise);
                    if (best == null || model.loss < best.loss) best = model;
                }
            }
            return best;
        }

        GaussianProcess(List<Observation> data, double[] y, double mean, double deviation, double length,
                        double noise) {
            this.data = data;
            this.mean = mean;
            this.deviation = deviation;
            this.length = length;
            this.noise = noise;
            lower = new double[data.size()][data.size()];
            double likelihood = 0;
            for (int i = 0; i < data.size(); i++) {
                for (int j = 0; j <= i; j++) {
                    double value = i == j ? 1 + noise : kernel(data.get(i).x, data.get(j).x);
                    for (int k = 0; k < j; k++) value -= lower[i][k] * lower[j][k];
                    lower[i][j] = i == j ? Math.sqrt(value) : value / lower[j][j];
                }
                likelihood += Math.log(lower[i][i]);
            }
            alpha = y.clone();
            forward(alpha);
            for (double value : alpha) likelihood += 0.5 * value * value;
            loss = likelihood;
            for (int i = data.size() - 1; i >= 0; i--) {
                for (int j = i + 1; j < data.size(); j++) alpha[i] -= lower[j][i] * alpha[j];
                alpha[i] /= lower[i][i];
            }
        }

        double[] predict(double[] x, double[] work) {
            double value = 0;
            for (int i = 0; i < data.size(); i++) {
                work[i] = kernel(x, data.get(i).x);
                value += alpha[i] * work[i];
            }
            forward(work);
            double variance = 1 + noise * 0.25;
            for (int i = 0; i < data.size(); i++) variance -= square(work[i]);
            return new double[]{mean + deviation * value, deviation * Math.sqrt(Math.max(0, variance))};
        }

        private void forward(double[] values) {
            for (int i = 0; i < data.size(); i++) {
                for (int j = 0; j < i; j++) values[i] -= lower[i][j] * values[j];
                values[i] /= lower[i][i];
            }
        }

        private double kernel(double[] a, double[] b) {
            double d = Math.sqrt(5) * distance(a, b) / length;
            return (1 + d + d * d / 3) * Math.exp(-d);
        }
    }
}
