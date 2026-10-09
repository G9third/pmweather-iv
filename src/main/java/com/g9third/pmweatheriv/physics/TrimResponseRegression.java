package com.g9third.pmweatheriv.physics;

import java.util.List;

/** Small least-squares fit for one independent, bounded live trim probe. */
final class TrimResponseRegression {
    private static final int MIN_SAMPLES = 40;
    private static final int MAX_LAG_TICKS = 20;
    private static final int LAG_STEP_TICKS = 2;
    private static final double MIN_EXCITATION_FRACTION = 0.15;
    private static final double MIN_DIAGONAL_RATIO = 1.0E-4;

    record Sample(double alphaPerPressure, double trim, double angleOfAttack,
                  double pitchRateOverSpeed, double elapsedTicks) {}

    record Fit(boolean accepted, String rejection, int samples, int rank,
               double conditionRatio, double trimExcitation, double slope,
               double slopeStandardError, double rSquared, int lagTicks) {
        String diagnostic() {
            return "samples=" + samples + " rank=" + rank
                + " conditionRatio=" + fmt(conditionRatio)
                + " trimExcitation=" + fmt(trimExcitation)
                + " slope=" + fmt(slope) + " slopeSE=" + fmt(slopeStandardError)
                + " rSquared=" + fmt(rSquared) + " lagTicks=" + lagTicks
                + " result=" + rejection;
        }

        private static String fmt(double value) {
            return Double.isFinite(value) ? String.format(java.util.Locale.ROOT, "%.8g", value) : "nan";
        }
    }

    private TrimResponseRegression() {}

    /** Uses identical response rows for every lag and returns the earliest confident lag. */
    static Fit fit(List<Sample> samples) {
        int n = samples == null ? 0 : samples.size();
        if (n < MIN_SAMPLES) return reject("TOO_FEW_SAMPLES", n, 0, 0);
        for (Sample sample : samples) {
            if (!valid(sample)) return reject("NON_FINITE_SAMPLE", n, 0, 0);
        }
        for (int i = 1; i < n; ++i) {
            if (Math.abs((samples.get(i).elapsedTicks() - samples.get(i - 1).elapsedTicks()) - 1.0) > 0.01) {
                return reject("NONCONTIGUOUS_SAMPLES", n, 0, 0);
            }
        }
        Fit bestRejection = reject("SLOPE_UNCERTAIN", n, 0, 0);
        double bestConfidence = Double.NEGATIVE_INFINITY;
        for (int lag = 0; lag <= MAX_LAG_TICKS; lag += LAG_STEP_TICKS) {
            Fit fit = fitAtLag(samples, lag);
            if (fit.accepted()) return fit;
            double confidence = finite(fit.slope()) && finite(fit.slopeStandardError())
                && fit.slopeStandardError() > 0.0
                ? Math.abs(fit.slope()) / fit.slopeStandardError()
                : Double.NEGATIVE_INFINITY;
            if (confidence > bestConfidence) {
                bestRejection = fit;
                bestConfidence = confidence;
            }
        }
        return bestRejection;
    }

    private static Fit fitAtLag(List<Sample> samples, int lagTicks) {
        int rows = samples.size();
        double[] y = new double[rows];
        double[] trim = new double[rows];
        double[] aoa = new double[rows];
        double[] rate = new double[rows];
        double[] elapsed = new double[rows];
        double yMean = 0.0;
        double trimMean = 0.0;
        double aoaMean = 0.0;
        double rateMean = 0.0;
        double timeMean = 0.0;
        for (int row = 0; row < rows; ++row) {
            Sample response = samples.get(row);
            // Before the first probe sample, the lagged native trim is the
            // known initial baseline value. This pads the history without
            // dropping response rows or inventing an animation time.
            Sample trimObservation = samples.get(Math.max(0, row - lagTicks));
            if (!valid(response) || !valid(trimObservation)) {
                return reject("NON_FINITE_SAMPLE", rows, 0, lagTicks);
            }
            y[row] = response.alphaPerPressure();
            trim[row] = trimObservation.trim();
            aoa[row] = response.angleOfAttack();
            rate[row] = response.pitchRateOverSpeed();
            elapsed[row] = response.elapsedTicks();
            yMean += y[row];
            trimMean += trim[row];
            aoaMean += aoa[row];
            rateMean += rate[row];
            timeMean += elapsed[row];
        }
        yMean /= rows;
        trimMean /= rows;
        aoaMean /= rows;
        rateMean /= rows;
        timeMean /= rows;

        double[] centeredY = new double[rows];
        double[] centeredTrim = new double[rows];
        double totalYSquares = 0.0;
        double totalTrimSquares = 0.0;
        double aoaScale = 0.0;
        double rateScale = 0.0;
        double timeScale = 0.0;
        for (int row = 0; row < rows; ++row) {
            centeredY[row] = y[row] - yMean;
            centeredTrim[row] = trim[row] - trimMean;
            totalYSquares += centeredY[row] * centeredY[row];
            totalTrimSquares += centeredTrim[row] * centeredTrim[row];
            aoaScale += square(aoa[row] - aoaMean);
            rateScale += square(rate[row] - rateMean);
            timeScale += square(elapsed[row] - timeMean);
        }
        double trimScale = Math.sqrt(totalTrimSquares / rows);
        aoaScale = Math.sqrt(aoaScale / rows);
        rateScale = Math.sqrt(rateScale / rows);
        timeScale = Math.sqrt(timeScale / rows);
        if (!finite(trimScale) || trimScale < 0.03) return reject("TRIM_UNEXCITED", rows, 0, lagTicks);

        // Constant and redundant predictors are handled by rank-revealing QR;
        // only predictors below physical noise floors are omitted up front.
        boolean includeAoa = aoaScale >= 1.0E-4;
        boolean includeRate = rateScale >= 1.0E-8;
        int nuisanceCount = 2 + (includeAoa ? 1 : 0) + (includeRate ? 1 : 0);
        double[][] nuisance = new double[nuisanceCount][rows];
        java.util.Arrays.fill(nuisance[0], 1.0);
        nuisance[1] = standardize(elapsed, timeMean, Math.max(timeScale, 1.0E-9));
        int column = 2;
        if (includeAoa) nuisance[column++] = standardize(aoa, aoaMean, aoaScale);
        if (includeRate) nuisance[column] = standardize(rate, rateMean, rateScale);

        Qr qr = decompose(nuisance, rows);
        if (qr.rank == 0) return reject("NUISANCE_RANK_ZERO", rows, 0, lagTicks);
        double condition = qr.diagonalRatio();
        if (!finite(condition) || condition < MIN_DIAGONAL_RATIO) {
            return new Fit(false, "NUISANCE_ILL_CONDITIONED", rows, qr.rank,
                condition, Double.NaN, Double.NaN, Double.NaN, Double.NaN, lagTicks);
        }

        double[] residualTrim = projectOut(centeredTrim, qr);
        double[] residualY = projectOut(centeredY, qr);
        double trimResidualSquares = dot(residualTrim, residualTrim);
        double excitation = totalTrimSquares > 1.0E-16
            ? Math.sqrt(trimResidualSquares / totalTrimSquares) : Double.NaN;
        if (!finite(excitation) || excitation < MIN_EXCITATION_FRACTION
            || !(trimResidualSquares > 1.0E-16)) {
            return new Fit(false, "TRIM_POORLY_EXCITED", rows, qr.rank,
                condition, excitation, Double.NaN, Double.NaN, Double.NaN, lagTicks);
        }

        double slope = dot(residualTrim, residualY) / trimResidualSquares;
        double sumSquares = 0.0;
        for (int row = 0; row < rows; ++row) {
            double residual = residualY[row] - slope * residualTrim[row];
            sumSquares += residual * residual;
        }
        int fullRank = qr.rank + 1;
        double degreesOfFreedom = rows - fullRank;
        if (!(degreesOfFreedom > 0.0)) {
            return new Fit(false, "INSUFFICIENT_DOF", rows, fullRank,
                condition, excitation, slope, Double.NaN, Double.NaN, lagTicks);
        }
        double residualVariance = sumSquares / degreesOfFreedom;
        double slopeStandardError = Math.sqrt(Math.max(0.0, residualVariance / trimResidualSquares));
        double rSquared = totalYSquares > 1.0E-24 ? 1.0 - sumSquares / totalYSquares : 0.0;
        boolean confident = finite(slope) && finite(slopeStandardError)
            && Math.abs(slope) > Math.max(1.0E-9, slopeStandardError * 3.0);
        return new Fit(confident, confident ? "ACCEPTED" : "SLOPE_UNCERTAIN",
            rows, fullRank, condition, excitation, slope, slopeStandardError, rSquared, lagTicks);
    }

    private static boolean valid(Sample sample) {
        return sample != null && finite(sample.alphaPerPressure()) && finite(sample.trim())
            && finite(sample.angleOfAttack()) && finite(sample.pitchRateOverSpeed())
            && finite(sample.elapsedTicks());
    }

    private static double[] projectOut(double[] values, Qr qr) {
        double[] residual = values.clone();
        for (int k = 0; k < qr.rank; ++k) {
            double projection = dot(qr.q[k], values);
            for (int row = 0; row < residual.length; ++row) residual[row] -= projection * qr.q[k][row];
        }
        return residual;
    }

    private static double[] standardize(double[] values, double mean, double scale) {
        double[] out = new double[values.length];
        for (int i = 0; i < values.length; ++i) out[i] = (values[i] - mean) / scale;
        return out;
    }

    private static Fit reject(String reason, int samples, int rank, int lagTicks) {
        return new Fit(false, reason, samples, rank, Double.NaN, Double.NaN,
            Double.NaN, Double.NaN, Double.NaN, lagTicks);
    }

    private static double square(double value) { return value * value; }
    private static boolean finite(double value) { return Double.isFinite(value); }

    private static double dot(double[] a, double[] b) {
        double sum = 0.0;
        for (int i = 0; i < a.length; ++i) sum += a[i] * b[i];
        return sum;
    }

    private static double norm(double[] values) { return Math.sqrt(dot(values, values)); }

    /** Modified Gram-Schmidt with column pivoting; dependent nuisance terms reduce rank safely. */
    private static Qr decompose(double[][] inputColumns, int rows) {
        int count = inputColumns.length;
        double[][] work = new double[count][rows];
        double[][] q = new double[count][rows];
        double referenceNorm = 0.0;
        for (int j = 0; j < count; ++j) {
            work[j] = inputColumns[j].clone();
            referenceNorm = Math.max(referenceNorm, norm(work[j]));
        }
        double rankFloor = Math.max(1.0E-9, referenceNorm * MIN_DIAGONAL_RATIO);
        int rank = 0;
        double maxDiagonal = 0.0;
        double minDiagonal = Double.POSITIVE_INFINITY;
        for (int k = 0; k < count; ++k) {
            int pivot = k;
            double pivotNorm = norm(work[k]);
            for (int j = k + 1; j < count; ++j) {
                double candidateNorm = norm(work[j]);
                if (candidateNorm > pivotNorm) {
                    pivot = j;
                    pivotNorm = candidateNorm;
                }
            }
            if (pivot != k) {
                double[] swappedColumn = work[k]; work[k] = work[pivot]; work[pivot] = swappedColumn;
            }
            double diagonal = norm(work[k]);
            if (!finite(diagonal) || diagonal < rankFloor) break;
            q[k] = work[k].clone();
            for (int row = 0; row < rows; ++row) q[k][row] /= diagonal;
            maxDiagonal = Math.max(maxDiagonal, diagonal);
            minDiagonal = Math.min(minDiagonal, diagonal);
            ++rank;
            for (int j = k + 1; j < count; ++j) {
                double first = dot(q[k], work[j]);
                for (int row = 0; row < rows; ++row) work[j][row] -= first * q[k][row];
                double correction = dot(q[k], work[j]);
                for (int row = 0; row < rows; ++row) work[j][row] -= correction * q[k][row];
            }
        }
        double ratio = rank == 0 || maxDiagonal == 0.0 ? Double.NaN : minDiagonal / maxDiagonal;
        return new Qr(q, rank, ratio);
    }

    private record Qr(double[][] q, int rank, double diagonalRatio) {}
}
