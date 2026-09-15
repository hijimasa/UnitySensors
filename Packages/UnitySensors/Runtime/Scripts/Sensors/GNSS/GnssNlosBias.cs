using System.Collections.Generic;
using UnityEngine;

using UnitySensors.DataType.Sensor;

namespace UnitySensors.Sensor.GNSS
{
    /// <summary>
    /// Where a set of biased pseudoranges puts the fix.
    /// </summary>
    /// <remarks>
    /// This is the piece that makes a simulated GNSS error correlate with the MAP
    /// rather than just with a random number generator. A satellite reaching the
    /// antenna only by reflection is measured too far away by the extra path length,
    /// and a receiver solving with that measurement lands somewhere specific. Drive
    /// the same street twice and the error repeats, which is exactly what a filter
    /// fusing GNSS with odometry cannot average away and what stationary Gaussian
    /// noise can never reproduce.
    ///
    /// The maths is the standard linearised single-point solution. For satellite i
    /// with unit vector e_i from the antenna and a pseudorange bias b_i,
    ///
    ///   dx = inv(G' G) G' b,   G row i = [-e_east, -e_north, -e_up, 1]
    ///
    /// The clock column is what absorbs a bias common to every satellite; only the
    /// part that differs between directions moves the position. That is why one
    /// biased satellite low on the horizon moves the fix much further than one
    /// overhead.
    /// </remarks>
    public static class GnssNlosBias
    {
        public struct HorizontalBias
        {
            public double east;
            public double north;
            public bool solved;
        }

        /// <summary>
        /// Horizontal displacement caused by the reflected satellites in
        /// <paramref name="satellites"/>. Satellites that are blocked outright or
        /// below the mask are not tracked and take no part.
        /// </summary>
        public static HorizontalBias Solve(IReadOnlyList<SatelliteObservation> satellites)
        {
            var bias = new HorizontalBias();
            if (satellites == null)
            {
                return bias;
            }

            var normal = new double[4, 4];
            var rhs = new double[4];
            int tracked = 0;
            bool anyBias = false;

            for (int i = 0; i < satellites.Count; i++)
            {
                SatelliteObservation satellite = satellites[i];
                double pseudorangeBias;
                if (satellite.visibility == SatelliteVisibility.LineOfSight)
                {
                    pseudorangeBias = 0.0;
                }
                else if (satellite.visibility == SatelliteVisibility.Nlos)
                {
                    pseudorangeBias = satellite.excessPathLength;
                    anyBias = true;
                }
                else
                {
                    continue;   // blocked outright, or below the mask
                }
                tracked++;

                float cosElevation = Mathf.Cos(satellite.elevation);
                double east = cosElevation * Mathf.Sin(satellite.azimuth);
                double north = cosElevation * Mathf.Cos(satellite.azimuth);
                double up = Mathf.Sin(satellite.elevation);
                // Last column is the receiver clock, which soaks up whatever bias
                // every satellite shares.
                double[] row = { -east, -north, -up, 1.0 };

                for (int r = 0; r < 4; r++)
                {
                    for (int c = 0; c < 4; c++)
                    {
                        normal[r, c] += row[r] * row[c];
                    }
                    rhs[r] += row[r] * pseudorangeBias;
                }
            }

            if (tracked < 4)
            {
                return bias;
            }
            if (!anyBias)
            {
                // Every signal is clean. Say so rather than running the solve and
                // returning a rounding error.
                bias.solved = true;
                return bias;
            }

            double[] solution;
            if (!Solve4x4(normal, rhs, out solution))
            {
                return bias;
            }
            bias.east = solution[0];
            bias.north = solution[1];
            bias.solved = true;
            return bias;
        }

        /// <summary>Gaussian elimination with partial pivoting.</summary>
        private static bool Solve4x4(double[,] a, double[] b, out double[] x)
        {
            x = new double[4];
            for (int col = 0; col < 4; col++)
            {
                int pivot = col;
                for (int row = col + 1; row < 4; row++)
                {
                    if (System.Math.Abs(a[row, col]) > System.Math.Abs(a[pivot, col]))
                    {
                        pivot = row;
                    }
                }
                // The normal matrix scales with the satellite count, so an absolute
                // floor is fine: a pivot this small means the satellites are
                // effectively coplanar.
                if (System.Math.Abs(a[pivot, col]) < 1e-12)
                {
                    return false;
                }
                if (pivot != col)
                {
                    for (int c = 0; c < 4; c++)
                    {
                        double swap = a[pivot, c];
                        a[pivot, c] = a[col, c];
                        a[col, c] = swap;
                    }
                    double swapB = b[pivot];
                    b[pivot] = b[col];
                    b[col] = swapB;
                }

                double invPivot = 1.0 / a[col, col];
                for (int row = col + 1; row < 4; row++)
                {
                    double factor = a[row, col] * invPivot;
                    if (factor == 0.0)
                    {
                        continue;
                    }
                    for (int c = col; c < 4; c++)
                    {
                        a[row, c] -= factor * a[col, c];
                    }
                    b[row] -= factor * b[col];
                }
            }

            for (int row = 3; row >= 0; row--)
            {
                double sum = b[row];
                for (int c = row + 1; c < 4; c++)
                {
                    sum -= a[row, c] * x[c];
                }
                x[row] = sum / a[row, row];
            }
            return true;
        }
    }
}
