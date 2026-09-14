using System;
using UnityEngine;

using UnitySensors.DataType.Sensor;

namespace UnitySensors.Sensor.GNSS
{
    /// <summary>
    /// Where the satellites are, and how good the geometry of a visible subset is.
    /// Pure maths, no scene access, so it can be unit-tested on its own.
    /// </summary>
    public static class GnssConstellation
    {
        /// <summary>Golden angle, the spacing that keeps a spiral lattice from forming spokes.</summary>
        private const float GoldenAngle = 2.39996322972865332f;

        /// <summary>
        /// Lay out <paramref name="count"/> satellites over the sky above
        /// <paramref name="elevationMaskRad"/>, uniformly in solid angle.
        /// </summary>
        /// <remarks>
        /// A synthetic constellation, not an almanac. It is deterministic (the same
        /// seed always gives the same sky) and well spread, which is what a geometry
        /// study needs. It is NOT the real thing: a real mid-latitude GNSS sky is
        /// denser at middle elevations and has a hole towards the pole, because the
        /// orbits are inclined. Feeding a real almanac in belongs to a later stage;
        /// until then, do not read absolute DOP values as if they were a site survey.
        ///
        /// Satellites are held still for the whole run. Over a SILS run of minutes
        /// the real geometry moves well under a degree, so the error is negligible
        /// and a frozen sky keeps runs repeatable.
        /// </remarks>
        public static SatelliteObservation[] Generate(int count, float elevationMaskRad, int seed)
        {
            count = Mathf.Max(0, count);
            elevationMaskRad = Mathf.Clamp(elevationMaskRad, 0.0f, 0.5f * Mathf.PI - 1e-3f);

            var satellites = new SatelliteObservation[count];
            if (count == 0)
            {
                return satellites;
            }

            // Uniform in solid angle over the cap means uniform in sin(elevation).
            float sinMask = Mathf.Sin(elevationMaskRad);
            // A seeded offset rotates and re-phases the whole lattice, so different
            // seeds are different skies rather than the same sky relabelled.
            var random = new System.Random(seed);
            float azimuthOffset = (float)random.NextDouble() * 2.0f * Mathf.PI;
            float elevationJitter = (float)random.NextDouble();

            for (int i = 0; i < count; i++)
            {
                float t = (i + elevationJitter) / count;
                float sinElevation = sinMask + t * (1.0f - sinMask);
                satellites[i] = new SatelliteObservation
                {
                    prn = (ushort)(i + 1),
                    elevation = Mathf.Asin(Mathf.Clamp(sinElevation, -1.0f, 1.0f)),
                    azimuth = Mathf.Repeat(azimuthOffset + i * GoldenAngle, 2.0f * Mathf.PI),
                    visibility = SatelliteVisibility.LineOfSight,
                    excessPathLength = 0.0f,
                    relativePowerDb = 0.0f,
                };
            }
            return satellites;
        }

        /// <summary>
        /// Unit vector from the antenna towards a satellite, in the ENU frame
        /// (x East, y North, z Up).
        /// </summary>
        public static Vector3 ToEnuDirection(float azimuthRad, float elevationRad)
        {
            float cosElevation = Mathf.Cos(elevationRad);
            return new Vector3(
                cosElevation * Mathf.Sin(azimuthRad),   // East
                cosElevation * Mathf.Cos(azimuthRad),   // North
                Mathf.Sin(elevationRad));               // Up
        }

        /// <summary>
        /// Same direction expressed in Unity's left-handed axes.
        /// </summary>
        /// <remarks>
        /// The simulator publishes its world as ROS ENU with x = Unity z and
        /// y = -Unity x (see GroundTruthPub), so East is Unity +z, North is Unity -x
        /// and Up is Unity +y. Note this is NOT the mapping GeoCoordinateConverter
        /// uses internally (it treats Unity x as easting); the two conventions
        /// coexist in this package, and the ROS-facing one is the one that matters
        /// for anything published alongside the ground truth.
        /// </remarks>
        public static Vector3 ToUnityDirection(float azimuthRad, float elevationRad)
        {
            Vector3 enu = ToEnuDirection(azimuthRad, elevationRad);
            return new Vector3(-enu.y, enu.z, enu.x);
        }

        /// <summary>
        /// Dilution of precision of the satellites marked
        /// <see cref="SatelliteVisibility.LineOfSight"/> in <paramref name="satellites"/>.
        /// </summary>
        /// <remarks>
        /// Standard construction: each usable satellite contributes a row
        /// [-e, -n, -u, 1] of the geometry matrix G (the unit vector towards it, plus
        /// the receiver clock term). With Q = inv(G'G), HDOP = sqrt(Q00 + Q11) and
        /// PDOP = sqrt(Q00 + Q11 + Q22). Fewer than four usable satellites, or a
        /// degenerate geometry, leaves the system unsolvable.
        /// </remarks>
        /// <returns>false when the geometry cannot be solved; the outputs are then
        /// <see cref="GnssSkyView.UnsolvableDop"/>.</returns>
        public static bool TryComputeDop(SatelliteObservation[] satellites, out float hdop, out float pdop)
        {
            hdop = GnssSkyView.UnsolvableDop;
            pdop = GnssSkyView.UnsolvableDop;
            if (satellites == null)
            {
                return false;
            }

            // Normal matrix G'G, accumulated without ever forming G.
            var normal = new double[4, 4];
            int usable = 0;
            foreach (SatelliteObservation satellite in satellites)
            {
                if (satellite.visibility != SatelliteVisibility.LineOfSight)
                {
                    continue;
                }
                usable++;

                Vector3 enu = ToEnuDirection(satellite.azimuth, satellite.elevation);
                double[] row = { -enu.x, -enu.y, -enu.z, 1.0 };
                for (int r = 0; r < 4; r++)
                {
                    for (int c = 0; c < 4; c++)
                    {
                        normal[r, c] += row[r] * row[c];
                    }
                }
            }

            if (usable < 4 || !TryInvert4x4(normal, out double[,] covariance))
            {
                return false;
            }

            double horizontal = covariance[0, 0] + covariance[1, 1];
            double position = horizontal + covariance[2, 2];
            if (horizontal < 0.0 || position < 0.0 || double.IsNaN(position))
            {
                return false;
            }

            hdop = (float)Math.Sqrt(horizontal);
            pdop = (float)Math.Sqrt(position);
            return true;
        }

        /// <summary>Gauss-Jordan inverse with partial pivoting.</summary>
        private static bool TryInvert4x4(double[,] matrix, out double[,] inverse)
        {
            const int n = 4;
            var work = new double[n, 2 * n];
            for (int r = 0; r < n; r++)
            {
                for (int c = 0; c < n; c++)
                {
                    work[r, c] = matrix[r, c];
                }
                work[r, n + r] = 1.0;
            }

            for (int col = 0; col < n; col++)
            {
                int pivot = col;
                for (int r = col + 1; r < n; r++)
                {
                    if (Math.Abs(work[r, col]) > Math.Abs(work[pivot, col]))
                    {
                        pivot = r;
                    }
                }
                // The normal matrix is scaled by the satellite count, so an absolute
                // threshold is fine here: a pivot this small means the satellites are
                // effectively coplanar.
                if (Math.Abs(work[pivot, col]) < 1e-12)
                {
                    inverse = null;
                    return false;
                }
                if (pivot != col)
                {
                    for (int c = 0; c < 2 * n; c++)
                    {
                        (work[col, c], work[pivot, c]) = (work[pivot, c], work[col, c]);
                    }
                }

                double scale = 1.0 / work[col, col];
                for (int c = 0; c < 2 * n; c++)
                {
                    work[col, c] *= scale;
                }
                for (int r = 0; r < n; r++)
                {
                    if (r == col)
                    {
                        continue;
                    }
                    double factor = work[r, col];
                    if (factor == 0.0)
                    {
                        continue;
                    }
                    for (int c = 0; c < 2 * n; c++)
                    {
                        work[r, c] -= factor * work[col, c];
                    }
                }
            }

            inverse = new double[n, n];
            for (int r = 0; r < n; r++)
            {
                for (int c = 0; c < n; c++)
                {
                    inverse[r, c] = work[r, n + c];
                }
            }
            return true;
        }
    }
}
