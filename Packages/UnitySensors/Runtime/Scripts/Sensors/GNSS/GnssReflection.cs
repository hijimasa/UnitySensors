using UnityEngine;

namespace UnitySensors.Sensor.GNSS
{
    /// <summary>
    /// One-bounce specular reflection search: the maths behind an NLOS path.
    /// Pure functions, no scene access, so they can be unit-tested on their own.
    /// </summary>
    /// <remarks>
    /// Every function here is coordinate-free. Specular reflection and the excess
    /// path length are built from dot products and a Householder reflection, both
    /// of which hold in ANY orthonormal basis regardless of handedness, so the
    /// sensor can work directly in Unity's left-handed axes without converting to
    /// ENU and back.
    /// </remarks>
    public static class GnssReflection
    {
        private const float GoldenAngle = 2.39996322972865332f;

        /// <summary>
        /// How many rays a sweep of the upper hemisphere needs for a given angular
        /// spacing: the hemisphere is 2*pi steradian and each ray covers about
        /// spacing^2 of it.
        /// </summary>
        public static int RayCountForSpacing(float spacingRad)
        {
            spacingRad = Mathf.Max(spacingRad, 1e-3f);
            return Mathf.Max(1, Mathf.CeilToInt(2.0f * Mathf.PI / (spacingRad * spacingRad)));
        }

        /// <summary>
        /// Directions for the launch sweep, spread uniformly in solid angle over the
        /// hemisphere above <paramref name="up"/>, in the same basis as
        /// <paramref name="up"/> and <paramref name="east"/>.
        /// </summary>
        /// <remarks>
        /// Only the upper hemisphere is swept. For a vertical wall the reflection
        /// leaves the elevation of the launch direction unchanged, so a satellite
        /// above the horizon can only be reached by a ray above the horizon.
        /// Reflections off the ground are a different mechanism (and one a GNSS
        /// antenna's ground plane is built to suppress), so they are out of scope
        /// here rather than accidentally included.
        /// </remarks>
        public static void FillLaunchDirections(Vector3[] directions, Vector3 up, Vector3 east, Vector3 north)
        {
            if (directions == null || directions.Length == 0)
            {
                return;
            }
            int count = directions.Length;
            for (int i = 0; i < count; i++)
            {
                // Uniform in solid angle over a hemisphere is uniform in sin(elevation).
                float sinElevation = (i + 0.5f) / count;
                float cosElevation = Mathf.Sqrt(Mathf.Max(0.0f, 1.0f - sinElevation * sinElevation));
                float azimuth = i * GoldenAngle;
                directions[i] = east * (cosElevation * Mathf.Sin(azimuth))
                              + north * (cosElevation * Mathf.Cos(azimuth))
                              + up * sinElevation;
            }
        }

        /// <summary>
        /// The satellite direction a ray launched along <paramref name="launch"/> and
        /// reflected off a surface with normal <paramref name="normal"/> would have
        /// come from.
        /// </summary>
        /// <remarks>
        /// A plane wave from direction s travels along -s, reflects to
        /// -s + 2(s.n)n, and reaches the antenna when that equals -launch. The
        /// Householder reflection is its own inverse, so the required s falls out of
        /// the launch direction directly: s = launch - 2(launch.n)n. That is why the
        /// sweep can be blind -- no surface has to be identified in advance.
        /// </remarks>
        public static Vector3 RequiredSatelliteDirection(Vector3 launch, Vector3 normal)
        {
            return launch - 2.0f * Vector3.Dot(launch, normal) * normal;
        }

        /// <summary>
        /// Extra distance the reflected signal travels compared with the direct one.
        /// </summary>
        /// <remarks>
        /// The satellite is effectively at infinity, so its wavefront is a plane.
        /// With the antenna at R, the reflection point at P, d = |P - R| and
        /// u = (P - R)/d, the path via P is longer than the direct path by
        /// (R - P).s + |P - R| = d(1 - u.s). This is the quantity that ends up on
        /// the pseudorange, and it is what makes an NLOS fix wrong by tens of metres
        /// in a street while pure multipath stays at the metre level.
        /// </remarks>
        public static float ExcessPathLength(float distance, Vector3 launch, Vector3 satellite)
        {
            return distance * (1.0f - Vector3.Dot(launch, satellite));
        }
    }
}
