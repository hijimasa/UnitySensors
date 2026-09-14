using System;

namespace UnitySensors.DataType.Sensor
{
    /// <summary>How a satellite's signal reaches the antenna, after ray tracing.</summary>
    public enum SatelliteVisibility
    {
        /// <summary>Direct line of sight. The only state a receiver can use cleanly.</summary>
        LineOfSight = 0,
        /// <summary>Geometry in the way and no reflected path found.</summary>
        Blocked = 1,
        /// <summary>Direct path blocked, but a reflected path reaches the antenna.</summary>
        Nlos = 2,
        /// <summary>Below the elevation mask, so not tracked at all.</summary>
        BelowMask = 3,
    }

    /// <summary>
    /// One satellite as seen from the antenna. Angles are in the ENU frame the
    /// simulator's world is published in: azimuth from North towards East,
    /// elevation above the local horizon.
    /// </summary>
    [Serializable]
    public struct SatelliteObservation
    {
        public ushort prn;
        public float azimuth;
        public float elevation;
        public SatelliteVisibility visibility;
        /// <summary>[m] extra path length of the reflected ray. 0 unless <see cref="SatelliteVisibility.Nlos"/>.</summary>
        public float excessPathLength;
        /// <summary>[dB] relative to a clean direct signal, so always &lt;= 0.</summary>
        public float relativePowerDb;
    }

    /// <summary>
    /// What the antenna can see of the sky right now.
    /// </summary>
    /// <remarks>
    /// This is the GEOMETRY only, on purpose. Turning it into a fix grade, a
    /// pseudorange error and a position is the consumer's job — in this project the
    /// GPS emulator on the ROS side — so that the decision logic can be tuned and
    /// unit-tested without rebuilding the simulator.
    /// </remarks>
    [Serializable]
    public struct GnssSkyView
    {
        /// <summary>Satellites with a direct line of sight above the elevation mask.</summary>
        public int usableSatellites;
        /// <summary>Horizontal dilution of precision of the usable set.</summary>
        public float hdop;
        /// <summary>Position dilution of precision of the usable set.</summary>
        public float pdop;
        /// <summary>Every tracked satellite, usable or not.</summary>
        public SatelliteObservation[] satellites;

        /// <summary>DOP reported when the geometry cannot be solved (fewer than four satellites).</summary>
        public const float UnsolvableDop = 99.0f;

        public static GnssSkyView None => new GnssSkyView
        {
            usableSatellites = 0,
            hdop = UnsolvableDop,
            pdop = UnsolvableDop,
            satellites = Array.Empty<SatelliteObservation>(),
        };
    }
}
