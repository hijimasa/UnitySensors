using System;

namespace UnitySensors.DataType.Sensor
{
    /// <summary>
    /// Which magnetic polarity a piece of tape presents to the sensor. Guide sensors
    /// (Roboteq MGS1600 / Accurate UDS-1213) follow one polarity as the track and
    /// report the opposite polarity, placed beside the track, as markers.
    /// </summary>
    public enum MagneticPolarity
    {
        Track = 0,
        Marker = 1,
    }

    /// <summary>Which track the sensor reports when it sees two (a fork or merge).</summary>
    public enum MagneticForkSelection
    {
        /// <summary>The track closest to the previously reported one (sticky).</summary>
        Nearest = 0,
        Left = 1,
        Right = 2,
    }

    /// <summary>One measurement of a magnetic guide sensor. Lateral positions are in
    /// metres, positive to the sensor's left, zero at the sensor centre.</summary>
    [Serializable]
    public struct MagneticGuideReading
    {
        public bool trackDetected;
        /// <summary>Lateral position of the selected track [m], +left. 0 when no track.</summary>
        public float trackPosition;
        public bool leftMarkerDetected;
        public bool rightMarkerDetected;
        /// <summary>Every track under the sensor, left to right (forks show two).</summary>
        public float[] trackPositions;

        public static MagneticGuideReading None => new MagneticGuideReading
        {
            trackDetected = false,
            trackPosition = 0.0f,
            leftMarkerDetected = false,
            rightMarkerDetected = false,
            trackPositions = Array.Empty<float>(),
        };
    }
}
