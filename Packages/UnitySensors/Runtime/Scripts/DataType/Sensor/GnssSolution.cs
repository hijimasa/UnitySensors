using System;

namespace UnitySensors.DataType.Sensor
{
    /// <summary>
    /// What kind of solution the receiver has. Ordered worst to best so grades can
    /// be compared directly.
    /// </summary>
    public enum RtkState
    {
        NoFix = 0,
        Single = 1,
        Float = 2,
        Fix = 3,
    }

    /// <summary>
    /// One epoch of a simulated GNSS receiver: the grade it reached, the error that
    /// puts on the reported position, and enough detail to explain why.
    /// </summary>
    /// <remarks>
    /// The horizontal error is deliberately reported in parts, because they behave
    /// differently and a consumer debugging a run needs to tell them apart:
    /// <list type="bullet">
    /// <item>a slowly drifting stochastic bias, which averaging does not remove;</item>
    /// <item>a displacement from reflected signals, which is a function of where the
    /// robot is standing and therefore repeats;</item>
    /// <item>a wrong fix, which is reported as a healthy fix and cannot be
    /// distinguished downstream at all.</item>
    /// </list>
    /// </remarks>
    [Serializable]
    public struct GnssSolution
    {
        public RtkState state;

        /// <summary>[m] total horizontal error applied to the reported position.</summary>
        public double errorEast;
        public double errorNorth;

        /// <summary>[m] the slowly drifting part alone.</summary>
        public double biasEast;
        public double biasNorth;

        /// <summary>[m] the part caused by reflected (NLOS) signals.</summary>
        public double nlosEast;
        public double nlosNorth;

        /// <summary>True while the ambiguities are resolved to the wrong integers.</summary>
        public bool wrongFix;
        /// <summary>[m] the standing offset that wrong fix produces.</summary>
        public double wrongFixEast;
        public double wrongFixNorth;

        /// <summary>Satellites with a clean line of sight above the mask.</summary>
        public int usableSatellites;
        /// <summary>Satellites reaching the antenna only by reflection.</summary>
        public int nlosSatellites;
        /// <summary>Satellites holding carrier phase long enough to resolve ambiguities.</summary>
        public int lockedSatellites;
        public float hdop;
        public float pdop;

        /// <summary>[m] 1-sigma the grade is worth, for the reported covariance.</summary>
        public double horizontalSigma;

        public static GnssSolution None => new GnssSolution
        {
            state = RtkState.NoFix,
            horizontalSigma = 100.0,
            hdop = GnssSkyView.UnsolvableDop,
            pdop = GnssSkyView.UnsolvableDop,
        };
    }
}
