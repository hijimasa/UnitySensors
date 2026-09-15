using System;
using System.Collections.Generic;
using UnityEngine;

using UnitySensors.DataType.Sensor;

namespace UnitySensors.Sensor.GNSS
{
    /// <summary>Sky requirement and error model of one solution grade.</summary>
    [Serializable]
    public struct RtkGrade
    {
        public int minSatellites;
        public float maxHdop;
        /// <summary>[m] steady-state 1-sigma of the slowly drifting bias.</summary>
        public float biasSigma;
        /// <summary>[s] correlation time of that bias.</summary>
        public float biasTau;
        /// <summary>[m] 1-sigma of the uncorrelated scatter on top.</summary>
        public float whiteSigma;
        /// <summary>Fraction of the geometric NLOS displacement this grade shows.</summary>
        public float nlosScale;
        /// <summary>[m] cap on that displacement, 0 for none.</summary>
        public float nlosLimit;
        /// <summary>[m] 1-sigma this grade is worth, for the reported covariance.</summary>
        public float reportedSigma;
    }

    /// <summary>
    /// A receiver's solution grade and the error that comes with it, driven by what
    /// the sky actually looks like from the antenna.
    /// </summary>
    /// <remarks>
    /// Two things make this worth having over a plain Gaussian on the truth.
    ///
    /// In FLOAT the error is a FIRST-ORDER GAUSS-MARKOV process, not white noise. It
    /// stays correlated for tens of seconds, so averaging does not remove it. That is
    /// what actually breaks a filter fusing GNSS with odometry; white noise is the
    /// easy case.
    ///
    /// The bias is CONTINUOUS across a grade change. Losing the fix does not teleport
    /// the solution: it starts from wherever it was and degrades towards the new
    /// grade's sigma with that grade's time constant, which is how a real receiver's
    /// float solution drifts out and re-converges back in.
    /// </remarks>
    [Serializable]
    public class RtkQualityModel
    {
        /// A conservative ambiguity-resolution threshold, and the centimetre-level
        /// jitter of a short-baseline fix. The 4.8 cm cap is a quarter of the L1
        /// wavelength, which bounds a fixed solution's carrier multipath whatever
        /// the reflected path length.
        public RtkGrade fix = new RtkGrade
        {
            minSatellites = 6, maxHdop = 2.5f, biasSigma = 0.010f, biasTau = 2.0f,
            whiteSigma = 0.008f, nlosScale = 0.05f, nlosLimit = 0.048f, reportedSigma = 0.02f,
        };
        /// A float solution on a short baseline sits in the decimetre range and
        /// wanders over tens of seconds.
        public RtkGrade floatGrade = new RtkGrade
        {
            minSatellites = 5, maxHdop = 6.0f, biasSigma = 0.350f, biasTau = 30.0f,
            whiteSigma = 0.030f, nlosScale = 0.1f, nlosLimit = 0.0f, reportedSigma = 0.30f,
        };
        /// Code-only single-point positioning: metre level, drifting even more slowly.
        public RtkGrade single = new RtkGrade
        {
            minSatellites = 4, maxHdop = 99.0f, biasSigma = 1.500f, biasTau = 60.0f,
            whiteSigma = 0.300f, nlosScale = 1.0f, nlosLimit = 0.0f, reportedSigma = 1.50f,
        };

        /// <summary>
        /// [s] of fix-capable sky needed before FLOAT -> FIX when per-satellite lock
        /// is not available. Any drop below fix-capable resets it.
        /// </summary>
        public float reconvergenceSeconds = 8.0f;
        /// <summary>
        /// [s] of unbroken carrier phase a satellite needs before it can help resolve
        /// the ambiguities.
        /// </summary>
        public float lockSecondsForFix = 8.0f;
        /// <summary>
        /// Start already converged, as a robot that has been sitting outdoors would
        /// be. False makes the receiver come up in FLOAT and converge, which is
        /// realistic but puts a transient at the start of every mission.
        /// </summary>
        public bool startConverged = true;

        /// <summary>
        /// Include the random part of the error. Turning it off leaves only what the
        /// geometry produces, which is the half that repeats when the robot returns
        /// to the same place -- useful when that repeatability is the thing being
        /// demonstrated or regression-tested.
        /// </summary>
        public bool stochasticError = true;

        /// <summary>
        /// Probability that resolving the ambiguities lands on the WRONG integers.
        /// A wrong fix is the nastiest failure a receiver has, because it is reported
        /// as a fix, with full confidence, and nothing downstream can tell: the
        /// quality flag says RTK fixed while the position is out by a decimetre or
        /// more. Real receivers are usually quoted around 1% in difficult conditions.
        /// </summary>
        public float wrongFixProbability = 0.01f;
        /// <summary>
        /// [m] bounds of the offset a wrong fix produces. An integer error in the
        /// double-differenced ambiguities lands the solution on a lattice point,
        /// which in practice means a decimetre or a few. Uniform between these
        /// because the real distribution depends on the baseline and constellation.
        /// </summary>
        public float wrongFixMinOffset = 0.08f;
        public float wrongFixMaxOffset = 0.40f;

        private RtkState _state = RtkState.Fix;
        private float _fixLockSeconds;
        private double _biasEast, _biasNorth;
        private bool _wrongFix;
        private double _wrongFixEast, _wrongFixNorth;
        private System.Random _random;
        private double _spareGaussian;
        private bool _hasSpareGaussian;

        public RtkState state { get { return _state; } }
        public bool wrongFix { get { return _wrongFix; } }

        public RtkQualityModel(int seed = 20260914)
        {
            Reset(seed);
        }

        public void Reset(int seed)
        {
            _random = new System.Random(seed);
            _hasSpareGaussian = false;
            _biasEast = _biasNorth = 0.0;
            _wrongFix = false;
            _wrongFixEast = _wrongFixNorth = 0.0;
            if (startConverged)
            {
                _state = RtkState.Fix;
                _fixLockSeconds = reconvergenceSeconds;
            }
            else
            {
                _state = RtkState.Float;
                _fixLockSeconds = 0.0f;
            }
        }

        /// <summary>
        /// Advance the model by <paramref name="dt"/> seconds.
        /// </summary>
        /// <param name="satellites">every tracked satellite, for the reflected-path
        /// displacement. May be null.</param>
        /// <param name="satellitesLocked">how many satellites have held carrier phase
        /// long enough to help resolve the ambiguities, or a negative number when
        /// per-satellite lock is not available (then the global timer decides).</param>
        public GnssSolution Update(int usableSatellites, float hdop, float pdop,
            IReadOnlyList<SatelliteObservation> satellites, float dt, int satellitesLocked)
        {
            if (!(dt > 0.0f))
            {
                dt = 0.0f;
            }

            // The sky sets the ceiling; ambiguity resolution decides whether we reach it.
            RtkState ceiling = RtkState.NoFix;
            if (usableSatellites >= fix.minSatellites && hdop <= fix.maxHdop)
            {
                ceiling = RtkState.Fix;
            }
            else if (usableSatellites >= floatGrade.minSatellites && hdop <= floatGrade.maxHdop)
            {
                ceiling = RtkState.Float;
            }
            else if (usableSatellites >= single.minSatellites)
            {
                ceiling = RtkState.Single;
            }

            RtkState previous = _state;
            if (ceiling == RtkState.Fix)
            {
                _fixLockSeconds += dt;
                bool resolved;
                if (satellitesLocked >= 0)
                {
                    // Per-satellite continuity: losing two satellites to a pole costs
                    // far less than losing all of them to a bridge, which a single
                    // global timer cannot express.
                    resolved = satellitesLocked >= fix.minSatellites;
                }
                else
                {
                    resolved = _fixLockSeconds >= reconvergenceSeconds;
                }
                _state = resolved ? RtkState.Fix : RtkState.Float;
            }
            else
            {
                // Anything short of fix-capable sky is a break in carrier-phase
                // continuity: the ambiguities have to be resolved from scratch.
                _fixLockSeconds = 0.0f;
                _state = ceiling;
            }

            if (_state == RtkState.Fix)
            {
                if (previous != RtkState.Fix)
                {
                    ResolveAmbiguities();
                }
            }
            else if (_wrongFix)
            {
                // Leaving the fix discards the integers, wrong ones included.
                _wrongFix = false;
                _wrongFixEast = _wrongFixNorth = 0.0;
            }

            RtkGrade grade = GradeOf(_state);

            // First-order Gauss-Markov, exact discretisation:
            //   b[k+1] = a*b[k] + sigma*sqrt(1 - a^2)*w,   a = exp(-dt/tau)
            // b carries over unchanged across a grade change, so the error walks into
            // its new regime over tau instead of jumping.
            double decay = (grade.biasTau > 0.0f) ? Math.Exp(-dt / grade.biasTau) : 0.0;
            double sigma = stochasticError ? grade.biasSigma : 0.0f;
            double drive = sigma * Math.Sqrt(Math.Max(0.0, 1.0 - decay * decay));
            _biasEast = decay * _biasEast + drive * NextGaussian();
            _biasNorth = decay * _biasNorth + drive * NextGaussian();

            var solution = new GnssSolution
            {
                state = _state,
                biasEast = _biasEast,
                biasNorth = _biasNorth,
                usableSatellites = usableSatellites,
                hdop = hdop,
                pdop = pdop,
                lockedSatellites = satellitesLocked,
                horizontalSigma = grade.reportedSigma,
                wrongFix = _wrongFix,
                wrongFixEast = _wrongFixEast,
                wrongFixNorth = _wrongFixNorth,
            };

            if (satellites != null && grade.nlosScale > 0.0f)
            {
                int nlos = 0;
                for (int i = 0; i < satellites.Count; i++)
                {
                    if (satellites[i].visibility == SatelliteVisibility.Nlos)
                    {
                        nlos++;
                    }
                }
                solution.nlosSatellites = nlos;

                GnssNlosBias.HorizontalBias bias = GnssNlosBias.Solve(satellites);
                if (bias.solved)
                {
                    double east = grade.nlosScale * bias.east;
                    double north = grade.nlosScale * bias.north;
                    // Cap it where the physics caps it. On a fixed solution the
                    // reflected signal reaches the position through the carrier
                    // phase, whose error cannot exceed a quarter wavelength however
                    // long the detour was; the direction still comes from geometry.
                    if (grade.nlosLimit > 0.0f)
                    {
                        double magnitude = Math.Sqrt(east * east + north * north);
                        if (magnitude > grade.nlosLimit)
                        {
                            double shrink = grade.nlosLimit / magnitude;
                            east *= shrink;
                            north *= shrink;
                        }
                    }
                    solution.nlosEast = east;
                    solution.nlosNorth = north;
                }
            }

            double white = stochasticError ? grade.whiteSigma : 0.0f;
            solution.errorEast = _biasEast + white * NextGaussian() + solution.nlosEast + _wrongFixEast;
            solution.errorNorth = _biasNorth + white * NextGaussian() + solution.nlosNorth + _wrongFixNorth;
            return solution;
        }

        private RtkGrade GradeOf(RtkState state)
        {
            switch (state)
            {
                case RtkState.Fix:
                    return fix;
                case RtkState.Float:
                    return floatGrade;
                default:
                    // NO_FIX keeps drifting on the single-point model so that when the
                    // sky reopens the solution reappears where it plausibly wandered
                    // to, rather than snapping back.
                    return single;
            }
        }

        private void ResolveAmbiguities()
        {
            if (_random.NextDouble() >= wrongFixProbability)
            {
                _wrongFix = false;
                _wrongFixEast = _wrongFixNorth = 0.0;
                return;
            }

            // Wrong integers. The offset is fixed for as long as this fix lasts: it is
            // not noise, it is the solution sitting on the wrong lattice point, and it
            // will sit there reporting a healthy RTK fix until something forces a
            // re-resolve.
            double magnitude = wrongFixMinOffset +
                _random.NextDouble() * Math.Max(0.0, wrongFixMaxOffset - wrongFixMinOffset);
            double bearing = 2.0 * Math.PI * _random.NextDouble();
            _wrongFix = true;
            _wrongFixEast = magnitude * Math.Sin(bearing);
            _wrongFixNorth = magnitude * Math.Cos(bearing);
        }

        /// <summary>Box-Muller, so the stream is reproducible from the seed alone.</summary>
        private double NextGaussian()
        {
            if (_hasSpareGaussian)
            {
                _hasSpareGaussian = false;
                return _spareGaussian;
            }
            double u1, u2;
            do
            {
                u1 = _random.NextDouble();
            } while (u1 <= double.Epsilon);
            u2 = _random.NextDouble();

            double magnitude = Math.Sqrt(-2.0 * Math.Log(u1));
            _spareGaussian = magnitude * Math.Sin(2.0 * Math.PI * u2);
            _hasSpareGaussian = true;
            return magnitude * Math.Cos(2.0 * Math.PI * u2);
        }
    }
}
