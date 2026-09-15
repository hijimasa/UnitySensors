using System;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnitySensors.DataType.Sensor;
using UnitySensors.Sensor.GNSS;

namespace UnitySensors.Tests.Editor
{
    /// <summary>
    /// The receiver model: which grade the sky allows, how long a lost fix takes to
    /// come back, and what error each grade puts on the position.
    /// </summary>
    public class RtkQualityModelTests
    {
        const int OpenSats = 14;
        const float OpenHdop = 0.8f;
        const int FloatSats = 5;      // float-capable, not fix-capable
        const float FloatHdop = 3.0f;
        const int SingleSats = 4;
        const float SingleHdop = 9.0f;
        const int NoSats = 2;
        const float NoHdop = 30.0f;

        static GnssSolution Run(RtkQualityModel model, int sats, float hdop, float seconds,
            int locked = -1, float dt = 0.2f)
        {
            var out_ = new GnssSolution();
            for (int i = 0; i < (int)(seconds / dt); i++)
            {
                out_ = model.Update(sats, hdop, hdop * 2.0f, null, dt, locked);
            }
            return out_;
        }

        [Test]
        public void StartsConvergedUnderOpenSky()
        {
            var model = new RtkQualityModel(1);
            Assert.AreEqual(RtkState.Fix, model.state);
            Assert.AreEqual(RtkState.Fix, model.Update(OpenSats, OpenHdop, 1.0f, null, 0.2f, -1).state);
        }

        [Test]
        public void ComesUpInFloatWhenNotStartedConverged()
        {
            var model = new RtkQualityModel(1);
            model.startConverged = false;
            model.Reset(1);

            Assert.AreEqual(RtkState.Float, model.Update(OpenSats, OpenHdop, 1.0f, null, 0.2f, -1).state);
            Assert.AreEqual(RtkState.Float, Run(model, OpenSats, OpenHdop, model.reconvergenceSeconds - 1.0f).state);
            Assert.AreEqual(RtkState.Fix, Run(model, OpenSats, OpenHdop, 2.0f).state);
        }

        [Test]
        public void SkyLadderSelectsTheGrade()
        {
            var model = new RtkQualityModel(1);
            Assert.AreEqual(RtkState.Float, model.Update(FloatSats, FloatHdop, 6.0f, null, 0.2f, -1).state);
            Assert.AreEqual(RtkState.Single, model.Update(SingleSats, SingleHdop, 18.0f, null, 0.2f, -1).state);
            Assert.AreEqual(RtkState.NoFix, model.Update(NoSats, NoHdop, 60.0f, null, 0.2f, -1).state);
        }

        [Test]
        public void LosingFixCostsTheFullReconvergenceTime()
        {
            var model = new RtkQualityModel(7);
            Assert.AreEqual(RtkState.Fix, model.state);

            // A single degraded epoch is enough to break carrier-phase continuity.
            Assert.AreEqual(RtkState.Float, model.Update(FloatSats, FloatHdop, 6.0f, null, 0.2f, -1).state);
            Assert.AreEqual(RtkState.Float, Run(model, OpenSats, OpenHdop, model.reconvergenceSeconds - 1.0f).state);
            Assert.AreEqual(RtkState.Fix, Run(model, OpenSats, OpenHdop, 2.0f).state);
        }

        [Test]
        public void BiasIsContinuousAcrossAGradeChange()
        {
            var model = new RtkQualityModel(3);
            var fixed_ = Run(model, OpenSats, OpenHdop, 30.0f);
            Assert.AreEqual(RtkState.Fix, fixed_.state);

            var firstFloat = model.Update(FloatSats, FloatHdop, 6.0f, null, 0.2f, -1);
            Assert.AreEqual(RtkState.Float, firstFloat.state);
            // The float solution starts from where the fixed one was; it must not
            // teleport to the float sigma in one step.
            double moved = Math.Sqrt(Math.Pow(firstFloat.biasEast - fixed_.biasEast, 2) +
                                     Math.Pow(firstFloat.biasNorth - fixed_.biasNorth, 2));
            Assert.Less(moved, 0.10);
        }

        [Test]
        public void FloatErrorGrowsToItsSigmaAndStaysCorrelated()
        {
            var model = new RtkQualityModel(11);
            Run(model, OpenSats, OpenHdop, 30.0f);

            const float dt = 0.2f;
            const int warmup = 1000;
            const int samples = 200000;
            var series = new double[samples];
            for (int i = 0; i < warmup + samples; i++)
            {
                var step = model.Update(FloatSats, FloatHdop, 6.0f, null, dt, -1);
                if (i >= warmup) series[i - warmup] = step.biasEast;
            }

            double sumSq = 0.0;
            for (int i = 0; i < samples; i++) sumSq += series[i] * series[i];
            double variance = sumSq / samples;
            double rms = Math.Sqrt(variance);
            Assert.Greater(rms, 0.8 * model.floatGrade.biasSigma);
            Assert.Less(rms, 1.2 * model.floatGrade.biasSigma);

            // The point of the Gauss-Markov model: one tau apart the error is still
            // correlated, so averaging does not remove it. White noise would give ~0.
            int lag = (int)(model.floatGrade.biasTau / dt);
            double cov = 0.0;
            for (int i = 0; i + lag < samples; i++) cov += series[i] * series[i + lag];
            double autocorrelation = (cov / (samples - lag)) / variance;
            Assert.AreEqual(Math.Exp(-1.0), autocorrelation, 0.08, "actual " + autocorrelation);
        }

        [Test]
        public void FixedErrorIsCentimetreLevel()
        {
            var model = new RtkQualityModel(5);
            double worst = 0.0;
            for (int i = 0; i < 1500; i++)
            {
                var step = model.Update(OpenSats, OpenHdop, 1.0f, null, 0.2f, -1);
                worst = Math.Max(worst, Math.Sqrt(step.errorEast * step.errorEast +
                                                  step.errorNorth * step.errorNorth));
            }
            Assert.Less(worst, 0.10, "a short-baseline fix must stay at the centimetre level");
        }

        [Test]
        public void EnoughLockedSatellitesFixWithoutWaitingOutTheGlobalTimer()
        {
            var model = new RtkQualityModel(1);
            model.startConverged = false;
            model.reconvergenceSeconds = 60.0f;   // absurd, to prove it is not what decides
            model.Reset(1);

            var step = model.Update(OpenSats, OpenHdop, 1.0f, null, 0.2f, model.fix.minSatellites);
            Assert.AreEqual(RtkState.Fix, step.state);
        }

        [Test]
        public void TooFewLockedSatellitesStayFloatUnderAPerfectSky()
        {
            var model = new RtkQualityModel(1);
            var step = model.Update(OpenSats, OpenHdop, 1.0f, null, 0.2f, model.fix.minSatellites - 1);
            Assert.AreEqual(RtkState.Float, step.state);
        }

        [Test]
        public void WrongFixesAreOffWhenTheProbabilityIsZero()
        {
            var model = new RtkQualityModel(4);
            model.wrongFixProbability = 0.0f;
            for (int i = 0; i < 50; i++)
            {
                model.Update(FloatSats, FloatHdop, 6.0f, null, 0.2f, -1);
                Run(model, OpenSats, OpenHdop, model.reconvergenceSeconds + 1.0f);
                Assert.IsFalse(model.wrongFix);
            }
        }

        [Test]
        public void AWrongFixIsStillReportedAsAFix()
        {
            var model = new RtkQualityModel(4);
            model.wrongFixProbability = 1.0f;
            model.Update(FloatSats, FloatHdop, 6.0f, null, 0.2f, -1);
            var step = Run(model, OpenSats, OpenHdop, model.reconvergenceSeconds + 1.0f);

            // This is the whole point: nothing downstream can tell.
            Assert.AreEqual(RtkState.Fix, step.state);
            Assert.IsTrue(step.wrongFix);
            double offset = Math.Sqrt(step.wrongFixEast * step.wrongFixEast +
                                      step.wrongFixNorth * step.wrongFixNorth);
            Assert.GreaterOrEqual(offset, model.wrongFixMinOffset - 1e-6);
            Assert.LessOrEqual(offset, model.wrongFixMaxOffset + 1e-6);
        }

        [Test]
        public void AWrongFixIsAStandingOffsetNotNoise()
        {
            var model = new RtkQualityModel(9);
            model.wrongFixProbability = 1.0f;
            model.Update(FloatSats, FloatHdop, 6.0f, null, 0.2f, -1);
            var first = Run(model, OpenSats, OpenHdop, model.reconvergenceSeconds + 1.0f);
            Assert.IsTrue(first.wrongFix);

            // Held for a minute the offset must not move: the solution is sitting on
            // the wrong lattice point, it is not being re-drawn every epoch.
            var later = Run(model, OpenSats, OpenHdop, 60.0f);
            Assert.AreEqual(first.wrongFixEast, later.wrongFixEast, 1e-12);
            Assert.AreEqual(first.wrongFixNorth, later.wrongFixNorth, 1e-12);
            Assert.AreEqual(first.wrongFixEast, later.errorEast - later.biasEast, 0.05,
                "the offset must show up in the reported error");
        }

        [Test]
        public void LosingTheFixDiscardsTheWrongIntegers()
        {
            var model = new RtkQualityModel(9);
            model.wrongFixProbability = 1.0f;
            model.Update(FloatSats, FloatHdop, 6.0f, null, 0.2f, -1);
            Assert.IsTrue(Run(model, OpenSats, OpenHdop, model.reconvergenceSeconds + 1.0f).wrongFix);

            var dropped = model.Update(FloatSats, FloatHdop, 6.0f, null, 0.2f, -1);
            Assert.IsFalse(dropped.wrongFix);
            Assert.AreEqual(0.0, dropped.wrongFixEast, 1e-12);
            Assert.AreEqual(0.0, dropped.wrongFixNorth, 1e-12);
        }

        [Test]
        public void AFixedSolutionCapsTheReflectionErrorAtAQuarterWavelength()
        {
            // A code bias of tens of metres cannot move a carrier-phase solution
            // further than a quarter of the wavelength.
            var model = new RtkQualityModel(1);
            Assert.AreEqual(0.048f, model.fix.nlosLimit, 1e-6f);
            Assert.Greater(model.fix.nlosScale, 0.0f,
                "not zero: a fixed solution does see carrier multipath");
            Assert.AreEqual(0.0f, model.floatGrade.nlosLimit, "a float solution is not bounded that way");

            var satellites = new List<SatelliteObservation>();
            float sinMask = Mathf.Sin(15.0f * Mathf.Deg2Rad);
            for (int i = 0; i < 12; i++)
            {
                satellites.Add(new SatelliteObservation
                {
                    prn = (ushort)(i + 1),
                    elevation = Mathf.Asin(sinMask + (i + 0.5f) / 12 * (1.0f - sinMask)),
                    azimuth = Mathf.Repeat(i * 2.39996323f, 2.0f * Mathf.PI),
                    visibility = SatelliteVisibility.LineOfSight,
                });
            }
            satellites.Add(new SatelliteObservation
            {
                prn = 200, azimuth = 0.5f * Mathf.PI, elevation = 20.0f * Mathf.Deg2Rad,
                visibility = SatelliteVisibility.Nlos, excessPathLength = 60.0f,
            });

            var step = model.Update(OpenSats, OpenHdop, 1.0f, satellites, 0.2f, -1);
            Assert.AreEqual(RtkState.Fix, step.state);
            double nlos = Math.Sqrt(step.nlosEast * step.nlosEast + step.nlosNorth * step.nlosNorth);
            Assert.Greater(nlos, 0.0, "the reflection is modelled, just bounded");
            Assert.LessOrEqual(nlos, model.fix.nlosLimit + 1e-9);
        }

        [Test]
        public void ReportedSigmaFollowsTheGrade()
        {
            var model = new RtkQualityModel(1);
            Assert.AreEqual(model.fix.reportedSigma,
                model.Update(OpenSats, OpenHdop, 1.0f, null, 0.2f, -1).horizontalSigma, 1e-6);
            Assert.AreEqual(model.floatGrade.reportedSigma,
                model.Update(FloatSats, FloatHdop, 6.0f, null, 0.2f, -1).horizontalSigma, 1e-6);
            Assert.AreEqual(model.single.reportedSigma,
                model.Update(SingleSats, SingleHdop, 18.0f, null, 0.2f, -1).horizontalSigma, 1e-6);
        }
    }
}
