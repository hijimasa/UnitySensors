using System;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnitySensors.DataType.Sensor;
using UnitySensors.Sensor.GNSS;

namespace UnitySensors.Tests.Editor
{
    /// <summary>
    /// Where a set of biased pseudoranges puts the fix. The assertions are on
    /// direction and equivariance rather than on numbers, because the value of this
    /// model is that the error points somewhere the map explains -- if a sign or an
    /// axis were wrong, the simulated rover would drift towards the wall instead of
    /// away from it and still look plausible.
    /// </summary>
    public class GnssNlosBiasTests
    {
        const float GoldenAngle = 2.39996323f;

        static List<SatelliteObservation> OpenSky(int count = 12, float azimuthOffset = 0.0f)
        {
            var sky = new List<SatelliteObservation>();
            float sinMask = Mathf.Sin(15.0f * Mathf.Deg2Rad);
            for (int i = 0; i < count; i++)
            {
                float sinElevation = sinMask + (i + 0.5f) / count * (1.0f - sinMask);
                sky.Add(new SatelliteObservation
                {
                    prn = (ushort)(i + 1),
                    elevation = Mathf.Asin(sinElevation),
                    azimuth = Mathf.Repeat(azimuthOffset + i * GoldenAngle, 2.0f * Mathf.PI),
                    visibility = SatelliteVisibility.LineOfSight,
                });
            }
            return sky;
        }

        static void AddReflected(List<SatelliteObservation> sky, float azimuth, float excess,
            float elevationDeg = 20.0f)
        {
            sky.Add(new SatelliteObservation
            {
                prn = 200,
                azimuth = azimuth,
                elevation = elevationDeg * Mathf.Deg2Rad,
                visibility = SatelliteVisibility.Nlos,
                excessPathLength = excess,
            });
        }

        [Test]
        public void NeedsFourSatellites()
        {
            Assert.IsFalse(GnssNlosBias.Solve(OpenSky(3)).solved);
            Assert.IsFalse(GnssNlosBias.Solve(new List<SatelliteObservation>()).solved);
        }

        [Test]
        public void CleanSkyProducesNoBias()
        {
            var bias = GnssNlosBias.Solve(OpenSky());
            Assert.IsTrue(bias.solved);
            Assert.AreEqual(0.0, bias.east, 1e-12);
            Assert.AreEqual(0.0, bias.north, 1e-12);
        }

        [Test]
        public void TheFixIsPushedAwayFromTheReflectedSatellite()
        {
            // A satellite due East measured too far away: the receiver concludes it is
            // further from it than it is, and the fix moves West.
            var sky = OpenSky();
            AddReflected(sky, 0.5f * Mathf.PI, 40.0f);
            var bias = GnssNlosBias.Solve(sky);
            Assert.IsTrue(bias.solved);
            Assert.Less(bias.east, -0.5);
            Assert.Less(Math.Abs(bias.north), Math.Abs(bias.east));
        }

        [Test]
        public void TheDirectionFollowsTheSatelliteBearing()
        {
            var sky = OpenSky();
            AddReflected(sky, 0.0f, 40.0f);
            var bias = GnssNlosBias.Solve(sky);
            Assert.IsTrue(bias.solved);
            Assert.Less(bias.north, -0.5, "a satellite due North must push the fix South");
            Assert.Less(Math.Abs(bias.east), Math.Abs(bias.north));
        }

        [Test]
        public void RotatingTheWholeSkyRotatesTheError()
        {
            // A swapped or sign-flipped axis survives every magnitude test but dies here.
            var sky = OpenSky(12, 0.0f);
            AddReflected(sky, 0.0f, 40.0f);
            var basis = GnssNlosBias.Solve(sky);

            var turned = OpenSky(12, 0.5f * Mathf.PI);
            AddReflected(turned, 0.5f * Mathf.PI, 40.0f);
            var rotated = GnssNlosBias.Solve(turned);

            Assert.IsTrue(basis.solved && rotated.solved);
            // Rotating by +90 deg (North towards East) maps (east, north) -> (north, -east).
            Assert.AreEqual(basis.north, rotated.east, 1e-4);
            Assert.AreEqual(-basis.east, rotated.north, 1e-4);
        }

        [Test]
        public void ScalesLinearlyWithTheExcessPathLength()
        {
            var small = OpenSky();
            AddReflected(small, 0.5f * Mathf.PI, 10.0f);
            var large = OpenSky();
            AddReflected(large, 0.5f * Mathf.PI, 30.0f);

            var a = GnssNlosBias.Solve(small);
            var b = GnssNlosBias.Solve(large);
            Assert.IsTrue(a.solved && b.solved);
            Assert.AreEqual(3.0 * a.east, b.east, 1e-4);
            Assert.AreEqual(3.0 * a.north, b.north, 1e-4);
        }

        [Test]
        public void ABiasCommonToEverySatelliteIsAbsorbedByTheClock()
        {
            // Every pseudorange long by the same amount is indistinguishable from a
            // receiver clock error, so the position must not move at all.
            var sky = new List<SatelliteObservation>();
            foreach (var satellite in OpenSky())
            {
                var reflected = satellite;
                reflected.visibility = SatelliteVisibility.Nlos;
                reflected.excessPathLength = 25.0f;
                sky.Add(reflected);
            }
            var bias = GnssNlosBias.Solve(sky);
            Assert.IsTrue(bias.solved);
            Assert.AreEqual(0.0, bias.east, 1e-6);
            Assert.AreEqual(0.0, bias.north, 1e-6);
        }

        [Test]
        public void LowSatellitesMoveTheFixMoreThanHighOnes()
        {
            // Geometry, not magnitude: the same extra path length on a satellite near
            // the horizon lands almost entirely in the horizontal, while one overhead
            // mostly trades against the clock and the vertical.
            var low = OpenSky();
            AddReflected(low, 0.5f * Mathf.PI, 40.0f, 10.0f);
            var high = OpenSky();
            AddReflected(high, 0.5f * Mathf.PI, 40.0f, 80.0f);

            var lowBias = GnssNlosBias.Solve(low);
            var highBias = GnssNlosBias.Solve(high);
            Assert.IsTrue(lowBias.solved && highBias.solved);
            Assert.Greater(Math.Sqrt(lowBias.east * lowBias.east + lowBias.north * lowBias.north),
                           Math.Sqrt(highBias.east * highBias.east + highBias.north * highBias.north));
        }

        [Test]
        public void StaysWithinTheExcessPathLength()
        {
            var sky = OpenSky();
            AddReflected(sky, 0.5f * Mathf.PI, 40.0f);
            var bias = GnssNlosBias.Solve(sky);
            Assert.IsTrue(bias.solved);
            Assert.Less(Math.Sqrt(bias.east * bias.east + bias.north * bias.north), 40.0);
        }

        [Test]
        public void BlockedSatellitesTakeNoPart()
        {
            // A satellite with no path at all is not tracked, so it neither anchors
            // the solution nor drags it.
            var sky = OpenSky();
            AddReflected(sky, 0.5f * Mathf.PI, 40.0f);
            var withBlocked = new List<SatelliteObservation>(sky);
            withBlocked.Add(new SatelliteObservation
            {
                prn = 300, azimuth = 1.0f, elevation = 0.5f,
                visibility = SatelliteVisibility.Blocked, excessPathLength = 999.0f,
            });

            var a = GnssNlosBias.Solve(sky);
            var b = GnssNlosBias.Solve(withBlocked);
            Assert.AreEqual(a.east, b.east, 1e-9);
            Assert.AreEqual(a.north, b.north, 1e-9);
        }
    }
}
