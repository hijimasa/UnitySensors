using System;
using NUnit.Framework;
using UnityEngine;
using UnitySensors.DataType.Sensor;
using UnitySensors.Sensor.GNSS;

namespace UnitySensors.Tests.Editor
{
    /// <summary>
    /// The constellation layout and the DOP maths in isolation, no scene needed.
    /// The assertions are on properties that must hold for any correct
    /// implementation (spread, rotation invariance, monotonicity), not on numbers
    /// that would pin down one particular synthetic sky.
    /// </summary>
    public class GnssConstellationTests
    {
        const float Mask = 15.0f * Mathf.Deg2Rad;

        static SatelliteObservation[] Rotated(SatelliteObservation[] source, float deltaAzimuth)
        {
            var rotated = (SatelliteObservation[])source.Clone();
            for (int i = 0; i < rotated.Length; i++)
            {
                rotated[i].azimuth = Mathf.Repeat(rotated[i].azimuth + deltaAzimuth, 2.0f * Mathf.PI);
            }
            return rotated;
        }

        [Test]
        public void GenerateRespectsCountAndMask()
        {
            var satellites = GnssConstellation.Generate(24, Mask, 1);
            Assert.AreEqual(24, satellites.Length);
            foreach (var satellite in satellites)
            {
                Assert.GreaterOrEqual(satellite.elevation, Mask - 1e-4f);
                Assert.LessOrEqual(satellite.elevation, 0.5f * Mathf.PI + 1e-4f);
                Assert.GreaterOrEqual(satellite.azimuth, 0.0f);
                Assert.Less(satellite.azimuth, 2.0f * Mathf.PI + 1e-4f);
                Assert.AreEqual(SatelliteVisibility.LineOfSight, satellite.visibility);
            }
        }

        [Test]
        public void GenerateIsDeterministicPerSeed()
        {
            var a = GnssConstellation.Generate(24, Mask, 7);
            var b = GnssConstellation.Generate(24, Mask, 7);
            var c = GnssConstellation.Generate(24, Mask, 8);

            for (int i = 0; i < a.Length; i++)
            {
                Assert.AreEqual(a[i].azimuth, b[i].azimuth, 1e-6f);
                Assert.AreEqual(a[i].elevation, b[i].elevation, 1e-6f);
            }
            bool anyDifferent = false;
            for (int i = 0; i < a.Length; i++)
            {
                if (Mathf.Abs(a[i].azimuth - c[i].azimuth) > 1e-3f)
                {
                    anyDifferent = true;
                }
            }
            Assert.IsTrue(anyDifferent, "a different seed should give a different sky");
        }

        [Test]
        public void GenerateSpreadsSatellitesAroundTheSky()
        {
            var satellites = GnssConstellation.Generate(24, Mask, 1);
            // Every 90-degree quadrant should hold at least a couple of satellites;
            // a lattice that collapsed into spokes would fail this.
            var perQuadrant = new int[4];
            foreach (var satellite in satellites)
            {
                perQuadrant[Mathf.FloorToInt(satellite.azimuth / (0.5f * Mathf.PI)) & 3]++;
            }
            foreach (int count in perQuadrant)
            {
                Assert.GreaterOrEqual(count, 2);
            }
        }

        [Test]
        public void EnuDirectionFollowsTheAzimuthConvention()
        {
            // Azimuth 0 = North, 90 deg = East, elevation 90 = straight up.
            var north = GnssConstellation.ToEnuDirection(0.0f, 0.0f);
            Assert.AreEqual(0.0f, north.x, 1e-5f);
            Assert.AreEqual(1.0f, north.y, 1e-5f);

            var east = GnssConstellation.ToEnuDirection(0.5f * Mathf.PI, 0.0f);
            Assert.AreEqual(1.0f, east.x, 1e-5f);
            Assert.AreEqual(0.0f, east.y, 1e-5f);

            var zenith = GnssConstellation.ToEnuDirection(0.0f, 0.5f * Mathf.PI);
            Assert.AreEqual(1.0f, zenith.z, 1e-5f);
        }

        [Test]
        public void UnityDirectionMatchesTheSimulatorsEnuMapping()
        {
            // The simulator publishes ROS x = Unity z, ROS y = -Unity x, ROS z = Unity y,
            // so East is Unity +z, North is Unity -x and Up is Unity +y.
            var north = GnssConstellation.ToUnityDirection(0.0f, 0.0f);
            Assert.AreEqual(-1.0f, north.x, 1e-5f);
            Assert.AreEqual(0.0f, north.z, 1e-5f);

            var east = GnssConstellation.ToUnityDirection(0.5f * Mathf.PI, 0.0f);
            Assert.AreEqual(1.0f, east.z, 1e-5f);
            Assert.AreEqual(0.0f, east.x, 1e-5f);

            var zenith = GnssConstellation.ToUnityDirection(0.0f, 0.5f * Mathf.PI);
            Assert.AreEqual(1.0f, zenith.y, 1e-5f);
        }

        [Test]
        public void DopNeedsFourUsableSatellites()
        {
            var satellites = GnssConstellation.Generate(24, Mask, 1);
            for (int i = 3; i < satellites.Length; i++)
            {
                satellites[i].visibility = SatelliteVisibility.Blocked;
            }
            Assert.IsFalse(GnssConstellation.TryComputeDop(satellites, out float hdop, out float pdop));
            Assert.AreEqual(GnssSkyView.UnsolvableDop, hdop);
            Assert.AreEqual(GnssSkyView.UnsolvableDop, pdop);
        }

        [Test]
        public void OpenSkyGivesGoodDop()
        {
            var satellites = GnssConstellation.Generate(24, Mask, 1);
            Assert.IsTrue(GnssConstellation.TryComputeDop(satellites, out float hdop, out float pdop));
            // A full, well spread sky is comfortably below the thresholds a receiver
            // uses to call a solution good.
            Assert.Less(hdop, 1.0f);
            Assert.Less(pdop, 2.0f);
            Assert.Greater(hdop, 0.0f);
            Assert.LessOrEqual(hdop, pdop);
        }

        [Test]
        public void BlockingOneSideDegradesDop()
        {
            var satellites = GnssConstellation.Generate(24, Mask, 1);
            Assert.IsTrue(GnssConstellation.TryComputeDop(satellites, out float openHdop, out _));

            // Everything to the east blocked, as a wall along the track would do.
            int remaining = 0;
            foreach (var satellite in satellites)
            {
                if (satellite.azimuth < Mathf.PI)
                {
                    continue;
                }
                remaining++;
            }
            Assume.That(remaining, Is.GreaterThanOrEqualTo(4));
            for (int i = 0; i < satellites.Length; i++)
            {
                if (satellites[i].azimuth < Mathf.PI)
                {
                    satellites[i].visibility = SatelliteVisibility.Blocked;
                }
            }

            Assert.IsTrue(GnssConstellation.TryComputeDop(satellites, out float canyonHdop, out _));
            Assert.Greater(canyonHdop, openHdop,
                "losing half the sky must make the horizontal geometry worse");
        }

        [Test]
        public void DopIsInvariantUnderARotationOfTheWholeSky()
        {
            var satellites = GnssConstellation.Generate(24, Mask, 1);
            Assert.IsTrue(GnssConstellation.TryComputeDop(satellites, out float hdop, out float pdop));
            Assert.IsTrue(GnssConstellation.TryComputeDop(Rotated(satellites, 1.1f), out float rotatedHdop,
                out float rotatedPdop));

            // HDOP mixes the two horizontal axes symmetrically, so naming them
            // differently cannot change it. A sign or axis slip in the geometry
            // matrix would break this.
            Assert.AreEqual(hdop, rotatedHdop, 1e-4f);
            Assert.AreEqual(pdop, rotatedPdop, 1e-4f);
        }

        [Test]
        public void MoreSatellitesNeverHurt()
        {
            var satellites = GnssConstellation.Generate(24, Mask, 1);
            for (int i = 8; i < satellites.Length; i++)
            {
                satellites[i].visibility = SatelliteVisibility.Blocked;
            }
            Assert.IsTrue(GnssConstellation.TryComputeDop(satellites, out float fewHdop, out _));

            satellites[8].visibility = SatelliteVisibility.LineOfSight;
            Assert.IsTrue(GnssConstellation.TryComputeDop(satellites, out float moreHdop, out _));
            Assert.LessOrEqual(moreHdop, fewHdop + 1e-5f);
        }

        [Test]
        public void CoplanarGeometryIsRejected()
        {
            // Four satellites all on the horizon and all at the same azimuth: the
            // geometry matrix is rank deficient and there is no solution.
            var satellites = new SatelliteObservation[4];
            for (int i = 0; i < satellites.Length; i++)
            {
                satellites[i] = new SatelliteObservation
                {
                    prn = (ushort)(i + 1),
                    azimuth = 0.0f,
                    elevation = 0.0f,
                    visibility = SatelliteVisibility.LineOfSight,
                };
            }
            Assert.IsFalse(GnssConstellation.TryComputeDop(satellites, out _, out _));
        }
    }
}
