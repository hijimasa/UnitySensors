using NUnit.Framework;
using UnityEngine;
using UnitySensors.Sensor.GNSS;

namespace UnitySensors.Tests.Editor
{
    /// <summary>
    /// The one-bounce reflection maths on its own: the direction a reflected path
    /// must have come from, and how much longer that path is.
    /// </summary>
    public class GnssReflectionTests
    {
        [Test]
        public void RayCountGrowsWithTheInverseSquareOfTheSpacing()
        {
            int coarse = GnssReflection.RayCountForSpacing(5.0f * Mathf.Deg2Rad);
            int fine = GnssReflection.RayCountForSpacing(2.5f * Mathf.Deg2Rad);
            // Halving the spacing must cost about four times as many rays.
            Assert.That(fine / (float)coarse, Is.EqualTo(4.0f).Within(0.1f));
            // 2.5 deg over a hemisphere is a few thousand rays, not a few million.
            Assert.Greater(fine, 2000);
            Assert.Less(fine, 6000);
        }

        [Test]
        public void LaunchDirectionsCoverTheUpperHemisphereOnly()
        {
            var directions = new Vector3[500];
            GnssReflection.FillLaunchDirections(directions, Vector3.up, Vector3.forward, -Vector3.right);

            var quadrant = new int[4];
            foreach (var d in directions)
            {
                Assert.That(d.magnitude, Is.EqualTo(1.0f).Within(1e-4f), "directions must be unit length");
                Assert.GreaterOrEqual(d.y, -1e-5f, "nothing below the horizon: ground bounce is out of scope");
                float azimuth = Mathf.Atan2(d.z, -d.x);   // east = +z, north = -x
                if (azimuth < 0.0f) azimuth += 2.0f * Mathf.PI;
                quadrant[Mathf.Min(3, Mathf.FloorToInt(azimuth / (0.5f * Mathf.PI)))]++;
            }
            foreach (int n in quadrant)
            {
                Assert.Greater(n, 50, "the sweep must not collapse into spokes");
            }
        }

        [Test]
        public void AWallReflectsASatelliteFromTheOppositeSide()
        {
            // Antenna at the origin, a wall off to +x whose face looks back at it.
            Vector3 normal = new Vector3(-1.0f, 0.0f, 0.0f);
            // Launch 45 degrees up, towards the wall.
            Vector3 launch = new Vector3(1.0f, 1.0f, 0.0f).normalized;

            Vector3 satellite = GnssReflection.RequiredSatelliteDirection(launch, normal);

            Assert.That(satellite.magnitude, Is.EqualTo(1.0f).Within(1e-4f));
            Assert.Less(satellite.x, 0.0f, "the satellite must be on the far side of the antenna from the wall");
            Assert.That(satellite.y, Is.EqualTo(launch.y).Within(1e-4f),
                "a vertical wall leaves the elevation untouched");
        }

        [Test]
        public void TheReflectionIsItsOwnInverse()
        {
            Vector3 normal = new Vector3(0.3f, 0.8f, -0.5f).normalized;
            Vector3 launch = new Vector3(-0.4f, 0.6f, 0.7f).normalized;

            Vector3 there = GnssReflection.RequiredSatelliteDirection(launch, normal);
            Vector3 back = GnssReflection.RequiredSatelliteDirection(there, normal);

            // This involution is what lets the sweep be blind: the same formula turns
            // a launch direction into a satellite direction and back.
            Assert.That(Vector3.Dot(back, launch), Is.EqualTo(1.0f).Within(1e-4f));
        }

        [Test]
        public void HeadOnIncidenceReflectsStraightBack()
        {
            Vector3 normal = new Vector3(0.0f, 0.0f, 1.0f);
            Vector3 launch = -normal;
            Vector3 satellite = GnssReflection.RequiredSatelliteDirection(launch, normal);
            Assert.That(Vector3.Dot(satellite, normal), Is.EqualTo(1.0f).Within(1e-4f));
        }

        [Test]
        public void ExcessPathLengthIsZeroForADirectPath()
        {
            Vector3 direction = new Vector3(0.2f, 0.9f, 0.3f).normalized;
            Assert.That(GnssReflection.ExcessPathLength(37.0f, direction, direction),
                        Is.EqualTo(0.0f).Within(1e-4f));
        }

        [Test]
        public void ExcessPathLengthEqualsTheDistanceAtRightAngles()
        {
            // Reflection point 10 m away, satellite at 90 degrees to it: the detour
            // costs exactly the distance to the wall.
            Vector3 launch = Vector3.forward;
            Vector3 satellite = Vector3.up;
            Assert.That(GnssReflection.ExcessPathLength(10.0f, launch, satellite),
                        Is.EqualTo(10.0f).Within(1e-4f));
        }

        [Test]
        public void ExcessPathLengthDoublesForASatelliteBehindTheAntenna()
        {
            // Straight back the way it came: the signal travels to the wall and back.
            Vector3 launch = Vector3.forward;
            Assert.That(GnssReflection.ExcessPathLength(12.0f, launch, -launch),
                        Is.EqualTo(24.0f).Within(1e-4f));
        }

        [Test]
        public void AStreetWallGivesTensOfMetresOfExcessPath()
        {
            // The case that matters: a wall 8 m away, a satellite 30 degrees up on
            // the far side. This is why an NLOS fix is wrong by tens of metres while
            // multipath with the direct signal present stays at the metre level.
            Vector3 normal = new Vector3(-1.0f, 0.0f, 0.0f);
            float elevation = 30.0f * Mathf.Deg2Rad;
            Vector3 launch = new Vector3(Mathf.Cos(elevation), Mathf.Sin(elevation), 0.0f);
            Vector3 satellite = GnssReflection.RequiredSatelliteDirection(launch, normal);

            float distance = 8.0f / Mathf.Cos(elevation);   // slant range to the wall
            float excess = GnssReflection.ExcessPathLength(distance, launch, satellite);

            Assert.Greater(excess, 5.0f);
            Assert.Less(excess, 40.0f);
            // Closed form for a vertical wall: 2 * d * cos(elevation)^2.
            Assert.That(excess, Is.EqualTo(2.0f * distance * Mathf.Cos(elevation) * Mathf.Cos(elevation))
                        .Within(1e-3f));
        }
    }
}
