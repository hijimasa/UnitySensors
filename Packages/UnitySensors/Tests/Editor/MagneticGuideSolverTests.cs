using NUnit.Framework;
using UnitySensors.DataType.Sensor;
using UnitySensors.Sensor.MagneticGuide;

namespace UnitySensors.Tests.Editor
{
    /// <summary>
    /// The guide-sensor solver in isolation: samples in, reading out. Geometry is
    /// 161 samples over 0.16 m, i.e. 1 mm pitch, sample 80 at the centre.
    /// </summary>
    public class MagneticGuideSolverTests
    {
        const float Width = 0.16f;
        const int Count = 161;

        static byte[] Samples(params (int from, int to, byte kind)[] runs)
        {
            var s = new byte[Count];
            foreach (var (from, to, kind) in runs)
            {
                for (int i = from; i <= to; i++) s[i] = kind;
            }
            return s;
        }

        static MagneticGuideReading Solve(byte[] s, MagneticForkSelection fork = MagneticForkSelection.Nearest,
            float previous = 0f, bool hadTrack = false)
            => MagneticGuideSolver.Solve(s, Width, fork, previous, hadTrack);

        [Test]
        public void SamplePosition_SpansMinusHalfToPlusHalf()
        {
            Assert.AreEqual(-0.08f, MagneticGuideSolver.SamplePosition(0, Count, Width), 1e-6f);
            Assert.AreEqual(0.0f, MagneticGuideSolver.SamplePosition(80, Count, Width), 1e-6f);
            Assert.AreEqual(0.08f, MagneticGuideSolver.SamplePosition(160, Count, Width), 1e-6f);
        }

        [Test]
        public void NoTape_NoTrackNoMarkers()
        {
            var r = Solve(new byte[Count]);
            Assert.IsFalse(r.trackDetected);
            Assert.AreEqual(0.0f, r.trackPosition);
            Assert.IsFalse(r.leftMarkerDetected);
            Assert.IsFalse(r.rightMarkerDetected);
            Assert.AreEqual(0, r.trackPositions.Length);
        }

        [Test]
        public void CentredTape_ReportsZero()
        {
            // 25 mm tape centred: samples 68..92
            var r = Solve(Samples((68, 92, MagneticGuideSolver.Track)));
            Assert.IsTrue(r.trackDetected);
            Assert.AreEqual(0.0f, r.trackPosition, 1e-6f);
            Assert.AreEqual(1, r.trackPositions.Length);
        }

        [Test]
        public void TapeToTheLeft_IsPositive()
        {
            // centre at sample 110 -> +30 mm
            var r = Solve(Samples((98, 122, MagneticGuideSolver.Track)));
            Assert.AreEqual(0.030f, r.trackPosition, 1e-6f);
        }

        [Test]
        public void OneMissingSample_DoesNotSplitTheTape()
        {
            var s = Samples((68, 92, MagneticGuideSolver.Track));
            s[80] = MagneticGuideSolver.None;
            var r = Solve(s);
            Assert.AreEqual(1, r.trackPositions.Length, "a single dropout is still one track");
            Assert.AreEqual(0.0f, r.trackPosition, 1e-6f);
        }

        [Test]
        public void Fork_ReportsBothTracks_LeftToRight()
        {
            var s = Samples((30, 50, MagneticGuideSolver.Track), (110, 130, MagneticGuideSolver.Track));
            var r = Solve(s);
            Assert.AreEqual(2, r.trackPositions.Length);
            Assert.AreEqual(0.040f, r.trackPositions[0], 1e-6f, "left first");
            Assert.AreEqual(-0.040f, r.trackPositions[1], 1e-6f);
        }

        [Test]
        public void Fork_SelectionLeftRightNearest()
        {
            var s = Samples((30, 50, MagneticGuideSolver.Track), (110, 130, MagneticGuideSolver.Track));
            Assert.AreEqual(0.040f, Solve(s, MagneticForkSelection.Left).trackPosition, 1e-6f);
            Assert.AreEqual(-0.040f, Solve(s, MagneticForkSelection.Right).trackPosition, 1e-6f);
            // Nearest sticks to what it was following
            Assert.AreEqual(-0.040f, Solve(s, MagneticForkSelection.Nearest, previous: -0.035f, hadTrack: true).trackPosition, 1e-6f);
            Assert.AreEqual(0.040f, Solve(s, MagneticForkSelection.Nearest, previous: 0.02f, hadTrack: true).trackPosition, 1e-6f);
        }

        [Test]
        public void Markers_AreSidedRelativeToTheTrack()
        {
            // track at +20 mm; marker at -10 mm is still to the RIGHT of the track
            var s = Samples((88, 112, MagneticGuideSolver.Track), (60, 80, MagneticGuideSolver.Marker));
            var r = Solve(s);
            Assert.IsTrue(r.trackDetected);
            Assert.IsTrue(r.rightMarkerDetected);
            Assert.IsFalse(r.leftMarkerDetected);

            var s2 = Samples((68, 92, MagneticGuideSolver.Track), (120, 140, MagneticGuideSolver.Marker));
            var r2 = Solve(s2);
            Assert.IsTrue(r2.leftMarkerDetected);
            Assert.IsFalse(r2.rightMarkerDetected);
        }

        [Test]
        public void MarkerWithoutTrack_SidedByTheSensorCentre()
        {
            var r = Solve(Samples((20, 40, MagneticGuideSolver.Marker)));
            Assert.IsFalse(r.trackDetected);
            Assert.IsTrue(r.rightMarkerDetected);
            Assert.IsFalse(r.leftMarkerDetected);
        }
    }
}
