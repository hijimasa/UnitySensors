using System.Collections.Generic;
using NUnit.Framework;
using UnitySensors.Sensor.GNSS;

namespace UnitySensors.Tests.Editor
{
    /// <summary>
    /// Per-satellite carrier phase continuity. What these protect is the distinction
    /// the tracker exists for: losing a couple of satellites and losing all of them
    /// must not cost the same. A global timer cannot tell those apart, and getting it
    /// wrong makes every brush past a pole look like driving under a bridge.
    /// </summary>
    public class CarrierLockTrackerTests
    {
        static List<ushort> Prns(int count, int first = 1)
        {
            var list = new List<ushort>();
            for (int i = 0; i < count; i++) list.Add((ushort)(first + i));
            return list;
        }

        static void Hold(CarrierLockTracker tracker, List<ushort> visible, float seconds, float dt = 0.2f)
        {
            for (int i = 0; i < (int)(seconds / dt); i++) tracker.Update(visible, dt);
        }

        [Test]
        public void CountsUpWhileTheSatelliteStaysInView()
        {
            var tracker = new CarrierLockTracker();
            Hold(tracker, Prns(6), 10.0f);
            Assert.AreEqual(6, tracker.LockedFor(8.0f));
            Assert.AreEqual(6, tracker.Locked());
            Assert.AreEqual(0ul, tracker.slips);
        }

        [Test]
        public void ASatelliteNotYetHeldLongEnoughDoesNotCount()
        {
            var tracker = new CarrierLockTracker();
            Hold(tracker, Prns(6), 3.0f);
            Assert.AreEqual(0, tracker.LockedFor(8.0f));
            Assert.AreEqual(6, tracker.Locked(), "they are tracked, just not for long enough yet");
        }

        [Test]
        public void LosingASatelliteIsASlipAndRestartsItsCount()
        {
            var tracker = new CarrierLockTracker();
            Hold(tracker, Prns(6), 10.0f);
            Assert.AreEqual(6, tracker.LockedFor(8.0f));

            tracker.Update(Prns(5), 0.2f);
            Assert.AreEqual(1ul, tracker.slips);
            Assert.AreEqual(5, tracker.LockedFor(8.0f), "the other five kept counting");

            Hold(tracker, Prns(6), 4.0f);
            Assert.AreEqual(5, tracker.LockedFor(8.0f));
            Hold(tracker, Prns(6), 5.0f);
            Assert.AreEqual(6, tracker.LockedFor(8.0f));
        }

        [Test]
        public void BrushingAPoleCostsFarLessThanDrivingUnderABridge()
        {
            var pole = new CarrierLockTracker();
            Hold(pole, Prns(10), 20.0f);
            pole.Update(Prns(8), 0.2f);          // two satellites clipped
            Hold(pole, Prns(10), 1.0f);

            var bridge = new CarrierLockTracker();
            Hold(bridge, Prns(10), 20.0f);
            bridge.Update(new List<ushort>(), 0.2f);   // everything gone
            Hold(bridge, Prns(10), 1.0f);

            Assert.AreEqual(8, pole.LockedFor(8.0f), "the eight that never dropped are still usable");
            Assert.AreEqual(0, bridge.LockedFor(8.0f), "nothing survived, so everything reconverges");
        }

        [Test]
        public void AFlickeringSatelliteIsNotMistakenForAFreshOne()
        {
            var tracker = new CarrierLockTracker();
            Hold(tracker, Prns(4), 10.0f);
            for (int i = 0; i < 5; i++)
            {
                tracker.Update(Prns(3), 0.2f);
                tracker.Update(Prns(4), 0.2f);
            }
            Assert.AreEqual(5ul, tracker.slips);
            Assert.AreEqual(4, tracker.Locked());
        }

        [Test]
        public void ResetForgetsEverything()
        {
            var tracker = new CarrierLockTracker();
            Hold(tracker, Prns(6), 10.0f);
            tracker.Update(Prns(3), 0.2f);
            Assert.Greater(tracker.slips, 0ul);

            tracker.Reset();
            Assert.AreEqual(0, tracker.Locked());
            Assert.AreEqual(0, tracker.LockedFor(0.1f));
            Assert.AreEqual(0ul, tracker.slips);
        }

        [Test]
        public void ZeroElapsedTimeChangesNothing()
        {
            var tracker = new CarrierLockTracker();
            Hold(tracker, Prns(5), 10.0f);
            int before = tracker.LockedFor(8.0f);
            tracker.Update(Prns(5), 0.0f);
            Assert.AreEqual(before, tracker.LockedFor(8.0f));
        }
    }
}
