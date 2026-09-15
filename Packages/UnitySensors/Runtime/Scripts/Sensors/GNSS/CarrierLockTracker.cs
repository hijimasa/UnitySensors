using System.Collections.Generic;

namespace UnitySensors.Sensor.GNSS
{
    /// <summary>
    /// Per-satellite carrier phase continuity, and what breaks it.
    /// </summary>
    /// <remarks>
    /// RTK stands on the carrier phase, and the carrier phase is only useful while
    /// it stays continuous. Every time a satellite disappears behind something the
    /// receiver loses count of the cycles, and when it comes back the integer
    /// ambiguity has to be resolved again from scratch. That is the cycle slip, and
    /// it is the real reason a fix does not return the instant the sky reopens.
    ///
    /// Tracking it per satellite rather than as one global timer matters in exactly
    /// the case worth simulating: brushing past a pole drops a couple of satellites
    /// while the rest keep counting and the fix returns quickly, whereas driving
    /// under a bridge drops all of them and the whole constellation reconverges.
    /// </remarks>
    public class CarrierLockTracker
    {
        private readonly Dictionary<ushort, float> _lockSeconds = new Dictionary<ushort, float>();
        private readonly List<ushort> _lost = new List<ushort>();
        private readonly HashSet<ushort> _visible = new HashSet<ushort>();
        private ulong _slips;

        /// <summary>Cycle slips counted since the last reset. Diagnostics only.</summary>
        public ulong slips { get { return _slips; } }

        /// <summary>
        /// Advance by <paramref name="dt"/> with the satellites currently in direct
        /// view. Anything absent has lost lock; a reflected signal counts as absent,
        /// because its carrier is not the satellite's own.
        /// </summary>
        public void Update(IReadOnlyList<ushort> lineOfSightPrns, float dt)
        {
            if (!(dt > 0.0f))
            {
                dt = 0.0f;
            }

            _visible.Clear();
            for (int i = 0; i < lineOfSightPrns.Count; i++)
            {
                _visible.Add(lineOfSightPrns[i]);
            }

            // Entries are kept rather than removed, so a satellite flickering in and
            // out is not mistaken for a fresh acquisition every time.
            _lost.Clear();
            foreach (var entry in _lockSeconds)
            {
                if (!_visible.Contains(entry.Key))
                {
                    _lost.Add(entry.Key);
                }
            }
            for (int i = 0; i < _lost.Count; i++)
            {
                if (_lockSeconds[_lost[i]] > 0.0f)
                {
                    _slips++;
                }
                _lockSeconds[_lost[i]] = 0.0f;
            }

            for (int i = 0; i < lineOfSightPrns.Count; i++)
            {
                ushort prn = lineOfSightPrns[i];
                float held;
                _lockSeconds[prn] = _lockSeconds.TryGetValue(prn, out held) ? held + dt : dt;
            }
        }

        /// <summary>How many satellites have held lock for at least <paramref name="seconds"/>.</summary>
        public int LockedFor(float seconds)
        {
            int count = 0;
            foreach (var entry in _lockSeconds)
            {
                if (entry.Value >= seconds)
                {
                    count++;
                }
            }
            return count;
        }

        /// <summary>Satellites holding any lock at all.</summary>
        public int Locked()
        {
            int count = 0;
            foreach (var entry in _lockSeconds)
            {
                if (entry.Value > 0.0f)
                {
                    count++;
                }
            }
            return count;
        }

        /// <summary>Forget everything, as a receiver restarting would.</summary>
        public void Reset()
        {
            _lockSeconds.Clear();
            _slips = 0;
        }
    }
}
