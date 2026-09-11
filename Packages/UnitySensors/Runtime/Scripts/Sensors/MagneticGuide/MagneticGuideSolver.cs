using System.Collections.Generic;
using UnityEngine;
using UnitySensors.DataType.Sensor;

namespace UnitySensors.Sensor.MagneticGuide
{
    /// <summary>
    /// Turns the per-sample tape classification of a guide sensor into a reading.
    /// Pure function of its inputs so it can be tested without physics.
    /// </summary>
    /// <remarks>
    /// Samples run from the sensor's right end to its left end; sample i sits at
    /// <c>-width/2 + i * pitch</c>. A run of consecutive Track samples is one track and
    /// its centre is the track position; a run of Marker samples is a marker, reported
    /// on the left or right according to its side of the selected track (or of the
    /// sensor centre when no track is seen). Runs separated by a single missing sample
    /// are still one run — a tape edge can straddle a sample point.
    /// </remarks>
    public static class MagneticGuideSolver
    {
        /// <summary>No tape under this sample.</summary>
        public const byte None = 0;
        public const byte Track = 1;
        public const byte Marker = 2;

        /// <summary>
        /// Lateral position [m, +left] of sample <paramref name="index"/> when
        /// <paramref name="count"/> samples span <paramref name="width"/>.
        /// </summary>
        public static float SamplePosition(int index, int count, float width)
        {
            if (count <= 1)
            {
                return 0.0f;
            }
            float pitch = width / (count - 1);
            return -0.5f * width + index * pitch;
        }

        public static MagneticGuideReading Solve(byte[] samples, float width,
            MagneticForkSelection forkSelection, float previousPosition, bool hadTrack)
        {
            var tracks = new List<float>();
            var markers = new List<float>();
            CollectRuns(samples, width, Track, tracks);
            CollectRuns(samples, width, Marker, markers);
            // CollectRuns walks right to left, so the lists are already right-to-left;
            // report tracks left to right.
            tracks.Reverse();

            var reading = MagneticGuideReading.None;
            reading.trackPositions = tracks.ToArray();
            if (tracks.Count > 0)
            {
                reading.trackDetected = true;
                reading.trackPosition = SelectTrack(tracks, forkSelection, previousPosition, hadTrack);
            }

            float reference = reading.trackDetected ? reading.trackPosition : 0.0f;
            foreach (float marker in markers)
            {
                if (marker > reference)
                {
                    reading.leftMarkerDetected = true;
                }
                else
                {
                    reading.rightMarkerDetected = true;
                }
            }
            return reading;
        }

        static float SelectTrack(List<float> tracksLeftToRight, MagneticForkSelection selection,
            float previousPosition, bool hadTrack)
        {
            switch (selection)
            {
                case MagneticForkSelection.Left:
                    return tracksLeftToRight[0];
                case MagneticForkSelection.Right:
                    return tracksLeftToRight[tracksLeftToRight.Count - 1];
                default:
                    // Stick to the track we were following; with no history take the
                    // one nearest the sensor centre.
                    float target = hadTrack ? previousPosition : 0.0f;
                    float best = tracksLeftToRight[0];
                    foreach (float t in tracksLeftToRight)
                    {
                        if (Mathf.Abs(t - target) < Mathf.Abs(best - target))
                        {
                            best = t;
                        }
                    }
                    return best;
            }
        }

        /// <summary>Centre positions of the runs of <paramref name="kind"/> samples.</summary>
        static void CollectRuns(byte[] samples, float width, byte kind, List<float> centres)
        {
            int count = samples.Length;
            int runStart = -1;
            int runEnd = -1;   // inclusive
            for (int i = 0; i <= count; i++)
            {
                bool hit = i < count && samples[i] == kind;
                if (hit)
                {
                    if (runStart < 0)
                    {
                        runStart = i;
                    }
                    runEnd = i;
                    continue;
                }
                if (runStart < 0)
                {
                    continue;
                }
                // Allow a one-sample gap: the next sample continues the run.
                if (i + 1 < count && samples[i + 1] == kind)
                {
                    continue;
                }
                centres.Add(0.5f * (SamplePosition(runStart, count, width) + SamplePosition(runEnd, count, width)));
                runStart = -1;
                runEnd = -1;
            }
        }
    }
}
