using System.Collections;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

using UnitySensors.DataType.Sensor;
using UnitySensors.Interface.Sensor;

namespace UnitySensors.Sensor.GNSS
{
    /// <summary>
    /// Ray-traced sky visibility at a GNSS antenna: which satellites reach it
    /// directly, which only reach it via a reflection, and how good the geometry of
    /// the clean set is.
    /// </summary>
    /// <remarks>
    /// This sensor deliberately stops at the geometry. It does not produce a
    /// position, a fix grade or an error — those depend on receiver behaviour that
    /// is far easier to tune and test outside the simulator, so the consumer (the
    /// GPS emulator on the ROS side) owns them. What cannot be done outside the
    /// simulator is knowing what the buildings block and where they bounce a signal
    /// from, and that is exactly what this provides.
    ///
    /// Two passes:
    ///  - one ray per satellite for the direct line of sight (a couple of dozen);
    ///  - if any satellite is blocked, a blind sweep of the upper hemisphere whose
    ///    hits are turned into candidate one-bounce paths (a few thousand). The
    ///    sweep is why no surface has to be identified in advance: the hit normal
    ///    gives the satellite direction that path would have come from
    ///    (see <see cref="GnssReflection"/>).
    ///
    /// Trigger colliders are ignored by default. The simulator uses triggers for
    /// objects that should be visible to sensors but not block movement (weeds,
    /// magnetic tape), and none of those should shadow or reflect a satellite.
    /// </remarks>
    public class GnssSkyViewSensor : UnitySensor, IGnssSkyViewInterface
    {
        [SerializeField, Min(0)]
        private int _satelliteCount = 24;
        [SerializeField, Range(0.0f, 60.0f)]
        private float _elevationMaskDeg = 15.0f;
        [SerializeField]
        private int _seed = 1;
        [SerializeField, Min(1.0f)]
        private float _maxRayDistance = 500.0f;
        [SerializeField]
        private LayerMask _layerMask = ~0;
        [SerializeField]
        private QueryTriggerInteraction _hitTriggers = QueryTriggerInteraction.Ignore;

        [SerializeField]
        private bool _findReflections = true;
        /// <summary>Angular spacing of the reflection sweep [deg]. Halving it quadruples the rays.</summary>
        [SerializeField, Range(0.5f, 10.0f)]
        private float _reflectionSpacingDeg = 2.5f;
        /// <summary>[dB] a one-bounce path loses. Negative.</summary>
        [SerializeField]
        private float _reflectionLossDb = -13.0f;

        private Transform _transform;
        /// Published state. Only ever written by Commit, so a publisher sampling it
        /// mid-update sees the previous complete measurement rather than a half-built
        /// one -- the reflection search spans frames, and during it every satellite
        /// is briefly back to "blocked, no reflection".
        private SatelliteObservation[] _satellites;
        /// Scratch the update works in.
        private SatelliteObservation[] _working;
        private int _pendingUsable;
        private float _pendingHdop = GnssSkyView.UnsolvableDop;
        private float _pendingPdop = GnssSkyView.UnsolvableDop;
        private Vector3[] _rayDirections;
        private GnssSkyView _skyView = GnssSkyView.None;

        // Reflection sweep buffers, allocated once and reused.
        private Vector3[] _launchDirections;
        private NativeArray<RaycastCommand> _sweepCommands;
        private NativeArray<RaycastHit> _sweepHits;
        private NativeArray<RaycastCommand> _confirmCommands;
        private NativeArray<RaycastHit> _confirmHits;
        /// Best few candidate bounces per satellite, ascending by excess path length.
        private float[] _candidateExcess;
        private Vector3[] _candidatePoint;
        /// Candidate slot each confirm ray belongs to, so the dense batch can be
        /// read back against the per-satellite lists.
        private int[] _confirmSlot;
        private float _matchCosine = 1.0f;
        private JobHandle _sweepHandle;
        private JobHandle _confirmHandle;

        /// Candidates kept per satellite. Only the shortest path matters, but the
        /// shortest is not always the one that still sees the sky from its
        /// reflection point, so keep a few and take the best that survives.
        private const int CandidatesPerSatellite = 4;

        public GnssSkyView skyView { get => _skyView; }
        public int satelliteCount { get => _satelliteCount; }
        public float elevationMaskDeg { get => _elevationMaskDeg; }

        /// <summary>Configure at runtime (avoids Reflection); the constellation is rebuilt.</summary>
        public void Configure(int satelliteCount, float elevationMaskDeg, int seed, float maxRayDistance,
            LayerMask layerMask, QueryTriggerInteraction hitTriggers)
        {
            _satelliteCount = Mathf.Max(0, satelliteCount);
            _elevationMaskDeg = Mathf.Clamp(elevationMaskDeg, 0.0f, 60.0f);
            _seed = seed;
            _maxRayDistance = Mathf.Max(1.0f, maxRayDistance);
            _layerMask = layerMask;
            _hitTriggers = hitTriggers;
            _satellites = null;
        }

        /// <summary>Configure the one-bounce reflection search.</summary>
        public void ConfigureReflections(bool findReflections, float spacingDeg, float lossDb)
        {
            _findReflections = findReflections;
            _reflectionSpacingDeg = Mathf.Clamp(spacingDeg, 0.5f, 10.0f);
            _reflectionLossDb = Mathf.Min(0.0f, lossDb);
            _satellites = null;
        }

        protected override void Init()
        {
            _transform = this.transform;
            BuildConstellation();
            // Measure straight away. The publisher runs on its own timer and can fire
            // before the sensor's first update, and an unmeasured sky view reads as
            // "no satellites at all" -- which a receiver model would quite reasonably
            // turn into a loss of fix. One real measurement here removes that
            // spurious dropout at spawn.
            Measure();
        }

        private void BuildConstellation()
        {
            ReleaseSweepBuffers();

            _satellites = GnssConstellation.Generate(_satelliteCount, _elevationMaskDeg * Mathf.Deg2Rad, _seed);
            _working = GnssConstellation.Generate(_satelliteCount, _elevationMaskDeg * Mathf.Deg2Rad, _seed);
            _rayDirections = new Vector3[_satellites.Length];
            for (int i = 0; i < _satellites.Length; i++)
            {
                _rayDirections[i] = GnssConstellation.ToUnityDirection(_satellites[i].azimuth, _satellites[i].elevation);
            }

            if (_findReflections && _satellites.Length > 0)
            {
                float spacing = _reflectionSpacingDeg * Mathf.Deg2Rad;
                int rays = GnssReflection.RayCountForSpacing(spacing);
                _launchDirections = new Vector3[rays];
                // World axes, not the sensor's: the sky does not rotate with the robot.
                GnssReflection.FillLaunchDirections(_launchDirections, Vector3.up, Vector3.forward, -Vector3.right);
                _sweepCommands = new NativeArray<RaycastCommand>(rays, Allocator.Persistent);
                _sweepHits = new NativeArray<RaycastHit>(rays, Allocator.Persistent);
                // Only the best few bounces per satellite are ever confirmed, so this
                // second batch is tiny next to the sweep -- a hundred rays, not
                // thousands.
                int candidates = _satellites.Length * CandidatesPerSatellite;
                _confirmCommands = new NativeArray<RaycastCommand>(candidates, Allocator.Persistent);
                _confirmHits = new NativeArray<RaycastHit>(candidates, Allocator.Persistent);
                _candidateExcess = new float[candidates];
                _candidatePoint = new Vector3[candidates];
                _confirmSlot = new int[candidates];
                // A reflected direction is spread about as widely as the launch grid,
                // so accept a match a little wider than one cell.
                _matchCosine = Mathf.Cos(1.5f * spacing);
            }

            _skyView = new GnssSkyView
            {
                usableSatellites = 0,
                hdop = GnssSkyView.UnsolvableDop,
                pdop = GnssSkyView.UnsolvableDop,
                satellites = _satellites,
            };
        }

        protected override IEnumerator UpdateSensor()
        {
            int blocked = MeasureDirect();
            if (blocked == 0 || _launchDirections == null)
            {
                Commit();
                yield return null;
                yield break;
            }

            // The reflection search is thousands of rays. Blocking on them the way
            // the direct pass does would make one update longer than the sensor's own
            // period, and the base class folds the update time back into the period,
            // so the sensor would end up running back to back and starve the main
            // thread -- ROS services on the same thread then time out. Yield until
            // the batch is done instead, exactly as the lidar does.
            ScheduleSweep(_transform.position, BuildQueryParameters());
            yield return new WaitUntil(() => _sweepHandle.IsCompleted);
            _sweepHandle.Complete();

            int candidates = CollectCandidates(BuildQueryParameters());
            if (candidates == 0)
            {
                Commit();
                yield return null;
                yield break;
            }

            ScheduleConfirm(candidates);
            yield return new WaitUntil(() => _confirmHandle.IsCompleted);
            _confirmHandle.Complete();

            ApplyConfirmedPaths(candidates);
            Commit();
            yield return null;
        }

        /// <summary>
        /// Trace every satellite and refresh <see cref="skyView"/>, reflections
        /// included. Blocking; <see cref="UpdateSensor"/> spreads the same work over
        /// frames instead.
        /// </summary>
        public void Measure()
        {
            int blocked = MeasureDirect();
            if (blocked == 0 || _launchDirections == null)
            {
                Commit();
                return;
            }
            QueryParameters queryParameters = BuildQueryParameters();
            ScheduleSweep(_transform.position, queryParameters);
            _sweepHandle.Complete();
            int candidates = CollectCandidates(queryParameters);
            if (candidates > 0)
            {
                ScheduleConfirm(candidates);
                _confirmHandle.Complete();
                ApplyConfirmedPaths(candidates);
            }
            Commit();
        }

        private QueryParameters BuildQueryParameters()
        {
            return new QueryParameters
            {
                layerMask = _layerMask,
                hitTriggers = _hitTriggers,
                hitMultipleFaces = false,
                hitBackfaces = false,
            };
        }

        /// <summary>One ray per satellite. Returns how many are blocked.</summary>
        private int MeasureDirect()
        {
            if (_satellites == null || _satellites.Length != _satelliteCount)
            {
                BuildConstellation();
            }

            Vector3 origin = _transform.position;
            int usable = 0;
            int blocked = 0;
            for (int i = 0; i < _working.Length; i++)
            {
                bool hit = Physics.Raycast(origin, _rayDirections[i], _maxRayDistance, _layerMask, _hitTriggers);
                _working[i].visibility = hit ? SatelliteVisibility.Blocked : SatelliteVisibility.LineOfSight;
                _working[i].excessPathLength = 0.0f;
                _working[i].relativePowerDb = 0.0f;
                _working[i].reflectionPoint = Vector3.zero;
                if (hit)
                {
                    blocked++;
                }
                else
                {
                    usable++;
                }
            }

            // DOP is computed from the clean signals only. A receiver that falls back
            // on a reflected signal gets a WORSE solution, not a better one, so an
            // NLOS satellite must never improve the reported geometry.
            GnssConstellation.TryComputeDop(_working, out float hdop, out float pdop);
            _pendingUsable = usable;
            _pendingHdop = hdop;
            _pendingPdop = pdop;
            return blocked;
        }

        private void ScheduleSweep(Vector3 origin, QueryParameters queryParameters)
        {
            for (int i = 0; i < _launchDirections.Length; i++)
            {
                _sweepCommands[i] = new RaycastCommand(origin, _launchDirections[i], queryParameters, _maxRayDistance);
            }
            _sweepHandle = RaycastCommand.ScheduleBatch(_sweepCommands, _sweepHits, 64, 1, default(JobHandle));
            JobHandle.ScheduleBatchedJobs();
        }

        /// <summary>
        /// Turn sweep hits into candidate bounces, keeping only the few shortest per
        /// satellite. Returns how many confirm rays are queued.
        /// </summary>
        private int CollectCandidates(QueryParameters queryParameters)
        {
            for (int i = 0; i < _candidateExcess.Length; i++)
            {
                _candidateExcess[i] = float.PositiveInfinity;
            }

            for (int i = 0; i < _launchDirections.Length; i++)
            {
                // A batch raycast that hit nothing leaves the collider null. Checking
                // it rather than colliderInstanceID keeps this working on the 2022.3
                // the package targets as well as the Unity 6 the simulator runs.
                if (_sweepHits[i].collider == null)
                {
                    continue;
                }
                Vector3 normal = _sweepHits[i].normal;
                Vector3 launch = _launchDirections[i];
                // Front faces only: a ray leaving the antenna must strike the side of
                // the surface it can bounce off.
                if (Vector3.Dot(launch, normal) >= 0.0f)
                {
                    continue;
                }

                Vector3 required = GnssReflection.RequiredSatelliteDirection(launch, normal);
                for (int s = 0; s < _satellites.Length; s++)
                {
                    if (_working[s].visibility != SatelliteVisibility.Blocked)
                    {
                        continue;
                    }
                    if (Vector3.Dot(required, _rayDirections[s]) < _matchCosine)
                    {
                        continue;
                    }

                    float excess = GnssReflection.ExcessPathLength(_sweepHits[i].distance, launch, _rayDirections[s]);
                    // Lift the point off the surface so the confirming ray does not
                    // immediately hit the face it is leaving.
                    InsertCandidate(s, excess, _sweepHits[i].point + normal * 0.01f);
                    break;   // one candidate per hit; another hit will serve another satellite
                }
            }

            // Pack the rays densely so exactly the queued ones are scheduled; gaps
            // would leave stale (or on the first pass, zero-direction) commands in
            // the batch.
            int queued = 0;
            for (int s = 0; s < _working.Length; s++)
            {
                for (int k = 0; k < CandidatesPerSatellite; k++)
                {
                    int slot = s * CandidatesPerSatellite + k;
                    if (float.IsPositiveInfinity(_candidateExcess[slot]))
                    {
                        break;
                    }
                    _confirmCommands[queued] =
                        new RaycastCommand(_candidatePoint[slot], _rayDirections[s], queryParameters, _maxRayDistance);
                    _confirmSlot[queued] = slot;
                    queued++;
                }
            }
            return queued;
        }

        /// <summary>Keep the candidate list for one satellite sorted, shortest first.</summary>
        private void InsertCandidate(int satellite, float excess, Vector3 point)
        {
            int baseIndex = satellite * CandidatesPerSatellite;
            for (int k = 0; k < CandidatesPerSatellite; k++)
            {
                if (excess >= _candidateExcess[baseIndex + k])
                {
                    continue;
                }
                for (int shift = CandidatesPerSatellite - 1; shift > k; shift--)
                {
                    _candidateExcess[baseIndex + shift] = _candidateExcess[baseIndex + shift - 1];
                    _candidatePoint[baseIndex + shift] = _candidatePoint[baseIndex + shift - 1];
                }
                _candidateExcess[baseIndex + k] = excess;
                _candidatePoint[baseIndex + k] = point;
                return;
            }
        }

        private void ScheduleConfirm(int queued)
        {
            _confirmHandle = RaycastCommand.ScheduleBatch(_confirmCommands.GetSubArray(0, queued),
                                                          _confirmHits.GetSubArray(0, queued), 32, 1,
                                                          default(JobHandle));
            JobHandle.ScheduleBatchedJobs();
        }

        /// <summary>Accept the shortest candidate whose reflection point still sees the sky.</summary>
        private void ApplyConfirmedPaths(int queued)
        {
            for (int i = 0; i < queued; i++)
            {
                if (_confirmHits[i].collider != null)
                {
                    continue;   // the sky is blocked from that reflection point too
                }
                int slot = _confirmSlot[i];
                int s = slot / CandidatesPerSatellite;
                // Candidates are ordered shortest first, so the first confirmed one
                // for a satellite is the strongest path; later ones are ignored.
                if (_working[s].visibility == SatelliteVisibility.Nlos)
                {
                    continue;
                }
                if (_working[s].visibility == SatelliteVisibility.Blocked)
                {
                    _working[s].visibility = SatelliteVisibility.Nlos;
                    _working[s].excessPathLength = _candidateExcess[slot];
                    // A flat loss, not a Fresnel model: the real figure depends on the
                    // material, the incidence angle and the right-hand to left-hand
                    // polarisation flip that the antenna then rejects. Tune it against
                    // measured C/N0 rather than trusting the default.
                    _working[s].relativePowerDb = _reflectionLossDb;
                    _working[s].reflectionPoint = _candidatePoint[slot];
                }
            }
        }

        /// <summary>Publish the finished measurement in one step.</summary>
        private void Commit()
        {
            System.Array.Copy(_working, _satellites, _working.Length);
            _skyView.usableSatellites = _pendingUsable;
            _skyView.hdop = _pendingHdop;
            _skyView.pdop = _pendingPdop;
            _skyView.satellites = _satellites;
        }

        private void ReleaseSweepBuffers()
        {
            if (_sweepCommands.IsCreated) _sweepCommands.Dispose();
            if (_sweepHits.IsCreated) _sweepHits.Dispose();
            if (_confirmCommands.IsCreated) _confirmCommands.Dispose();
            if (_confirmHits.IsCreated) _confirmHits.Dispose();
            _launchDirections = null;
            _candidateExcess = null;
            _candidatePoint = null;
            _confirmSlot = null;
        }

        protected override void OnSensorDestroy()
        {
            _sweepHandle.Complete();
            _confirmHandle.Complete();
            ReleaseSweepBuffers();
        }
    }
}
