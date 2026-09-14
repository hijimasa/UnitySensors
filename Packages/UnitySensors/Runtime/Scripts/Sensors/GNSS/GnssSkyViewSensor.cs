using System.Collections;
using UnityEngine;

using UnitySensors.DataType.Sensor;
using UnitySensors.Interface.Sensor;

namespace UnitySensors.Sensor.GNSS
{
    /// <summary>
    /// Ray-traced sky visibility at a GNSS antenna: which satellites reach it
    /// directly, and how good the geometry of that set is.
    /// </summary>
    /// <remarks>
    /// This sensor deliberately stops at the geometry. It does not produce a
    /// position, a fix grade or an error — those depend on receiver behaviour that
    /// is far easier to tune and test outside the simulator, so the consumer (the
    /// GPS emulator on the ROS side) owns them. What cannot be done outside the
    /// simulator is knowing what the buildings block, and that is exactly what this
    /// provides.
    ///
    /// One ray per satellite per update: a couple of dozen rays at a few hertz,
    /// which is nothing next to the LiDARs' tens of thousands per frame. A plain
    /// loop is used rather than a RaycastCommand batch because at this count the
    /// batch's setup costs more than it saves; reflected paths (which need
    /// thousands of rays) will want the batch API.
    ///
    /// Trigger colliders are ignored by default. The simulator uses triggers for
    /// objects that should be visible to sensors but not block movement (weeds,
    /// magnetic tape), and none of those should shadow a satellite. Buildings are
    /// solid colliders and block normally.
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

        private Transform _transform;
        private SatelliteObservation[] _satellites;
        private Vector3[] _rayDirections;
        private GnssSkyView _skyView = GnssSkyView.None;

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

        protected override void Init()
        {
            _transform = this.transform;
            BuildConstellation();
        }

        private void BuildConstellation()
        {
            _satellites = GnssConstellation.Generate(_satelliteCount, _elevationMaskDeg * Mathf.Deg2Rad, _seed);
            _rayDirections = new Vector3[_satellites.Length];
            for (int i = 0; i < _satellites.Length; i++)
            {
                _rayDirections[i] = GnssConstellation.ToUnityDirection(_satellites[i].azimuth, _satellites[i].elevation);
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
            Measure();
            yield return null;
        }

        /// <summary>Trace every satellite and refresh <see cref="skyView"/>.</summary>
        public void Measure()
        {
            if (_satellites == null || _satellites.Length != _satelliteCount)
            {
                BuildConstellation();
            }

            Vector3 origin = _transform.position;
            int usable = 0;
            for (int i = 0; i < _satellites.Length; i++)
            {
                bool blocked = Physics.Raycast(origin, _rayDirections[i], _maxRayDistance, _layerMask, _hitTriggers);
                // No reflected path is searched for yet, so a blocked satellite is
                // simply unusable rather than NLOS.
                _satellites[i].visibility = blocked ? SatelliteVisibility.Blocked : SatelliteVisibility.LineOfSight;
                _satellites[i].excessPathLength = 0.0f;
                _satellites[i].relativePowerDb = 0.0f;
                if (!blocked)
                {
                    usable++;
                }
            }

            GnssConstellation.TryComputeDop(_satellites, out float hdop, out float pdop);
            _skyView.usableSatellites = usable;
            _skyView.hdop = hdop;
            _skyView.pdop = pdop;
            _skyView.satellites = _satellites;
        }

        protected override void OnSensorDestroy()
        {
        }
    }
}
