using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnitySensors.DataType.Geometry;
using UnitySensors.DataType.Sensor;
using UnitySensors.Interface.Geometry;
using UnitySensors.Interface.Sensor;

namespace UnitySensors.Sensor.GNSS
{
    /// <summary>
    /// A GNSS receiver: the antenna's position, degraded the way the environment
    /// would degrade it.
    /// </summary>
    /// <remarks>
    /// Without a <see cref="GnssSkyViewSensor"/> this reports the exact truth, which
    /// is what it always did. With one, the sky it traces drives a receiver model:
    /// how many satellites are usable decides the solution grade, the satellites that
    /// only arrive by reflection displace the fix in a direction the map explains,
    /// and carrier phase continuity decides how long a lost fix takes to come back.
    ///
    /// The model lives here, in the simulator, rather than in a node beside it. A
    /// sensor's output is the simulator's business: a ROS user spawns a robot and
    /// gets a realistic fix on the standard topic, with nothing extra to launch. It
    /// also avoids shipping twenty-odd satellites over the wire every epoch merely so
    /// something else can count them.
    /// </remarks>
    public class GNSSSensor : UnitySensor, IGeoCoordinateInterface, IGnssSolutionInterface
    {
        [SerializeField]
        private GeoCoordinateSystem _coordinateSystem;

        /// <summary>Sky source. Left empty, the receiver reports the exact truth.</summary>
        [SerializeField]
        private GnssSkyViewSensor _skyView;

        [SerializeField]
        private RtkQualityModel _model = new RtkQualityModel();

        [SerializeField]
        private int _seed = 20260914;

        private Transform _transform;

        [SerializeField]
        private GeoCoordinate _coordinate;
        private GnssSolution _solution = GnssSolution.None;
        private readonly CarrierLockTracker _carrierLock = new CarrierLockTracker();
        private readonly List<ushort> _lineOfSight = new List<ushort>();
        private float _lastMeasured = -1.0f;
        private bool _skyViewSearched;

        public GeoCoordinate coordinate { get => _coordinate; }
        public GnssSolution solution { get => _solution; }
        public RtkQualityModel model { get => _model; }

        /// <summary>
        /// Configure sensor at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(GeoCoordinateSystem coordinateSystem)
        {
            _coordinateSystem = coordinateSystem;
        }

        /// <summary>Seed the receiver model. The sky source is found on this link.</summary>
        public void ConfigureReceiver(int seed)
        {
            _seed = seed;
            _model.Reset(_seed);
        }

        protected override void Init()
        {
            _transform = this.transform;
            _model.Reset(_seed);
            // Produce a value straight away. The publisher runs on its own timer and
            // can fire before the sensor's first update; an unset coordinate is a
            // null reference, and an unset solution reads as no fix at all.
            _solution = GnssSolution.None;
            _solution.state = RtkState.Fix;
            _solution.horizontalSigma = _model.fix.reportedSigma;
            if (_coordinateSystem != null)
            {
                _coordinate = ToCoordinate(_transform.position, 0.0, 0.0);
            }
        }

        /// <summary>
        /// The sky sensor on this link, looked up on first use.
        /// </summary>
        /// <remarks>
        /// Lazily, because the two sensors are separate entries in the URDF and
        /// resolving at construction would silently depend on which one is written
        /// first -- a trap that shows up as a receiver that never degrades.
        /// </remarks>
        private GnssSkyViewSensor SkyView()
        {
            if (_skyView == null && !_skyViewSearched)
            {
                _skyView = GetComponent<GnssSkyViewSensor>();
                _skyViewSearched = true;
            }
            return _skyView;
        }

        protected override IEnumerator UpdateSensor()
        {
            Vector3 position = _transform.position;

            if (_coordinateSystem == null)
            {
                yield return null;
                yield break;
            }

            GnssSkyViewSensor skyView = SkyView();
            if (skyView == null)
            {
                // No sky traced: the honest answer is the truth, not a guess at what
                // a receiver might have done.
                _solution = GnssSolution.None;
                _solution.state = RtkState.Fix;
                _solution.horizontalSigma = _model.fix.reportedSigma;
                _coordinate = ToCoordinate(position, 0.0, 0.0);
                yield return null;
                yield break;
            }

            float now = Time.time;
            float dt = (_lastMeasured < 0.0f) ? this.dt : Mathf.Max(0.0f, now - _lastMeasured);
            _lastMeasured = now;

            GnssSkyView sky = skyView.skyView;
            SatelliteObservation[] satellites = sky.satellites;

            _lineOfSight.Clear();
            if (satellites != null)
            {
                for (int i = 0; i < satellites.Length; i++)
                {
                    if (satellites[i].visibility == SatelliteVisibility.LineOfSight)
                    {
                        _lineOfSight.Add(satellites[i].prn);
                    }
                }
            }
            _carrierLock.Update(_lineOfSight, dt);

            _solution = _model.Update(sky.usableSatellites, sky.hdop, sky.pdop, satellites, dt,
                _carrierLock.LockedFor(_model.lockSecondsForFix));

            _coordinate = ToCoordinate(position, _solution.errorEast, _solution.errorNorth);
            yield return null;
        }

        /// <summary>
        /// Antenna position plus a horizontal error, as a geodetic coordinate.
        /// </summary>
        /// <remarks>
        /// The host publishes its world as ROS ENU, where east is Unity +z and north
        /// is Unity -x. That is NOT the mapping the geodetic converter assumes (it
        /// takes Unity x as easting), so going through GetCoordinate would produce a
        /// fix rotated ninety degrees against the simulator's own ground truth --
        /// wrong by a metre for every metre driven. The ENU offset is therefore
        /// stated explicitly.
        /// </remarks>
        private GeoCoordinate ToCoordinate(Vector3 worldPosition, double errorEast, double errorNorth)
        {
            Vector3 local = _coordinateSystem.ToLocal(worldPosition);
            double east = local.z + errorEast;
            double north = -local.x + errorNorth;
            return _coordinateSystem.GetCoordinateFromEnu(east, north, local.y);
        }

        protected override void OnSensorDestroy()
        {
        }
    }
}
