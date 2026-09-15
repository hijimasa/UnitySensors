using UnityEngine;

using UnitySensors.DataType.Sensor;
using UnitySensors.Sensor.GNSS;

namespace UnitySensors.Visualization.Sensor
{
    /// <summary>
    /// Draws the route every satellite's signal actually took to the antenna.
    /// </summary>
    /// <remarks>
    /// A GNSS error in a street is hard to believe from numbers alone: the fix is
    /// simply wrong and nothing on screen says why. Drawing the paths makes the
    /// mechanism visible -- lines reaching straight up to the sky, stubs where a
    /// building is in the way, and the reflected ones bending off a wall, which are
    /// the signals that actually move the solution.
    ///
    /// One <see cref="LineRenderer"/> per satellite, created once. That is a couple
    /// of dozen renderers, which costs nothing next to a lidar point cloud, and it
    /// gives width and colour in a built player where <c>Debug.DrawLine</c> draws
    /// nothing at all.
    /// </remarks>
    public class GnssSkyViewVisualizer : MonoBehaviour
    {
        [SerializeField]
        private GnssSkyViewSensor _source;

        /// <summary>[m] how far a clear line of sight is drawn towards the sky.</summary>
        [SerializeField, Min(1.0f)]
        private float _rayLength = 25.0f;
        /// <summary>[m] length of the stub drawn for a satellite that is blocked.</summary>
        [SerializeField, Min(0.0f)]
        private float _blockedLength = 3.0f;
        /// <summary>[m] width of a clear line of sight. These are context.</summary>
        [SerializeField, Min(0.001f)]
        private float _width = 0.05f;
        /// <summary>
        /// [m] width of a reflected path. Deliberately heavier than the rest: among
        /// twenty clear signals the two or three that bounced are the ones moving
        /// the fix, and they are the only thing in the picture worth looking at.
        /// </summary>
        [SerializeField, Min(0.001f)]
        private float _nlosWidth = 0.16f;

        [SerializeField]
        private Color _lineOfSightColor = new Color(0.25f, 0.95f, 0.35f, 1.0f);
        /// <summary>Reflected paths: the ones that put the fix in the wrong place.</summary>
        [SerializeField]
        private Color _nlosColor = new Color(1.0f, 0.65f, 0.1f, 1.0f);
        [SerializeField]
        private Color _blockedColor = new Color(0.8f, 0.2f, 0.2f, 0.45f);

        private Transform _transform;
        private LineRenderer[] _lines;
        private Material _material;
        private int _layer = -1;

        /// <summary>Configure at runtime (avoids Reflection).</summary>
        public void Configure(GnssSkyViewSensor source, int layer)
        {
            _source = source;
            _layer = layer;
        }

        private void Awake()
        {
            _transform = this.transform;
            if (_source == null)
            {
                _source = GetComponent<GnssSkyViewSensor>();
            }
        }

        private void OnEnable()
        {
            SetVisible(true);
        }

        private void OnDisable()
        {
            SetVisible(false);
        }

        private void OnDestroy()
        {
            if (_lines != null)
            {
                foreach (LineRenderer line in _lines)
                {
                    if (line != null)
                    {
                        Destroy(line.gameObject);
                    }
                }
                _lines = null;
            }
            if (_material != null)
            {
                Destroy(_material);
            }
        }

        private void SetVisible(bool visible)
        {
            if (_lines == null)
            {
                return;
            }
            foreach (LineRenderer line in _lines)
            {
                if (line != null)
                {
                    line.enabled = visible;
                }
            }
        }

        private void LateUpdate()
        {
            if (_source == null)
            {
                return;
            }
            GnssSkyView sky = _source.skyView;
            SatelliteObservation[] satellites = sky.satellites;
            if (satellites == null || satellites.Length == 0)
            {
                return;
            }

            EnsureLines(satellites.Length);
            Vector3 antenna = _transform.position;

            for (int i = 0; i < satellites.Length; i++)
            {
                LineRenderer line = _lines[i];
                SatelliteObservation satellite = satellites[i];
                Vector3 direction = GnssConstellation.ToUnityDirection(satellite.azimuth, satellite.elevation);

                switch (satellite.visibility)
                {
                    case SatelliteVisibility.LineOfSight:
                        line.positionCount = 2;
                        line.SetPosition(0, antenna);
                        line.SetPosition(1, antenna + direction * _rayLength);
                        SetColor(line, _lineOfSightColor);
                        break;

                    case SatelliteVisibility.Nlos:
                        // Antenna -> the wall it bounced off -> on towards the sky.
                        // The kink is the whole story: that detour is the extra path
                        // length the receiver measures as range.
                        line.positionCount = 3;
                        line.SetPosition(0, antenna);
                        line.SetPosition(1, satellite.reflectionPoint);
                        line.SetPosition(2, satellite.reflectionPoint + direction * _rayLength);
                        SetColor(line, _nlosColor);
                        line.widthMultiplier = _nlosWidth;
                        continue;

                    default:
                        line.positionCount = 2;
                        line.SetPosition(0, antenna);
                        line.SetPosition(1, antenna + direction * _blockedLength);
                        SetColor(line, _blockedColor);
                        break;
                }
                line.widthMultiplier = _width;
            }

            // A constellation can shrink if the sensor is reconfigured; hide the rest.
            for (int i = satellites.Length; i < _lines.Length; i++)
            {
                _lines[i].positionCount = 0;
            }
        }

        private static void SetColor(LineRenderer line, Color color)
        {
            line.startColor = color;
            line.endColor = color;
        }

        private void EnsureLines(int count)
        {
            if (_lines != null && _lines.Length >= count)
            {
                return;
            }
            if (_material == null)
            {
                // Sprites/Default is in the project's always-included shader list, so
                // a material built here survives into a player; a render-pipeline
                // shader looked up by name would not.
                _material = new Material(Shader.Find("Sprites/Default"));
            }

            var lines = new LineRenderer[count];
            for (int i = 0; i < count; i++)
            {
                if (_lines != null && i < _lines.Length)
                {
                    lines[i] = _lines[i];
                    continue;
                }
                var holder = new GameObject("GnssRay" + i);
                holder.transform.SetParent(_transform, false);
                if (_layer >= 0)
                {
                    holder.layer = _layer;
                }
                LineRenderer line = holder.AddComponent<LineRenderer>();
                line.material = _material;
                line.useWorldSpace = true;
                line.numCapVertices = 0;
                line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                line.receiveShadows = false;
                line.positionCount = 0;
                line.enabled = enabled;
                lines[i] = line;
            }
            _lines = lines;
        }
    }
}
