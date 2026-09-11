using System.Collections;
using UnityEngine;

using UnitySensors.DataType.Sensor;
using UnitySensors.Interface.Sensor;

namespace UnitySensors.Sensor.MagneticGuide
{
    /// <summary>
    /// Magnetic guide (line) sensor of the AGV kind — Roboteq MGS1600 / Accurate
    /// UDS-1213: a bar of Hall elements that reports where a magnetic tape lies under
    /// it, whether a tape is present at all, and markers of the opposite polarity to
    /// its left and right.
    /// </summary>
    /// <remarks>
    /// The bar is modelled as <c>width / pitch + 1</c> sample points along the sensor's
    /// local left axis. Each point casts a ray along the sensor's -up direction and
    /// counts as "over tape" when the nearest hit within <c>maxHeight</c> carries a
    /// <see cref="MagneticTape"/> and lies at least <c>minHeight</c> away — closer than
    /// the real sensor's operating range means the sensor is dragging on the tape,
    /// further means the field is too weak. <see cref="MagneticGuideSolver"/> turns the
    /// samples into a reading.
    ///
    /// Axes follow the URDF convention after import: the link's +x (forward) is Unity
    /// +z, +y (left) is Unity -x and +z (up) is Unity +y, so "left" here is
    /// <c>-transform.right</c> and "down" is <c>-transform.up</c>.
    /// </remarks>
    public class MagneticGuideSensor : UnitySensor, IMagneticGuideInterface
    {
        [SerializeField, Min(0.001f)]
        private float _width = 0.16f;
        [SerializeField, Min(0.0001f)]
        private float _pitch = 0.001f;
        [SerializeField, Min(0.0f)]
        private float _minHeight = 0.01f;
        [SerializeField, Min(0.0f)]
        private float _maxHeight = 0.06f;
        [SerializeField]
        private MagneticForkSelection _forkSelection = MagneticForkSelection.Nearest;
        [SerializeField, Min(0.0f)]
        private float _positionNoiseSigma = 0.0f;
        [SerializeField]
        private LayerMask _layerMask = ~0;

        private Transform _transform;
        private byte[] _samples;
        private RaycastHit[] _hits = new RaycastHit[8];
        private MagneticGuideReading _reading = MagneticGuideReading.None;

        public MagneticGuideReading reading { get => _reading; }
        public float width { get => _width; }
        public float pitch { get => _pitch; }
        public int sampleCount { get => Mathf.Max(2, Mathf.RoundToInt(_width / _pitch) + 1); }

        /// <summary>Configure at runtime (before or after Init; the sample buffer is resized lazily).</summary>
        public void Configure(float width, float pitch, float minHeight, float maxHeight,
            MagneticForkSelection forkSelection, float positionNoiseSigma)
        {
            _width = Mathf.Max(0.001f, width);
            _pitch = Mathf.Max(0.0001f, pitch);
            _minHeight = Mathf.Max(0.0f, minHeight);
            _maxHeight = Mathf.Max(_minHeight, maxHeight);
            _forkSelection = forkSelection;
            _positionNoiseSigma = Mathf.Max(0.0f, positionNoiseSigma);
            _samples = null;
        }

        protected override void Init()
        {
            _transform = this.transform;
        }

        protected override IEnumerator UpdateSensor()
        {
            Measure();
            yield return null;
        }

        protected override void OnSensorDestroy()
        {
        }

        /// <summary>Take one measurement now (synchronous; used by the update loop and by tests).</summary>
        public MagneticGuideReading Measure()
        {
            if (_transform == null)
            {
                _transform = this.transform;
            }
            int count = sampleCount;
            if (_samples == null || _samples.Length != count)
            {
                _samples = new byte[count];
            }

            Vector3 origin = _transform.position;
            Vector3 left = -_transform.right;
            Vector3 down = -_transform.up;
            float rayLength = _maxHeight + 0.001f;

            for (int i = 0; i < count; i++)
            {
                float y = MagneticGuideSolver.SamplePosition(i, count, _width);
                _samples[i] = Classify(origin + left * y, down, rayLength);
            }

            var next = MagneticGuideSolver.Solve(_samples, _width, _forkSelection,
                _reading.trackPosition, _reading.trackDetected);
            if (next.trackDetected)
            {
                float p = next.trackPosition;
                if (_positionNoiseSigma > 0.0f)
                {
                    p += Gaussian() * _positionNoiseSigma;
                }
                // The real sensor reports whole millimetres (its pitch).
                p = Mathf.Round(p / _pitch) * _pitch;
                next.trackPosition = Mathf.Clamp(p, -0.5f * _width, 0.5f * _width);
            }
            _reading = next;
            return _reading;
        }

        byte Classify(Vector3 origin, Vector3 down, float rayLength)
        {
            int n = Physics.RaycastNonAlloc(origin, down, _hits, rayLength, _layerMask, QueryTriggerInteraction.Collide);
            float bestDistance = float.MaxValue;
            byte kind = MagneticGuideSolver.None;
            for (int k = 0; k < n; k++)
            {
                RaycastHit hit = _hits[k];
                if (hit.distance >= bestDistance)
                {
                    continue;
                }
                MagneticTape tape = hit.collider.GetComponentInParent<MagneticTape>();
                if (tape == null)
                {
                    // Something else (the floor, the robot) is nearer than any tape:
                    // the tape below it is shielded. Keep the nearest non-tape hit as
                    // the cutoff.
                    bestDistance = hit.distance;
                    kind = MagneticGuideSolver.None;
                    continue;
                }
                bestDistance = hit.distance;
                kind = hit.distance < _minHeight
                    ? MagneticGuideSolver.None
                    : (tape.polarity == MagneticPolarity.Marker ? MagneticGuideSolver.Marker : MagneticGuideSolver.Track);
            }
            return kind;
        }

        static float Gaussian()
        {
            float u1 = 1.0f - Random.value;
            float u2 = Random.value;
            return Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Cos(2.0f * Mathf.PI * u2);
        }
    }
}
