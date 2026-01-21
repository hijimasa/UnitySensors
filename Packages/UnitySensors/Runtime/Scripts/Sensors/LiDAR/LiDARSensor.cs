using UnityEngine;
using Unity.Collections;
using Unity.Jobs;

using UnitySensors.DataType.LiDAR;
using UnitySensors.DataType.Sensor;
using UnitySensors.DataType.Sensor.PointCloud;
using UnitySensors.Interface.Sensor;

namespace UnitySensors.Sensor.LiDAR
{
    public abstract class LiDARSensor : UnitySensor, IPointCloudInterface<PointXYZI>
    {
        [SerializeField]
        private ScanPattern _scanPattern;
        [SerializeField]
        private int _pointsNumPerScan = 1;
        [SerializeField]
        private float _minRange = 0.5f;
        [SerializeField]
        private float _maxRange = 100.0f;
        [SerializeField]
        private float _gaussianNoiseSigma = 0.0f;
        [SerializeField]
        private float _maxIntensity = 255.0f;

        private PointCloud<PointXYZI> _pointCloud;
        protected JobHandle _jobHandle;

        protected ScanPattern scanPattern { get => _scanPattern; }
        protected float minRange { get => _minRange; }
        protected float maxRange { get => _maxRange; }
        protected float gaussianNoiseSigma { get => _gaussianNoiseSigma; }
        protected float maxIntensity { get => _maxIntensity; }
        public PointCloud<PointXYZI> pointCloud { get => _pointCloud; }
        public int pointsNum { get => _pointsNumPerScan; }

        /// <summary>
        /// Configure LiDAR sensor parameters at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(ScanPattern pattern, float minRng, float maxRng, float noiseSigma, int pointsPerScan)
        {
            _scanPattern = pattern;
            _minRange = minRng;
            _maxRange = maxRng;
            _gaussianNoiseSigma = noiseSigma;
            _pointsNumPerScan = pointsPerScan;
        }

        protected override void Init()
        {
            if (_scanPattern == null)
            {
                return;
            }
            Initialize();
        }

        public virtual void Initialize()
        {
            _pointsNumPerScan = Mathf.Clamp(_pointsNumPerScan, 1, scanPattern.size);
            _pointCloud = new PointCloud<PointXYZI>()
            {
                points = new NativeArray<PointXYZI>(_pointsNumPerScan, Allocator.Persistent)
            };
        }

        protected override void OnSensorDestroy()
        {
            _pointCloud.Dispose();
        }

        /// <summary>
        /// Completes any pending job that writes to pointCloud.
        /// Must be called before reading from pointCloud.points to avoid race conditions.
        /// </summary>
        public void CompleteJob()
        {
            _jobHandle.Complete();
        }
    }
}
