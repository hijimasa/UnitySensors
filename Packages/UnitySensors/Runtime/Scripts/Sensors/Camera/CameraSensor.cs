using System.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using UnitySensors.Interface.Sensor;

namespace UnitySensors.Sensor.Camera
{
    [RequireComponent(typeof(UnityEngine.Camera))]
    public abstract class CameraSensor : UnitySensor, ICameraInterface, ITextureInterface
    {
        [SerializeField]
        protected internal Vector2Int _resolution = new Vector2Int(640, 480);
        [SerializeField]
        protected internal float _fov = 30.0f;

        protected RenderTexture _rt = null;
        protected UnityEngine.Camera _camera;
        protected Texture2D _texture;

        public UnityEngine.Camera m_camera { get => _camera; }

        public virtual Texture2D texture0 { get => _texture; }

        public virtual Texture2D texture1 { get => _texture; }

        public float texture0FarClipPlane { get => _camera.farClipPlane; }

        /// <summary>
        /// Configure camera sensor parameters at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(Vector2Int resolution, float fov)
        {
            _resolution = resolution;
            _fov = fov;
        }

        /// <summary>
        /// Re-run Init() with the current configuration. Call after Configure()
        /// when the component was added at runtime: Awake() has already built
        /// the sensor with default settings by the time Configure() can run,
        /// so the render targets and camera parameters need to be rebuilt.
        /// (Same convention as LiDARSensor.Initialize().)
        /// </summary>
        public void Initialize()
        {
            ReleaseSensorResources();
            Init();
        }

        /// <summary>Release everything Init() created so it can run again.</summary>
        protected virtual void ReleaseSensorResources() { }

        protected override void Init()
        {
            _camera = GetComponent<UnityEngine.Camera>();
            _camera.fieldOfView = _fov;
            _camera.enabled = false;
        }
    }
}
