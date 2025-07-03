using UnityEngine;
using UnityEngine.Rendering;

namespace UnitySensors.Sensor.Camera
{
    public class FisheyeCameraSensor : CameraSensor
    {
        public enum CameraModel
        {
            Equidistant,
            EUCM
        }
        [SerializeField]
        private Material _fisheyeMat;
        [SerializeField]
        private int _cubemapResolution = 1024;
        [SerializeField, Range(90, 360)]
        private float _viewAngle = 210;
        [SerializeField]
        internal CameraModel _cameraModel = CameraModel.Equidistant;
        [SerializeField, Range(0.0f, 1.0f)]
        internal float _alpha = 1.0f;
        [SerializeField, Min(0)]
        internal float _beta = 0.0f;
        [SerializeField]
        internal Vector2 _focalLength = new Vector2(1.0f, 1.0f);
        [SerializeField]
        internal Vector2 _principalPoint = new Vector2(512f, 512f);
        private RenderTexture _cubemap;
        protected override void Init()
        {
            base.Init();
#if UNITY_6000_0_OR_NEWER
            var cubemapDescriptor = new RenderTextureDescriptor(_cubemapResolution, _cubemapResolution, RenderTextureFormat.ARGB32, 24)
            {
                dimension = TextureDimension.Cube
            };
            cubemapDescriptor.depthStencilFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.D24_UNorm_S8_UInt;
            _cubemap = new RenderTexture(cubemapDescriptor);
            
            var rtDescriptor = new RenderTextureDescriptor(_resolution.x, _resolution.y, RenderTextureFormat.ARGB32, 24);
            rtDescriptor.depthStencilFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.D24_UNorm_S8_UInt;
            _rt = new RenderTexture(rtDescriptor);
#else
            _cubemap = new RenderTexture(_cubemapResolution, _cubemapResolution, 24, RenderTextureFormat.ARGB32)
            {
                dimension = TextureDimension.Cube
            };
            _rt = new RenderTexture(_resolution.x, _resolution.y, 24, RenderTextureFormat.ARGB32);
#endif
            _texture = new Texture2D(_resolution.x, _resolution.y, TextureFormat.RGBA32, false);
            
            // Create material from shader if not assigned
            if (_fisheyeMat == null)
            {
                var shader = Shader.Find("UnitySensors/FisheyeCamera");
                if (shader != null)
                {
                    _fisheyeMat = new Material(shader);
                }
                else
                {
                    Debug.LogError("FisheyeCamera shader not found. Please assign _fisheyeMat manually.");
                }
            }
        }

        protected override void UpdateSensor()
        {
            if (_fisheyeMat == null) return;
            
            m_camera.RenderToCubemap(_cubemap);

            _fisheyeMat.SetInteger("_Equidistant", _cameraModel == CameraModel.Equidistant ? 1 : 0);
            _fisheyeMat.SetFloat("_Angle", _viewAngle);
            _fisheyeMat.SetFloat("_alpha", _alpha);
            _fisheyeMat.SetFloat("_beta", _beta);
            _fisheyeMat.SetFloat("_fx", _focalLength.x / _resolution.x);
            _fisheyeMat.SetFloat("_fy", _focalLength.y / _resolution.y);
            _fisheyeMat.SetFloat("_cx", _principalPoint.x / _resolution.x);
            _fisheyeMat.SetFloat("_cy", _principalPoint.y / _resolution.y);
            var eulerAngles = transform.rotation.eulerAngles;
            var rot = Quaternion.Euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
            var mat = Matrix4x4.TRS(Vector3.zero, rot, Vector3.one);
            _fisheyeMat.SetMatrix("_WorldTransform", mat);
            Graphics.Blit(_cubemap, _rt, _fisheyeMat);

            if (!LoadTexture(_rt, ref _texture)) return;
            onSensorUpdated?.Invoke();
        }
        protected override void OnSensorDestroy()
        {
            _cubemap.Release();
            _rt.Release();
        }
    }
}