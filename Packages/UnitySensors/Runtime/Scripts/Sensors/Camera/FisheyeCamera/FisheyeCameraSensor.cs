using System.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using UnitySensors.Utils.Texture;

namespace UnitySensors.Sensor.Camera
{
    public class FisheyeCameraSensor : CameraSensor
    {
        public enum CameraModel
        {
            UCM,
            EUCM,
            DS,
            KB4,
            OCAM,
            Equidistant
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
        internal float _xi = 0.34f;
        [SerializeField]
        internal Vector4 _kb4 = new Vector4(-0.01f, 0.03f, -0.02f, 0.005f);
        [SerializeField]
        internal Vector4 _affineCoeffs = new Vector4(1.0f, 0.0f, 0.0f, 1.0f);//c d e 1
        [SerializeField]
        internal float _a0 = 190.87f;
        [SerializeField]
        internal float _a1 = 0.0f;
        [SerializeField]
        internal float _a2 = 0.0f;
        [SerializeField]
        internal float _a3 = -0.000003f;
        [SerializeField]
        internal float _a4 = 0.0f;
        [SerializeField]
        internal Vector2 _focalLength = new Vector2(1.0f, 1.0f);
        [SerializeField]
        internal Vector2 _principalPoint = new Vector2(512f, 512f);
        private RenderTexture _cubemap;
        private TextureLoader _textureLoader;
        protected override void Init()
        {
            base.Init();
#if UNITY_6000_0_OR_NEWER
            // Unity 6000+ requires depth buffer for render textures used with cameras
            _cubemap = new RenderTexture(_cubemapResolution, _cubemapResolution, 24, RenderTextureFormat.ARGB32)
            {
                dimension = TextureDimension.Cube
            };
            _rt = new RenderTexture(_resolution.x, _resolution.y, 24, RenderTextureFormat.ARGB32);
#else
            _cubemap = new RenderTexture(_cubemapResolution, _cubemapResolution, 0, RenderTextureFormat.ARGB32)
            {
                dimension = TextureDimension.Cube
            };
            _rt = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.ARGB32);
#endif
            _texture = new Texture2D(_resolution.x, _resolution.y, TextureFormat.RGBA32, false);
            _textureLoader = new TextureLoader
            {
                source = _rt,
                destination = _texture
            };
        }

        protected override void ReleaseSensorResources()
        {
            if (_camera != null) _camera.targetTexture = null;
            if (_cubemap != null)
            {
                _cubemap.Release();
                Destroy(_cubemap);
                _cubemap = null;
            }
            if (_rt != null)
            {
                _rt.Release();
                Destroy(_rt);
                _rt = null;
            }
            if (_texture != null)
            {
                Destroy(_texture);
                _texture = null;
            }
        }

        protected override IEnumerator UpdateSensor()
        {
            m_camera.RenderToCubemap(_cubemap);

            _fisheyeMat.SetFloat("_CameraModel", (int)_cameraModel);
            _fisheyeMat.SetFloat("_Angle", _viewAngle);
            _fisheyeMat.SetFloat("_alpha", _alpha);
            _fisheyeMat.SetFloat("_beta", _beta);
            _fisheyeMat.SetFloat("_xi", _xi);
            _fisheyeMat.SetVector("_kb4", _kb4);
            _fisheyeMat.SetVector("_affineCoeffs", _affineCoeffs);
            _fisheyeMat.SetFloat("_a0", _a0);
            _fisheyeMat.SetFloat("_a1", _a1);
            _fisheyeMat.SetFloat("_a2", _a2);
            _fisheyeMat.SetFloat("_a3", _a3);
            _fisheyeMat.SetFloat("_a4", _a4);
            _fisheyeMat.SetFloat("_fx", _focalLength.x / _resolution.x);
            _fisheyeMat.SetFloat("_fy", _focalLength.y / _resolution.y);
            _fisheyeMat.SetFloat("_cx", _principalPoint.x / _resolution.x);
            _fisheyeMat.SetFloat("_cy", 1 - _principalPoint.y / _resolution.y);
            _fisheyeMat.SetFloat(" _resolutionX", _resolution.x);
            _fisheyeMat.SetFloat(" _resolutionY", _resolution.y);
            var eulerAngles = transform.rotation.eulerAngles;
            var rot = Quaternion.Euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);
            var mat = Matrix4x4.TRS(Vector3.zero, rot, Vector3.one);
            _fisheyeMat.SetMatrix("_WorldTransform", mat);
            Graphics.Blit(_cubemap, _rt, _fisheyeMat);

            yield return _textureLoader.LoadTextureAsync();
        }
        protected override void OnSensorDestroy()
        {
            _cubemap.Release();
            _rt.Release();
        }
    }
}