using UnityEngine;
using UnityEngine.Rendering;

namespace UnitySensors.Sensor.Camera
{
    public class PanoramicCameraSensor : CameraSensor
    {
        [SerializeField]
        private Material _panoramicMat;
        [SerializeField]
        protected Vector2Int _cubemapResolution = new Vector2Int(1024, 1024);
        private RenderTexture _cubemap;
        protected override void Init()
        {
            base.Init();
#if UNITY_6000_0_OR_NEWER
            var cubemapDescriptor = new RenderTextureDescriptor(_cubemapResolution.x, _cubemapResolution.y, RenderTextureFormat.ARGB32, 24)
            {
                dimension = TextureDimension.Cube
            };
            cubemapDescriptor.depthStencilFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.D24_UNorm_S8_UInt;
            _cubemap = new RenderTexture(cubemapDescriptor);
            
            var rtDescriptor = new RenderTextureDescriptor(_resolution.x, _resolution.y, RenderTextureFormat.ARGB32, 24);
            rtDescriptor.depthStencilFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.D24_UNorm_S8_UInt;
            _rt = new RenderTexture(rtDescriptor);
#else
            _cubemap = new RenderTexture(_cubemapResolution.x, _cubemapResolution.y, 24, RenderTextureFormat.ARGB32)
            {
                dimension = TextureDimension.Cube
            };
            _rt = new RenderTexture(_resolution.x, _resolution.y, 24, RenderTextureFormat.ARGB32);
#endif
            _texture = new Texture2D(_resolution.x, _resolution.y, TextureFormat.RGBA32, false);
            
            // Create material from shader if not assigned
            if (_panoramicMat == null)
            {
                var shader = Shader.Find("UnitySensors/Panoramic");
                if (shader != null)
                {
                    _panoramicMat = new Material(shader);
                }
                else
                {
                    Debug.LogError("PanoramicCamera shader not found. Please assign _panoramicMat manually.");
                }
            }
        }

        protected override void UpdateSensor()
        {
            if (_panoramicMat == null) return;
            
            _panoramicMat.SetVector("_Rotation", transform.rotation.eulerAngles);
            m_camera.RenderToCubemap(_cubemap);
            Graphics.Blit(_cubemap, _rt, _panoramicMat);

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