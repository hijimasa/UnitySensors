using UnityEngine;
using UnityEngine.Rendering;

namespace UnitySensors.Sensor.Camera
{
    public class RGBCameraSensor : CameraSensor
    {
        protected override void Init()
        {
            base.Init();
#if UNITY_6000_0_OR_NEWER
            var descriptor = new RenderTextureDescriptor(_resolution.x, _resolution.y, RenderTextureFormat.ARGB32, 24);
            descriptor.depthStencilFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.D24_UNorm_S8_UInt;
            _rt = new RenderTexture(descriptor);
#else
            _rt = new RenderTexture(_resolution.x, _resolution.y, 24, RenderTextureFormat.ARGB32);
#endif
            _camera.targetTexture = _rt;

            _texture = new Texture2D(_resolution.x, _resolution.y, TextureFormat.RGBA32, false);
        }

        protected override void UpdateSensor()
        {
            if (!LoadTexture(_rt, ref _texture)) return;

            if (onSensorUpdated != null)
                onSensorUpdated.Invoke();
        }

        protected override void OnSensorDestroy()
        {
            _rt.Release();
        }
    }
}
