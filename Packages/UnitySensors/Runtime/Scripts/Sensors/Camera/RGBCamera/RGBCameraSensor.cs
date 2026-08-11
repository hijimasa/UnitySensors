using System.Collections;
using UnityEngine;
using UnitySensors.Utils.Texture;

namespace UnitySensors.Sensor.Camera
{
    public class RGBCameraSensor : CameraSensor
    {
        private TextureLoader _textureLoader;
        protected override void Init()
        {
            base.Init();
#if UNITY_6000_0_OR_NEWER
            // Unity 6000+ requires depth buffer for render textures used with cameras
            _rt = new RenderTexture(_resolution.x, _resolution.y, 24, RenderTextureFormat.ARGB32);
#else
            _rt = new RenderTexture(_resolution.x, _resolution.y, 0, RenderTextureFormat.ARGB32);
#endif
            _camera.targetTexture = _rt;

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
            _camera.Render();
            yield return _textureLoader.LoadTextureAsync();
        }

        protected override void OnSensorDestroy()
        {
            _rt.Release();
        }
    }
}
