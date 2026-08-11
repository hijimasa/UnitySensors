using UnityEngine;
using UnityEngine.UI;
using UnitySensors.Attribute;
using UnitySensors.Sensor;
using UnitySensors.Interface.Sensor;

namespace UnitySensors.Visualization.Sensor
{
    public class TextureVisualizer : Visualizer
    {
        private enum SourceTexture
        {
            Texture0,
            Texture1
        }

        [SerializeField, Interface(typeof(ITextureInterface))]
        private Object _source;
        [SerializeField]
        private SourceTexture _sourceTexture;
        [SerializeField]
        private RawImage _image;
        private ITextureInterface _sourceInterface;

        public void Configure(Object source, RawImage image, bool useTexture1 = false)
        {
            _source = source;
            _image = image;
            _sourceTexture = useTexture1 ? SourceTexture.Texture1 : SourceTexture.Texture0;
        }

        private void Start()
        {
            _sourceInterface = _source as ITextureInterface;
            var rectTransform = _image.GetComponent<RectTransform>();
            rectTransform.sizeDelta = new(rectTransform.sizeDelta.x, rectTransform.sizeDelta.x * _sourceInterface.texture0.height / _sourceInterface.texture0.width);

            if (_source is UnitySensor)
            {
                (_source as UnitySensor).onSensorUpdateComplete += Visualize;
            }

        }

        protected override void Visualize()
        {
            if (!_image || _sourceInterface == null) return;
            _image.texture = _sourceTexture == SourceTexture.Texture0 ? _sourceInterface.texture0 : _sourceInterface.texture1;
        }

        private void OnDestroy()
        {
            // Runtime toggling: without this, a destroyed visualizer keeps
            // receiving sensor updates and touches released buffers.
            if (_source is UnitySensor)
            {
                (_source as UnitySensor).onSensorUpdateComplete -= Visualize;
            }
        }

    }
}