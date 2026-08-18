using UnityEngine;

using RosMessageTypes.Sensor;

using UnitySensors.Attribute;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Serializer.Std;

namespace UnitySensors.ROS.Serializer.Sensor
{
    [System.Serializable]
    public class CompressedImageMsgSerializer : RosMsgSerializer<CompressedImageMsg>
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
        private HeaderSerializer _header;
        [SerializeField, Range(1, 100)]
        private int quality = 75;

        private ITextureInterface _sourceInterface;

        /// <summary>
        /// Configure serializer at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(ITextureInterface source, HeaderSerializer header, int sourceTextureIndex, int jpegQuality = 0)
        {
            _source = source as Object;
            _sourceInterface = source;
            _header = header;
            _sourceTexture = (SourceTexture)sourceTextureIndex;
            // 0 は「指定なし」。既定の 75 のまま置いておく。品質は転送量に
            // 直接効くので、帯域が苦しい構成では呼び出し側から下げられるようにする。
            if (jpegQuality > 0) quality = Mathf.Clamp(jpegQuality, 1, 100);
        }

        public override void Init()
        {
            base.Init();
            _header.Init();
            _sourceInterface = _source as ITextureInterface;
            _msg.format = "jpeg";
        }

        public override CompressedImageMsg Serialize()
        {
            _msg.header = _header.Serialize();
            _msg.data = (_sourceTexture == SourceTexture.Texture0 ? _sourceInterface.texture0 : _sourceInterface.texture1).EncodeToJPG(quality);
            return _msg;
        }
    }
}
