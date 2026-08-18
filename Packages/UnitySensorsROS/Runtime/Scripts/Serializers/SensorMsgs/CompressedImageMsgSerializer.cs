using System;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Experimental.Rendering;

using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

using UnitySensors.Attribute;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Serializer.Std;

namespace UnitySensors.ROS.Serializer.Sensor
{
    /// <summary>
    /// Publishes a camera image as JPEG.
    /// </summary>
    /// <remarks>
    /// The encoding runs on a worker thread. It costs a few milliseconds per image,
    /// which is minor on its own but not when a simulator carries a dozen cameras:
    /// done inline it would add over a hundred milliseconds to every frame, and the
    /// frame is also where physics runs. So each call hands the pixels to a worker
    /// and publishes the image the previous call started, which puts the encoding
    /// off the critical path at the cost of one publish period of latency. The
    /// timestamp travels with the pixels, so a message still carries the time its
    /// image was taken rather than the time it went out.
    ///
    /// Texture2D.EncodeToJPG cannot be used here - it touches the texture object and
    /// so is main thread only. EncodeArrayToJPG takes plain bytes instead. Should it
    /// turn out to be unusable off the main thread, the first failure falls back to
    /// encoding inline and says so, rather than leaving the stream dead.
    /// </remarks>
    [System.Serializable]
    public class CompressedImageMsgSerializer : RosMsgSerializer<CompressedImageMsg>
    {
        private enum SourceTexture
        {
            Texture0,
            Texture1
        }

        [SerializeField, Interface(typeof(ITextureInterface))]
        private UnityEngine.Object _source;
        [SerializeField]
        private SourceTexture _sourceTexture;

        [SerializeField]
        private HeaderSerializer _header;
        [SerializeField, Range(1, 100)]
        private int quality = 75;

        private ITextureInterface _sourceInterface;

        // 符号化に渡す画素。ワーカーが読んでいる間は書き換えないよう、次の
        // 取り込みの前に必ず完了を待つ。そのため 1 枚で足りる。
        private byte[] _pixels;
        private Task<byte[]> _pending;
        private byte[] _encoded;
        private GraphicsFormat _format;
        private uint _width;
        private uint _height;
        // 画素を取り込んだ時刻。符号化を待つ間ぶんだけ publish より前になる。
        private int _pendingSec;
        private uint _pendingNanosec;
        private int _encodedSec;
        private uint _encodedNanosec;
        private bool _encodeInline;

        /// <summary>
        /// Configure serializer at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(ITextureInterface source, HeaderSerializer header, int sourceTextureIndex, int jpegQuality = 0)
        {
            _source = source as UnityEngine.Object;
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

            Texture2D texture = SourceTexture2D();
            _format = texture.graphicsFormat;
            _width = (uint)texture.width;
            _height = (uint)texture.height;
            _pixels = new byte[texture.GetRawTextureData<byte>().Length];
        }

        private Texture2D SourceTexture2D()
        {
            return _sourceTexture == SourceTexture.Texture0
                ? _sourceInterface.texture0 : _sourceInterface.texture1;
        }

        private byte[] Encode()
        {
            return ImageConversion.EncodeArrayToJPG(_pixels, _format, _width, _height, 0, quality);
        }

        public override CompressedImageMsg Serialize()
        {
            // 前回投げたぶんを回収する。符号化は publish 周期よりずっと短いので
            // 通常は完了済み。まだなら待つしかないが、待つ時間は次の取り込みを
            // 安全にするためにも必要。
            if (_pending != null)
            {
                try
                {
                    _encoded = _pending.GetAwaiter().GetResult();
                    _encodedSec = _pendingSec;
                    _encodedNanosec = _pendingNanosec;
                }
                catch (Exception e)
                {
                    Debug.LogWarning("JPEG encoding failed on a worker thread; "
                                     + "falling back to encoding inline. " + e.Message);
                    _encodeInline = true;
                }
                _pending = null;
            }

            HeaderMsg header = _header.Serialize();
            _pendingSec = header.stamp.sec;
            _pendingNanosec = header.stamp.nanosec;
            SourceTexture2D().GetRawTextureData<byte>().CopyTo(_pixels);

            if (_encodeInline)
            {
                _encoded = Encode();
                _encodedSec = _pendingSec;
                _encodedNanosec = _pendingNanosec;
            }
            else if (_encoded == null)
            {
                // 1 枚目だけは裏に回すと publish するものが無いので、その場で作る。
                _encoded = Encode();
                _encodedSec = _pendingSec;
                _encodedNanosec = _pendingNanosec;
            }
            else
            {
                _pending = Task.Run(() => Encode());
            }

            _msg.header = header;
            _msg.header.stamp.sec = _encodedSec;
            _msg.header.stamp.nanosec = _encodedNanosec;
            _msg.data = _encoded;
            return _msg;
        }

        public override void OnDestroy()
        {
            // ワーカーが _pixels を読んでいる最中に消えないよう、完了を待つ。
            if (_pending != null)
            {
                try { _pending.GetAwaiter().GetResult(); } catch (Exception) { }
                _pending = null;
            }
            base.OnDestroy();
        }
    }
}
