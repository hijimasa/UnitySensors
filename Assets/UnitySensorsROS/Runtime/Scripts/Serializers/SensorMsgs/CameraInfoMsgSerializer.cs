using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;

using UnitySensors.Attribute;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Serializer.Std;

namespace UnitySensors.ROS.Serializer.Sensor
{
    [System.Serializable]
    public class CameraInfoMsgSerializer : RosMsgSerializer<CameraInfoMsg>
    {
        [SerializeField, Interface(typeof(ICameraInterface))]
        private Object _source;

        [SerializeField]
        private HeaderSerializer _header;

        private ICameraInterface _sourceInterface;

        public override void Init()
        {
            base.Init();
            _header.Init();
            _sourceInterface = _source as ICameraInterface;
        }

        public override CameraInfoMsg Serialize()
        {
            _msg = CameraInfoGenerator.ConstructCameraInfoMessage(_sourceInterface.m_camera, _header.Serialize());
            return _msg;
        }

        public override void SetObject(UnityEngine.Object obj)
        {
            if (_source == null)
            {
                _source = new Object();
            }
            _source = obj;
        }

        public override void SetHeaderObject(UnityEngine.Object obj)
        {
            if (_header == null)
            {
                _header = new HeaderSerializer();
            }
            if (_header.source == null)
            {
                _header.source = new Object();
            }
            _header.source = obj;
        }

        public override void SetFrameId(string frame_id)
        {
            _header.frame_id = frame_id;
        }
    }
}
