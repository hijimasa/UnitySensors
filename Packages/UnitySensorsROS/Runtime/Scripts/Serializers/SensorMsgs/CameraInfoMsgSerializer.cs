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

        /// <summary>
        /// Configure serializer at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(ICameraInterface source, HeaderSerializer header)
        {
            _source = source as Object;
            _sourceInterface = source;
            _header = header;
        }

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
    }
}
