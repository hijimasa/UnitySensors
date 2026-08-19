using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;

using UnitySensors.Attribute;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Serializer.Std;

namespace UnitySensors.ROS.Serializer.Sensor
{
    [System.Serializable]
    public class IMUMsgSerializer : RosMsgSerializer<ImuMsg>
    {
        [SerializeField, Interface(typeof(IImuDataInterface))]
        private Object _source;
        [SerializeField]
        private HeaderSerializer _header;

        private IImuDataInterface _sourceInterface;

        /// <summary>
        /// Configure serializer at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(IImuDataInterface source, HeaderSerializer header)
        {
            _source = source as Object;
            _sourceInterface = source;
            _header = header;
        }

        public override void Init()
        {
            base.Init();
            _header.Init();
            _sourceInterface = _source as IImuDataInterface;
        }

        public override ImuMsg Serialize()
        {
            _msg.header = _header.Serialize();
            _msg.linear_acceleration = _sourceInterface.acceleration.To<FLU>();
            _msg.orientation = _sourceInterface.rotation.To<FLU>();
            // Angular velocity is a pseudo-vector: mapping it from Unity's
            // left-handed frame to ROS's right-handed FLU needs a negation on
            // top of the axis swap (same convention as Unity Robotics' own
            // examples, e.g. -rigidbody.angularVelocity.To<FLU>()). Without it
            // a CCW yaw in ROS terms is published as a negative z rate.
            _msg.angular_velocity = (-_sourceInterface.angularVelocity).To<FLU>();
            return _msg;
        }
    }
}
