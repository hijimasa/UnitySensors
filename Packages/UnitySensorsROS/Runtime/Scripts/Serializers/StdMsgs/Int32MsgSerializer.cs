using UnityEngine;
using RosMessageTypes.Std;

using UnitySensors.Attribute;
using UnitySensors.Interface.Std;

namespace UnitySensors.ROS.Serializer.Std
{
    [System.Serializable]
    public class Int32MsgSerializer : RosMsgSerializer<Int32Msg>
    {
        [SerializeField, Interface(typeof(IIntStateInterface))]
        private Object _source;

        private IIntStateInterface _sourceInterface;

        /// <summary>
        /// Configure serializer at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(IIntStateInterface source)
        {
            _source = source as Object;
            _sourceInterface = source;
        }

        public override void Init()
        {
            base.Init();
            _sourceInterface = _source as IIntStateInterface;
        }

        public override Int32Msg Serialize()
        {
            _msg.data = _sourceInterface.state;
            return _msg;
        }
    }
}
