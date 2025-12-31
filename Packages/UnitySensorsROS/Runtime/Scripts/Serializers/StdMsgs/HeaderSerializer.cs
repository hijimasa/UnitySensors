using System;
using UnityEngine;
using RosMessageTypes.Std;

using UnitySensors.Attribute;
using UnitySensors.Interface.Std;

namespace UnitySensors.ROS.Serializer.Std
{
    [System.Serializable]
    public class HeaderSerializer : RosMsgSerializer<HeaderMsg>
    {
        [SerializeField, Interface(typeof(ITimeInterface))]
        private UnityEngine.Object _source;
        [SerializeField]
        private string _frame_id;

        private ITimeInterface _sourceInterface;

        /// <summary>
        /// Configure header serializer at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(ITimeInterface source, string frameId)
        {
            _source = source as UnityEngine.Object;
            _sourceInterface = source;
            _frame_id = frameId;
        }

        public override void Init()
        {
            base.Init();
            _sourceInterface = _source as ITimeInterface;

            _msg = new HeaderMsg();

            _msg.frame_id = _frame_id;
#if ROS2
#else
            _msg.seq = 0;
#endif
        }

        public override HeaderMsg Serialize()
        {
#if ROS2
            int sec = (int)Math.Truncate(_sourceInterface.time);
#else
            uint sec = (uint)Math.Truncate(_sourceInterface.time);
#endif
            _msg.stamp.sec = sec;
            _msg.stamp.nanosec = (uint)((_sourceInterface.time - sec) * 1e+9);
#if ROS2
#else
            _msg.seq++;
#endif
            return _msg;
        }
    }
}
