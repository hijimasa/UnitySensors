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
        [Interface(typeof(ITimeInterface))]
        public UnityEngine.Object source;

        public string frame_id;

        private ITimeInterface _sourceInterface;

        public override void Init()
        {
            base.Init();
            _sourceInterface = source as ITimeInterface;

            _msg = new HeaderMsg();

            _msg.frame_id = frame_id;
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
