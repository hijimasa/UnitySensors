using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using UnitySensors.ROS.Serializer;

namespace UnitySensors.ROS.Publisher
{
    public class RosMsgPublisher<T, TT> : MonoBehaviour where T : RosMsgSerializer<TT> where TT : Message, new()
    {
        public float frequency = 10.0f;

        public string topicName;

        public T serializer;

        private ROSConnection _ros;

        private float _time;
        private float _dt;

        private float _frequency_inv;

        protected virtual void Start()
        {
            _dt = 0.0f;
            _frequency_inv = 1.0f / frequency;

            _ros = ROSConnection.GetOrCreateInstance();
            _ros.RegisterPublisher<TT>(topicName);

            serializer.Init();
        }

        protected virtual void Update()
        {
            _dt += Time.deltaTime;
            if (_dt < _frequency_inv) return;

            _ros.Publish(topicName, serializer.Serialize());

            _dt -= _frequency_inv;
        }

        private void OnDestroy()
        {
            serializer.OnDestroy();
        }
    }
}
