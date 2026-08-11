using UnityEngine;
using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor.PointCloud;
using UnitySensors.Interface.Sensor;
using UnitySensors.Sensor;

namespace UnitySensors.Visualization.Sensor
{
    public class LiDARPointCloudVisualizer : PointCloudVisualizer<PointXYZI>
    {
        [SerializeField, Interface(typeof(IPointCloudInterface<PointXYZI>))]
        private Object _source;

        public void Configure(Object source, Utils.PointCloud.PointUtilitiesSO pointUtilitiesSO, int drawLayer = 0)
        {
            _source = source;
            base.Configure(pointUtilitiesSO, drawLayer);
        }

        protected override void Start()
        {
            base.SetSource(_source as IPointCloudInterface<PointXYZI>);
            base.Start();
            // Subscribe only once fully initialized, so a sensor update can
            // never reach a visualizer whose buffers do not exist yet.
            if (_source is UnitySensor)
            {
                (_source as UnitySensor).onSensorUpdateComplete += Visualize;
            }
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