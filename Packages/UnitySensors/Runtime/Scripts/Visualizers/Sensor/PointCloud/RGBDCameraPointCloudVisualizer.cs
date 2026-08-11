using UnityEngine;
using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor.PointCloud;
using UnitySensors.Interface.Sensor;
using UnitySensors.Sensor;
using UnitySensors.Sensor.Camera;

namespace UnitySensors.Visualization.Sensor
{
    public class RGBDCameraPointCloudVisualizer : PointCloudVisualizer<PointXYZRGB>
    {
        [SerializeField, Interface(typeof(IPointCloudInterface<PointXYZRGB>))]
        private Object _source;

        public void Configure(Object source, Utils.PointCloud.PointUtilitiesSO pointUtilitiesSO, int drawLayer = 0)
        {
            _source = source;
            base.Configure(pointUtilitiesSO, drawLayer);
            // OnEnable has already run when the component is added at runtime,
            // so mirror its point-cloud conversion switch here
            if (_source is DepthCameraSensor)
                (_source as DepthCameraSensor).convertToPointCloud = true;
            else if (_source is RGBDCameraSensor)
                (_source as RGBDCameraSensor).convertToPointCloud = true;
        }

        private void OnEnable()
        {
            if (_source is DepthCameraSensor)
                (_source as DepthCameraSensor).convertToPointCloud = true;
            else if (_source is RGBDCameraSensor)
                (_source as RGBDCameraSensor).convertToPointCloud = true;
        }

        protected override void Start()
        {
            base.SetSource(_source as IPointCloudInterface<PointXYZRGB>);
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