using UnitySensors.DataType.Sensor;
using UnitySensors.Interface.Sensor.PointCloud;

namespace UnitySensors.Interface.Sensor
{
    public interface IPointCloudInterface<T> where T : struct, IPointInterface
    {
        public PointCloud<T> pointCloud { get; }
        public int pointsNum { get; }
        /// <summary>
        /// Completes any pending job that writes to pointCloud.
        /// Must be called before reading from pointCloud.points to avoid race conditions.
        /// </summary>
        public void CompleteJob();
    }
}