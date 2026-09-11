using UnitySensors.DataType.Sensor;

namespace UnitySensors.Interface.Sensor
{
    public interface IMagneticGuideInterface
    {
        public MagneticGuideReading reading { get; }
    }
}
