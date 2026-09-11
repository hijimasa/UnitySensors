using UnityEngine;
using UnitySensors.DataType.Sensor;

namespace UnitySensors.Sensor.MagneticGuide
{
    /// <summary>
    /// Marks a collider as magnetic tape for <see cref="MagneticGuideSensor"/>.
    /// Put it on (or above) the collider that outlines the tape; the sensor finds it
    /// with GetComponentInParent on whatever its rays hit. The collider should be a
    /// trigger so the tape does not bump the robot.
    /// </summary>
    public class MagneticTape : MonoBehaviour
    {
        [SerializeField]
        private MagneticPolarity _polarity = MagneticPolarity.Track;

        public MagneticPolarity polarity { get => _polarity; set => _polarity = value; }
    }
}
