using UnityEngine;

namespace UnitySensors.Sensor.Contact
{
    /// <summary>
    /// Forwards OnCollision* messages from a collider-only child GameObject to
    /// the ContactSensor on the owning body link. Added automatically by
    /// ContactSensor; not meant to be attached by hand.
    /// </summary>
    [AddComponentMenu("")]
    public class ContactEventRelay : MonoBehaviour
    {
        private ContactSensor _sensor;

        public void Initialize(ContactSensor sensor)
        {
            _sensor = sensor;
        }

        private void OnCollisionEnter(Collision collision)
        {
            if (_sensor != null) _sensor.HandleRelayedCollisionEnter(collision);
        }

        private void OnCollisionStay(Collision collision)
        {
            if (_sensor != null) _sensor.HandleRelayedCollisionStay(collision);
        }

        private void OnCollisionExit(Collision collision)
        {
            if (_sensor != null) _sensor.HandleRelayedCollisionExit(collision);
        }
    }
}
