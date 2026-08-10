using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor;
using UnitySensors.Interface.Sensor;
using UnitySensors.Interface.Std;

namespace UnitySensors.Sensor.Contact
{
    public class ContactSensor : UnitySensor, IContactDataInterface, IBoolStateInterface, IWrenchInterface
    {
        [SerializeField, ReadOnly]
        private bool _isContact;
        [SerializeField, ReadOnly]
        private List<ContactData> _contacts = new List<ContactData>();

        private Dictionary<Collider, ContactData> _activeContacts = new Dictionary<Collider, ContactData>();
        private List<Collider> _removeBuffer = new List<Collider>();
        private Vector3 _totalForce;
        private Vector3 _totalTorque;

        public bool isContact { get => _isContact; }
        public Vector3 totalForce { get => _totalForce; }
        public Vector3 totalTorque { get => _totalTorque; }
        public Vector3 localTotalForce { get => transform.InverseTransformDirection(_totalForce); }
        public Vector3 localTotalTorque { get => transform.InverseTransformDirection(_totalTorque); }
        public IReadOnlyList<ContactData> contacts { get => _contacts; }

        // Generic serializer sources: bumper state and the net contact wrench
        // in the sensor's local frame.
        bool IBoolStateInterface.state { get => _isContact; }
        Vector3 IWrenchInterface.force { get => localTotalForce; }
        Vector3 IWrenchInterface.torque { get => localTotalTorque; }

        protected override void Init()
        {
            // OnCollision* messages are delivered to the GameObject that holds
            // the touched collider, which for articulated robots is usually a
            // collider-only child of the body link. Put a relay on each of those
            // children so their events reach this sensor. Children that have
            // their own Rigidbody/ArticulationBody belong to another body, so
            // recursion stops there; use RegisterCollider() to cover colliders
            // outside this default scan.
            AttachRelays(transform);
        }

        /// <summary>
        /// Relay OnCollision* events from the given collider's GameObject to
        /// this sensor. Use this when the collider sits outside the default
        /// scan, e.g. on a child GameObject that carries its own body.
        /// </summary>
        public void RegisterCollider(Collider target)
        {
            if (target == null || target.gameObject == gameObject) return;
            if (target.GetComponent<ContactEventRelay>() == null)
            {
                target.gameObject.AddComponent<ContactEventRelay>().Initialize(this);
            }
        }

        private void AttachRelays(Transform target)
        {
            if (target != transform &&
                (target.GetComponent<Rigidbody>() != null || target.GetComponent<ArticulationBody>() != null))
            {
                return;
            }

            if (target != transform &&
                target.GetComponent<Collider>() != null &&
                target.GetComponent<ContactEventRelay>() == null)
            {
                target.gameObject.AddComponent<ContactEventRelay>().Initialize(this);
            }

            for (int i = 0; i < target.childCount; i++)
            {
                AttachRelays(target.GetChild(i));
            }
        }

        // Colliders on the same GameObject as the body report here directly;
        // collider-only children report through ContactEventRelay.
        private void OnCollisionEnter(Collision collision)
        {
            UpdateContact(collision);
        }

        private void OnCollisionStay(Collision collision)
        {
            UpdateContact(collision);
        }

        private void OnCollisionExit(Collision collision)
        {
            _activeContacts.Remove(collision.collider);
        }

        internal void HandleRelayedCollisionEnter(Collision collision)
        {
            UpdateContact(collision);
        }

        internal void HandleRelayedCollisionStay(Collision collision)
        {
            UpdateContact(collision);
        }

        internal void HandleRelayedCollisionExit(Collision collision)
        {
            _activeContacts.Remove(collision.collider);
        }

        private void UpdateContact(Collision collision)
        {
            if (collision.contactCount == 0) return;

            ContactPoint contactPoint = collision.GetContact(0);
            ContactData contact = new ContactData();
            contact.colliderName = collision.collider.name;
            contact.position = contactPoint.point;
            contact.normal = contactPoint.normal;
            contact.force = collision.impulse / Time.fixedDeltaTime;
            _activeContacts[collision.collider] = contact;
        }

        protected override IEnumerator UpdateSensor()
        {
            // A collider destroyed while touching never sends OnCollisionExit,
            // so drop destroyed keys before aggregating.
            _removeBuffer.Clear();
            foreach (Collider collider in _activeContacts.Keys)
            {
                if (collider == null) _removeBuffer.Add(collider);
            }
            foreach (Collider collider in _removeBuffer)
            {
                _activeContacts.Remove(collider);
            }

            _contacts.Clear();
            _totalForce = Vector3.zero;
            _totalTorque = Vector3.zero;

            Vector3 origin = transform.position;
            foreach (ContactData contact in _activeContacts.Values)
            {
                _contacts.Add(contact);
                _totalForce += contact.force;
                _totalTorque += Vector3.Cross(contact.position - origin, contact.force);
            }
            _isContact = _contacts.Count > 0;

            yield return null;
        }

        protected override void OnSensorDestroy()
        {
        }
    }
}
