using UnityEngine;

using UnitySensors.DataType.Geometry;
using UnitySensors.Utils.Geometry;

namespace UnitySensors.Sensor.GNSS
{
    public class GeoCoordinateSystem : MonoBehaviour
    {
        [SerializeField]
        private GeoCoordinate _coordinate = new GeoCoordinate(35.71020206575301, 139.81070039691542, 3.0f);

        private Transform _transform;
        private GeoCoordinateConverter _converter;

        public GeoCoordinate coordinate { get => _coordinate; }

        private void Awake()
        {
            _transform = this.transform;
            _converter = new GeoCoordinateConverter(_coordinate);
        }

        /// <summary>
        /// Configure the geodetic origin at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(GeoCoordinate coordinate)
        {
            _coordinate = coordinate;
            _converter = new GeoCoordinateConverter(_coordinate);
        }

        public GeoCoordinate GetCoordinate(Vector3 worldPosition)
        {
            Vector3 localPosition = _transform.InverseTransformPoint(worldPosition);
            return _converter.Convert(new Vector3D(localPosition));
        }

        /// <summary>Local position of <paramref name="worldPosition"/> in this origin's frame.</summary>
        public Vector3 ToLocal(Vector3 worldPosition)
        {
            return _transform.InverseTransformPoint(worldPosition);
        }

        /// <summary>
        /// Geodetic position of an ENU offset from this origin.
        /// </summary>
        /// <remarks>
        /// <see cref="GetCoordinate"/> reads a Unity world position through the
        /// converter's own axis convention, which takes Unity x as easting and z as
        /// northing. A host application whose world frame does not agree with that
        /// -- one publishing ROS ENU, where east is Unity +z and north is Unity -x --
        /// would otherwise get a fix rotated by ninety degrees against its own
        /// ground truth, an error that grows with distance from the origin and is
        /// invisible unless something decodes the fix and compares. Such an
        /// application should state the ENU offset here instead.
        /// </remarks>
        public GeoCoordinate GetCoordinateFromEnu(double east, double north, double up)
        {
            return _converter.Convert(new Vector3D((float)east, (float)up, (float)north));
        }
    }
}