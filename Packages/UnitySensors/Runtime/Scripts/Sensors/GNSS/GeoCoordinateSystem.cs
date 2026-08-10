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
    }
}