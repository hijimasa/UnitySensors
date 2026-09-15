using UnityEngine;
using RosMessageTypes.Sensor;

using UnitySensors.Attribute;
using UnitySensors.DataType.Sensor;
using UnitySensors.Interface.Geometry;
using UnitySensors.Interface.Sensor;
using UnitySensors.ROS.Serializer.Std;

namespace UnitySensors.ROS.Serializer.Sensor
{
    [System.Serializable]
    public class NavSatFixMsgSerializer : RosMsgSerializer<NavSatFixMsg>
    {
        private enum Status
        {
            NO_FIX,
            FIX,
            SBAS_FIX,
            GBAS_FIX
        }

        private enum Service
        {
            GPS,
            GLONASS,
            COMPASS,
            GALILEO
        }

        [SerializeField, Interface(typeof(IGeoCoordinateInterface))]
        private Object _source;
        /// <summary>
        /// Optional receiver solution. With it, the status and the covariance follow
        /// the grade the receiver actually reached instead of the fixed values below.
        /// </summary>
        [SerializeField, Interface(typeof(IGnssSolutionInterface))]
        private Object _solutionSource;
        [SerializeField]
        private HeaderSerializer _header;
        /// <summary>Vertical sigma as a multiple of the horizontal one.</summary>
        [SerializeField]
        private float _verticalSigmaRatio = 2.0f;

        [SerializeField]
        private Status _status = Status.FIX;
        [SerializeField]
        private Service _service = Service.GPS;

        private IGeoCoordinateInterface _sourceInterface;
        private IGnssSolutionInterface _solutionInterface;

        /// <summary>
        /// Configure serializer at runtime (avoids Reflection overhead)
        /// </summary>
        public void Configure(IGeoCoordinateInterface source, HeaderSerializer header)
        {
            _source = source as Object;
            _sourceInterface = source;
            _header = header;
        }

        /// <summary>Also report the receiver's solution grade and its covariance.</summary>
        public void ConfigureSolution(IGnssSolutionInterface solution)
        {
            _solutionSource = solution as Object;
            _solutionInterface = solution;
        }

        public override void Init()
        {
            base.Init();
            _header.Init();
            _sourceInterface = _source as IGeoCoordinateInterface;
            _solutionInterface = _solutionSource as IGnssSolutionInterface;

            _msg.status = new NavSatStatusMsg();
            _msg.status.service = (ushort)Mathf.Pow(2, (int)(_service));
            _msg.position_covariance = new double[9];
        }
        public override NavSatFixMsg Serialize()
        {
            _msg.header = _header.Serialize();
            _msg.latitude = _sourceInterface.coordinate.latitude;
            _msg.longitude = _sourceInterface.coordinate.longitude;
            _msg.altitude = _sourceInterface.coordinate.altitude;

            if (_solutionInterface == null)
            {
                _msg.status.status = (sbyte)((int)(_status) - 1);
                _msg.position_covariance_type = 0;   // COVARIANCE_TYPE_UNKNOWN
                return _msg;
            }

            GnssSolution solution = _solutionInterface.solution;
            // NavSatFix has four status values and none of them separates an RTK
            // fixed solution from an RTK float one -- both are ground-based
            // augmentation. The grade therefore travels in the covariance, which is
            // what consumers such as robot_localization actually read, and in the
            // GnssSolution message alongside for anything that needs the distinction.
            switch (solution.state)
            {
                case RtkState.NoFix:
                    _msg.status.status = (sbyte)Status.NO_FIX - 1;
                    break;
                case RtkState.Single:
                    _msg.status.status = (sbyte)Status.FIX - 1;
                    break;
                default:
                    _msg.status.status = (sbyte)Status.GBAS_FIX - 1;
                    break;
            }

            if (solution.state == RtkState.NoFix)
            {
                _msg.position_covariance_type = 0;   // COVARIANCE_TYPE_UNKNOWN
                return _msg;
            }

            double variance = solution.horizontalSigma * solution.horizontalSigma;
            double vertical = variance * _verticalSigmaRatio * _verticalSigmaRatio;
            for (int i = 0; i < 9; i++) _msg.position_covariance[i] = 0.0;
            _msg.position_covariance[0] = variance;
            _msg.position_covariance[4] = variance;
            _msg.position_covariance[8] = vertical;
            // DIAGONAL_KNOWN, not APPROXIMATED: these come from the grade the model
            // is in, not from a rule of thumb over HDOP.
            _msg.position_covariance_type = 2;
            return _msg;
        }
    }
}