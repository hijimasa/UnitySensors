using System;
using System.Globalization;
using System.Text;

namespace UnitySensors.Sensor.GNSS
{
    /// <summary>
    /// NMEA 0183 sentence formatting.
    /// </summary>
    /// <remarks>
    /// Published as nmea_msgs/Sentence so a stack can be driven by the very NMEA
    /// driver it runs against real hardware, with no serial device in between. It
    /// is also the only place the RTK fix/float distinction survives as a byte
    /// stream, which is what the robot-side gpsd chain reads.
    /// </remarks>
    public static class NmeaProtocol
    {
        /// <summary>GGA fix quality field.</summary>
        public enum FixQuality
        {
            NoFix = 0,
            Standard = 1,
            Differential = 2,
            RtkFix = 4,
            RtkFloat = 5,
        }

        /// <summary>XOR checksum of the payload between '$' and '*', two upper-case hex digits.</summary>
        public static string Checksum(string payload)
        {
            int sum = 0;
            for (int i = 0; i < payload.Length; i++)
            {
                sum ^= payload[i];
            }
            return sum.ToString("X2", CultureInfo.InvariantCulture);
        }

        /// <summary>
        /// $GPGGA with checksum. No trailing CRLF: a sentence on a topic is one
        /// message, and whatever writes it to a wire adds the framing.
        /// </summary>
        /// <param name="altitude">
        /// Ellipsoidal height. It goes in the altitude field with a zero geoid
        /// separation, because a receiving parser reconstructs the ellipsoidal
        /// height as altitude + geoid and would otherwise be handed it twice.
        /// </param>
        public static string Gga(double latitude, double longitude, double altitude, FixQuality quality,
            DateTime utc, int satellites, double hdop)
        {
            var payload = new StringBuilder(96);
            payload.Append("GPGGA,");
            payload.Append(FormatUtc(utc)).Append(',');
            payload.Append(FormatDegreesMinutes(latitude, 2)).Append(',').Append(latitude >= 0.0 ? 'N' : 'S').Append(',');
            payload.Append(FormatDegreesMinutes(longitude, 3)).Append(',').Append(longitude >= 0.0 ? 'E' : 'W').Append(',');
            payload.Append(((int)quality).ToString(CultureInfo.InvariantCulture)).Append(',');
            payload.Append(satellites.ToString("00", CultureInfo.InvariantCulture)).Append(',');
            payload.Append(hdop.ToString("F2", CultureInfo.InvariantCulture)).Append(',');
            payload.Append(altitude.ToString("F3", CultureInfo.InvariantCulture)).Append(",M,");
            payload.Append("0.000,M,,");
            return Wrap(payload.ToString());
        }

        /// <summary>$GPRMC with checksum.</summary>
        public static string Rmc(double latitude, double longitude, FixQuality quality, DateTime utc,
            double speedMetresPerSecond, double courseDegrees)
        {
            char status = (quality == FixQuality.NoFix) ? 'V' : 'A';
            char mode;
            switch (quality)
            {
                case FixQuality.RtkFix:
                    mode = 'R';
                    break;
                case FixQuality.RtkFloat:
                    mode = 'F';
                    break;
                case FixQuality.Differential:
                    mode = 'D';
                    break;
                case FixQuality.NoFix:
                    mode = 'N';
                    break;
                default:
                    mode = 'A';
                    break;
            }

            var payload = new StringBuilder(96);
            payload.Append("GPRMC,");
            payload.Append(FormatUtc(utc)).Append(',').Append(status).Append(',');
            payload.Append(FormatDegreesMinutes(latitude, 2)).Append(',').Append(latitude >= 0.0 ? 'N' : 'S').Append(',');
            payload.Append(FormatDegreesMinutes(longitude, 3)).Append(',').Append(longitude >= 0.0 ? 'E' : 'W').Append(',');
            payload.Append((speedMetresPerSecond / 0.514444).ToString("F2", CultureInfo.InvariantCulture)).Append(',');
            payload.Append(courseDegrees.ToString("F2", CultureInfo.InvariantCulture)).Append(',');
            payload.Append(utc.ToString("ddMMyy", CultureInfo.InvariantCulture)).Append(",,,");
            payload.Append(mode);
            return Wrap(payload.ToString());
        }

        private static string Wrap(string payload)
        {
            return "$" + payload + "*" + Checksum(payload);
        }

        /// <summary>Degrees -> NMEA ddmm.mmmmmm / dddmm.mmmmmm.</summary>
        private static string FormatDegreesMinutes(double degrees, int degreeDigits)
        {
            double absolute = Math.Abs(degrees);
            int whole = (int)absolute;
            double minutes = (absolute - whole) * 60.0;
            return whole.ToString(new string('0', degreeDigits), CultureInfo.InvariantCulture)
                 + minutes.ToString("00.000000", CultureInfo.InvariantCulture);
        }

        private static string FormatUtc(DateTime utc)
        {
            return utc.ToString("HHmmss.fff", CultureInfo.InvariantCulture);
        }
    }
}
