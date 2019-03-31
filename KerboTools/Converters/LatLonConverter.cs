using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;

namespace KerboTools.Converters
{
    /// <summary>
    /// Convert a length in meters to a human readable string.
    /// </summary>
    class LatLonConverter : IValueConverter
    {
        public string PositiveSuffix { get; set; }

        public string NegativeSuffix { get; set; }

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if(value is double coordinate)
            {
                var suffix = coordinate < 0 ? NegativeSuffix : PositiveSuffix;
                var pos = Math.Abs(coordinate);

                var degrees = Math.Truncate(pos);
                var totalMinutes = (pos - degrees) * 60;
                var minutes = Math.Truncate(totalMinutes);
                var seconds = (totalMinutes - minutes) * 60;

                return string.Format("{0,3:0}° {1,2:0}' {2,4:0.0}\" {3}", degrees, minutes, seconds, suffix);
            }
            return "n/a";
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
