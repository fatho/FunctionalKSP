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
    class LengthConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if(value is double meters)
            {
                if (Math.Abs(meters) < 100000)
                {
                    return string.Format("{0:0}m", meters);
                }
                else
                {
                    return string.Format("{0:0.0}km", meters / 1000);
                }
            }
            return "n/a";
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
