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
    /// Convert a velocity in meters per second to a human readable string.
    /// </summary>
    class VelocityConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if(value is double velocity)
            {
                if (Math.Abs(velocity) < 20)
                {
                    return string.Format("{0:0.0}m/s", velocity);
                }
                else
                {
                    return string.Format("{0:0}m/s", velocity);
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
