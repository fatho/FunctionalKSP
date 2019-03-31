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
    /// Convert an angle in rad to a human readable string using degrees.
    /// </summary>
    class RadConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if(value is double rad)
            {
                return string.Format("{0:0.0} deg", rad / Math.PI * 180);
            }
            return "n/a";
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
