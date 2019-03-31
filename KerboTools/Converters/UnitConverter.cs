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
    class UnitConverter : IValueConverter
    {
        private string fmt = "0";
        private int decimals = 0;

        public UnitConverter()
        {
            ConversionFactor = 1;
            Unit = "";
        }

        public int Decimals
        {
            get { return decimals; }
            set
            {
                decimals = value;
                StringBuilder fmt = new StringBuilder("0");
                if (decimals > 0)
                {
                    for (int i = 0; i < decimals; i++)
                    {
                        fmt.Append("0");
                    }
                }
                this.fmt = fmt.ToString();
            }
        }

        public float ConversionFactor { get; set; }

        public string Unit { get; set; }

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if(value is double dbl)
            {
                return dbl.ToString(this.fmt) + Unit;
            }
            else if(value is float flt)
            {
                return flt.ToString(this.fmt) + Unit;
            }
            return "n/a";
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
