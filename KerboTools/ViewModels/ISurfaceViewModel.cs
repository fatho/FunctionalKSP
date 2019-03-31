using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KerboTools.ViewModels
{
    interface ISurfaceViewModel
    {
        double Latitude { get; }
        double Longitude { get; }

        double RadarAltitude { get; }
        double Velocity { get; }

        double AtmosphericPressure { get; }
        double DynamicPressure { get; }

        string Biome { get; }
    }
}
