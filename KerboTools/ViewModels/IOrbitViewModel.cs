using ANewDawn.Mission;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KerboTools.ViewModels
{
    interface IOrbitViewModel
    {
        string Body { get; }

        double Altitude { get; }
        double Velocity { get; }

        double Periapsis { get; }
        double Apoapsis { get; }

        Time TimeToApoapsis { get; }
        Time TimeToPeriapsis { get; }
        
        double Inclination { get; }
    }
}
