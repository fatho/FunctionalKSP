using ANewDawn.Mission;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KerboTools.ViewModels
{
    interface IVesselViewModel
    {
        string Name { get; }
        Time MissionTime { get; }

        IOrbitViewModel Orbit { get; }

        ISurfaceViewModel Surface { get;  }

        IVesselStatusViewModel Status { get; }
    }
}
