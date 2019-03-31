using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KerboTools.ViewModels
{
    interface IVesselStatusViewModel
    {
        

        bool Radiators { get; set; }
        bool ReactionWheels { get; set; }
        bool ResourceHarvesters { get; set; }
        bool ResourceHarvestersActive { get; set; }
        bool SAS { get; set; }
        bool RCS { get; set; }
        bool SolarPanels { get; set; }
        bool Parachutes { get; set; }
        bool Abort { get; set; }
        bool Antennas { get; set; }
        bool Wheels { get; set; }
        bool CargoBays { get; set; }
        bool Brakes { get; set; }
        bool Gear { get; set; }
        bool Intakes { get; set; }
        bool Legs { get; set; }
        bool Lights { get; set; }

        int Stage { get; }

        IActionGroupStatus ActionGroups { get; }

        IResourceStatus TotalResources { get; }
        IResourceStatus StageResources { get; }
    }

    interface IResourceStatus
    {
        double CurrentLiquidFuel { get; }
        double CurrentOxidizer { get; }
        double CurrentSolidFuel { get; }
        double CurrentXenon { get; }
        double CurrentMonoPropellant { get; }
        double CurrentElectricCharge { get; }

        double MaxLiquidFuel { get; }
        double MaxOxidizer { get; }
        double MaxSolidFuel { get; }
        double MaxXenon { get; }
        double MaxMonoPropellant { get; }
        double MaxElectricCharge { get; }
    }

    interface IActionGroupStatus
    {
        bool this[int index] { get; set; }
    }
}
