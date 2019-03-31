using ANewDawn.Mission;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace KerboTools.ViewModels
{
    class VesselStatusViewModel : IVesselStatusViewModel, IDisposable, INotifyPropertyChanged
    {
        private readonly Mission mission;
        private readonly Control control;
        private readonly Vessel vessel;
        private readonly ActionGroupStatusViewModel actionGroups;
        private readonly ResourceStatusViewModel totalResources;
        private ResourceStatusViewModel stageResources;

        private bool sas;
        private bool rcs;
        private bool solarpanels;
        private bool parachutes;
        private bool abort;
        private bool antennas;
        private bool cargobays;
        private bool brakes;
        private bool gear;
        private bool lights;
        private int stage;

        private readonly ANewDawn.RefCountedStream<bool, bool> sasStream;
        private readonly ANewDawn.RefCountedStream<bool, bool> rcsStream;
        private readonly ANewDawn.RefCountedStream<bool, bool> solarpanelsStream;
        private readonly ANewDawn.RefCountedStream<bool, bool> parachutesStream;
        private readonly ANewDawn.RefCountedStream<bool, bool> abortStream;
        private readonly ANewDawn.RefCountedStream<bool, bool> antennasStream;
        private readonly ANewDawn.RefCountedStream<bool, bool> cargobaysStream;
        private readonly ANewDawn.RefCountedStream<bool, bool> brakesStream;
        private readonly ANewDawn.RefCountedStream<bool, bool> gearStream;
        private readonly ANewDawn.RefCountedStream<bool, bool> lightsStream;
        private readonly ANewDawn.RefCountedStream<int, int> stageStream;

        public double LiquidFuel => 0;
        public double Oxidizer => 0;
        public double MonoPropellant => 0;
        public double ElectricCharge => 0;

        public bool Radiators { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }
        public bool ReactionWheels { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }
        public bool ResourceHarvesters { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }
        public bool ResourceHarvestersActive { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }
        public bool Wheels { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }
        public bool Legs { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }
        public bool Intakes { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }

        public bool SAS
        {
            get { return sas; }
            set
            {
                sas = value;
                control.SAS = value;
            }
        }
        public bool RCS
        {
            get { return rcs; }
            set
            {
                rcs = value;
                control.RCS = value;
            }
        }
        public bool SolarPanels
        {
            get { return solarpanels; }
            set
            {
                solarpanels = value;
                control.SolarPanels = value;
            }
        }
        public bool Parachutes
        {
            get { return parachutes; }
            set
            {
                parachutes = value;
                control.Parachutes = value;
            }
        }
        public bool Abort
        {
            get { return abort; }
            set
            {
                abort = value;
                control.Abort = value;
            }
        }
        public bool Antennas
        {
            get { return antennas; }
            set
            {
                antennas = value;
                control.Antennas = value;
            }
        }
        public bool CargoBays
        {
            get { return cargobays; }
            set
            {
                cargobays = value;
                control.CargoBays = value;
            }
        }
        public bool Brakes
        {
            get { return brakes; }
            set
            {
                brakes = value;
                control.Brakes = value;
            }
        }
        public bool Gear
        {
            get { return gear; }
            set
            {
                gear = value;
                control.Gear = value;
            }
        }
        public bool Lights
        {
            get { return lights; }
            set
            {
                lights = value;
                control.Lights = value;
            }
        }

        public int Stage
        {
            get { return stage; }
        }

        public IActionGroupStatus ActionGroups => actionGroups;

        public IResourceStatus TotalResources => totalResources;

        public IResourceStatus StageResources => stageResources;

        public VesselStatusViewModel(Mission mission, Vessel vessel)
        {
            this.mission = mission;
            this.control = vessel.Control;
            this.vessel = vessel;
            this.actionGroups = new ActionGroupStatusViewModel(mission, vessel);
            this.totalResources = new ResourceStatusViewModel(mission, vessel.Resources);

            this.sasStream = mission.Streams.UseStream<bool>(() => control.SAS);
            this.sasStream.UnderlyingStream.AddCallback(newVal => { this.sas = newVal; NotifyPropertyChanged("SAS"); });
            this.sasStream.UnderlyingStream.Rate = 1;
            this.sasStream.UnderlyingStream.Start(false);

            this.rcsStream = mission.Streams.UseStream<bool>(() => control.RCS);
            this.rcsStream.UnderlyingStream.AddCallback(newVal => { this.rcs = newVal; NotifyPropertyChanged("RCS"); });
            this.rcsStream.UnderlyingStream.Rate = 1;
            this.rcsStream.UnderlyingStream.Start(false);

            this.solarpanelsStream = mission.Streams.UseStream<bool>(() => control.SolarPanels);
            this.solarpanelsStream.UnderlyingStream.AddCallback(newVal => { this.solarpanels = newVal; NotifyPropertyChanged("SolarPanels"); });
            this.solarpanelsStream.UnderlyingStream.Rate = 1;
            this.solarpanelsStream.UnderlyingStream.Start(false);

            this.parachutesStream = mission.Streams.UseStream<bool>(() => control.Parachutes);
            this.parachutesStream.UnderlyingStream.AddCallback(newVal => { this.parachutes = newVal; NotifyPropertyChanged("Parachutes"); });
            this.parachutesStream.UnderlyingStream.Rate = 1;
            this.parachutesStream.UnderlyingStream.Start(false);

            this.abortStream = mission.Streams.UseStream<bool>(() => control.Abort);
            this.abortStream.UnderlyingStream.AddCallback(newVal => { this.abort = newVal; NotifyPropertyChanged("Abort"); });
            this.abortStream.UnderlyingStream.Rate = 1;
            this.abortStream.UnderlyingStream.Start(false);

            this.antennasStream = mission.Streams.UseStream<bool>(() => control.Antennas);
            this.antennasStream.UnderlyingStream.AddCallback(newVal => { this.antennas = newVal; NotifyPropertyChanged("Antennas"); });
            this.antennasStream.UnderlyingStream.Rate = 1;
            this.antennasStream.UnderlyingStream.Start(false);

            this.cargobaysStream = mission.Streams.UseStream<bool>(() => control.CargoBays);
            this.cargobaysStream.UnderlyingStream.AddCallback(newVal => { this.cargobays = newVal; NotifyPropertyChanged("CargoBays"); });
            this.cargobaysStream.UnderlyingStream.Rate = 1;
            this.cargobaysStream.UnderlyingStream.Start(false);

            this.brakesStream = mission.Streams.UseStream<bool>(() => control.Brakes);
            this.brakesStream.UnderlyingStream.AddCallback(newVal => { this.brakes = newVal; NotifyPropertyChanged("Brakes"); });
            this.brakesStream.UnderlyingStream.Rate = 1;
            this.brakesStream.UnderlyingStream.Start(false);

            this.gearStream = mission.Streams.UseStream<bool>(() => control.Gear);
            this.gearStream.UnderlyingStream.AddCallback(newVal => { this.gear = newVal; NotifyPropertyChanged("Gear"); });
            this.gearStream.UnderlyingStream.Rate = 1;
            this.gearStream.UnderlyingStream.Start(false);

            this.lightsStream = mission.Streams.UseStream<bool>(() => control.Lights);
            this.lightsStream.UnderlyingStream.AddCallback(newVal => { this.lights = newVal; NotifyPropertyChanged("Lights"); });
            this.lightsStream.UnderlyingStream.Rate = 1;
            this.lightsStream.UnderlyingStream.Start(false);

            this.stageStream = mission.Streams.UseStream<int>(() => control.CurrentStage);
            this.stageStream.UnderlyingStream.AddCallback(newVal => { this.stage = newVal; NotifyPropertyChanged("Stage"); this.RecreateStageResources(); });
            this.stageStream.UnderlyingStream.Rate = 1;
            this.stageStream.UnderlyingStream.Start(false);
        }

        private void RecreateStageResources()
        {
            Application.Current.Dispatcher.BeginInvoke(System.Windows.Threading.DispatcherPriority.Background, new Action(() =>
            {
                if (stageResources != null)
                {
                    stageResources.Dispose();
                }
                for (var i = -1; i <= stage; i++)
                {
                    Console.WriteLine(i);
                    var res = vessel.ResourcesInDecoupleStage(i);
                    foreach(var n in res.Names)
                    {
                        Console.WriteLine(n);
                        Console.WriteLine(res.Amount(n));
                    }
                }
                var resources = vessel.ResourcesInDecoupleStage(stage - 1, false);
                stageResources = new ResourceStatusViewModel(mission, resources);
                NotifyPropertyChanged("StageResources");
            }));
        }

        public event PropertyChangedEventHandler PropertyChanged;

        private void NotifyPropertyChanged(String propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public void Dispose()
        {
            DisposeUtil.DisposeFields(this);
        }
    }

    class ActionGroupStatusViewModel : IActionGroupStatus, IDisposable, INotifyPropertyChanged
    {
        const int NUM_AGS = 10;

        private readonly bool[] status;
        private readonly ANewDawn.RefCountedStream<bool, bool>[] ags;

        private readonly Control control;

        public ActionGroupStatusViewModel(Mission mission, Vessel vessel)
        {
            this.status = new bool[NUM_AGS];
            this.ags = new ANewDawn.RefCountedStream<bool, bool>[NUM_AGS];
            this.control = vessel.Control;

            var control = vessel.Control;

            for (int i = 0; i < NUM_AGS; i++)
            {
                uint ag = (uint)i;
                this.ags[i] = mission.Streams.UseStream<bool>(() => control.GetActionGroup(ag));
                this.ags[i].UnderlyingStream.AddCallback(newStatus =>
                {
                    this.status[ag] = newStatus;
                    NotifyPropertyChanged("Item[]");
                });
                this.ags[i].UnderlyingStream.Rate = 1;
                this.ags[i].UnderlyingStream.Start(false);
            }
        }

        public bool this[int index]
        {
            get { return status[index]; }
            set
            {
                status[index] = value;
                control.SetActionGroup((uint)index, value);
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        private void NotifyPropertyChanged(String propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public void Dispose()
        {
            DisposeUtil.DisposeFields(this);
        }
    }


    class ResourceStatusViewModel : IResourceStatus, IDisposable, INotifyPropertyChanged
    {
        private Resources resources;
        private readonly string[] NAMES = { "LiquidFuel", "Oxidizer", "MonoPropellant", "ElectricCharge", "SolidFuel", "Xenon" };

        private readonly ANewDawn.RefCountedStream<float, float>[] currentStream;
        private readonly ANewDawn.RefCountedStream<float, float>[] maxStream;
        private readonly float[] current;
        private readonly float[] max;

        public double CurrentLiquidFuel => this.current[0];

        public double CurrentOxidizer => this.current[1];

        public double CurrentMonoPropellant => this.current[2];

        public double CurrentElectricCharge => this.current[3];

        public double MaxLiquidFuel => this.max[0];

        public double MaxOxidizer => this.max[1];

        public double MaxMonoPropellant => this.max[2];

        public double MaxElectricCharge => this.max[3];

        public double CurrentSolidFuel => this.current[4];

        public double CurrentXenon => this.current[5];

        public double MaxSolidFuel => this.max[4];

        public double MaxXenon => this.max[5];

        public ResourceStatusViewModel(Mission mission, Resources resources)
        {
            this.resources = resources;

            this.currentStream = new ANewDawn.RefCountedStream<float, float>[NAMES.Length];
            this.maxStream = new ANewDawn.RefCountedStream<float, float>[NAMES.Length];
            this.current = new float[NAMES.Length];
            this.max = new float[NAMES.Length];

            for (int i = 0; i < NAMES.Length; i++)
            {
                int index = i;
                string name = NAMES[i];
                this.currentStream[i] = mission.Streams.UseStream<float>(() => resources.Amount(name));
                this.currentStream[i].UnderlyingStream.AddCallback(newVal =>
                {
                    this.current[index] = newVal;
                    NotifyPropertyChanged("Current" + NAMES[index]);
                });
                this.currentStream[i].UnderlyingStream.Rate = 2;
                this.currentStream[i].UnderlyingStream.Start(false);

                this.maxStream[i] = mission.Streams.UseStream<float>(() => resources.Max(name));
                this.maxStream[i].UnderlyingStream.AddCallback(newVal =>
                {
                    this.max[index] = newVal;
                    NotifyPropertyChanged("Max" + NAMES[index]);
                });
                this.maxStream[i].UnderlyingStream.Rate = 2;
                this.maxStream[i].UnderlyingStream.Start(false);
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        private void NotifyPropertyChanged(String propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public void Dispose()
        {
            DisposeUtil.DisposeFields(this);
        }
    }
}