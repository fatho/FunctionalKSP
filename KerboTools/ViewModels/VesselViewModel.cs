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
    class VesselViewModel : IVesselViewModel, IDisposable, INotifyPropertyChanged
    {
        private readonly Mission mission;
        private readonly ANewDawn.RefCountedStream<double, double> missionTime;
        private readonly ANewDawn.RefCountedStream<Orbit, Orbit> orbit;

        private CurrentOrbitViewModel currentOrbit;
        private SurfaceViewModel surface;
        private VesselStatusViewModel status;

        public VesselViewModel(Mission mission, Vessel vessel)
        {
            this.mission = mission;

            this.missionTime = mission.Streams.UseStream<double>(() => vessel.MET);
            this.missionTime.UnderlyingStream.AddCallback(this.Tick);
            this.missionTime.UnderlyingStream.Start(false);
            this.missionTime.UnderlyingStream.Rate = 2;

            this.orbit = mission.Streams.UseStream<Orbit>(() => vessel.Orbit);
            this.orbit.UnderlyingStream.AddCallback(this.NewOrbit);
            this.orbit.UnderlyingStream.Start(false);
            this.orbit.UnderlyingStream.Rate = 2;

            this.Name = vessel.Name;

            this.surface = new SurfaceViewModel(mission, vessel);
            this.status = new VesselStatusViewModel(mission, vessel);
        }

        private void NewOrbit(Orbit orbit)
        {
            Application.Current.Dispatcher.BeginInvoke(System.Windows.Threading.DispatcherPriority.Background, new Action(() =>
            {
                if (currentOrbit != null)
                {
                    currentOrbit.Dispose();
                }
                if(orbit != null)
                {
                    currentOrbit = new CurrentOrbitViewModel(mission, orbit);
                }
                else
                {
                    currentOrbit = null;
                }
                NotifyPropertyChanged("Orbit");
            }));
        }

        private void Tick(double met)
        {
            this.MissionTime = new Time(met);
            NotifyPropertyChanged("MissionTime");
        }

        public string Name { get; private set; }

        public Time MissionTime { get; private set; }

        public IOrbitViewModel Orbit => currentOrbit;

        public ISurfaceViewModel Surface => surface;

        public IVesselStatusViewModel Status => status;

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
