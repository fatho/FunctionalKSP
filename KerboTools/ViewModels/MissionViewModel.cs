using ANewDawn.Mission;
using KRPC.Client.Services.SpaceCenter;
using KRPC.Client.Services.KRPC;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace KerboTools.ViewModels
{
    class MissionViewModel : INotifyPropertyChanged, IDisposable, IMissionViewModel
    {
        public event PropertyChangedEventHandler PropertyChanged;

        private readonly Mission mission;
        private readonly KRPC.Client.Services.SpaceCenter.Service spaceCenter;
        private readonly KRPC.Client.Services.KRPC.Service krpc;
        private readonly ANewDawn.RefCountedStream<double, double> universalTime;
        private ANewDawn.RefCountedStream<Vessel, Vessel> activeVessel;
        private readonly ANewDawn.RefCountedStream<GameScene, GameScene> scene;
        private VesselViewModel vesselView;

        public MissionViewModel(Mission mission)
        {
            this.mission = mission;
            var ksc = mission.SpaceCenter;
            this.krpc = mission.Connection.KRPC();
            this.spaceCenter = ksc;
            this.universalTime = mission.Streams.UseStream<double>(() => ksc.UT);
            this.universalTime.UnderlyingStream.AddCallback(this.Tick);
            this.universalTime.UnderlyingStream.Rate = 1;
            this.universalTime.UnderlyingStream.Start(false);

            this.scene = this.mission.Streams.UseStream<GameScene>(() => krpc.CurrentGameScene);
            this.scene.UnderlyingStream.AddCallback(this.SceneChanged);
            this.scene.UnderlyingStream.Start(false);
            this.scene.UnderlyingStream.Rate = 2;
        }

        private void SceneChanged(GameScene scene)
        {
            Application.Current.Dispatcher.BeginInvoke(System.Windows.Threading.DispatcherPriority.Background, new Action(() =>
            {
                if (scene == GameScene.Flight)
                {
                    this.activeVessel = mission.Streams.UseStream<Vessel>(() => spaceCenter.ActiveVessel);
                    this.activeVessel.UnderlyingStream.AddCallback(this.VesselSwitched);
                    this.activeVessel.UnderlyingStream.Start(false);
                }
                else if (this.activeVessel != null)
                {
                    ((IDisposable)this.activeVessel).Dispose();
                    this.activeVessel = null;
                    this.VesselSwitched(null);
                }
            }));
        }

        private void VesselSwitched(Vessel vessel)
        {
            Application.Current.Dispatcher.BeginInvoke(System.Windows.Threading.DispatcherPriority.Background, new Action(() =>
            {
                if (this.vesselView != null)
                {
                    this.vesselView.Dispose();
                    this.vesselView = null;
                }
                if (vessel != null)
                {
                    this.vesselView = new VesselViewModel(mission, vessel);
                }
                else
                {
                    this.vesselView = null;
                }
                NotifyPropertyChanged("ActiveVessel");
            }));
        }

        private void Tick(double newTime)
        {
            this.UniversalTime = new Time(newTime);
            NotifyPropertyChanged("UniversalTime");
        }

        private void NotifyPropertyChanged(String propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public Time UniversalTime { get; private set; }

        public IVesselViewModel ActiveVessel => this.vesselView;

        public void Dispose()
        {
            DisposeUtil.DisposeFields(this);
        }
    }
}
