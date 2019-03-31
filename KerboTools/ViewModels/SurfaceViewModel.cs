using ANewDawn.Mission;
using KRPC.Client.Services.SpaceCenter;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KerboTools.ViewModels
{
    class SurfaceViewModel : ISurfaceViewModel, IDisposable, INotifyPropertyChanged
    {
        private readonly ANewDawn.RefCountedStream<double, double> latitude;
        private readonly ANewDawn.RefCountedStream<double, double> longitude;
        private readonly ANewDawn.RefCountedStream<double, double> radarAltitude;
        private readonly ANewDawn.RefCountedStream<double, double> velocity;
        private readonly ANewDawn.RefCountedStream<float, float> atmosphericPressure;
        private readonly ANewDawn.RefCountedStream<float, float> dynamicPressure;
        private readonly ANewDawn.RefCountedStream<string, string> biome;

        public SurfaceViewModel(Mission mission, Vessel vessel)
        {
            var flight = vessel.Flight(vessel.Orbit.Body.ReferenceFrame);

            this.latitude = mission.Streams.UseStream<double>(() => flight.Latitude);
            this.latitude.UnderlyingStream.AddCallback(newVal => { this.Latitude = newVal; NotifyPropertyChanged("Latitude"); });
            this.latitude.UnderlyingStream.Start();

            this.longitude = mission.Streams.UseStream<double>(() => flight.Longitude);
            this.longitude.UnderlyingStream.AddCallback(newVal => { this.Longitude = newVal; NotifyPropertyChanged("Longitude"); });
            this.longitude.UnderlyingStream.Start();

            this.radarAltitude = mission.Streams.UseStream<double>(() => flight.SurfaceAltitude);
            this.radarAltitude.UnderlyingStream.AddCallback(newVal => { this.RadarAltitude = newVal; NotifyPropertyChanged("RadarAltitude"); });
            this.radarAltitude.UnderlyingStream.Start();

            this.velocity = mission.Streams.UseStream<double>(() => flight.Speed);
            this.velocity.UnderlyingStream.AddCallback(newVal => { this.Velocity = newVal; NotifyPropertyChanged("Velocity"); });
            this.velocity.UnderlyingStream.Start();

            this.atmosphericPressure = mission.Streams.UseStream<float>(() => flight.StaticPressure);
            this.atmosphericPressure.UnderlyingStream.AddCallback(newVal => { this.AtmosphericPressure = newVal; NotifyPropertyChanged("AtmosphericPressure"); });
            this.atmosphericPressure.UnderlyingStream.Start();

            this.dynamicPressure = mission.Streams.UseStream<float>(() => flight.DynamicPressure);
            this.dynamicPressure.UnderlyingStream.AddCallback(newVal => { this.DynamicPressure = newVal; NotifyPropertyChanged("DynamicPressure"); });
            this.dynamicPressure.UnderlyingStream.Start();

            this.biome = mission.Streams.UseStream<string>(() => vessel.Biome);
            this.biome.UnderlyingStream.AddCallback(newVal => { this.Biome = newVal; NotifyPropertyChanged("Biome"); });
            this.biome.UnderlyingStream.Start();

            this.latitude.UnderlyingStream.Rate = 2;
            this.longitude.UnderlyingStream.Rate = 2;
            this.radarAltitude.UnderlyingStream.Rate = 2;
            this.velocity.UnderlyingStream.Rate = 2;
            this.atmosphericPressure.UnderlyingStream.Rate = 2;
            this.dynamicPressure.UnderlyingStream.Rate = 2;
            this.biome.UnderlyingStream.Rate = 2;
        }

        public double Latitude { get; private set; }

        public double Longitude { get; private set; }

        public double RadarAltitude { get; private set; }

        public double Velocity { get; private set; }

        public double AtmosphericPressure { get; private set; }
    
        public double DynamicPressure { get; private set; }

        public string Biome { get; private set; }
    
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
