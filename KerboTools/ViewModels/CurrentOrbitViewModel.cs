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
    class CurrentOrbitViewModel : IOrbitViewModel, IDisposable, INotifyPropertyChanged
    {
        private readonly ANewDawn.RefCountedStream<double, double> altitude;
        private readonly ANewDawn.RefCountedStream<double, double> velocity;
        private readonly ANewDawn.RefCountedStream<double, double> periapsis;
        private readonly ANewDawn.RefCountedStream<double, double> apoapsis;
        private readonly ANewDawn.RefCountedStream<double, double> timeToApoapsis;
        private readonly ANewDawn.RefCountedStream<double, double> timeToPeriapsis;
        private readonly ANewDawn.RefCountedStream<double, double> inclination;

        public CurrentOrbitViewModel(Mission mission, Orbit orbit)
        {
            var bodyRadius = orbit.Body.EquatorialRadius;
            this.Body = orbit.Body.Name;

            this.altitude = mission.Streams.UseStream<double>(() => orbit.Radius);
            this.altitude.UnderlyingStream.AddCallback(newVal => { this.Altitude = newVal - bodyRadius; NotifyPropertyChanged("Altitude"); });
            this.altitude.UnderlyingStream.Start(true);

            this.velocity = mission.Streams.UseStream<double>(() => orbit.OrbitalSpeed);
            this.velocity.UnderlyingStream.AddCallback(newVal => { this.Velocity = newVal; NotifyPropertyChanged("Velocity"); });
            this.velocity.UnderlyingStream.Start(true);

            this.periapsis = mission.Streams.UseStream<double>(() => orbit.PeriapsisAltitude);
            this.periapsis.UnderlyingStream.AddCallback(newVal => { this.Periapsis = newVal; NotifyPropertyChanged("Periapsis"); });
            this.periapsis.UnderlyingStream.Start(true);

            this.apoapsis = mission.Streams.UseStream<double>(() => orbit.ApoapsisAltitude);
            this.apoapsis.UnderlyingStream.AddCallback(newVal => { this.Apoapsis = newVal; NotifyPropertyChanged("Apoapsis"); });
            this.apoapsis.UnderlyingStream.Start(true);

            this.timeToApoapsis = mission.Streams.UseStream<double>(() => orbit.TimeToApoapsis);
            this.timeToApoapsis.UnderlyingStream.AddCallback(newVal => { this.TimeToApoapsis = new Time(newVal); NotifyPropertyChanged("TimeToApoapsis"); });
            this.timeToApoapsis.UnderlyingStream.Start(true);

            this.timeToPeriapsis = mission.Streams.UseStream<double>(() => orbit.TimeToPeriapsis);
            this.timeToPeriapsis.UnderlyingStream.AddCallback(newVal => { this.TimeToPeriapsis = new Time(newVal); NotifyPropertyChanged("TimeToPeriapsis"); });
            this.timeToPeriapsis.UnderlyingStream.Start(true);

            this.inclination = mission.Streams.UseStream<double>(() => orbit.Inclination);
            this.inclination.UnderlyingStream.AddCallback(newVal => { this.Inclination = newVal; NotifyPropertyChanged("Inclination"); });
            this.inclination.UnderlyingStream.Start(true);

            this.altitude.UnderlyingStream.Rate = 2;
            this.velocity.UnderlyingStream.Rate = 2;
            this.periapsis.UnderlyingStream.Rate = 2;
            this.apoapsis.UnderlyingStream.Rate = 2;
            this.timeToApoapsis.UnderlyingStream.Rate = 2;
            this.timeToPeriapsis.UnderlyingStream.Rate = 2;
            this.inclination.UnderlyingStream.Rate = 2;
        }

        public double Altitude { get; private set; }

        public double Velocity { get; private set; }

        public double Periapsis { get; private set; }

        public double Apoapsis { get; private set; }

        public Time TimeToApoapsis { get; private set; }

        public Time TimeToPeriapsis { get; private set; }

        public double Inclination { get; private set; }

        public string Body { get; private set; }

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
