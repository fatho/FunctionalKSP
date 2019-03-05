module ANewDawn.Extensions

open ANewDawn.Math
open ANewDawn.Units
open KRPC.Client.Services.SpaceCenter
open KRPC.Client

type RemoteObject with
    member obj.Connection with get(): Connection = downcast obj.connection

type Orbit with
    member orbit.OrbitalElements
        with get(): Kepler.OrbitalElements = {
            semiMajorAxis = orbit.SemiMajorAxis.As<m>();
            eccentricity = orbit.Eccentricity;
            argumentOfPeriapsis = orbit.ArgumentOfPeriapsis.As<rad>();
            longitudeOfAscendingNode = orbit.LongitudeOfAscendingNode.As<rad>();
            inclination = orbit.Inclination.As<rad>();
            meanAnomalyAtEpoch = orbit.MeanAnomalyAtEpoch.As<rad>();
            epoch = orbit.Epoch.As<s>();
        }