module ANewDawn.Launch

open KRPC.Client;
open KRPC.Client.Services;
open KRPC.Client.Services.SpaceCenter
open System.Threading
open System

open ANewDawn.Units
open ANewDawn.StreamExtensions
open ANewDawn.Extensions
open ANewDawn.Math
open ANewDawn.Mission
open ANewDawn.Control
open ANewDawn
    
type Profile = {
    /// desired altitude of apoapsis [m]
    TargetApoapsis: float<m>;
    /// compass direction of launch [°]
    LaunchAzimuth: float<deg>;
    /// time to wait before ignition [s]
    Countdown: int;
    /// minimum altitude before starting turn [m]
    MinTurnAlt: float<m>;
    /// minimum speed before starting turn [m/s]
    MinTurnSpeed: float<m/s>;
    /// final altitude of burn
    MaxTurnAlt: float<m>;
    /// parameter describing the turn shap
    TurnExponent: float;
}
    
/// The Kerbin launch profile defines a 45 degree pitch at roughly 10km altitude.
let KerbinProfile: Profile = {
    TargetApoapsis = 80000.<m>;
    LaunchAzimuth = 90.<deg>;
    Countdown = 5;
    MinTurnAlt = 700.<m>;
    MinTurnSpeed = 100.<m/s>;
    MaxTurnAlt = 70000.<m>;
    TurnExponent = 0.4
}
    
// Compute current turn angle as an exponentially shaped curve w.r.t altitude
let turnPitch profile (startAlt: float<m>) (currentAlt: float<m>) =
    90.<deg> - 90.<deg> * ((currentAlt - startAlt) / (profile.MaxTurnAlt - startAlt)) ** profile.TurnExponent;

let launch (mission: Mission) (profile: Profile) =
    let ship = mission.ActiveVessel
    if ship.Situation <> VesselSituation.Landed && ship.Situation <> VesselSituation.PreLaunch
        then invalidArg "vessel" "Not grounded"
            
    printfn "Preparing launch"

    let conn = ship.Connection
    let control = ship.Control
    let flight = ship.Flight(ship.SurfaceReferenceFrame);
        
    control.Throttle <- 0.f

    use altitudeS = mission.Streams.UseStream<m>(fun () -> flight.MeanAltitude)
    use airSpeedS = mission.Streams.UseStream<m/s>(fun () -> flight.TrueAirSpeed)
    use apoapsisS = mission.Streams.UseStream<m>(fun () -> ship.Orbit.ApoapsisAltitude);
        
    // compass direction vector in surface reference frame of vessel, 0 deg is north, 90 deg is east
    let compassDir (angle: float<deg>) =
        let a = angle / degPerRad / 1.<rad>
        { x = 0. (* up *); y = cos a (* north *); z = sin a (* east *) }
            
    let forwardAtPitch (pitch: float<deg>) (azimuth: float<deg>) =
        let pitchRad = pitch / degPerRad / 1.<rad>
        let azimuthRad = azimuth / degPerRad / 1.<rad>
        let upwards = sin pitchRad
        let sidewards = cos pitchRad
        {x = upwards; y = cos azimuthRad * sidewards; z = sin azimuthRad * sidewards }
        
    let upAtPitch (pitch: float<deg>) (azimuth: float<deg>) =
        let pitchRad = pitch / degPerRad / 1.<rad>
        let azimuthRad = azimuth / degPerRad / 1.<rad>
        let upwards = cos pitchRad
        let sidewards = - sin pitchRad
        {x = upwards; y = cos azimuthRad * sidewards; z = sin azimuthRad * sidewards }
            
    printfn "Launching in"
    for count in { 0 .. profile.Countdown - 1 } do
        printfn "  %d s" (profile.Countdown - count)
        Thread.Sleep(1000)

    control.Throttle <- 1.f
            
    // Lift-off
    let stage () =
        printfn "Staging!"
        control.ActivateNextStage() |> ignore
   
    printfn "Ignition"
    stage ()

    AttitudeControl.loop mission ship.SurfaceReferenceFrame <| seq {
        // Go straight up until high enough and fast enough to start turning
        // (Turning to early might result in accidental collisions with launch clamps
        // or result in the vessel spinning out of control)
        while altitudeS.Value < profile.MinTurnAlt || 
               floatU airSpeedS.Value < profile.MinTurnSpeed do

            if Staging.shouldStage ship then
                stage ()
            
            let fw = forwardAtPitch 89.9<deg> profile.LaunchAzimuth
            let up = upAtPitch 89.9<deg> profile.LaunchAzimuth
        
            yield { forward = fw; top = Some(up) }

        printfn "Starting gravity turn"

        let turnStartAlt = altitudeS.Value

        let throttlePid = new PidLoop<m, 1>(0.01</m>, 0.001</(m s)>, 0.<_>, 0., 1.)
        throttlePid.Setpoint <- profile.TargetApoapsis

        /// Perform the gravity turn maneuver by following a pre-defined curve
        while apoapsisS.Value < profile.TargetApoapsis - 10.<m> ||
                altitudeS.Value < profile.MaxTurnAlt do
            if Staging.shouldStage ship then
                stage ()

            let apo = apoapsisS.Value

            control.Throttle <- float32U <| throttlePid.Update(mission.UniversalTime, apo)

            let desiredPitch = turnPitch profile turnStartAlt altitudeS.Value
        
            let fw = forwardAtPitch desiredPitch profile.LaunchAzimuth
            let up = upAtPitch desiredPitch profile.LaunchAzimuth
            yield { forward = fw; top = Some(up) }
    }

    control.Throttle <- 0.f
    
    printfn "Reached space"
