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
open ANewDawn.Math
open ANewDawn.Math
open ANewDawn.Math
open ANewDawn.Math
    
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
    /// Maximum dynamic pressure
    MaxQ: float<Pa>;
}
    
/// The Kerbin launch profile defines a 45 degree pitch at roughly 10km altitude.
let KerbinProfile: Profile = {
    TargetApoapsis = 80000.<m>;
    LaunchAzimuth = 90.<deg>;
    Countdown = 5;
    MinTurnAlt = 700.<m>;
    MinTurnSpeed = 100.<m/s>;
    MaxTurnAlt = 70000.<m>;
    TurnExponent = 0.4;
    MaxQ = 1_000_000_000.<Pa>;
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
    use dynamicPressureS = mission.Streams.UseStream<Pa>(fun () -> flight.DynamicPressure)
        
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

        let apoThrottlePid = new PidLoop<m, 1>(0.01</m>, 0.001</(m s)>, 0.<_>, 0., 1.)
        apoThrottlePid.Setpoint <- profile.TargetApoapsis
        
        let maxQThrottlePid = new PidLoop<Pa, 1>(0.01</Pa>, 0.001</(Pa s)>, 0.<_>, 0., 1.)
        maxQThrottlePid.Setpoint <- profile.MaxQ

        /// Perform the gravity turn maneuver by following a pre-defined curve
        while (apoapsisS.Value < profile.TargetApoapsis - 10.<m> ||
                altitudeS.Value < profile.MaxTurnAlt) do
            if Staging.shouldStage ship then
                stage ()

            let apo = apoapsisS.Value

            let apoThrottle = float32U <| apoThrottlePid.Update(mission.UniversalTime, apo)
            let qThrottle = float32U <| maxQThrottlePid.Update(mission.UniversalTime, floatU dynamicPressureS.Value)

            printfn "AT: %.2f   QT: %.2f" apoThrottle qThrottle

            control.Throttle <- min apoThrottle qThrottle

            let desiredPitch = turnPitch profile turnStartAlt altitudeS.Value
        
            let fw = forwardAtPitch desiredPitch profile.LaunchAzimuth
            let up = upAtPitch desiredPitch profile.LaunchAzimuth
            yield { forward = fw; top = Some(up) }
    }

    control.Throttle <- 0.f
    
    printfn "Coasting to space"


let launchToInclination (mission: Mission) (profile: Profile) (targetOrbit: Orbit) =
    let ship = mission.ActiveVessel
    let body = ship.Orbit.Body
    assert (targetOrbit.Body = body)

    let reference = body.NonRotatingReferenceFrame
    let flight = ship.Flight(reference)
    let shipLatitude = flight.Latitude.As<deg>() / degPerRad
    let shipLongitude = flight.Longitude.As<deg>() / degPerRad

    let targetInclination = 
        let inclination = targetOrbit.Inclination.As<rad>()
        if inclination < shipLatitude then
            printfn "Cannot launch to inclination below latitude %.2f, adjusting inclination." (shipLatitude * degPerRad)
            shipLatitude
        else
            inclination

    // compute inertial (w/o accounting for body's rotation) launch azimuth
    let inertialAzimuth = 1.<rad> * asin (Util.cosRad targetInclination / Util.cosRad shipLatitude)

    // desired orbital velocity
    let bodyRadius = floatU (body.EquatorialRadius.As<m>())
    let orbitRadius = bodyRadius + profile.TargetApoapsis
    let vOrbit = VisViva.orbitalVelocity (floatU <| body.GravitationalParameter.As<m^3/s^2>()) orbitRadius orbitRadius

    // ship movement due to body rotation (assuming we are standing still)
    let vEqRot = floatU (body.RotationalSpeed.As<rad/s>()) / Util.pi.As<rad>() * bodyRadius

    // compute rotation-adjusted velocity vector
    let vXRot = vOrbit * Util.sinRad inertialAzimuth - vEqRot * Util.cosRad shipLatitude
    let vYRot = vOrbit * Util.cosRad inertialAzimuth

    // find launch angle of rotated vector
    let rotAzimuth = 90.<deg> - 1.<rad> * degPerRad * atan2 vYRot vXRot
    let modifiedProfile = { profile with LaunchAzimuth = rotAzimuth }
    printfn "Launch Azimuth: %.1f" rotAzimuth

    // compute time when launch site intersects target orbit
    // TODO: only works for equatorial launchsites, I think
    let mutable targetLongitude = targetOrbit.LongitudeOfAscendingNode.As<rad>()
    let shipOrbitalLongitude = shipLongitude + body.RotationAngle.As<rad>()

    while targetLongitude < shipOrbitalLongitude do
        targetLongitude <- targetLongitude + 360.<deg> / degPerRad
    
    let launchEta = (targetLongitude - shipOrbitalLongitude) / (floatU body.RotationalSpeed).As<rad/s>()
    printfn "Launch ETA: %.0f s" launchEta
    
    mission.WarpTo(mission.UniversalTime + launchEta)

    launch mission modifiedProfile