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

/// Determine whether we should activate the next stage
let shouldStage (ship: Vessel) = ship.AvailableThrust < 0.1f

let launch (mission: Mission) (profile: Profile) =
    let ship = mission.ActiveVessel
    if ship.Situation <> VesselSituation.Landed && ship.Situation <> VesselSituation.PreLaunch
        then invalidArg "vessel" "Not grounded"
            
    printfn "Preparing launch"

    let conn = ship.Connection
    let control = ship.Control
    let flight = ship.Flight(ship.SurfaceReferenceFrame);

    let availableTorqueS = mission.Streams.UseStream<N m>(fun () -> ship.AvailableTorque)
    let moiS = mission.Streams.UseStream<kg m^2>(fun () -> ship.MomentOfInertia)
    let orbitalAngularVelocityS = mission.Streams.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.OrbitalReferenceFrame))

    let combineTorques (pos: vec3<N m>, neg: vec3<N m>): vec3<N m> = Vec3.zip (fun a b -> min (abs a) (abs b)) pos neg
    let transformAngVel av = -(Vec3.pack<rad/s> <| mission.SpaceCenter.TransformDirection(Vec3.unpack av, ship.OrbitalReferenceFrame, ship.ReferenceFrame))

    let steering = new AttitudeController(availableTorqueS.Map(combineTorques), moiS, orbitalAngularVelocityS.Map(transformAngVel))

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
        
    let launchAlt = altitudeS.Value
            
    printfn "Launching in"
    for count in { 0 .. profile.Countdown - 1 } do
        printfn "  %d s" (profile.Countdown - count)
        Thread.Sleep(1000)

    control.Throttle <- 1.f
            
    // Lift-off
    let stage () =
        control.ActivateNextStage() |> ignore
   
    printfn "Ignition"
    stage ()
    
    let steer fw up =
        let shipTarget = Vec3.pack <| mission.SpaceCenter.TransformDirection(Vec3.unpack fw, ship.SurfaceReferenceFrame, ship.ReferenceFrame)
        let shipUp = Vec3.pack <| mission.SpaceCenter.TransformDirection(Vec3.unpack up, ship.SurfaceReferenceFrame, ship.ReferenceFrame)
        let controls = steering.UpdatePrediction(mission.UniversalTime, { forward = shipTarget; top = shipUp })
        control.Pitch <- float32U controls.x
        control.Yaw <- float32U controls.z
        control.Roll <- float32U controls.y

    // Go straight up until high enough and fast enough to start turning
    // (Turning to early might result in accidental collisions with launch clamps
    // or result in the vessel spinning out of control)
    while altitudeS.Value < profile.MinTurnAlt || 
           floatU airSpeedS.Value < profile.MinTurnSpeed do

        if shouldStage ship then
            printfn "Staging!"
            stage ()
            
        let fw = forwardAtPitch 89.9<deg> profile.LaunchAzimuth
        let up = upAtPitch 89.9<deg> profile.LaunchAzimuth
        
        steer fw up

    printfn "Starting gravity turn"

    let turnStartAlt = altitudeS.Value

    /// Perform the gravity turn maneuver by following a pre-defined curve
    while apoapsisS.Value < profile.TargetApoapsis - 10.<m> ||
            altitudeS.Value < profile.MaxTurnAlt do
        if shouldStage ship then
            printfn "Staging!"
            stage ()

        let apo = apoapsisS.Value
        let throttle =
            if apo > profile.TargetApoapsis
            then 0.f
            else min 1.f (float32 (profile.TargetApoapsis - apo) / 1000.f)

        control.Throttle <- throttle

        let desiredPitch = turnPitch profile turnStartAlt altitudeS.Value
        
        let fw = forwardAtPitch desiredPitch profile.LaunchAzimuth
        let up = upAtPitch desiredPitch profile.LaunchAzimuth
        steer fw up

    control.Pitch <- 0.f
    control.Yaw <- 0.f
    control.Roll <- 0.f
    control.Throttle <- 0.f

    printfn "Reached space"
