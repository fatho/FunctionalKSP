module FunctionalKSP.Launch

open KRPC.Client;
open KRPC.Client.Services;
open KRPC.Client.Services.SpaceCenter
open System.Threading
open System

open Units
open Telemetry
open Extensions;
open FunctionalKSP.Math
open FunctionalKSP.Control
    
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

let launch (ksc: SpaceCenter.Service) (ship: Vessel) (profile: Profile) =
    if ship.Situation <> VesselSituation.Landed && ship.Situation <> VesselSituation.PreLaunch
        then invalidArg "vessel" "Not grounded"
            
    printfn "Preparing launch"

    let conn = ship.Connection
    let control = ship.Control
    let pilot = ship.AutoPilot
    let body = ship.Orbit.Body;
    let flight = ship.Flight(ship.SurfaceReferenceFrame);
    let steering = new Rocket.AtmosphericSteering()
    let coastSteering = new Rocket.ZeroGSteering(ksc, ship)
    use clock = new Clock(conn, ksc)

    control.Throttle <- 0.f

    use altitudeS = conn.UseStream<m>(fun () -> flight.MeanAltitude)
    use airSpeedS = conn.UseStream<m/s>(fun () -> flight.TrueAirSpeed)
    use apoapsisS = conn.UseStream<m>(fun () -> ship.Orbit.ApoapsisAltitude);
    let availableTorqueS = conn.UseStream<N m>(fun () -> ship.AvailableTorque)
    let moiS = conn.UseStream<kg m^2>(fun () -> ship.MomentOfInertia)
    let orbitalAngularVelocityS = conn.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.OrbitalReferenceFrame))
        
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

    let north = compassDir 0.<deg>
    let east = compassDir 90.<deg>

    let launchAlt = altitudeS.Value
    
    printfn "pitch = %f deg" flight.Pitch
    printfn "altitude = %f m" launchAlt

    printfn "f(90, 90) %O" (forwardAtPitch 90.<deg> 90.<deg>)
    printfn "f(45, 90) %O" (forwardAtPitch 45.<deg> 90.<deg>)
    printfn "f( 0, 90) %O" (forwardAtPitch 0.<deg> 90.<deg>)
    printfn "f(45,  0) %O" (forwardAtPitch 45.<deg> 0.<deg>)
        
    printfn "u(90, 90) %O" (upAtPitch 90.<deg> 90.<deg>)
    printfn "u(45, 90) %O" (upAtPitch 45.<deg> 90.<deg>)
    printfn "u( 0, 90) %O" (upAtPitch 0.<deg> 90.<deg>)
    printfn "u(45,  0) %O" (upAtPitch 45.<deg> 0.<deg>)
            
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
    
    let mutable dataCounter = 0
    let dumpData desiredPitch =
        dataCounter <- dataCounter + 1
        if dataCounter >= 10 then
            dataCounter <- 0
            printfn "%6.0f %6.2f %6.2f %6.2f" altitudeS.Value flight.Heading flight.Pitch desiredPitch

    let combineTorques (pos: vec3<N m>, neg: vec3<N m>): vec3<N m> = Vec3.zip (fun a b -> min (abs a) (abs b)) pos neg

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
        
        let shipAngVel = Vec3.pack<rad/s> <| ksc.TransformDirection(Vec3.unpack orbitalAngularVelocityS.Value, ship.OrbitalReferenceFrame, ship.ReferenceFrame)
        let shipTarget = Vec3.pack <| ksc.TransformDirection(Vec3.unpack fw, ship.SurfaceReferenceFrame, ship.ReferenceFrame)
        let shipUp = Vec3.pack <| ksc.TransformDirection(Vec3.unpack up, ship.SurfaceReferenceFrame, ship.ReferenceFrame)

        let time = clock.Tick()
        let input = {
            Rocket.sampleTime = time;
            Rocket.targetForward = shipTarget;
            Rocket.targetTop = shipUp;
            Rocket.controlTorque = combineTorques availableTorqueS.Value;
            Rocket.momentOfInertia = moiS.Value;
            Rocket.angularVelocity = shipAngVel;
        }
        let controls = steering.UpdatePrediction(input)
        control.Pitch <- float32U controls.x
        control.Yaw <- float32U controls.z
        control.Roll <- float32U controls.y
        dumpData 89.9
                
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
        let shipAngVel = -(Vec3.pack<rad/s> <| ksc.TransformDirection(Vec3.unpack orbitalAngularVelocityS.Value, ship.OrbitalReferenceFrame, ship.ReferenceFrame))
        let shipTarget = Vec3.pack <| ksc.TransformDirection(Vec3.unpack fw, ship.SurfaceReferenceFrame, ship.ReferenceFrame)
        let shipUp = Vec3.pack <| ksc.TransformDirection(Vec3.unpack up, ship.SurfaceReferenceFrame, ship.ReferenceFrame)

        let time = clock.Tick()
        let input = {
            Rocket.sampleTime = time;
            Rocket.targetForward = shipTarget;
            Rocket.targetTop = shipUp;
            Rocket.controlTorque = combineTorques availableTorqueS.Value;
            Rocket.momentOfInertia = moiS.Value;
            Rocket.angularVelocity = shipAngVel;
        }
        let atmosphericControls = steering.UpdatePrediction(input)
        
        control.Pitch <- float32U atmosphericControls.x
        control.Yaw <- float32U atmosphericControls.z
        control.Roll <- float32U atmosphericControls.y
        
        dumpData (float desiredPitch)

    control.Pitch <- 0.f
    control.Yaw <- 0.f
    control.Roll <- 0.f
    control.Throttle <- 0.f

    printfn "Reached space"
