module FunctionalKSP.Launch

open KRPC.Client;
open KRPC.Client.Services;
open KRPC.Client.Services.SpaceCenter
open System.Threading
open System

open Units
open Telemetry
open Extensions;
open FunctionalKSP.LinearAlgebra
    
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

    let test = new FunctionalKSP.Control.Rocket.Steering(ksc, ship)

    let conn = ship.Connection
    let control = ship.Control
    let pilot = ship.AutoPilot
    let body = ship.Orbit.Body;
    let flight = ship.Flight(ship.SurfaceReferenceFrame);

    control.SAS <- true
    //Thread.Sleep(100)
    //control.SASMode <- SASMode.StabilityAssist
    control.Throttle <- 0.f

    use altitudeS = conn.UseStream<m>(fun () -> flight.MeanAltitude)
    use airSpeedS = conn.UseStream<m/s>(fun () -> flight.TrueAirSpeed)
    use pitchS = conn.UseStream<deg>(fun () -> flight.Pitch)
    use apoapsisS = conn.UseStream<m>(fun () -> ship.Orbit.ApoapsisAltitude);

    let downVectorS = conn.UseStream<1>(fun () -> ksc.TransformDirection((0., 0., 1.), ship.ReferenceFrame, ship.SurfaceReferenceFrame))
    
    // compass direction vector in surface reference frame of vessel, 0 deg is north, 90 deg is east
    let compassDir (angle: float<deg>) =
        let a = angle / degPerRad / 1.<rad>
        { x = 0. (* up *); y = cos a (* north *); z = sin a (* east *) }

    let north = compassDir 0.<deg>
    let east = compassDir 90.<deg>
    /// reliably compute heading while pointing upwards on the launchpad
    let launchpadHeading () = Vec3.compassHeading north east downVectorS.Value

    let launchAlt = altitudeS.Value
    
    printfn "pitch = %f deg" flight.Pitch

    printfn "heading = %f deg" (launchpadHeading ())
    printfn "east: %s" (east.ToString())
    printfn "north: %s" (north.ToString())
    printfn "down: %s" (downVectorS.Value.ToString())


    printfn "north = %f deg" (Vec3.compassHeading north east north)
    printfn "east = %f deg" (Vec3.compassHeading north east east)
    printfn "south = %f deg" (Vec3.compassHeading north east -north)
    printfn "west = %f deg" (Vec3.compassHeading north east -east)

    printfn "altitude = %f m" launchAlt
            
    printfn "Launching in"
    for count in { 0 .. profile.Countdown - 1 } do
        printfn "  %d s" (profile.Countdown - count)
        Thread.Sleep(1000)

    //pilot.ReferenceFrame <- ship.SurfaceReferenceFrame
    //pilot.TargetPitchAndHeading(90.f, flight.Heading)
    //pilot.TargetRoll <- Single.NaN
    //pilot.Engage()

    control.Throttle <- 1.f
            
    // Lift-off
    let stage () = control.ActivateNextStage() |> ignore
   
    printfn "Ignition"
    stage ()
    
    let mutable dataCounter = 0
    let dumpData desiredPitch =
        dataCounter <- dataCounter + 1
        if dataCounter >= 10 then
            dataCounter <- 0
            printfn "%6.0f %6.2f %6.2f %6.2f %6.2f" altitudeS.Value flight.Heading flight.Pitch desiredPitch (launchpadHeading ())

    let steerPitch desiredPitch =
        let actualPitch = floatU pitchS.Value
        let pitchError = desiredPitch - actualPitch
        let pitchGain = 1. / 10.<deg>
        control.Pitch <- clamp -1. 1. (pitchGain * pitchError) |> float32U
    
    let steerRoll rollError =
        let rollGain = 1. / 45.<deg>
        control.Roll <- clamp -0.3 0.3 (rollGain * rollError) |> float32U

    let steerYaw yawError =
        let yawGain =  1. / 10.<deg>
        control.Yaw <- clamp -0.4 0.4 (yawGain * yawError) |> float32U

    let initialHeading = launchpadHeading ()

    // Go straight up until high enough and fast enough to start turning
    // (Turning to early might result in accidental collisions with launch clamps
    // or result in the vessel spinning out of control)
    while altitudeS.Value < profile.MinTurnAlt || 
           floatU airSpeedS.Value < profile.MinTurnSpeed do

        if shouldStage ship then
            printfn "Staging!"
            stage ()
            
        /// If we're sufficiently high, rotate the ship to the launch direction
        // (if it wasn't rotated correctly already pre-launch)
        let desiredHeading =
            if altitudeS.Value > launchAlt + 25.<m>
            then profile.LaunchAzimuth
            else initialHeading

        let currentHeading = launchpadHeading ()
        let headingError = desiredHeading - currentHeading
        /// Make sure we take shortest path along the circle for correcting the heading
        let rollError =
            if headingError > 180.<deg> then headingError - 360.<deg>
            elif headingError < -180.<deg> then headingError + 360.<deg>
            else headingError

        let yawError = rollError

        steerPitch 89.9<deg>
        steerRoll -rollError
                            
        Thread.Sleep(100)
        dumpData 90.
                
    printfn "Starting gravity turn"

    let turnStartAlt = altitudeS.Value

    /// Perform the gravity turn maneuver by following a pre-defined curve
    while apoapsisS.Value < profile.TargetApoapsis ||
            altitudeS.Value < profile.MaxTurnAlt do
        if shouldStage ship then
            printfn "Staging!"
            stage ()

        let desiredPitch = turnPitch profile turnStartAlt altitudeS.Value
        steerPitch desiredPitch

        let currentHeading = launchpadHeading ()
        let headingError = profile.LaunchAzimuth - currentHeading
        /// Make sure we take shortest path along the circle for correcting the heading
        let rollError =
            if headingError > 180.<deg> then headingError - 360.<deg>
            elif headingError < -180.<deg> then headingError + 360.<deg>
            else headingError

        steerRoll -rollError
        steerYaw rollError

        let apo = apoapsisS.Value
        control.Throttle <- if apo > profile.TargetApoapsis
                                then 0.f
                                else min 1.f (max 0.1f (float32 (profile.TargetApoapsis - apo) / 1000.f))

        Thread.Sleep(100)
        dumpData (float desiredPitch)

    control.Pitch <- 0.f
    control.Yaw <- 0.f
    control.Roll <- 0.f
    control.Throttle <- 0.f
    // pilot.Disengage()

    printfn "Reached space"