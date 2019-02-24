module FunctionalKSP.Maneuver

open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter

open Units
open Extensions
open Telemetry
open FunctionalKSP.LinearAlgebra
open System.Threading
open FunctionalKSP.Control
open FunctionalKSP.Control.Rocket

type Apsis = Apoapsis | Periapsis

/// Add a maneuver node for circularizing at the next following Apoapsis or Periapsis.
let addCircularizationNode (clock: Clock) (ship: Vessel) (apsis: Apsis): Node = 
    printfn "Computing circularization burn"

    let (eta, radius) =
        match apsis with
        | Apoapsis -> (ship.Orbit.TimeToApoapsis * 1.<s>, ship.Orbit.Apoapsis * 1.<m>)
        | Periapsis -> (ship.Orbit.TimeToPeriapsis * 1.<s>, ship.Orbit.Periapsis * 1.<m>)
    
    printfn "Next %s in %.1f s at %.0f m" (apsis.ToString()) eta radius

    let mu = float ship.Orbit.Body.GravitationalParameter * 1.<m^3/s^2>
    let sma = ship.Orbit.SemiMajorAxis * 1.<m>
    
    let actualVelocity = Mechanics.Orbital.VisViva.orbitalVelocity mu radius sma
    let desiredVelocity = Mechanics.Orbital.VisViva.orbitalVelocity mu radius radius
    let deltaV = desiredVelocity - actualVelocity

    printfn "Required deltaV is %.1f m/s" deltaV

    let node = ship.Control.AddNode((clock.Time + eta) / 1.<s>, prograde=float32U deltaV / 1.f<m/s>)

    node

 /// Add a Hohmann transfer node bringing the chosen apsis to the targetAltitude.
let addHohmannNode (clock: Clock) (ship: Vessel) (apsis: Apsis) (targetAlt: float<m>) =
    printfn "Computing Hohmann transfer"

    // place node at opposite apsis
    let (eta, radius) =
        match apsis with
        | Periapsis -> (ship.Orbit.TimeToApoapsis * 1.<s>, ship.Orbit.Apoapsis * 1.<m>)
        | Apoapsis -> (ship.Orbit.TimeToPeriapsis * 1.<s>, ship.Orbit.Periapsis * 1.<m>)

    printfn "Next %s in %.1f s at %.0f m" (apsis.ToString()) eta radius

    let mu = float ship.Orbit.Body.GravitationalParameter * 1.<m^3/s^2>
    let currentSma = ship.Orbit.SemiMajorAxis * 1.<m>
    let newSma = (radius + floatU (ship.Orbit.Body.EquatorialRadius.As<m>()) + targetAlt) / 2.
    
    let actualVelocity = Mechanics.Orbital.VisViva.orbitalVelocity mu radius currentSma 
    let desiredVelocity = Mechanics.Orbital.VisViva.orbitalVelocity mu radius newSma
    let deltaV = desiredVelocity - actualVelocity

    printfn "Required deltaV is %.1f m/s" deltaV

    let node = ship.Control.AddNode((clock.Time + eta) / 1.<s>, prograde=float32U deltaV / 1.f<m/s>)

    node

let executeNext (clock: Clock) (steering: Control.Rocket.Steering) (warpHeadroom: float<s>) =
    printfn "Executing maneuver node"

    let ship = steering.Vessel
    let conn = ship.Connection
    let pilot = ship.AutoPilot
    let control = ship.Control
    let node = control.Nodes.[0]

    // 1. Orient ship along burn vector
    steering.Mode <- SteeringMode.LockTarget (Vec3.unitY, node.ReferenceFrame)
    
    use facingS = conn.UseStream<1>(fun () -> ship.Direction(node.ReferenceFrame))

    printfn "Orienting ship"

    while Vec3.angle facingS.Value Vec3.unitY > 0.1<deg> / degPerRad do
        clock.Tick() |> steering.Update

    // 2. Compute burn time
    let dv = node.RemainingDeltaV * 1.<m/s>
    let Isp = float ship.SpecificImpulse * 1.<s>
    let ve = Isp * 9.80665<m/s^2> // effective exhaust velocity
    let m0 = float ship.Mass * 1.<kg> // initial mass
        
    printfn "dv = %.1f m/s ; Isp = %.1f s ; ve = %.1f m/s ; m0 = %f kg" dv Isp ve m0

    // Tsiolkovsky rocket equation solved for final mass
    let mf = Mechanics.Orbital.Tsiolkovsky.finalMass dv ve m0

    // compute mass of exhaust per second
    let F = floatU (ship.AvailableThrust.As<N>())
    let fuelFlow: float<kg/s> = F / ve

    let burnTime = (m0 - mf) / fuelFlow

    printfn "mf = %f kg ; flow = %.1f ; burn time = %.1f s" mf fuelFlow burnTime

    // 3. Wait for burn

    let nodeUT = node.UT.As<s>()

    printfn "Time: %.0f s" clock.Time
    printfn "ETA %.1f s" (nodeUT - clock.Time)

    /// warp to node, but leave headroom for final orientation corrections
    if clock.Time < nodeUT - burnTime / 2. - warpHeadroom then
        clock.WarpTo(nodeUT - burnTime / 2. - warpHeadroom)
        
    while clock.Tick() < nodeUT - burnTime / 2. do
        steering.Update clock.Time

    // 4. Burn

    printfn "Commencing burn"

    use burnVectorS = conn.UseStream<m/s>(fun () -> node.RemainingBurnVector())
    use thrustS = conn.UseStream<N>(fun () -> ship.AvailableThrust)
    use massS = conn.UseStream<kg>(fun () -> ship.Mass)

    let initialBurnDir = Vec3.norm burnVectorS.Value

    control.Throttle <- 1.f;

    let rec doBurn () =
        let burnMag = Vec3.mag burnVectorS.Value
        let burnDir = burnVectorS.Value / burnMag
        // check if deviating more than 60 deg
        let deviating = Vec3.dot initialBurnDir burnDir < 0.5
        // check if burn completed
        let completed = burnMag < 0.05<m/s>

        if not (deviating || completed) then
            let accel = floatU thrustS.Value / floatU massS.Value
            let horizon = 0.5<s>

            // scale down throttle when approaching end of burn
            let desiredAcceleration = burnMag / horizon
            let newThrottle = clamp 0. 1. (desiredAcceleration / accel)
            control.Throttle <- float32U newThrottle
            steering.Mode <- Rocket.LockTarget (burnDir, node.ReferenceFrame)
            clock.Tick() |> steering.Update
            doBurn ()

    doBurn ()

    // 5. Done

    control.Throttle <- 0.f
    steering.Mode <- KillRotation
    printfn "Done"
    node.Remove()
