module FunctionalKSP.Maneuver

open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter

open Units
open Extensions
open Telemetry
open FunctionalKSP.LinearAlgebra
open System.Threading

type Apsis = Apoapsis | Periapsis

/// Add a maneuver node for circularizing at the next following Apoapsis or Periapsis.
let addCircularizationNode (ksc: SpaceCenter.Service) (ship: Vessel) (apsis: Apsis): Node = 
    printfn "Computing circularization burn"

    let (eta, radius) =
        match apsis with
        | Apoapsis -> (ship.Orbit.TimeToApoapsis * 1.<s>, ship.Orbit.Apoapsis * 1.<m>)
        | Periapsis -> (ship.Orbit.TimeToPeriapsis * 1.<s>, ship.Orbit.Periapsis * 1.<m>)
    
    printfn "Next %s in %.1f s at %.0f m" (apsis.ToString()) eta radius

    let mu = float ship.Orbit.Body.GravitationalParameter * 1.<m^3/s^2>
    let sma = ship.Orbit.SemiMajorAxis * 1.<m>
    
    let actualVelocity = Mechanics.Orbital.orbitalVelocity mu radius sma
    let desiredVelocity = Mechanics.Orbital.orbitalVelocity mu radius radius
    let deltaV = desiredVelocity - actualVelocity

    printfn "Required deltaV is %.1f m/s" deltaV

    let node = ship.Control.AddNode((ksc.UT.As<s>() + eta) / 1.<s>, prograde=float32U deltaV / 1.f<m/s>)

    node


let executeNext (ship: Vessel) =
    printfn "Executing maneuver node"

    let conn = ship.Connection
    let pilot = ship.AutoPilot
    let control = ship.Control
    let node = control.Nodes.[0]

    // 1. Orient ship along burn vector
    pilot.ReferenceFrame <- node.ReferenceFrame
    pilot.TargetDirection <- (0., 1., 0.)
    pilot.Engage()
    
    printfn "Autopilot engaged"

    // 2. Compute burn time
    let dv = node.RemainingDeltaV * 1.<m/s>
    let Isp = float ship.SpecificImpulse * 1.<s>
    let ve = Isp * 9.80665<m/s^2> // effective exhaust velocity
    let m0 = float ship.Mass * 1.<kg> // initial mass
        
    printfn "dv = %.1f m/s ; Isp = %.1f s ; ve = %.1f m/s ; m0 = %f kg" dv Isp ve m0

    // Tsiolkovsky rocket equation solved for final mass
    let mf = m0 / exp (dv / ve)

    // compute mass of exhaust per second
    let F = floatU (ship.AvailableThrust.As<N>())
    let fuelFlow: float<kg/s> = F / ve

    let burnTime = (m0 - mf) / fuelFlow

    printfn "mf = %f kg ; flow = %.1f ; burn time = %.1f s" mf fuelFlow burnTime

    // 3. Wait for burn
    
    use nodeEta = conn.UseStream<s>(fun () -> node.TimeTo)

    printfn "ETA %.1f" nodeEta.Value
    
    while nodeEta.Value >= burnTime / 2. do
        Thread.Sleep(100)

    // 4. Burn

    printfn "Commencing burn"

    use burnVectorS = conn.UseStream<m/s>(fun () -> node.RemainingBurnVector())
    use thrustS = conn.UseStream<N>(fun () -> ship.AvailableThrust)
    use massS = conn.UseStream<kg>(fun () -> ship.Mass)

    let initialBurnDir = Vec3.norm burnVectorS.Value

    control.Throttle <- 1.f;

    let burnStream = Seq.initInfinite (fun _ -> burnVectorS.Value)
    let accelerationStream = Seq.initInfinite (fun _ -> thrustS.Value / massS.Value)

    let throttleStream = seq {
            for (burn, accel) in Seq.zip burnStream accelerationStream do
            let burnMag = Vec3.mag burn
            let burnDir = burn / burnMag
            // check if deviating more than 60 deg
            let deviating = Vec3.dot initialBurnDir burnDir < 0.5
            // check if burn completed
            let completed = burnMag < 0.05<m/s>
            // compute approximate remaining burn time
            let tRemaining = burnMag / floatU accel
            // scale down throttle when approaching end of burn
            let throttlePerSeconds = 1.<1/s>
            let newThrottle = min 1. (max 0.05 (tRemaining * throttlePerSeconds))
            yield not deviating && not completed, newThrottle
        }
   
    throttleStream
    |> Seq.takeWhile (fun (keepGoing, _) -> keepGoing)
    |> Seq.iter (fun (_, throttle) -> control.Throttle <- float32 throttle)
        
    control.Throttle <- 0.f
    
    printfn "Done"
    node.Remove()

    // 5. Done
    pilot.Disengage()
    ()