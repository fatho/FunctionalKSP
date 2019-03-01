namespace ANewDawn

open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter

open ANewDawn.Units
open ANewDawn.Extensions
open ANewDawn.Math
open ANewDawn.Control
open ANewDawn.Mission

type Apsis = Apoapsis | Periapsis


module Maneuver =
    /// Add a maneuver node for circularizing at the next following Apoapsis or Periapsis.
    let addCircularizationNode (mission: Mission) (apsis: Apsis): Node = 
        printfn "Computing circularization burn"
        let ship = mission.ActiveVessel

        let (eta, radius) =
            match apsis with
            | Apoapsis -> (ship.Orbit.TimeToApoapsis * 1.<s>, ship.Orbit.Apoapsis * 1.<m>)
            | Periapsis -> (ship.Orbit.TimeToPeriapsis * 1.<s>, ship.Orbit.Periapsis * 1.<m>)
    
        printfn "Next %s in %.1f s at %.0f m" (apsis.ToString()) eta radius

        let mu = float ship.Orbit.Body.GravitationalParameter * 1.<m^3/s^2>
        let sma = ship.Orbit.SemiMajorAxis * 1.<m>
    
        let actualVelocity = VisViva.orbitalVelocity mu radius sma
        let desiredVelocity =VisViva.orbitalVelocity mu radius radius
        let deltaV = desiredVelocity - actualVelocity

        printfn "Required deltaV is %.1f m/s" deltaV

        let node = ship.Control.AddNode((mission.UniversalTime + eta) / 1.<s>, prograde=float32U deltaV / 1.f<m/s>)

        node

     /// Add a Hohmann transfer node bringing the chosen apsis to the targetAltitude.
    let addHohmannNode (mission: Mission) (apsis: Apsis) (targetAlt: float<m>) =
        printfn "Computing Hohmann transfer"
        let ship = mission.ActiveVessel

        // place node at opposite apsis
        let (eta, radius) =
            match apsis with
            | Periapsis -> (ship.Orbit.TimeToApoapsis * 1.<s>, ship.Orbit.Apoapsis * 1.<m>)
            | Apoapsis -> (ship.Orbit.TimeToPeriapsis * 1.<s>, ship.Orbit.Periapsis * 1.<m>)

        printfn "Next %s in %.1f s at %.0f m" (apsis.ToString()) eta radius

        let mu = float ship.Orbit.Body.GravitationalParameter * 1.<m^3/s^2>
        let currentSma = ship.Orbit.SemiMajorAxis * 1.<m>
        let newSma = (radius + floatU (ship.Orbit.Body.EquatorialRadius.As<m>()) + targetAlt) / 2.
    
        let actualVelocity = VisViva.orbitalVelocity mu radius currentSma 
        let desiredVelocity = VisViva.orbitalVelocity mu radius newSma
        let deltaV = desiredVelocity - actualVelocity

        printfn "Required deltaV is %.1f m/s" deltaV

        let node = ship.Control.AddNode((mission.UniversalTime + eta) / 1.<s>, prograde=float32U deltaV / 1.f<m/s>)

        node

        
    let addInterceptionNode (mission: Mission) (target: Orbit) =
        let current = mission.ActiveVessel.Orbit
        assert (current.Body.Name = target.Body.Name)

        let body = current.Body
        let now = mission.UniversalTime
        let mu = floatU <| body.GravitationalParameter.As<m^3/s^2>()
        
        let velocityAt (orbit: Orbit) (ut: float<s>) =
            //let scale = sqrt (mu * orbit.SemiMajorAxis) / orbit.RadiusAt(ut)
            //let E = orbit.EccentricAnomalyAtUT(ut)
            //scale * { x = - sin E; y = sqrt (1 - square orbit.Eccentricity) * cos E; z = 0 }
            let p1 = Vec3.pack <| orbit.PositionAt(float ut - 0.5, body.NonRotatingReferenceFrame)
            let p2 = Vec3.pack <| orbit.PositionAt(float ut + 0.5, body.NonRotatingReferenceFrame)
            orbit.OrbitalSpeedAt(float ut).As<m/s>() * Vec3.norm (p2 - p1)

        let evaluate tDeparture tArrival =
            let p1 = Vec3.pack <| current.PositionAt(tDeparture / 1.<s>, body.NonRotatingReferenceFrame)
            let p2 = Vec3.pack <| target.PositionAt(tArrival / 1.<s>, body.NonRotatingReferenceFrame)
            let result = Lambert.lambert p1 p2 (tArrival - tDeparture) 0 mu
            match result with
            | Lambert.Solution (v1, v2) -> 
                let actualV1 = velocityAt current tDeparture
                let actualV2 = velocityAt target tArrival
                let dv1 = v1 - actualV1
                let dv2 = actualV2 - v2
                // Some (v1 - actualV1, actualV2 - v2)
                Some (Vec3.mag dv1 + Vec3.mag dv2)
            | _ -> None

        
        for dtDep in { 600.<s> .. 60.<s> .. 3600.<s> } do
            for tFlight in { 600.<s> .. 60.<s> .. 1200.<s> } do
                let dv = evaluate (now + dtDep) (now + dtDep + tFlight)
                printfn "DP: %f   FL: %f  dV: %O" dtDep tFlight dv

        ()

    /// Perform a maneuver that orients the vessel in the given direction, does not perform roll
    let orient (mission: Mission) (ref: ReferenceFrame) (direction: vec3<1>) (thresholdPos: float<deg>) (thresholdVel: float<deg/s>) (timeout: float<s>): float<deg> * float<deg/s> =
        let ship = mission.ActiveVessel
        use angVelS = mission.Streams.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.Orbit.Body.NonRotatingReferenceFrame))
        use shipFacingS = mission.Streams.UseStream<1>(fun () -> ship.Direction(ref))

        //use steering = new KosSteering()
        
        let tMax = mission.UniversalTime + timeout
        
        let mutable remainingAngle = infinity.As<deg>()
        let mutable remainingAngularVel = infinity.As<deg/s>()

        AttitudeControl.loop mission ref <| seq {
            while not (mission.UniversalTime >= tMax || (remainingAngle <= thresholdPos && remainingAngularVel <= thresholdVel)) do
                yield { forward = direction; top = None }
                remainingAngle <- Vec3.angle shipFacingS.Value direction * degPerRad
                remainingAngularVel <- Vec3.mag angVelS.Value * degPerRad
        }
        remainingAngle, remainingAngularVel

    /// Perform a maneuver that kills all rotation in the orbital reference frame.
    /// Returns the residual angular velocity that was not killed.
    let killRotation (mission: Mission) (threshold: float<deg/s>) (timeout: float<s>): vec3<deg/s> =
        let ship = mission.ActiveVessel
        use angVelS = mission.Streams.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.Orbit.Body.NonRotatingReferenceFrame))
        let tMax = mission.UniversalTime + timeout
        let mutable remainingAngularVel = infinity.As<deg/s>()
        
        AttitudeControl.loopAngVel mission <| seq {
            while not (mission.UniversalTime >= tMax || remainingAngularVel <= threshold) do
                yield Vec3.zero<_>
                remainingAngularVel <- Vec3.mag angVelS.Value * degPerRad
        }
        angVelS.Value * degPerRad

    let executeNext (mission: Mission) (warpHeadroom: float<s>) =
        printfn "Executing maneuver node"

        let ship = mission.ActiveVessel
        let control = ship.Control
        let node = control.Nodes.[0]

        // TODO: compute burn time across multiple stages

        // 2. Compute initial burn time
        let computeBurnTime dv =
            let Isp = float ship.SpecificImpulse * 1.<s>
            let ve = Isp * 9.80665<m/s^2> // effective exhaust velocity
            let m0 = float ship.Mass * 1.<kg> // initial mass
        
            printfn "dv = %.1f m/s ; Isp = %.1f s ; ve = %.1f m/s ; m0 = %f kg" dv Isp ve m0

            // Tsiolkovsky rocket equation solved for final mass
            let mf = Tsiolkovsky.finalMass dv ve m0

            // compute mass of exhaust per second
            let F = floatU (ship.AvailableThrust.As<N>())
            let fuelFlow: float<kg/s> = F / ve
            let burnTime = (m0 - mf) / fuelFlow

            printfn "mf = %f kg ; flow = %.1f ; burn time = %.1f s" mf fuelFlow burnTime

            burnTime

        let initialBurnTime = computeBurnTime (node.RemainingDeltaV * 1.<m/s>)

        
        // 1. Orient ship along burn vector

        let nodeUT = node.UT.As<s>()
        let burnStartT = nodeUT - initialBurnTime / 2.
        let eta = burnStartT - mission.UniversalTime
        
        printfn "Orienting ship prograde"

        let (residualAngle, residualAngVel) = orient mission node.ReferenceFrame Vec3.unitY 0.5<deg> 0.1<deg/s> eta

        printfn "ship oriented, angle = %.2f angvel = %.2f" residualAngle residualAngVel

        // 3. Wait for burn

        printfn "Time: %.0f s" mission.UniversalTime
        printfn "ETA (node): %.1f s" (nodeUT - mission.UniversalTime)
        printfn "ETA (burn): %.1f s" (burnStartT - mission.UniversalTime)

        /// warp to node, but leave headroom for final orientation corrections
        if mission.UniversalTime < burnStartT - warpHeadroom then
            mission.WarpTo(nodeUT - initialBurnTime / 2. - warpHeadroom)
        
        // final orientation corrections
        
        use burnVectorS = mission.Streams.UseStream<m/s>(fun () -> node.RemainingBurnVector())
        use thrustS = mission.Streams.UseStream<N>(fun () -> ship.AvailableThrust)
        use massS = mission.Streams.UseStream<kg>(fun () -> ship.Mass)

        // let accel = thrustS.Value / massS.Value

        let throttlePid = new PidLoop<m/s, 1>(0.2<s/m>, 0.1</m>, 0.<_>, 0., 1.)
        
        let initialBurnDir = Vec3.norm burnVectorS.Value

        AttitudeControl.loop mission node.ReferenceFrame <| seq {
            while mission.UniversalTime < burnStartT do
                yield { forward = Vec3.unitY; top = None }

            printfn "Commencing burn"
                
            let isDeviating () = Vec3.angle burnVectorS.Value initialBurnDir > 45.<deg> / degPerRad
            let isCompleted () = Vec3.mag burnVectorS.Value < 0.05<m/s>
            
            control.Throttle <- 1.f;

            while not (isCompleted () || isDeviating ()) do

                if Staging.shouldStage ship then
                    ship.Control.ActivateNextStage() |> ignore
                    // recompute burn time
                    printfn "Recomputing burn"
                    let newBurnTime = computeBurnTime (Vec3.mag burnVectorS.Value)
                    printfn "Remaining burn time %.1f s" newBurnTime

                let throttle = throttlePid.Update(mission.UniversalTime, - Vec3.mag burnVectorS.Value)
                control.Throttle <- float32U throttle

                yield { forward = Vec3.unitY; top = None }
                
            control.Throttle <- 0.f;
        }

        // 5. Done

        node.Remove()
