namespace ANewDawn

open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter

open ANewDawn.Units
open ANewDawn.Extensions
open ANewDawn.Math
open ANewDawn.Math.Util
open ANewDawn.Control
open ANewDawn.Mission

type Apsis = Apoapsis | Periapsis


module Maneuver =
    open System

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
                
    let addInterceptionNode (mission: Mission) (target: Orbit) (departureTimeRange: float<s> * float<s>) (flightTimeRange: float<s> * float<s>) =
        let current = mission.ActiveVessel.Orbit
        assert (current.Body.Name = target.Body.Name)

        let body = current.Body
        let now = mission.UniversalTime
        let mu = floatU <| body.GravitationalParameter.As<m^3/s^2>()

        let currentElems = current.OrbitalElements
        let targetElems = target.OrbitalElements

        let evaluate tDeparture tArrival =
            let (p1, v1) = Kepler.stateVectors mu currentElems tDeparture
            let (p2, v2) = Kepler.stateVectors mu targetElems tArrival
            let result = Lambert.lambert p1 p2 (tArrival - tDeparture) 0 mu
            match result with
            | Lambert.Solution (transV1, transV2) -> 
                let dv1 = transV1 - v1
                let dv2 = v2 - transV2
                // Some (v1 - actualV1, actualV2 - v2)
                Some (dv1, dv2)
            | _ -> None

        let deltaV (dv1, dv2) = Vec3.mag dv1 + Vec3.mag dv2

        // TODO: use orbital periods to guide search
        let currentPeriod = Kepler.orbitalPeriod mu currentElems.semiMajorAxis
        //let targetPeriod = Kepler.orbitalPeriod mu targetElems.semiMajorAxis
        
        let mutable minDV = Double.PositiveInfinity.As<m/s>()
        let mutable minParams: float<s> * float<s> = 0.<_>, 0.<_>
        let mutable minBurns: vec3<m/s> * vec3<m/s> = Vec3.zero<_>, Vec3.zero<_>
                
        printfn "Computing optimal interception trajectory"

        let depTimStep = (snd departureTimeRange - fst departureTimeRange) / 200.
        let flightTimeStep = (snd flightTimeRange - fst flightTimeRange) / 200.

        for dtDep in { fst departureTimeRange .. depTimStep .. snd departureTimeRange } do
            for tFlight in { fst flightTimeRange .. flightTimeStep .. snd flightTimeRange } do
                let dv = evaluate (now + dtDep) (now + dtDep + tFlight)
                match dv with
                | None -> ()
                | Some(dv1, dv2) ->
                    let dv = deltaV (dv1, dv2)
                    if dv < minDV then
                        minDV <- dv
                        minParams <- dtDep, tFlight
                        minBurns <- dv1, dv2

        printfn "Min: %f m/s at %O" minDV minParams
        printfn "Burns: %O" minBurns
        
        let startTime = now + fst minParams
        let rStart, vStart = Kepler.stateVectors mu currentElems startTime
        let dirs = Kepler.maneuverDirections rStart vStart
        let burnStart = fst minBurns

        let vPrograde = Vec3.dot burnStart dirs.prograde
        let vRadial = Vec3.dot burnStart dirs.radial
        let vNormal = Vec3.dot burnStart dirs.normal
        let f v = float32U (v / 1.<m/s>)

        mission.ActiveVessel.Control.AddNode((now + fst minParams) / 1.<s>, prograde=f vPrograde, normal=f vNormal, radial=f vRadial)

    /// Add a node at the closest approach with the target for catching up with it.
    let addCatchUpTargetNode (mission: Mission) (target: Orbit) =
        let current = mission.ActiveVessel.Orbit
        assert (current.Body.Name = target.Body.Name)

        let body = current.Body
        let now = mission.UniversalTime
        let mu = floatU <| body.GravitationalParameter.As<m^3/s^2>()

        let currentElems = current.OrbitalElements
        let targetElems = target.OrbitalElements

        let tIntercept = current.TimeOfClosestApproach(target).As<s>()

        let (p1, v1) = Kepler.stateVectors mu currentElems tIntercept
        let (p2, v2) = Kepler.stateVectors mu targetElems tIntercept
        let rvel = v2 - v1

        printfn "eta = %.1f s  d = %.1f m rvel = %O   (%.1f)" (tIntercept - now) (Vec3.mag (p2 - p1)) rvel (Vec3.mag rvel)

        let dirs = Kepler.maneuverDirections p1 v1

        let vPrograde = Vec3.dot rvel dirs.prograde
        let vRadial = Vec3.dot rvel dirs.radial
        let vNormal = Vec3.dot rvel dirs.normal
        let f v = float32U (v / 1.<m/s>)

        let burnTimeFactor = 1. + (Vec3.mag v1 - Vec3.mag v2) / (2. * Vec3.mag v2)
        
        mission.ActiveVessel.Control.AddNode(tIntercept / 1.<s>, prograde=f vPrograde, normal=f vNormal, radial=f vRadial), burnTimeFactor
        
    /// Add a maneuver node for deorbiting, reaching the given altitude at the desired surface position
    let addDeorbitNode (mission: Mission) (latLon: float<deg> * float<deg>) (altitude: float<m>) (tMax: float<s>) : Node  =
        let current = mission.ActiveVessel.Orbit
        let body = current.Body
        let now = mission.UniversalTime
        let mu = floatU <| body.GravitationalParameter.As<m^3/s^2>()

        let rot0 = body.InitialRotation.As<rad>()
        let rotVel = floatU <| body.RotationalSpeed.As<rad/s>()
        let r = floatU <| body.EquatorialRadius.As<m>()

        let targetPositionAtT t =
            let (lat, lon) = latLon
            let lonAtT = lon + (rot0 + t * rotVel) * degPerRad
            let dir = { x = cosRad (lonAtT / degPerRad); y = sinRad (lat / degPerRad); z = sinRad (lonAtT / degPerRad) }
            dir * (r + altitude)
        
        let currentElems = current.OrbitalElements
        
        let evaluate tDeparture tArrival =
            let (p1, v1) = Kepler.stateVectors mu currentElems tDeparture
            let p2 = targetPositionAtT tArrival
            let result = Lambert.lambert p1 p2 (tArrival - tDeparture) 0 mu
            match result with
            | Lambert.Solution (transV1, _transV2) -> 
                let dv = transV1 - v1
                Some dv
            | _ -> None
            
        // TODO: use orbital periods to guide search
        let currentPeriod = Kepler.orbitalPeriod mu currentElems.semiMajorAxis
        let orbMax = tMax / currentPeriod
        //let targetPeriod = Kepler.orbitalPeriod mu targetElems.semiMajorAxis
        
        let mutable minDV = Double.PositiveInfinity.As<m/s>()
        let mutable minParams: float<s> * float<s> = 0.<_>, 0.<_>
        let mutable minBurn: vec3<m/s> = Vec3.zero<_>
                
        printfn "Computing optimal interception trajectory"

        for depOrbits in { 0. .. 0.005 .. orbMax } do
            for flightOrbits in { 0.005 .. 0.001 .. 1. } do
                let dtDep = currentPeriod * depOrbits
                let tFlight = currentPeriod * flightOrbits
                let dv = evaluate (now + dtDep) (now + dtDep + tFlight)
                match dv with
                | None -> ()
                | Some dv ->
                    if Vec3.mag dv < minDV then
                        minDV <- Vec3.mag dv
                        minParams <- dtDep, tFlight
                        minBurn <- dv

        printfn "Min: %f at %O" minDV minParams
        printfn "Burn: %O" minBurn

        let startTime = now + fst minParams
        let rStart, vStart = Kepler.stateVectors mu currentElems startTime
        let prograde = Vec3.norm vStart
        let radial = Vec3.norm rStart
        let normal = Vec3.cross prograde radial

        let vPrograde = Vec3.dot minBurn prograde
        let vRadial = Vec3.dot minBurn radial
        let vNormal = Vec3.dot minBurn normal
        let f v = float32U (v / 1.<m/s>)

        mission.ActiveVessel.Control.AddNode((now + fst minParams) / 1.<s>, prograde=f vPrograde, normal=f vNormal, radial=f vRadial)

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

    type ExecutionMode = {
        earlyStartFactor: float
    }

    let executeNextExt (mission: Mission) (warpHeadroom: float<s>) (mode: ExecutionMode) =
        printfn "Executing maneuver node"

        let ship = mission.ActiveVessel
        let control = ship.Control
        let node = control.Nodes.[0]

        // TODO: compute burn time across multiple stages
        let currentStage = control.CurrentStage

        let stageInfo stage =
            let stageParts = query {
                for part in ship.Parts.All do
                where (part.DecoupleStage = stage - 1)
                select part
            }
            let engines = query {
                for part in stageParts do
                let engine = part.Engine
                where (engine <> null)
                select (floatU <| engine.MaxVacuumThrust.As<N>(), floatU <| engine.VacuumSpecificImpulse.As<s>())
            }
            printfn "  Parts in stage %d" stage
            if Seq.isEmpty engines then
                None
            else
                for p in stageParts do
                    printfn "    part %s, active %d, decouple %d" p.Name p.Stage p.DecoupleStage
                let thrust = engines |> Seq.sumBy fst
                let isp = thrust / Seq.sumBy (fun (f, isp) -> f / isp) engines
                let propellantMass = query {
                        for part in stageParts do
                        sumBy (part.Mass.As<kg>() - part.DryMass.As<kg>())
                    }

                Some (thrust, isp, propellantMass)

        let decoupledDryMass stage = query {
                for part in ship.Parts.All do
                where (part.DecoupleStage = stage)
                sumBy (part.DryMass.As<kg>())
            }

        let rec computeBetterBurnTime (dv: float<m/s>) (m0: float<kg>) (stage: int): float<s> =
            printfn "Stage %d:" stage
            if stage = -1 then
                0.<s>
            else
                let nextStage mfStage dvStage =
                    let decoupledMass = decoupledDryMass (stage - 1)
                    
                    printfn "  dv in this stage = %.1f m/s" dvStage
                    printfn "  decoupling %.1f kg" decoupledMass

                    computeBetterBurnTime (dv - dvStage) (mfStage - decoupledMass) (stage - 1)

                match stageInfo stage with
                | Some(F, Isp, propellant) -> 
                    let ve = Isp * 9.80665<m/s^2> // effective exhaust velocity
                    printfn "  dv = %.1f m/s ; Isp = %.1f s ; ve = %.1f m/s ; F = %.1f" dv Isp ve F

                    // Tsiolkovsky rocket equation solved for final mass
                    let mf = Tsiolkovsky.finalMass dv ve m0

                    // compute mass of exhaust per second
                    let fuelFlow: float<kg/s> = F / ve

                    let burnTimeStage = propellant / fuelFlow
                
                    printfn "  m0 = %.1f kg; mf = %.1f kg ; m_prop = %.1f kg ; flow = %.1f " m0 mf propellant fuelFlow
                    printfn "  stage burnt out after %.1f s" burnTimeStage

                    if m0 - mf <= propellant then
                        // current stage is enough
                        let burnTime = (m0 - mf) / fuelFlow

                        printfn "  burn time %.1f s is enough" burnTime

                        burnTime
                    else
                        // current stage is not enough
                        let mfStage = m0 - propellant
                        let dvStage = Tsiolkovsky.deltaV ve m0 mfStage
                        
                        let remaining = nextStage mfStage dvStage

                        burnTimeStage + remaining
                | None ->
                    printfn "No engines in this stage"
                    nextStage m0 0.<m/s>
        
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

        let initialBurnTime = computeBetterBurnTime (node.RemainingDeltaV.As<m/s>()) (floatU <| ship.Mass.As<kg>()) control.CurrentStage // computeBurnTime (node.RemainingDeltaV * 1.<m/s>)

        
        printfn "Overall burn time: %.1f s" initialBurnTime
        
        // 1. Orient ship along burn vector

        let nodeUT = node.UT.As<s>()

        let burnStartT = nodeUT - initialBurnTime * mode.earlyStartFactor

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
            mission.WarpTo(burnStartT - warpHeadroom)
        
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
                    let newBurnTime = computeBetterBurnTime (node.RemainingDeltaV.As<m/s>()) (floatU <| ship.Mass.As<kg>()) control.CurrentStage
                    printfn "Remaining burn time %.1f s" newBurnTime

                let throttle = throttlePid.Update(mission.UniversalTime, - Vec3.mag burnVectorS.Value)
                control.Throttle <- float32U throttle

                yield { forward = Vec3.unitY; top = None }
                
            control.Throttle <- 0.f;
        }

        // 5. Done

        node.Remove()
        
    let executeNext (mission: Mission) (warpHeadroom: float<s>) = executeNextExt mission warpHeadroom { earlyStartFactor = 0.5 }

    let suicideBurn (mission: Mission) =
        let ship = mission.ActiveVessel
        let body = ship.Orbit.Body
        let flight = ship.Flight(body.NonRotatingReferenceFrame)
        
        use altS = mission.Streams.UseStream<m>(fun () -> flight.SurfaceAltitude)
        use thrustS = mission.Streams.UseStream<N>(fun () -> ship.AvailableThrust)
        use massS = mission.Streams.UseStream<kg>(fun () -> ship.Mass)
        use velV = mission.Streams.UseStream<m/s>(fun () -> flight.Velocity)
        use posS = mission.Streams.UseStream<m>(fun () -> ship.Position(body.NonRotatingReferenceFrame))
        use situationS = mission.Streams.UseStream(fun () -> ship.Situation)
        
        AttitudeControl.loop mission body.NonRotatingReferenceFrame <| seq {
            let mutable waiting = true
            while waiting do
                let vertical = Vec3.norm posS.Value
                let accel: float<m/s^2> = floatU thrustS.Value / floatU massS.Value
                let vel = Vec3.dot vertical velV.Value
                let burnDist: float<m> = 0.5 * vel * vel / accel
                printfn "%.1f m        %.1f m    %.1f m/s    %.1f m/s^2" altS.Value burnDist vel accel
                if burnDist > altS.Value + 10.<m> then
                    ship.Control.Throttle <- 1.f
                    waiting <- false
                yield { forward = vertical; top = None }

            while situationS.Value = VesselSituation.Flying do
                let vertical = Vec3.norm posS.Value
                let accel: float<m/s^2> = floatU thrustS.Value / floatU massS.Value
                let vel = Vec3.dot vertical velV.Value
                let burnDist: float<m> = 0.5 * vel * vel / accel
                printfn "%.1f m        %.1f m    %.1f m/s    %.1f m/s^2" altS.Value burnDist vel accel
                if vel >= 0.<_> then
                    ship.Control.Throttle <- 0.f
                yield { forward = vertical; top = None }
        }

