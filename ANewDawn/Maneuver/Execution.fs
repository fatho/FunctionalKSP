namespace ANewDawn.Maneuver

open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter

open ANewDawn.Units
open ANewDawn.Extensions
open ANewDawn.Math
open ANewDawn.Math.Util
open ANewDawn.Control
open ANewDawn.Mission

module Execution =
    open ANewDawn
    open ANewDawn.Staging

    /// Perform a maneuver that orients the vessel in the given direction, does not perform roll
    let orient (mission: Mission) (ref: ReferenceFrame) (direction: vec3<1>) (thresholdPos: float<deg>) (thresholdVel: float<deg/s>) (timeout: float<s>): float<deg> * float<deg/s> =
        let ship = mission.ActiveVessel
        use angVelS = mission.Streams.UseStream<rad/s>(fun () -> ship.AngularVelocity(ship.Orbit.Body.NonRotatingReferenceFrame))
        use shipFacingS = mission.Streams.UseStream<1>(fun () -> ship.Direction(ref))

        //use steering = new KosSteering()
        
        let tMax = mission.UniversalTime + timeout
        
        let mutable remainingAngle = infinity.As<deg>()
        let mutable remainingAngularVel = infinity.As<deg/s>()
        let controlProfile =
            if ship.Mass.As<kg>() > 30_000.f<kg> then
                AttitudeControl.Heavy
            else
                AttitudeControl.Medium

        AttitudeControl.loopWithProfile mission controlProfile ref <| seq {
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
        printfn "Executing maneuver node fopr %s" mission.ActiveVessel.Name

        let ship = mission.ActiveVessel
        let control = ship.Control
        let node = control.Nodes.[0]

        //let stageInfo stage =
        //    let stageParts = query {
        //        for part in ship.Parts.All do
        //        where (part.DecoupleStage = stage - 1 || (part.Stage = stage && part.DecoupleStage = -1))
        //        select part
        //    }
        //    let engines = query {
        //        for part in stageParts do
        //        let engine = part.Engine
        //        where (engine <> null)
        //        select (floatU <| engine.MaxVacuumThrust.As<N>(), floatU <| engine.VacuumSpecificImpulse.As<s>())
        //    }
        //    printfn "  Parts in stage %d" stage
        //    if Seq.isEmpty engines then
        //        None
        //    else
        //        for p in stageParts do
        //            printfn "    part %s, active %d, decouple %d" p.Name p.Stage p.DecoupleStage
        //        let thrust = engines |> Seq.sumBy fst
        //        let isp = thrust / Seq.sumBy (fun (f, isp) -> f / isp) engines
        //        let propellantMass = query {
        //                for part in stageParts do
        //                sumBy (part.Mass.As<kg>() - part.DryMass.As<kg>())
        //            }

        //        Some (thrust, isp, propellantMass)

        let decoupledDryMass stage = query {
                for part in ship.Parts.All do
                where (part.DecoupleStage = stage)
                sumBy (part.DryMass.As<kg>())
            }

        let rec computeBetterBurnTime (dv: float<m/s>) (stages: List<TwrInfo>): float<s> =
            match stages with
            | [] -> 0.<s>
            | current :: later ->
                let nextStage dvStage =
                    printfn "  dv in this stage = %.1f m/s" dvStage
                    computeBetterBurnTime (dv - dvStage) later
                
                if current.thrust > 0.<N> then
                    let ve = current.isp * 9.80665<m/s^2> // effective exhaust velocity
                    printfn "  dv = %.1f m/s ; Isp = %.1f s ; ve = %.1f m/s ; F = %.1f" dv current.isp ve current.thrust

                    // Tsiolkovsky rocket equation solved for final mass
                    let mf = Tsiolkovsky.finalMass dv ve current.initialMass

                    // compute mass of exhaust per second
                    let fuelFlow: float<kg/s> = current.thrust / ve

                    let burnTimeStage = current.propellant / fuelFlow
                
                    printfn "  m0 = %.1f kg; mf = %.1f kg ; m_prop = %.1f kg ; flow = %.1f " current.initialMass mf current.propellant fuelFlow
                    printfn "  stage burnt out after %.1f s" burnTimeStage

                    if current.initialMass - mf <= current.propellant then
                        // current stage is enough
                        let burnTime = (current.initialMass - mf) / fuelFlow

                        printfn "  burn time %.1f s is enough" burnTime

                        burnTime
                    else
                        // current stage is not enough
                        let mfStage = current.initialMass - current.propellant
                        let dvStage = Tsiolkovsky.deltaV ve current.initialMass mfStage
                        
                        let remaining = nextStage dvStage

                        burnTimeStage + remaining
                else
                    printfn "No engines in this stage"
                    nextStage 0.<m/s>
        
        // 2. Compute initial burn time

        let twrInfo = Staging.computeStageEngineInfo ship
        let initialBurnTime = computeBetterBurnTime (node.RemainingDeltaV.As<m/s>()) twrInfo // computeBurnTime (node.RemainingDeltaV * 1.<m/s>)

        
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

        let controlProfile =
            //if ship.Mass.As<kg>() > 30_000.f<kg> then
            //    AttitudeControl.Heavy
            //else
                AttitudeControl.Medium

        AttitudeControl.loopWithProfile mission controlProfile node.ReferenceFrame <| seq {
            while mission.UniversalTime < burnStartT do
                yield { forward = Vec3.unitY; top = None }

            printfn "Commencing burn"
                
            let isCompleted () = Vec3.mag burnVectorS.Value < 0.05<m/s>
            // TODO: not suitable for very long burns
            let isDeviating () = Vec3.angle burnVectorS.Value initialBurnDir > 45.<deg> / degPerRad
            
            control.Throttle <- 1.f;

            while not (isCompleted () || isDeviating ()) do

                if not (Staging.hasThrust ship) then
                    ship.Control.ActivateNextStage() |> ignore

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

