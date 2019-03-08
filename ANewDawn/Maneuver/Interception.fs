namespace ANewDawn.Maneuver

open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter

open ANewDawn.Units
open ANewDawn.Extensions
open ANewDawn.Math
open ANewDawn.Math.Util
open ANewDawn.Control
open ANewDawn.Mission

module Interception =
    open System
    
    type LambertTimes = 
        { departureTime: float<s> // universal time of departure
        ; flightTime: float<s> // duration of the flight
        }

    type LambertSolution =
        { vesselPosition: vec3<m>
        ; vesselVelocity: vec3<m/s>
        ; targetPosition: vec3<m>
        ; targetVelocity: vec3<m/s>
        ; departureVelocity: vec3<m/s>
        ; arrivalVelocity: vec3<m/s>
        ; times: LambertTimes
        }

    let addLambertNode (mission: Mission) (times: seq<LambertTimes>) (targetAtTime: float<s> -> vec3<m> * vec3<m/s>) (evalSolution: LambertSolution -> float): Option<Node> =
        let current = mission.ActiveVessel.Orbit

        let body = current.Body
        let mu = floatU <| body.GravitationalParameter.As<m^3/s^2>()

        let currentElems = current.OrbitalElements

        let evaluate times =
            let (p1, v1) = Kepler.stateVectors mu currentElems times.departureTime
            let (p2, v2) = targetAtTime (times.departureTime + times.flightTime)
            let result = Lambert.lambert p1 p2 times.flightTime 0 mu
            match result with
            | Lambert.Solution (transV1, transV2) -> 
                Some {
                    vesselPosition = p1
                    vesselVelocity = v1
                    targetPosition = p2
                    targetVelocity = v2
                    departureVelocity = transV1
                    arrivalVelocity = transV2
                    times = times
                }
            | _ -> None
        
        let mutable minScore = Double.PositiveInfinity
        let mutable bestSolution: Option<LambertSolution> = None
        let mutable transferBurn: vec3<m/s> = Vec3.zero<_>
                
        printfn "Computing optimal interception trajectory"
        
        for time in times do
            let dv = evaluate time
            match dv with
            | None -> ()
            | Some(solution) ->
                let score = evalSolution solution
                if score < minScore then
                    minScore <- score
                    bestSolution <- Some(solution)
                    transferBurn <- solution.departureVelocity - solution.vesselVelocity

        match bestSolution with
        | None ->
            printfn "No solution found!"
            None
        | Some(solution) -> 
            printfn "Best solution has score: %.1f" minScore
            printfn "Departure time: %.1f" solution.times.departureTime
            printfn "Flight time: %.1f" solution.times.flightTime

            let startTime = solution.times.departureTime
            let rStart, vStart = Kepler.stateVectors mu currentElems startTime
            let dirs = Kepler.maneuverDirections rStart vStart

            let vPrograde = Vec3.dot transferBurn dirs.prograde
            let vRadial = Vec3.dot transferBurn dirs.radial
            let vNormal = Vec3.dot transferBurn dirs.normal
            let f v = float32U (v / 1.<m/s>)

            let node = mission.ActiveVessel.Control.AddNode(startTime / 1.<s>, prograde=f vPrograde, normal=f vNormal, radial=f vRadial)
            Some(node)

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
        

    let addVesselInterceptionNode (mission: Mission) (target: Orbit) (departureTimeRange: float<s> * float<s>) (flightTimeRange: float<s> * float<s>) =
        assert (mission.ActiveVessel.Orbit.Body.Name = target.Body.Name)
    
        let now = mission.UniversalTime

        let depTimStep = (snd departureTimeRange - fst departureTimeRange) / 2000.
        let flightTimeStep = (snd flightTimeRange - fst flightTimeRange) / 200.

        let times = seq {
            for dtDep in { fst departureTimeRange .. depTimStep .. snd departureTimeRange } do
                for tFlight in { fst flightTimeRange .. flightTimeStep .. snd flightTimeRange } do
                    yield { departureTime = now + dtDep; flightTime = tFlight }
        }

        let body = target.Body
        let mu = floatU <| body.GravitationalParameter.As<m^3/s^2>()

        let targetElems = target.OrbitalElements
        let evaluateTarget time = Kepler.stateVectors mu targetElems time
                
        // score solutions by total delta-V needed for reaching the target orbit
        let evaluateSolution solution =
            let dv1 = solution.departureVelocity - solution.vesselVelocity
            let dv2 = solution.targetVelocity - solution.arrivalVelocity
            let totalDV = Vec3.mag dv1 + Vec3.mag dv2
            // score is unitless
            float totalDV

        addLambertNode mission times evaluateTarget evaluateSolution
   
        
    /// Intercepting another planet is more complicated, because we very much don't want to be in the exact same place as the target planet
    /// at the arrival time. Ideally, we are a few hundred/thousand kilometers apart from its center of mass.
    let addPlanetaryInterceptionNode (mission: Mission) (target: CelestialBody) (departureTimeRange: float<s> * float<s>) (flightTimeRange: float<s> * float<s>) =
    
        // TODO: support more than just polar encounters

        let now = mission.UniversalTime

        let depTimStep = (snd departureTimeRange - fst departureTimeRange) / 200.
        let flightTimeStep = (snd flightTimeRange - fst flightTimeRange) / 200.

        let times = seq {
            for dtDep in { fst departureTimeRange .. depTimStep .. snd departureTimeRange } do
                for tFlight in { fst flightTimeRange .. flightTimeStep .. snd flightTimeRange } do
                    yield { departureTime = now + dtDep; flightTime = tFlight }
        }
                
        let centralBody = target.Orbit.Body
        let mu = floatU <| centralBody.GravitationalParameter.As<m^3/s^2>()

        let targetElems = target.Orbit.OrbitalElements
        let rSoi = floatU <| target.SphereOfInfluence.As<m>()

        let evaluateTarget time =
            let (r, v) = Kepler.stateVectors mu targetElems time
            let normal = Vec3.norm (Vec3.cross r v)
            let prograde = Vec3.norm v
            let radial = Vec3.cross prograde normal
            let insertionAngle = -3.5<deg>
            let progradeAngle = -12.<deg>
            let pd = cosDeg insertionAngle * rSoi * prograde
            let rd = cosDeg insertionAngle * rSoi * radial
            let insertionPos =
                sinDeg insertionAngle * rSoi * normal + pd * cosDeg progradeAngle + rd * sinDeg progradeAngle
            (r + insertionPos, v)
                
        // score solutions by total delta-V needed for reaching the target orbit
        let evaluateSolution solution =
            let dv1 = solution.departureVelocity - solution.vesselVelocity
            let dv2 = solution.targetVelocity - solution.arrivalVelocity
            let totalDV = Vec3.mag dv1 // + Vec3.mag dv2
            // score is unitless
            float totalDV

        addLambertNode mission times evaluateTarget evaluateSolution
        
    /// Add a maneuver node for deorbiting, reaching the given altitude at the desired surface position
    let addSurfaceInterceptionNode (mission: Mission) (latLon: float<deg> * float<deg>) (altitude: float<m>) ((tMin, tMax): float<s> * float<s>): Option<Node> =
        let current = mission.ActiveVessel.Orbit
        let body = current.Body
        let now = mission.UniversalTime
        let mu = floatU <| body.GravitationalParameter.As<m^3/s^2>()

        let rot0 = body.InitialRotation.As<rad>()
        let rotVel = floatU <| body.RotationalSpeed.As<rad/s>()
        let r = floatU <| body.EquatorialRadius.As<m>()
        
        let currentElems = current.OrbitalElements
        let currentPeriod = Kepler.orbitalPeriod mu currentElems.semiMajorAxis
        let orbMin = tMin / currentPeriod
        let orbMax = tMax / currentPeriod
        let orbStep = (orbMax - orbMin) / 300.

        let targetPositionAtT t =
            let (lat, lon) = latLon
            let lonAtT = lon + (rot0 + t * rotVel) * degPerRad
            let dir = { x = cosRad (lonAtT / degPerRad); y = sinRad (lat / degPerRad); z = sinRad (lonAtT / degPerRad) }
            let down = Vec3.unitY
            let vel = r * 1.</rad> * rotVel * Vec3.cross dir down
            dir * (r + altitude), vel
        
        let times = seq {
            for depOrbits in { orbMin .. orbStep .. orbMax } do
                for flightOrbits in { 0.005 .. 0.0005 .. 1. } do
                    yield { departureTime = now + currentPeriod * depOrbits; flightTime = currentPeriod * flightOrbits }
        }

        
        // score solutions by delta-V needed for deorbiting
        let evaluateSolution solution =
            let dv1 = solution.departureVelocity - solution.vesselVelocity
            float (Vec3.mag dv1)

        addLambertNode mission times targetPositionAtT evaluateSolution
            
