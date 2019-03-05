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

    type LambertSolution =
        { vesselPosition: vec3<m>
        ; vesselVelocity: vec3<m/s>
        ; targetPosition: vec3<m>
        ; targetVelocity: vec3<m/s>
        ; departureVelocity: vec3<m/s>
        ; arrivalVelocity: vec3<m/s>
        ; departureTime: float<s>
        ; flightTime: float<s>
        }
    
    let addLambertNode (mission: Mission) (target: Orbit) (departureTimes: seq<float<s>>) (flightTimes: seq<float<s>>) (evalSolution: LambertSolution -> float): Option<Node> =
        let current = mission.ActiveVessel.Orbit
        assert (current.Body.Name = target.Body.Name)

        let body = current.Body
        let mu = floatU <| body.GravitationalParameter.As<m^3/s^2>()

        let currentElems = current.OrbitalElements
        let targetElems = target.OrbitalElements

        let evaluate tDeparture tArrival =
            let (p1, v1) = Kepler.stateVectors mu currentElems tDeparture
            let (p2, v2) = Kepler.stateVectors mu targetElems tArrival
            let result = Lambert.lambert p1 p2 (tArrival - tDeparture) 0 mu
            match result with
            | Lambert.Solution (transV1, transV2) -> 
                Some {
                    vesselPosition = p1
                    vesselVelocity = v1
                    targetPosition = p2
                    targetVelocity = v2
                    departureVelocity = transV1
                    arrivalVelocity = transV2
                    departureTime = tDeparture
                    flightTime = tArrival - tDeparture
                }
            | _ -> None
        
        let mutable minScore = Double.PositiveInfinity
        let mutable bestSolution: Option<LambertSolution> = None
        let mutable transferBurn: vec3<m/s> = Vec3.zero<_>
                
        printfn "Computing optimal interception trajectory"
        
        for tDep in departureTimes do
            for tFlight in flightTimes do
                let dv = evaluate tDep (tDep + tFlight)
                match dv with
                | None -> ()
                | Some(solution) ->
                    let score = evalSolution solution
                    if score < minScore then
                        bestSolution <- Some(solution)
                        transferBurn <- solution.departureVelocity - solution.vesselVelocity

        match bestSolution with
        | None ->
            printfn "No solution found!"
            None
        | Some(solution) -> 
            let startTime = solution.departureTime
            let rStart, vStart = Kepler.stateVectors mu currentElems startTime
            let dirs = Kepler.maneuverDirections rStart vStart

            let vPrograde = Vec3.dot transferBurn dirs.prograde
            let vRadial = Vec3.dot transferBurn dirs.radial
            let vNormal = Vec3.dot transferBurn dirs.normal
            let f v = float32U (v / 1.<m/s>)

            let node = mission.ActiveVessel.Control.AddNode(solution.departureTime / 1.<s>, prograde=f vPrograde, normal=f vNormal, radial=f vRadial)
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
        