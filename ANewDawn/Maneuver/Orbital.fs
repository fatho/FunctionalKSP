namespace ANewDawn.Maneuver

open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter

open ANewDawn.Units
open ANewDawn.Extensions
open ANewDawn.Math
open ANewDawn.Math.Util
open ANewDawn.Control
open ANewDawn.Mission

type Apsis = Apoapsis | Periapsis

module Orbital =
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
