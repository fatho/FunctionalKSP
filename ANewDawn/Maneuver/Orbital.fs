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

        printfn "Maneuver %.1f s at %.0f m" eta radius

        let mu = float ship.Orbit.Body.GravitationalParameter * 1.<m^3/s^2>
        let bodyR = floatU (ship.Orbit.Body.EquatorialRadius.As<m>())
        let currentSma = ship.Orbit.SemiMajorAxis * 1.<m>
        let newSma = (radius + bodyR + targetAlt) / 2.
        let currentAlt =
            match apsis with
            | Apoapsis -> ship.Orbit.Apoapsis.As<m>() - bodyR
            | Periapsis -> ship.Orbit.Periapsis.As<m>() - bodyR
        let oldSma = (radius + bodyR + currentAlt) / 2.

        printfn "Cur SMA: %.0f m" currentSma
        printfn "Old SMA: %.0f m" oldSma
        printfn "New SMA: %.0f m" newSma
    
        let actualVelocity = VisViva.orbitalVelocity mu radius currentSma 
        let desiredVelocity = VisViva.orbitalVelocity mu radius newSma
                
        printfn "Actual vel : %.0f m/s" actualVelocity
        printfn "Desired vel: %.0f m/s" desiredVelocity

        let deltaV = desiredVelocity - actualVelocity

        printfn "Required deltaV is %.1f m/s" deltaV

        let node = ship.Control.AddNode((mission.UniversalTime + eta) / 1.<s>, prograde=float32U deltaV / 1.f<m/s>)

        node