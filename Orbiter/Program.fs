// Learn more about F# at http://fsharp.org
// See the 'F# Tutorial' project for more help.

open KRPC.Client
open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter

open System.Threading
open System

open ANewDawn.Math
open ANewDawn.Units
open ANewDawn.Mission
open ANewDawn
open ANewDawn.Math.Lambert
open ANewDawn.Math.Kepler

[<EntryPoint>]
let main argv = 
    let kerbinMu = 3.5316000e12<m^3/s^2>
    //let p1 = Vec3.unitX * 700_000.<m>
    //let p2 = -p1  //(Vec3.unitX * cos (-135.<deg/rad> / degPerRad) + Vec3.unitZ * sin (-135.<deg/rad> / degPerRad)) * 685_000.<m> // Vec3.unitZ * 700_000.<m>
    //let result = Lambert.lambert p1 p2 -800.<s> 0 kerbinMu
    //match result with
    //| Solution (v1, v2) ->
    //  printfn "V1 = %O    |V1| = %.1f" v1 (Vec3.mag v1)
    //  printfn "V2 = %O    |V2| = %.1f" v2 (Vec3.mag v2)
    //| NoSolution ->
    //  printfn "No solution"
    //| Failed ->
    //  printfn "Failed"


    let elems = {
        semiMajorAxis = 12_000_000.<m>;
        eccentricity = 0.;
        argumentOfPeriapsis = 0.<rad>;
        longitudeOfAscendingNode = 0.<rad>;
        inclination = 0.<rad>;
        meanAnomalyAtEpoch = 1.7<rad>;
        epoch = 0.<s>;
    }

    let op = 2. * Util.pi / sqrt (kerbinMu / Util.cube elems.semiMajorAxis)

    printfn "%f" op

    let state0 = Kepler.stateVectors kerbinMu elems 0.<s>
    let state1 = Kepler.stateVectors kerbinMu elems (2000. * 138984.376574<s>)

    printfn "%O" state0
    printfn "%O" state1


    //use conn = new Connection()
    //use mission = new Mission(conn)
    //let tgt = mission.SpaceCenter.TargetVessel
    //Maneuver.addInterceptionNode mission tgt.Orbit
    //// Launch.launch mission { Launch.KerbinProfile with MinTurnAlt = 500.<m> }
    //let _transfer: Node = Maneuver.addHohmannNode mission Apsis.Apoapsis 100_000.<m>
    //Maneuver.executeNext mission 30.<s>
    //let _circ: Node = Maneuver.addCircularizationNode mission Apsis.Apoapsis
    //Maneuver.executeNext mission 30.<s>

    0