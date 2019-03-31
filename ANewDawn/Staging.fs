module ANewDawn.Staging

open KRPC.Client.Services.SpaceCenter
open ANewDawn.Units
open ANewDawn.Control

let hasFlameout (ship: Vessel) = query {
    for engine in ship.Parts.Engines do
    exists (engine.Active && engine.AvailableThrust < 0.1f)
}

/// Determine whether we should activate the next stage
let hasThrust (ship: Vessel) = ship.AvailableThrust >= 0.1f


// TODO: detect more sophisticated staging setups, such as dropping empty tanks etc.

type TwrInfo =
    { thrust: float<N>
    ; isp: float<s>
    ; propellant: float<kg> 
    ; initialMass: float<kg>
    }

/// Get information about the engines that will be in use in the given stage.
let stageEngineInfo (ship: Vessel) stage: Option<TwrInfo> =
    let debug = true

    let engines = Seq.toList <| query {
        for e in ship.Parts.Engines do
        where (e.Part.Stage = stage && e.Part.DecoupleStage < stage)
    }

    let mutable searched = Set.empty
    let mutable dryMass = 0.
    let mutable wetMass = 0.

    let rec scanStageMass (current: Part) =
        if not (searched.Contains current.id) then
            printfn "  * %s ---- %O" current.Name current.Crossfeed
            searched <- searched.Add current.id
            // TODO: needs adjustment for different fuel types
            dryMass <- dryMass + current.DryMass
            wetMass <- wetMass + current.Mass
            // Stop traversing when we reach a decoupler
            if current.Decoupler = null then
                if current.Parent <> null then
                    scanStageMass current.Parent
                for ch in current.Children do
                    scanStageMass ch
        
    for e in engines do
        scanStageMass e.Part

    if debug then
        printfn "Stage %d of %s" stage ship.Name
        printfn "  Wet mass: %.1f kg" wetMass
        printfn "  Dry mass: %.1f kg" dryMass
        printfn "  Engines:"
        for e in engines do
            printfn "  - %s (%.0f N, %.0f s)" e.Part.Name e.MaxVacuumThrust e.VacuumSpecificImpulse
            for p in e.PropellantNames do
                printfn "   %s" p
         

    if engines.IsEmpty then
        None
    else
        let propellantMass = wetMass - dryMass
        let thrustAndIsp = query {
            for e in engines do
            select (floatU <| e.MaxVacuumThrust.As<N>(), floatU <| e.VacuumSpecificImpulse.As<s>())
        }
        let thrust = thrustAndIsp |> Seq.sumBy fst
        let isp = thrust / Seq.sumBy (fun (f, isp) -> f / isp) thrustAndIsp
        Some { thrust = thrust; isp = isp; propellant = propellantMass.As<kg>(); initialMass = System.Double.NaN.As<kg>() }
    

// Depth-first traversal of all parts that are still on the Vessel when in the given stage
let findStageParts (ship: Vessel) stage =
    let mutable searched = Set.empty
    let mutable parts = List.empty

    let rec go (current: Part) =
        if not (searched.Contains current.id) then
            let decoupler = current.Decoupler
            let clamp = current.LaunchClamp
            // If we reach a decoupler or launch clamp, check if it's still coupled in the given stage
            if (decoupler = null && clamp = null)
                || (decoupler <> null && decoupler.Staged && decoupler.Part.Stage < stage)
                || (clamp <> null && clamp.Part.Stage < stage) then
                searched <- searched.Add current.id
                parts <- current :: parts
                for ch in current.Children do
                    go ch

    go ship.Parts.Root

    searched, parts


let computeStageEngineInfo (ship: Vessel) =
    
    let maxStage = ship.Control.CurrentStage

    let mutable emptiedTanks = Set.empty
    let mutable stageInfo = List.empty
    
    for curStage in Seq.rev {0..maxStage} do
        printfn "Processing stage %d" curStage
        // Find all parts of the ship in that stage
        let partIds, parts = findStageParts ship curStage

        // Compute intial mass
        let mInit = query {
            for p in parts do
            sumBy ((if emptiedTanks.Contains p.id then p.DryMass else p.Mass).As<kg>())
        }

        //for p in parts do
        //    printfn "  %s" p.Name

        // Find all active engines in that stage
        let engines = Seq.toList <| query {
            for e in ship.Parts.Engines do
            where (e.Part.Stage >= curStage && partIds.Contains(e.Part.id))
        }

        // Drain reachable tanks
        let mutable drainedPropellantMass = 0.<kg>

        let rec drainTanks (current: Part) =
            if partIds.Contains current.id && not (emptiedTanks.Contains current.id) then
                printfn "  * %s ---- %O" current.Name current.Crossfeed
                emptiedTanks <- emptiedTanks.Add current.id
                // TODO: needs adjustment for different fuel types
                drainedPropellantMass <- drainedPropellantMass + (current.Mass - current.DryMass).As<kg>()
                // Stop traversing when we cannot crossfeed
                if current.Crossfeed then
                    if current.Parent <> null then
                        drainTanks current.Parent
                    for ch in current.Children do
                        drainTanks ch
                    for ch in current.FuelLinesFrom do
                        drainTanks ch

        for e in engines do
            drainTanks e.Part

        let info = 
            if engines.IsEmpty then
                { thrust = 0.<_>; isp = 0.<_>; propellant = 0.<_>; initialMass = mInit }
            else
                let thrustAndIsp = query {
                    for e in engines do
                    select (floatU <| e.MaxVacuumThrust.As<N>(), floatU <| e.VacuumSpecificImpulse.As<s>())
                }
                let thrust = thrustAndIsp |> Seq.sumBy fst
                let isp = thrust / Seq.sumBy (fun (f, isp) -> f / isp) thrustAndIsp
                { thrust = thrust; isp = isp; propellant = drainedPropellantMass; initialMass = mInit }

        stageInfo <- info :: stageInfo

        printfn "  %O" info
    
    List.rev stageInfo
