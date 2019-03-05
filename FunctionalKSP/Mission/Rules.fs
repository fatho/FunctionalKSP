/// This class contains functions for verifying that I play by my self-imposed rules.
module FunctionalKSP.Mission.Rules

open KRPC.Client;
open KRPC.Client.Services.SpaceCenter
open System.Linq
open System.Threading
open System
open Microsoft.FSharp.Linq

/// Check whether there's a part on the ship that allows code execution.
/// Without such a part, using kRPC would be "cheating"
let isControllableVessel (ship: Vessel): bool =
    query {
        for part in ship.Parts.All do
        for pmod in part.Modules do
        exists (pmod.Name = "kOSProcessor" || (pmod.Name = "ModuleCommand" && pmod.HasField("Hibernation")))
    }

let requireControllableVessel (ship: Vessel) =
    if not (isControllableVessel ship) then invalidOp (sprintf "The vessel %s has no controllable parts" ship.Name)