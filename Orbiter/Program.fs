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

[<EntryPoint>]
let main argv = 
    use conn = new Connection()
    use mission = new Mission(conn)
    Launch.launch mission Launch.KerbinProfile
    0