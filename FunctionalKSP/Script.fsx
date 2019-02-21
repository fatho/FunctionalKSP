// Learn more about F# at http://fsharp.org
// See the 'F# Tutorial' project for more help.
#r "../packages/KRPC.Client.0.4.8/lib/net45/KRPC.Client.dll"
#r "../packages/Google.Protobuf.3.6.1/lib/net45/Google.Protobuf.dll"
#load "Units.fs"
#load "Launch.fs"
#load "Rules.fs"

open FunctionalKSP

open KRPC.Client
open KRPC.Client.Services.SpaceCenter

let orbiter () = 
    /// Make connection
    use conn = new Connection()
    let ksc = conn.SpaceCenter()
    Rules.requireControllableVessel ksc.ActiveVessel

    /// Run stuff
    Launch.launch ksc.ActiveVessel Launch.KerbinProfile

orbiter ()
