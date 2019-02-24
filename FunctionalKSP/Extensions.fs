module FunctionalKSP.Extensions

open KRPC.Client

type RemoteObject with
    member obj.Connection with get(): Connection = downcast obj.connection

let clamp<[<Measure>] 'u> (lo: float<'u>) (hi: float<'u>) (v: float<'u>): float<'u> = min hi (max lo v)