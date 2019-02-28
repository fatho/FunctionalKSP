module ANewDawn.Extensions

open KRPC.Client

type RemoteObject with
    member obj.Connection with get(): Connection = downcast obj.connection
