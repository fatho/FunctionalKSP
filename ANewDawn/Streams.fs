namespace ANewDawn

open KRPC.Client
open KRPC.Schema;
open KRPC.Client.Services.KRPC;
open System

open Units;
open ANewDawn.Math
open System.Collections.Generic

/// Some stream providing values
type IStream<'a> =
    abstract Value: 'a with get
        
// TODO: MAKE THIS THREADSAFE

type RefCount(initial: int) =
    let mutable count = initial

    member this.AddRef(): unit =
        count <- count + 1

    /// Returns true if the last reference was removed
    member this.RemoveRef(): bool =
        if count > 0 then
            count <- count - 1
            count = 0
        else
            invalidOp "refcount is already zero"

type RefCountedStream<'a, 'b when 'b: equality>(stream: Stream<'a>, refcount: RefCount, map: 'a -> 'b, holder: Dictionary<Object, RefCount>) =
    do refcount.AddRef()

    member val UnderlyingStream = stream with get
    member this.Value with get() = stream.Get() |> map

    member this.WaitForChange() =
        lock this.UnderlyingStream.Condition <| fun () ->
            let current = this.Value
            while this.Value = current do
                this.UnderlyingStream.Wait()
            this.Value

    interface IStream<'b> with
        member this.Value with get() = this.Value

    interface IDisposable with
        member this.Dispose() =
            if refcount.RemoveRef() then
                stream.Remove()
                holder.Remove(downcast stream) |> ignore

type StreamId = StreamId of (string)

type Streams(conn: Connection) =
    let streams = new Dictionary<Object, RefCount>()
    let refCountFor (stream: Object) =
        // The Stream objects correctly implement Equals and GetHashCode
        if streams.ContainsKey(stream) then
            streams.[stream]
        else
            let refcount = new RefCount(0)
            streams.Add(stream, refcount)
            refcount
     
    member this.UseStream<[<Measure>] 'u>(expr: System.Linq.Expressions.Expression<Func<float32>>): RefCountedStream<float32, float32<'u>> =
        let stream = conn.AddStream(expr)
        let refcount = refCountFor stream
        new RefCountedStream<_, _>(stream, refcount, LanguagePrimitives.Float32WithMeasure<'u>, streams)
    
    member this.UseStream<[<Measure>] 'u>(expr: System.Linq.Expressions.Expression<Func<float>>): RefCountedStream<float, float<'u>> =
        let stream = conn.AddStream(expr)
        let refcount = refCountFor stream
        new RefCountedStream<_, _>(stream, refcount, LanguagePrimitives.FloatWithMeasure<'u>, streams)
        
    member this.UseStream<[<Measure>]'u>(expr: System.Linq.Expressions.Expression<Func<float * float * float>>): RefCountedStream<float * float * float, vec3<'u>> =
        let stream = conn.AddStream(expr)
        let refcount = refCountFor stream
        new RefCountedStream<_, _>(stream, refcount, Vec3.pack<'u>, streams)
        
    member this.UseStream<[<Measure>]'u>(expr: System.Linq.Expressions.Expression<Func<(float * float * float) * (float * float * float)>>): RefCountedStream<(float * float * float)*(float * float * float), vec3<'u> * vec3<'u>> =
        let stream = conn.AddStream(expr)
        let refcount = refCountFor stream
        new RefCountedStream<_, _>(stream, refcount, (fun (a, b) -> Vec3.pack<'u> a, Vec3.pack<'u> b), streams)

    member this.UseStream<'a when 'a: equality>(expr: System.Linq.Expressions.Expression<Func<'a>>): RefCountedStream<'a, 'a> =
        let stream = conn.AddStream(expr)
        let refcount = refCountFor stream
        new RefCountedStream<_, _>(stream, refcount, id, streams)
        
type MapStream<'a, 'b>(inner: IStream<'a>, map: 'a -> 'b) =
    interface IStream<'b> with
        member this.Value with get() = inner.Value |> map

module StreamExtensions =
    type IStream<'a> with
        member this.Map<'a, 'b>(f: 'a -> 'b) =
            new MapStream<_, _>(this, f)