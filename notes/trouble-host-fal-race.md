# trouble-host: Filter Accept List race between `Scanner::scan` and `Central::connect`

**Repo:** https://github.com/embassy-rs/trouble
**Affects:** `host/src/scan.rs`, `host/src/central.rs` (observed on `dc15b9e`, current `main`)

## Summary

`Scanner::scan` and `Central::connect` both mutate the controller's Filter
Accept List (FAL) via the shared helper `Central::set_accept_filter`, but
they synchronize against two *different* command-state machines
(`scan_command_state` vs `connect_command_state`). Nothing serializes the two
against each other. If a `scan()` call begins while a `connect()` is already
in flight (or vice versa), the FAL gets clobbered mid-flight: `Clear FAL` and
`Add Device To FAL` commands from one path interleave with the other path's
own `Clear`/`Add` sequence.

In the most easily-reproduced failure mode, the in-flight `LeCreateConn`
future never resolves: `LE Enhanced Connection Complete` arrives at the host
but the `connect()` future doesn't complete and the connection is effectively
wedged until something else cancels it.

## Where the race lives

`Scanner::scan` (`host/src/scan.rs`):

```rust
host.scan_command_state.request().await;
self.central.set_accept_filter(config.filter_accept_list).await?;
// ... LeSetScanParams, LeSetScanEnable
```

`Central::connect` (`host/src/central.rs`):

```rust
host.connect_command_state.request().await;
self.set_accept_filter(config.scan_config.filter_accept_list).await?;
host.async_command(LeCreateConn::new(/* initiator_filter_policy = true */ ...)).await?;
// ... await connections.accept(...)
```

`set_accept_filter` itself is non-atomic from the controller's point of view:

```rust
host.command(LeClearFilterAcceptList::new()).await?;
for entry in filter_accept_list {
    host.command(LeAddDeviceToFilterAcceptList::new(entry.0, *entry.1)).await?;
}
```

`scan_command_state` and `connect_command_state` gate their own paths but not
each other, so a typical interleaving looks like:

1. App A: `connect()` — `connect_command_state.request()` taken
2. App A: `Clear FAL`, `Add device 1`, issues `LeCreateConn` (initiator armed)
3. App B (or the same app from a different task): `scan()` — `scan_command_state.request()` taken (independent gate)
4. App B: `Clear FAL`, `Add device 2`, `LeSetScanParams`, `LeSetScanEnable`
5. Controller's FAL no longer matches what either path believes; the
   already-issued `LeCreateConn` is now operating against an FAL that the
   `scan()` path also expects to own. `LE Connection Complete` may still
   arrive (it did in our captures) but the `connect()` future never
   completes.

The bug shows up immediately when a host has a background passive scan loop
*and* a connection task running concurrently, both hitting different bonded
addresses through the same `Stack`.

## Reproduction (sketch)

```rust
let host = stack.build();
let mut central = host.central;
let mut scanner = host.scanner;

// Task A — opportunistic auto-connect via passive scan
loop {
    let _session = scanner.scan(&ScanConfig {
        filter_accept_list: &fal_a,
        active: false, ..Default::default()
    }).await.unwrap();
    let addr = wait_for_advert().await;
    drop(_session); // stops the scan
    spawn_task_b(addr); // hands off to a slot task
}

// Task B — connect to the discovered device
async fn task_b(addr: BdAddr) {
    central.connect(&ConnectConfig {
        scan_config: ScanConfig { filter_accept_list: &[(AddrKind::RANDOM, &addr)], .. },
        ..
    }).await.unwrap(); // <-- can wedge if Task A re-enters scan() before this resolves
}
```

If Task A re-enters `scan()` between Task B's `set_accept_filter` and the
moment the controller delivers `LE Connection Complete`, Task B's
`connect()` future never resolves.

In our captures we see two `LeClearFilterAcceptList` commands and three
`LeAddDeviceToFilterAcceptList` commands (one from the connect path, two from
the re-entered scan path) bracketing the wedged `LeCreateConn`. The
`LE Enhanced Connection Complete` subevent arrives but `connect()` doesn't
return.

## Fix options

1. **Single command-state for FAL access.** Have both `scan()` and
   `connect()` (and the ext variants) take the same gate around the
   `set_accept_filter` + initiator/scanner-enable sequence — e.g. an
   `fal_command_state` that both must hold across the FAL writes and the
   subsequent `LeSetScanEnable` / `LeCreateConn`. This is the smallest
   correctness fix.
2. **Make `set_accept_filter` an internal critical section that also blocks
   the other state.** Either by reusing the connect gate from scan and vice
   versa, or by an explicit mutex around the helper.
3. **Document the requirement** that callers must serialize scan and connect
   externally. This is what we're doing as a workaround (see below) but it's
   a sharp edge — it isn't obvious from the public API that two methods on
   different objects (`Scanner` vs `Central`) need to be mutually exclusive.

## Workaround in our application

We pre-reserve a "connecting" slot synchronously in our manager loop *before*
signalling the slot task to call `central.connect()`, and the manager's
background scan loop gates on `any_slot_connecting()` so it won't re-enter
`Scanner::scan` while a slot's connect is still in flight. This eliminates
the race window in practice but requires application-level discipline that
the trouble-host API doesn't currently advertise as necessary.

## Notes

- We only tested the non-extended path (`Central::connect` +
  `Scanner::scan`), but `connect_ext` has the same structure and the same
  shared `set_accept_filter` call, so we expect the bug to apply identically.
- Hardware: CYW43439 on RP2040, controlled via `bt-hci` over the CYW43 HCI
  channel.
- trouble-host commit: `dc15b9e` (latest `main` at time of writing).
