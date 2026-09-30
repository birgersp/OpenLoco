# Experimental path-based signalling

Path-based signalling (PBS) allows more than one train into a signalled block
when the routes selected by those trains do not cross or merge. It is an
opt-in experimental feature; the original block-signalling code remains the
default and is unchanged when PBS is disabled.

## Enabling the feature

Add the following key to `openloco.yml` and restart OpenLoco:

```yaml
enable_path_based_signals: true
```

While the option is enabled, all railway signals use path reservations. Set the
value to `false` or remove the key to restore normal block signalling. The
option is deliberately not stored in saved games, so enabling it never makes a
save depend on an experimental save-format extension.

For predictable results, change this setting from the title screen rather than
while a game is running.

## Behaviour and current scope

When a train reaches a signal, OpenLoco plans its route up to the next signal,
dead end, or the 256-piece safety limit. It then reserves the physical track
resources used by that route as one atomic operation:

- A train moves only when the complete route can be reserved.
- Routes on the same tile may coexist when their track connection masks do not
  overlap.
- Crossing, merging, following, and head-on routes conflict.
- A train follows the junction choices made while creating its reservation.
- Each track piece is released after the train's tail clears it.
- A crash discards the future route but keeps every track piece under the wreck
  blocked until that wreck is removed. Pickup, sale, reversal, and world reset
  discard the affected reservations.
- Building or removing real track or signals invalidates planned routes.
  Affected trains reserve again at their next track boundary before moving.
- A blocked route is reused for a short retry window. This avoids repeating
  pathfinding every update while still allowing changed orders to replan.

Reservations are derived runtime state and are not added to the Locomotion save
format. After loading, routing history identifies trains already inside a
signal block so they reserve again before their next track boundary. Physical
train occupation is also checked whenever a reservation is made.

This first implementation intentionally omits signal priorities, reserving
through several signals, separate path-signal graphics, and a reservation
overlay. Those features can be added without changing the reservation core.

## Design for maintainers

The implementation is split into two layers:

- `PathSignalReservations` is a small, deterministic reservation table. It
  knows about owners, routes, and geometric conflict masks, but not vehicles,
  map storage, configuration, or global game state. Its behaviour is covered by
  focused unit tests. Claims are indexed by map position and reservations are
  indexed by owner. Conflict checks inspect only claims at the requested map
  positions; tail release and vehicle removal inspect only the affected train's
  route. Bulk topology invalidation visits every stored route step once.
- `PathSignals` adapts Locomotion track pieces and vehicles to that table. It is
  the isolated policy boundary that a future extension API or alternative
  signalling strategy would need to replace or expose.

The existing engine has no facility for native code plugins. Files historically
called `plugin.dat` and `plugin2.dat` are object-index data, not dynamically
loaded game code. PBS therefore lives in the engine for now, behind a disabled-
by-default configuration switch and a narrow adapter interface.

Engine hooks are limited to five responsibilities:

1. Plan and acquire a route when a train reaches a signal.
2. Use the reserved choice when that train reaches a junction.
3. Release track after the tail clears it.
4. Release train-owned state on crashes, deletion, pickup, or reversal.
5. Invalidate and safely rebuild derived state after real track or signal
   topology changes.

Keeping policy and storage out of `VehicleHead.cpp` makes these hooks easy to
remove, replace, or redirect to a future extension API.

## Testing

`PathSignalReservationsTests.cpp` covers independent paths, crossing conflicts,
atomic failure, progressive tail release, owner removal, topology reset, and
preservation of live connection flags. It also exercises overlapping claims,
crash blockers, and bulk route invalidation with many owners. Map-level
integration scenarios should additionally exercise crashes, track and signal
edits, terminus platforms, flat junctions, merges, opposing trains, long trains,
pickup, and save/load before the feature is promoted from experimental status.

For manual acceptance testing, use a copy of a save and verify the following
with PBS both enabled and disabled:

1. Two non-conflicting trains can enter a flat junction at the same time, while
   crossing, merging, following, and opposing trains wait at their signals.
2. A long train keeps every piece protected until its tail clears that piece.
3. A crashed train continues to block its physical footprint, but releases its
   unused route. Removing the wreck then releases the footprint.
4. Picking up, selling, reversing, or deleting a train releases its reservation.
5. Adding or removing track or signals invalidates affected plans without a
   crash, hang, or stale reservation.
6. Saving and loading rebuilds safe runtime reservations without changing the
   save format.
