# tailscale-client

Standalone Tailscale VPN node. This is the tailnet connection every other part
of the project rides on — ROS 2 DDS discovery and the Zenoh bridge both reach
peer machines over it.

It is **its own compose project**, not part of the robot stack. Keep it that
way: the robot stack's `docker compose down` must never take the VPN with it,
and a second `tailscale` service defined elsewhere would race a competing
identity onto the same hostname.

## Setup (once per machine)

```bash
cd tailscale-client
cp .env.example .env      # fill in TS_AUTHKEY and TS_HOSTNAME
docker compose up -d
```

Then leave it running. Verify:

```bash
docker exec tailscale tailscale status
```

## Configuration

Everything lives in `.env`, which is gitignored. See `.env.example` for the
three variables and what they do.

`ts-state/` is this machine's node identity and private keys. It is gitignored
too — it must not be shared between machines, or they fight over one identity.

> An earlier version of this compose file (on the `rpi` branch) hardcoded
> `TS_AUTHKEY` inline, which leaked the key into git history. That key was
> revoked. Keys go in `.env` only.

## MagicDNS: `--accept-dns=true` is not enough on its own

`TS_EXTRA_ARGS=--accept-dns=true` makes MagicDNS work **inside this
container** and nowhere else. Verified:

```
# inside the container — works
$ docker exec tailscale getent hosts yahav-rpi-1
100.105.8.88   yahav-rpi-1.tail2d2cb3.ts.net

# on the host — does not
$ getent hosts yahav-rpi-1
(nothing)
```

The reason is that `/etc/resolv.conf` is per **mount** namespace, while
`network_mode: host` shares only the **network** namespace. tailscaled
rewrites the resolv.conf it can see — the one Docker bind-mounts into this
container — so the host and every other container keep their original DNS.
Running tailscale in a container can never push DNS config outward.

Packets are unaffected: `tailscale0` is a real host interface (kernel mode),
so routing to `100.x.y.z` works everywhere. It is only *name lookup* that is
confined.

### Making peer names resolve elsewhere

The MagicDNS resolver at `100.100.100.100` **is** reachable from the host and
from other containers — it is routed over `tailscale0`. Bare names fail only
because the search domain is missing; fully-qualified ones work already:

```
$ dig +short yahav-rpi-1.tail2d2cb3.ts.net @100.100.100.100
100.105.8.88
```

So any container that needs to reach peers by name should point at that
resolver itself. For `zenoh_bridge` in
`AutonomousWarfare/AutonomousWarfare/docker-compose.yml`, whose endpoint list
is written as `-e tcp/shay-laptop:7447` and friends, that means adding:

```yaml
    dns:
      - 100.100.100.100
    dns_search:
      - tail2d2cb3.ts.net
```

Alternatively, drop the DNS dependency entirely and use the `100.x` addresses
directly — Tailscale IPs are stable per node.

Note that static `/etc/hosts` entries on the host do **not** help containers
either: Docker generates a fresh `/etc/hosts` per container.
