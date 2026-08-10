#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
 measure_zenoh_latency.py  --  Eye-To-Zion / Autonomous Robot Combat
===============================================================================

PURPOSE
-------
Measure round-trip latency, one-way latency (estimated), packet loss, jitter
and out-of-order delivery across the team-comms path used by this project:

    tactical_brain (rclpy)  ->  DDS domain 0  ->  zenoh-bridge-ros2dds (peer)
                            ->  TCP :7447 over Tailscale  ->  the other node

Nothing about this path has ever been measured in this repo (see
`metrics_report.md`, entry 7: latency / packet loss / recovery time are all
marked "requires measurement"). This script produces the numbers.

It works as a ping/pong pair of rclpy nodes:

    * `--role responder`   subscribes to the ping topic and echoes each
                           message back, byte-for-byte, on the pong topic.
    * `--role initiator`   publishes sequenced + monotonic-timestamped
                           messages on the ping topic, matches replies on the
                           pong topic by sequence number, and computes stats.

Because both timestamps are taken on the initiator's own monotonic clock, NO
clock synchronisation between the two machines is required. That is also why
the primary figure is RTT, not one-way latency.


###############################################################################
#  #1 GOTCHA -- READ THIS OR YOUR TEST WILL SILENTLY MEASURE NOTHING          #
###############################################################################
#                                                                             #
#  The zenoh bridge runs a STRICT ALLOWLIST ("deny all by default").          #
#  Only topics whose fully-qualified name matches the allow regex are ever    #
#  declared to Zenoh -- in EITHER direction. A denied topic does not error,   #
#  does not warn, and does not log. It simply never crosses the bridge, and   #
#  your initiator sits there reporting 100% packet loss forever.              #
#                                                                             #
#    Forward_Command_Post/zenoh/config.json5                                  #
#        publishers / subscribers / services / actions : ["^/teams/.*"]       #
#                                                                             #
#    AutonomousWarfare/.../zenoh_config.json5.template   (ROBOT SIDE)         #
#        publishers / subscribers : ["^/teams/team_@@MY_TEAM_IDX@@/.*$"]      #
#        (@@MY_TEAM_IDX@@ is substituted at container start from the          #
#         MY_TEAM_IDX env var in AutonomousWarfare/AutonomousWarfare/.env,    #
#         currently MY_TEAM_IDX=0)                                            #
#                                                                             #
#  The ROBOT-side allowlist is the NARROWER of the two. It is not enough for  #
#  a test topic to start with /teams/ -- it must start with                   #
#                                                                             #
#        /teams/team_<MY_TEAM_IDX>/                                           #
#                                                                             #
#  e.g. /teams/team_0/latency_ping. A topic like /teams/arena/latency_ping    #
#  passes the command-post allowlist but is DENIED by the robot, so it will   #
#  never make a round trip. /latency_ping, /ping, /test, /scan, /map, /tf,    #
#  /cmd_vel, /odometry/filtered -- all denied, all silent.                    #
#                                                                             #
#  This script defaults to allowlist-compatible topic names and prints a      #
#  loud warning if you override them with something that does not match.      #
#  Heed the warning.                                                          #
#                                                                             #
###############################################################################


OTHER SHARP EDGES (all verified against the checked-in config)
--------------------------------------------------------------
* PUBLISH-RATE CAP. Forward_Command_Post/zenoh/config.json5 sets
  `pub_max_frequencies: ["^/teams/.*=40"]`. Anything above 40 Hz crossing the
  bridge is THROTTLED BY THE BRIDGE -- the dropped messages are the bridge
  doing its job, not the network losing packets. Keep `--rate` well under 40
  (default 10 Hz, which is the real tactical update cadence) or your loss
  figure is meaningless.

* DDS MAY BYPASS ZENOH ENTIRELY. The command-post config sets
  `ros_automatic_discovery_range: "SUBNET"`, and both compose files use
  `network_mode: host` with `ROS_DOMAIN_ID=0`. If the two machines you are
  testing happen to sit on the SAME LAN subnet, plain DDS multicast discovery
  can connect them directly and your "zenoh-tailscale" run will actually be
  measuring raw DDS over the LAN. To be sure you are measuring the Zenoh
  path, put the two machines on different subnets (or verify by stopping the
  bridge container: if messages still flow, you are not on the Zenoh path).

* QoS MUST MATCH ON BOTH ENDS. A BEST_EFFORT publisher and a RELIABLE
  subscriber DO NOT MATCH in DDS -- you get zero messages and no error.
  Pass `--reliable` on BOTH ends or on NEITHER. The script prints which
  reliability it used in every summary, because a loss percentage without
  that label is not interpretable.

* RELATIVE vs ABSOLUTE topic names. tactical_brain/team_comms.py creates its
  publishers with RELATIVE names (`teams/team_0/positions`, no leading
  slash), which resolve to `/teams/team_0/positions` only because those nodes
  run in the root namespace. This script always uses absolute names (leading
  `/`) so no namespace can silently remap them out of the allowlist.


PREREQUISITES
-------------
1. ROS 2 Humble installed and SOURCED on both machines:
       source /opt/ros/humble/setup.bash
   (on the robot, ROS 2 lives inside the `ros2_humble` container -- see
    "RUN COMMANDS" below.)
2. `rclpy` and `std_msgs` importable (they come with the Humble desktop or
   ros-base install).
3. ROS_DOMAIN_ID=0 on both ends -- this is the domain the bridge attaches to
   (`domain: 0` in Forward_Command_Post/zenoh/config.json5).
4. The zenoh bridge container running on BOTH machines:
       robot:        etz_zenoh_bridge   (AutonomousWarfare compose)
       command post: zenoh_bridge       (Forward_Command_Post compose)
   Check with: docker ps --filter name=zenoh
5. Tailscale up and the two nodes able to reach each other on tcp/7447:
       tailscale status
       nc -vz <peer-tailscale-ip-or-magicdns-name> 7447
6. Python 3.8+ (uses only the standard library beyond rclpy).


===============================================================================
 EXACT RUN COMMANDS -- THIS TEST NEEDS TWO MACHINES
===============================================================================

Start the RESPONDER first, wait until it prints "responder ready", then start
the INITIATOR. Both must use the same --team-idx and the same reliability.

--- ON THE ROBOT (Raspberry Pi 5, AutonomousWarfare stack) -------------------

  ROS 2 on the robot lives inside the `ros2_humble` container, so run the
  script in there. The repo is bind-mounted, but measurement_scripts/ is not
  under ./ros2_ws, so copy it in first:

      docker cp /home/yahav/Eye-To-Zion---Autonomous-Robot-Combat/measurement_scripts/measure_zenoh_latency.py \
                ros2_humble:/tmp/measure_zenoh_latency.py

      docker exec -it ros2_humble bash -lc '
          source /opt/ros/humble/setup.bash
          export ROS_DOMAIN_ID=0
          python3 /tmp/measure_zenoh_latency.py --role responder --team-idx 0
      '

  (If you run ROS 2 natively on the Pi instead, drop the docker wrapper and
   just `source /opt/ros/humble/setup.bash` first.)

--- ON THE COMMAND POST (overhead-cam PC) -----------------------------------

      source /opt/ros/humble/setup.bash
      export ROS_DOMAIN_ID=0
      python3 /home/yahav/Eye-To-Zion---Autonomous-Robot-Combat/measurement_scripts/measure_zenoh_latency.py \
          --role initiator \
          --team-idx 0 \
          --layer zenoh-tailscale \
          --count 1000 --rate 10 \
          --csv  /tmp/etz_lat_zenoh_tailscale.csv \
          --summary-csv /tmp/etz_lat_summary.csv

  The roles are interchangeable -- you can just as well put the initiator on
  the robot and the responder on the command post. Run it both ways if you
  suspect an asymmetric path (see the one-way caveat below).


-------------------------------------------------------------------------------
 MEASURING THE THREE LAYERS (run the script three times, compare --layer tags)
-------------------------------------------------------------------------------
`--layer` is a free-text tag recorded in the output and in --summary-csv, so
three runs land as three comparable rows in one file.

  1) dds-local        Both roles on the SAME machine. Measures rclpy + DDS
                      only; the bridge is not involved at all.
                          --layer dds-local
                      (run responder and initiator in two terminals on the
                       command post, or two `docker exec` shells on the robot)

  2) zenoh-lan        Two machines on the same LAN, traffic crossing the
                      Zenoh bridge. Stop Tailscale (or point the bridge's
                      `-e tcp/...` connect endpoints at LAN IPs) so the hop
                      is LAN, not VPN.
                          --layer zenoh-lan

  3) zenoh-tailscale  The real deployment: two machines, bridge to bridge
                      over the 100.x.y.z Tailscale addresses.
                          --layer zenoh-tailscale

Subtracting layer 1 from layer 2 gives the bridge's own cost; subtracting
layer 2 from layer 3 gives the Tailscale/WAN cost.

Payload-size sensitivity: the real messages are small JSON blobs (team_comms.py
publishes ~100-200 byte std_msgs/String payloads). Re-run any layer with
`--payload-bytes 1024` / `4096` / `16384` to see where size starts to matter.


===============================================================================
 EXPECTED OUTPUT FORMAT
===============================================================================
The block below is an ILLUSTRATIVE SAMPLE showing the FORMAT ONLY.
The numbers in it are INVENTED PLACEHOLDERS, not measurements from this
system. Nothing in this repo has measured this path yet -- that is the whole
point of this script. Do not copy these numbers anywhere.

    ================================================================
     ZENOH / DDS LATENCY REPORT   (ILLUSTRATIVE SAMPLE -- FAKE DATA)
    ================================================================
     layer tag ............ zenoh-tailscale
     ping topic ........... /teams/team_0/latency_ping
     pong topic ........... /teams/team_0/latency_pong
     reliability .......... BEST_EFFORT  (history KEEP_LAST, depth 1)
     payload padding ...... 0 bytes
     rate / count ......... 10.000 Hz, 1000 messages (20 warmup discarded)
     per-msg timeout ...... 2.000 s
     wall duration ........ 100.3 s
    ----------------------------------------------------------------
     DELIVERY
       sent ............... 980
       replied ............ 971
       lost ............... 9        ( 0.918 %)
       late (> timeout) ... 0
       duplicates ......... 0
       out-of-order ....... 2
    ----------------------------------------------------------------
     ROUND-TRIP TIME (ms)          ONE-WAY ESTIMATE = RTT/2 (ms)
       min ......   18.412          min ......    9.206
       mean .....   31.087          mean .....   15.543
       median ...   29.640          median ...   14.820
       p95 ......   47.930          p95 ......   23.965
       p99 ......   63.115          p99 ......   31.557
       max ......   88.204          max ......   44.102
       stddev ...    9.771
       jitter ...    6.204   (mean |RTT[i] - RTT[i-1]|, arrival order)
    ----------------------------------------------------------------
     NOTE ON ONE-WAY: the one-way column is simply RTT/2. That assumes a
     SYMMETRIC path -- equal latency in both directions. Over Tailscale
     (DERP relay vs direct, asymmetric uplinks, NAT hairpinning) that
     assumption can be badly wrong. Treat one-way as an ESTIMATE only.
     A true one-way number needs synchronised clocks (PTP/chrony) on both
     machines; this script deliberately does not pretend to have that.
    ----------------------------------------------------------------
     per-sample CSV ....... /tmp/etz_lat_zenoh_tailscale.csv
     summary row appended . /tmp/etz_lat_summary.csv
    ================================================================

Exit status: 0 on a completed run, 1 on a usage/environment error, 2 if the
initiator received ZERO replies (almost always the allowlist gotcha above).

===============================================================================
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys
import threading
import time

DEFAULT_PING_SUFFIX = "latency_ping"
DEFAULT_PONG_SUFFIX = "latency_pong"


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        prog="measure_zenoh_latency.py",
        description=(
            "Measure RTT / one-way-estimate / packet loss / jitter across the "
            "DDS + zenoh-bridge-ros2dds + Tailscale team-comms path. "
            "Run --role responder on one machine and --role initiator on the "
            "other. Topics MUST live under /teams/team_<idx>/ or the bridge "
            "allowlist silently drops them."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  # on the robot (inside the ros2_humble container)\n"
            "  python3 measure_zenoh_latency.py --role responder --team-idx 0\n\n"
            "  # on the command post\n"
            "  python3 measure_zenoh_latency.py --role initiator --team-idx 0 \\\n"
            "      --layer zenoh-tailscale --count 1000 --rate 10 \\\n"
            "      --csv /tmp/etz_lat.csv --summary-csv /tmp/etz_lat_summary.csv\n"
        ),
    )

    p.add_argument(
        "--role", required=True, choices=("initiator", "responder"),
        help="'responder' echoes every ping straight back; 'initiator' sends "
             "sequenced pings, matches the replies and computes the stats.",
    )

    default_idx = os.environ.get("MY_TEAM_IDX", "0")
    p.add_argument(
        "--team-idx", default=default_idx, metavar="N",
        help="Team index used to build the default topic names, matching the "
             "bridge allowlist ^/teams/team_<N>/.*$ . Defaults to the "
             "MY_TEAM_IDX env var if set, else 0. (current default: %(default)s)",
    )
    p.add_argument(
        "--ping-topic", default=None, metavar="TOPIC",
        help="Override the ping topic. Default: /teams/team_<idx>/%s" % DEFAULT_PING_SUFFIX,
    )
    p.add_argument(
        "--pong-topic", default=None, metavar="TOPIC",
        help="Override the pong topic. Default: /teams/team_<idx>/%s" % DEFAULT_PONG_SUFFIX,
    )

    p.add_argument("--count", type=int, default=1000, metavar="N",
                   help="Number of pings the initiator sends (default: %(default)s).")
    p.add_argument("--rate", type=float, default=10.0, metavar="HZ",
                   help="Send rate in Hz. Default %(default)s Hz, matching the real "
                        "tactical update cadence. NOTE: the bridge caps /teams/* at "
                        "40 Hz (pub_max_frequencies), so rates near/above 40 will "
                        "show bridge throttling as if it were packet loss.")
    p.add_argument("--timeout", type=float, default=2.0, metavar="SEC",
                   help="Per-message reply timeout in seconds (default: %(default)s). "
                        "A reply slower than this is counted as lost, and also "
                        "reported separately as 'late'.")
    p.add_argument("--payload-bytes", type=int, default=0, metavar="N",
                   help="Pad each message with N bytes of filler so payload-size "
                        "sensitivity can be measured. 0 (default) = no padding, "
                        "which is closest to the real ~100-200 byte JSON messages.")
    p.add_argument("--warmup", type=int, default=20, metavar="N",
                   help="Discard the first N samples from the statistics to exclude "
                        "discovery / first-packet effects (default: %(default)s). "
                        "They are still written to the per-sample CSV, flagged.")
    p.add_argument("--settle", type=float, default=3.0, metavar="SEC",
                   help="Seconds to wait after creating the pub/sub before the first "
                        "ping, to let DDS + Zenoh discovery complete "
                        "(default: %(default)s). Discovery over Tailscale is not "
                        "instant; too small a value shows up as an initial loss burst.")

    p.add_argument("--layer", default="unspecified", metavar="LABEL",
                   help="Free-text label for the path being measured, recorded in the "
                        "output and CSV. Suggested: dds-local | zenoh-lan | "
                        "zenoh-tailscale (default: %(default)s).")

    p.add_argument("--reliable", action="store_true",
                   help="Use RELIABLE QoS instead of the default BEST_EFFORT. MUST be "
                        "set the same way on BOTH ends -- a BEST_EFFORT publisher and "
                        "a RELIABLE subscriber do not match and you will see zero "
                        "messages with no error. Loss numbers are only interpretable "
                        "alongside this setting, so it is always printed.")
    p.add_argument("--depth", type=int, default=1, metavar="N",
                   help="QoS history depth, KEEP_LAST (default: %(default)s, i.e. "
                        "SensorData-like). Must also match on both ends in spirit; "
                        "a depth of 1 at high rate can overwrite queued samples.")

    p.add_argument("--csv", default=None, metavar="PATH",
                   help="Write one row per sent message (seq, rtt, status) to this CSV.")
    p.add_argument("--summary-csv", default=None, metavar="PATH",
                   help="Append ONE summary row per run to this CSV (created with a "
                        "header if absent). Point all three --layer runs at the same "
                        "file to get a ready-made comparison table.")

    p.add_argument("--quiet", action="store_true",
                   help="Suppress the responder's periodic progress line.")
    return p


# ---------------------------------------------------------------------------
# Topic naming + the allowlist warning
# ---------------------------------------------------------------------------
def resolve_topics(args):
    idx = str(args.team_idx)
    ping = args.ping_topic or "/teams/team_%s/%s" % (idx, DEFAULT_PING_SUFFIX)
    pong = args.pong_topic or "/teams/team_%s/%s" % (idx, DEFAULT_PONG_SUFFIX)
    if not ping.startswith("/"):
        ping = "/" + ping
    if not pong.startswith("/"):
        pong = "/" + pong
    return ping, pong


def warn_about_allowlist(ping, pong, team_idx):
    """Loudly warn if a topic cannot cross the zenoh bridge allowlist.

    Two allowlists are in play and the robot's is the narrower one:
      Forward_Command_Post/zenoh/config.json5   -> ^/teams/.*
      zenoh_config.json5.template (robot side)  -> ^/teams/team_<MY_TEAM_IDX>/.*$
    """
    strict_prefix = "/teams/team_%s/" % team_idx
    problems = []
    for label, topic in (("ping", ping), ("pong", pong)):
        if not topic.startswith("/teams/"):
            problems.append(
                (label, topic, "DENIED by BOTH allowlists -- does not start with /teams/")
            )
        elif not topic.startswith(strict_prefix):
            problems.append(
                (label, topic,
                 "passes the command post (^/teams/.*) but is DENIED by the ROBOT "
                 "allowlist (^/teams/team_%s/.*$)" % team_idx)
            )
    if not problems:
        return

    bar = "!" * 78
    sys.stderr.write("\n%s\n" % bar)
    sys.stderr.write("!! ZENOH BRIDGE ALLOWLIST WARNING -- THIS TEST WILL LIKELY MEASURE NOTHING\n")
    sys.stderr.write("%s\n" % bar)
    for label, topic, why in problems:
        sys.stderr.write("!!  %-4s topic : %s\n" % (label, topic))
        sys.stderr.write("!!             -> %s\n" % why)
    sys.stderr.write("!!\n")
    sys.stderr.write("!!  The bridge runs a deny-all-by-default allowlist. A denied topic is\n")
    sys.stderr.write("!!  never declared to Zenoh in either direction. There is NO error and\n")
    sys.stderr.write("!!  NO log line -- it just never crosses, and you will see 100% loss.\n")
    sys.stderr.write("!!\n")
    sys.stderr.write("!!  Use topics under  %s...  e.g.\n" % strict_prefix)
    sys.stderr.write("!!      %slatency_ping\n" % strict_prefix)
    sys.stderr.write("!!      %slatency_pong\n" % strict_prefix)
    sys.stderr.write("%s\n\n" % bar)
    sys.stderr.flush()
    # Give a human a beat to actually read it before the wall of output.
    time.sleep(2.0)


# ---------------------------------------------------------------------------
# ROS import, deferred so --help works without a sourced environment
# ---------------------------------------------------------------------------
def import_ros():
    try:
        import rclpy  # noqa: F401
        from rclpy.node import Node  # noqa: F401
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)  # noqa: F401
        from std_msgs.msg import String  # noqa: F401
    except ImportError as exc:
        sys.stderr.write(
            "\nERROR: could not import the ROS 2 Python packages (%s).\n\n"
            "The ROS 2 environment does not look sourced. Fix it with:\n\n"
            "    source /opt/ros/humble/setup.bash\n"
            "    export ROS_DOMAIN_ID=0\n\n"
            "On the robot, ROS 2 lives inside the `ros2_humble` container, so run\n"
            "this script in there instead:\n\n"
            "    docker exec -it ros2_humble bash -lc \\\n"
            "        'source /opt/ros/humble/setup.bash && python3 /tmp/%s ...'\n\n"
            % (exc, os.path.basename(__file__))
        )
        sys.exit(1)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    return rclpy, Node, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, String


def make_qos(args, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy):
    qos = QoSProfile(depth=max(1, args.depth))
    qos.history = HistoryPolicy.KEEP_LAST
    qos.durability = DurabilityPolicy.VOLATILE
    qos.reliability = (ReliabilityPolicy.RELIABLE if args.reliable
                       else ReliabilityPolicy.BEST_EFFORT)
    return qos


def reliability_label(args):
    return "RELIABLE" if args.reliable else "BEST_EFFORT"


# ---------------------------------------------------------------------------
# Responder
# ---------------------------------------------------------------------------
def run_responder(args, ping_topic, pong_topic):
    (rclpy, Node, QoSProfile, ReliabilityPolicy, HistoryPolicy,
     DurabilityPolicy, String) = import_ros()

    qos = make_qos(args, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)

    rclpy.init()
    node = rclpy.create_node("etz_latency_responder")

    pub = node.create_publisher(String, pong_topic, qos)
    state = {"n": 0, "last_report": time.monotonic()}

    def on_ping(msg):
        # Echo the payload back BYTE FOR BYTE. The initiator's send timestamp
        # travels inside it, so the initiator never needs this machine's clock
        # and no clock sync is required. Echoing verbatim also preserves the
        # --payload-bytes padding on the return leg, so a size sweep loads
        # both directions equally.
        out = String()
        out.data = msg.data
        pub.publish(out)
        state["n"] += 1
        now = time.monotonic()
        if not args.quiet and now - state["last_report"] >= 5.0:
            node.get_logger().info("echoed %d messages" % state["n"])
            state["last_report"] = now

    node.create_subscription(String, ping_topic, on_ping, qos)

    print("=" * 64)
    print(" ETZ LATENCY RESPONDER")
    print("=" * 64)
    print("  listening on ... %s" % ping_topic)
    print("  echoing to ..... %s" % pong_topic)
    print("  reliability .... %s (KEEP_LAST, depth %d)"
          % (reliability_label(args), max(1, args.depth)))
    print("  ROS_DOMAIN_ID .. %s" % os.environ.get("ROS_DOMAIN_ID", "(unset -> 0)"))
    print("")
    print("  responder ready -- now start the initiator on the other machine.")
    print("  Ctrl-C to stop.")
    print("=" * 64)
    sys.stdout.flush()

    # ExternalShutdownException is what rclpy raises when the process is sent
    # SIGINT/SIGTERM -- i.e. the normal way this responder is stopped. Catch it
    # so a plain Ctrl-C or `kill` exits quietly instead of dumping a traceback
    # that looks like a failure in the middle of a measurement session.
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print("\nresponder stopped after echoing %d messages." % state["n"])
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


# ---------------------------------------------------------------------------
# Initiator
# ---------------------------------------------------------------------------
def run_initiator(args, ping_topic, pong_topic):
    (rclpy, Node, QoSProfile, ReliabilityPolicy, HistoryPolicy,
     DurabilityPolicy, String) = import_ros()

    qos = make_qos(args, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)

    rclpy.init()
    node = rclpy.create_node("etz_latency_initiator")
    pub = node.create_publisher(String, ping_topic, qos)

    lock = threading.Lock()
    sent_at = {}            # seq -> monotonic_ns at publish
    replies = []            # (seq, recv_ns, arrival_index) in ARRIVAL order
    seen_seqs = set()       # O(1) duplicate check, kept in step with `replies`
    dup_count = [0]
    arrival_counter = [0]

    def on_pong(msg):
        # Timestamp FIRST, parse second: keep everything that could be slow
        # out of the measured interval.
        recv_ns = time.monotonic_ns()
        try:
            data = json.loads(msg.data)
            seq = int(data["seq"])
        except (ValueError, KeyError, TypeError):
            return
        with lock:
            if seq not in sent_at:
                return                      # not ours (stale run / other node)
            if seq in seen_seqs:
                dup_count[0] += 1
                return
            seen_seqs.add(seq)
            arrival_counter[0] += 1
            replies.append((seq, recv_ns, arrival_counter[0]))

    node.create_subscription(String, pong_topic, on_pong, qos)

    # Spin in a background thread so the send loop can keep an exact cadence
    # and the receive callback timestamps as close to arrival as possible.
    executor_stop = threading.Event()

    def spin_loop():
        while not executor_stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    spinner = threading.Thread(target=spin_loop, daemon=True)
    spinner.start()

    padding = "x" * max(0, args.payload_bytes)
    period = 1.0 / args.rate if args.rate > 0 else 0.0

    print("=" * 64)
    print(" ETZ LATENCY INITIATOR")
    print("=" * 64)
    print("  layer tag ...... %s" % args.layer)
    print("  ping topic ..... %s" % ping_topic)
    print("  pong topic ..... %s" % pong_topic)
    print("  reliability .... %s (KEEP_LAST, depth %d)"
          % (reliability_label(args), max(1, args.depth)))
    print("  count / rate ... %d messages @ %.3f Hz" % (args.count, args.rate))
    print("  padding ........ %d bytes" % max(0, args.payload_bytes))
    print("  ROS_DOMAIN_ID .. %s" % os.environ.get("ROS_DOMAIN_ID", "(unset -> 0)"))
    if args.rate >= 40.0:
        print("")
        print("  WARNING: --rate %.1f Hz is at or above the bridge's 40 Hz cap for"
              % args.rate)
        print("           /teams/* (pub_max_frequencies in the command post config)."
              )
        print("           Messages the BRIDGE throttles will be counted as LOSS.")
    print("")
    print("  waiting %.1fs for DDS + Zenoh discovery ..." % args.settle)
    sys.stdout.flush()

    deadline = time.monotonic() + args.settle
    while time.monotonic() < deadline:
        time.sleep(0.05)

    print("  sending ...")
    sys.stdout.flush()

    t_wall_start = time.monotonic()
    t0 = time.monotonic()
    for seq in range(args.count):
        payload = {
            "seq": seq,
            # Monotonic clock, NOT wall clock: immune to NTP steps, suspend,
            # and leap seconds. Only ever compared against this same clock.
            "t_send_ns": 0,
            "layer": args.layer,
        }
        if padding:
            payload["pad"] = padding

        # Stamp as late as possible: build the JSON with a placeholder, then
        # substitute the real timestamp so serialisation cost is outside the
        # measured window.
        blob = json.dumps(payload)
        send_ns = time.monotonic_ns()
        blob = blob.replace('"t_send_ns": 0', '"t_send_ns": %d' % send_ns, 1)

        msg = String()
        msg.data = blob
        with lock:
            sent_at[seq] = send_ns
        pub.publish(msg)

        if period:
            # Absolute schedule so the send rate does not drift over a long run.
            target = t0 + (seq + 1) * period
            slack = target - time.monotonic()
            if slack > 0:
                time.sleep(slack)

        if not args.quiet and (seq + 1) % max(1, args.count // 10) == 0:
            with lock:
                got = len(replies)
            print("    sent %d/%d, replies so far %d" % (seq + 1, args.count, got))
            sys.stdout.flush()

    # Drain: give in-flight replies a full timeout window to land.
    print("  draining for %.1fs ..." % args.timeout)
    sys.stdout.flush()
    drain_deadline = time.monotonic() + args.timeout
    while time.monotonic() < drain_deadline:
        time.sleep(0.05)
    wall_duration = time.monotonic() - t_wall_start

    executor_stop.set()
    spinner.join(timeout=2.0)
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

    with lock:
        snapshot_sent = dict(sent_at)
        snapshot_replies = list(replies)
        duplicates = dup_count[0]

    stats = compute_stats(args, snapshot_sent, snapshot_replies, duplicates, wall_duration)
    print_report(args, ping_topic, pong_topic, stats)

    if args.csv:
        write_sample_csv(args, snapshot_sent, snapshot_replies, args.csv)
        print("  per-sample CSV ....... %s" % args.csv)
    if args.summary_csv:
        append_summary_csv(args, ping_topic, pong_topic, stats, args.summary_csv)
        print("  summary row appended . %s" % args.summary_csv)
    print("=" * 64)

    if stats["replied"] == 0:
        sys.stderr.write(
            "\nERROR: ZERO replies received.\n"
            "The overwhelmingly likely cause is the zenoh bridge allowlist:\n"
            "  * is the topic under /teams/team_%s/ on BOTH ends?\n"
            "  * is the responder actually running, and did it print 'ready'?\n"
            "  * do both ends use the same reliability (--reliable or not)?\n"
            "    A BEST_EFFORT publisher never matches a RELIABLE subscriber.\n"
            "  * same ROS_DOMAIN_ID (0) on both ends?\n"
            "  * are both zenoh bridge containers up (docker ps | grep zenoh)\n"
            "    and can they reach each other on tcp/7447 over Tailscale?\n"
            % args.team_idx
        )
        return 2
    return 0


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------
def percentile(sorted_values, pct):
    """Nearest-rank percentile. Deterministic and easy to defend in a report."""
    if not sorted_values:
        return float("nan")
    k = max(1, int(math.ceil(pct / 100.0 * len(sorted_values))))
    return sorted_values[min(k, len(sorted_values)) - 1]


def compute_stats(args, sent_at, replies, duplicates, wall_duration):
    timeout_ns = args.timeout * 1e9
    warmup = max(0, args.warmup)

    # RTT per seq, plus arrival order (needed for jitter + out-of-order).
    per_seq = {}
    by_arrival = []
    for seq, recv_ns, arrival_idx in sorted(replies, key=lambda r: r[2]):
        rtt_ns = recv_ns - sent_at[seq]
        per_seq[seq] = rtt_ns
        by_arrival.append((seq, rtt_ns))

    counted_seqs = [s for s in sorted(sent_at) if s >= warmup]
    sent_counted = len(counted_seqs)

    on_time, late = [], []
    for seq in counted_seqs:
        rtt = per_seq.get(seq)
        if rtt is None:
            continue
        (on_time if rtt <= timeout_ns else late).append(rtt)

    rtts_ms = sorted(v / 1e6 for v in on_time)
    replied = len(on_time)
    lost = sent_counted - replied
    loss_pct = (100.0 * lost / sent_counted) if sent_counted else float("nan")

    # Out-of-order: a reply whose seq is lower than the highest seq already
    # seen. Counted over arrival order, warmup excluded.
    out_of_order = 0
    highest = -1
    for seq, _ in by_arrival:
        if seq < warmup:
            continue
        if seq < highest:
            out_of_order += 1
        else:
            highest = seq

    # Jitter: mean absolute successive difference of RTTs, in ARRIVAL order
    # (that is what a receiver actually experiences).
    diffs = []
    prev = None
    for seq, rtt_ns in by_arrival:
        if seq < warmup or rtt_ns > timeout_ns:
            continue
        cur = rtt_ns / 1e6
        if prev is not None:
            diffs.append(abs(cur - prev))
        prev = cur

    def agg(fn, default=float("nan")):
        try:
            return fn()
        except Exception:
            return default

    return {
        "layer": args.layer,
        "reliability": reliability_label(args),
        "depth": max(1, args.depth),
        "rate_hz": args.rate,
        "count": args.count,
        "warmup": warmup,
        "payload_bytes": max(0, args.payload_bytes),
        "timeout_s": args.timeout,
        "wall_duration_s": wall_duration,
        "sent": sent_counted,
        "replied": replied,
        "lost": lost,
        "loss_pct": loss_pct,
        "late": len(late),
        "duplicates": duplicates,
        "out_of_order": out_of_order,
        "rtt_min_ms": rtts_ms[0] if rtts_ms else float("nan"),
        "rtt_mean_ms": agg(lambda: statistics.fmean(rtts_ms)),
        "rtt_median_ms": agg(lambda: statistics.median(rtts_ms)),
        "rtt_p95_ms": percentile(rtts_ms, 95),
        "rtt_p99_ms": percentile(rtts_ms, 99),
        "rtt_max_ms": rtts_ms[-1] if rtts_ms else float("nan"),
        "rtt_stddev_ms": agg(lambda: statistics.pstdev(rtts_ms)) if len(rtts_ms) > 1 else 0.0,
        "jitter_ms": agg(lambda: statistics.fmean(diffs)) if diffs else float("nan"),
        "_rtts_ms": rtts_ms,
    }


def f(x, width=8, prec=3):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "%*s" % (width, "n/a")
    return "%*.*f" % (width, prec, x)


def print_report(args, ping_topic, pong_topic, s):
    print("")
    print("=" * 64)
    print(" ZENOH / DDS LATENCY REPORT")
    print("=" * 64)
    print("  layer tag ............ %s" % s["layer"])
    print("  ping topic ........... %s" % ping_topic)
    print("  pong topic ........... %s" % pong_topic)
    print("  reliability .......... %s  (history KEEP_LAST, depth %d)"
          % (s["reliability"], s["depth"]))
    print("  payload padding ...... %d bytes" % s["payload_bytes"])
    print("  rate / count ......... %.3f Hz, %d messages (%d warmup discarded)"
          % (s["rate_hz"], s["count"], s["warmup"]))
    print("  per-msg timeout ...... %.3f s" % s["timeout_s"])
    print("  wall duration ........ %.1f s" % s["wall_duration_s"])
    print("-" * 64)
    print("  DELIVERY")
    print("    sent ............... %d" % s["sent"])
    print("    replied ............ %d" % s["replied"])
    print("    lost ............... %d        (%s %%)" % (s["lost"], f(s["loss_pct"], 6, 3)))
    print("    late (> timeout) ... %d" % s["late"])
    print("    duplicates ......... %d" % s["duplicates"])
    print("    out-of-order ....... %d" % s["out_of_order"])
    print("-" * 64)
    print("  ROUND-TRIP TIME (ms)          ONE-WAY ESTIMATE = RTT/2 (ms)")
    for name, key in (("min", "rtt_min_ms"), ("mean", "rtt_mean_ms"),
                      ("median", "rtt_median_ms"), ("p95", "rtt_p95_ms"),
                      ("p99", "rtt_p99_ms"), ("max", "rtt_max_ms")):
        v = s[key]
        half = v / 2.0 if isinstance(v, float) and not math.isnan(v) else float("nan")
        label = ("%s " % name).ljust(10, ".")
        print("    %s %s          %s %s" % (label, f(v, 9), label, f(half, 9)))
    print("    %s %s" % ("stddev ".ljust(10, "."), f(s["rtt_stddev_ms"], 9)))
    print("    %s %s   (mean |RTT[i] - RTT[i-1]|, arrival order)"
          % ("jitter ".ljust(10, "."), f(s["jitter_ms"], 9)))
    print("-" * 64)
    print("  NOTE ON ONE-WAY: the one-way column is simply RTT/2, which ASSUMES A")
    print("  SYMMETRIC PATH -- equal latency in each direction. Over Tailscale that")
    print("  can be badly wrong (DERP relay vs direct connection, asymmetric")
    print("  uplinks, NAT hairpinning). Treat one-way as an ESTIMATE ONLY. A true")
    print("  one-way figure needs synchronised clocks on both machines; this script")
    print("  deliberately does not pretend to have them.")
    print("-" * 64)


def write_sample_csv(args, sent_at, replies, path):
    per_seq = {}
    arrival_of = {}
    for seq, recv_ns, arrival_idx in replies:
        per_seq[seq] = recv_ns
        arrival_of[seq] = arrival_idx
    timeout_ns = args.timeout * 1e9

    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow([
            "layer", "reliability", "rate_hz", "payload_bytes",
            "seq", "warmup", "status", "rtt_ms", "one_way_estimate_ms",
            "send_monotonic_ns", "recv_monotonic_ns", "arrival_index",
        ])
        for seq in sorted(sent_at):
            send_ns = sent_at[seq]
            recv_ns = per_seq.get(seq)
            is_warmup = 1 if seq < max(0, args.warmup) else 0
            if recv_ns is None:
                w.writerow([args.layer, reliability_label(args), args.rate,
                            max(0, args.payload_bytes), seq, is_warmup,
                            "lost", "", "", send_ns, "", ""])
                continue
            rtt_ns = recv_ns - send_ns
            status = "ok" if rtt_ns <= timeout_ns else "late"
            w.writerow([args.layer, reliability_label(args), args.rate,
                        max(0, args.payload_bytes), seq, is_warmup, status,
                        "%.6f" % (rtt_ns / 1e6), "%.6f" % (rtt_ns / 2e6),
                        send_ns, recv_ns, arrival_of.get(seq, "")])


SUMMARY_FIELDS = [
    "timestamp_utc", "layer", "reliability", "qos_depth", "rate_hz",
    "payload_bytes", "count", "warmup", "timeout_s", "ping_topic", "pong_topic",
    "sent", "replied", "lost", "loss_pct", "late", "duplicates", "out_of_order",
    "rtt_min_ms", "rtt_mean_ms", "rtt_median_ms", "rtt_p95_ms", "rtt_p99_ms",
    "rtt_max_ms", "rtt_stddev_ms", "jitter_ms", "one_way_mean_estimate_ms",
    "wall_duration_s",
]


def append_summary_csv(args, ping_topic, pong_topic, s, path):
    exists = os.path.exists(path) and os.path.getsize(path) > 0
    with open(path, "a", newline="") as fh:
        w = csv.writer(fh)
        if not exists:
            w.writerow(SUMMARY_FIELDS)
        mean = s["rtt_mean_ms"]
        one_way = mean / 2.0 if isinstance(mean, float) and not math.isnan(mean) else ""
        w.writerow([
            time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            s["layer"], s["reliability"], s["depth"], s["rate_hz"],
            s["payload_bytes"], s["count"], s["warmup"], s["timeout_s"],
            ping_topic, pong_topic,
            s["sent"], s["replied"], s["lost"], "%.4f" % s["loss_pct"],
            s["late"], s["duplicates"], s["out_of_order"],
            "%.4f" % s["rtt_min_ms"], "%.4f" % s["rtt_mean_ms"],
            "%.4f" % s["rtt_median_ms"], "%.4f" % s["rtt_p95_ms"],
            "%.4f" % s["rtt_p99_ms"], "%.4f" % s["rtt_max_ms"],
            "%.4f" % s["rtt_stddev_ms"],
            ("%.4f" % s["jitter_ms"]) if not math.isnan(s["jitter_ms"]) else "",
            ("%.4f" % one_way) if one_way != "" else "",
            "%.3f" % s["wall_duration_s"],
        ])


# ---------------------------------------------------------------------------
def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.count <= 0:
        sys.stderr.write("ERROR: --count must be > 0\n")
        return 1
    if args.rate <= 0:
        sys.stderr.write("ERROR: --rate must be > 0\n")
        return 1
    if args.warmup >= args.count:
        sys.stderr.write("ERROR: --warmup (%d) must be < --count (%d)\n"
                         % (args.warmup, args.count))
        return 1

    ping_topic, pong_topic = resolve_topics(args)
    warn_about_allowlist(ping_topic, pong_topic, args.team_idx)

    if args.role == "responder":
        return run_responder(args, ping_topic, pong_topic)
    return run_initiator(args, ping_topic, pong_topic)


if __name__ == "__main__":
    sys.exit(main())
