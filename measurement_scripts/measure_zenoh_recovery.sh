#!/usr/bin/env bash
# =============================================================================
#  measure_zenoh_recovery.sh  --  Eye-To-Zion / Autonomous Robot Combat
# =============================================================================
#
#  PURPOSE
#  -------
#  Measure how long the team-comms link takes to RECOVER after a disconnect.
#
#  The current stack is zenoh-bridge-ros2dds running in `peer` mode (no central
#  broker), bridging DDS domain 0 over tcp/7447 across Tailscale. Recovery time
#  has never been measured in this repo -- metrics_report.md entry 7 lists it
#  as "requires measurement", with only an unverified ~3-10 s guess.
#
#      BASELINE FOR COMPARISON: the SUPERSEDED Redis implementation
#      (ai_vision/redis_bridge/redis_bridge.py, recoverable with
#       `git show fce9305:ai_vision/redis_bridge/redis_bridge.py`) had a
#      HARDCODED 2-SECOND RECONNECT RETRY LOOP. So its recovery time was
#      bounded below by the retry interval and averaged roughly 1 s of dead
#      time plus the reconnect itself -- but it needed a CENTRAL REDIS SERVER,
#      a single point of failure. Zenoh peer mode has no such single point;
#      this script measures what that costs (or saves) in recovery time.
#      Anything this script reports is a real measurement; the 2 s figure is
#      the documented behaviour of the OLD system, not of Zenoh.
#
#  Method: a publisher on the FAR machine streams messages at a steady rate on
#  an allowlisted /teams/ topic. This script subscribes locally, confirms the
#  traffic is flowing, cuts the link, holds the outage, restores the link, and
#  polls until the first message arrives again.
#
#      recovery time      = T(first message after restore) - T(restore issued)
#      blackout duration  = T(first message after restore) - T(cut issued)
#                           (= outage hold + recovery + command latency)
#
#  Both are reported, per trial and aggregated.
#
#
# #############################################################################
# #  #1 GOTCHA -- THE ZENOH ALLOWLIST WILL SILENTLY EAT YOUR PROBE TOPIC      #
# #############################################################################
# #                                                                           #
# #  The bridge runs a STRICT DENY-ALL-BY-DEFAULT ALLOWLIST. A topic that     #
# #  does not match is never declared to Zenoh in EITHER direction -- no      #
# #  error, no warning, no log line. It just never crosses, and this script   #
# #  will report "traffic never established" no matter how healthy the link.  #
# #                                                                           #
# #    Forward_Command_Post/zenoh/config.json5                                #
# #        publishers/subscribers/services/actions : ["^/teams/.*"]           #
# #                                                                           #
# #    AutonomousWarfare/.../zenoh_config.json5.template   (ROBOT SIDE)       #
# #        publishers/subscribers : ["^/teams/team_@@MY_TEAM_IDX@@/.*$"]      #
# #        (rendered at container start from MY_TEAM_IDX in .env; today 0)    #
# #                                                                           #
# #  The ROBOT side is the NARROWER of the two, so the probe topic must be    #
# #  under  /teams/team_<MY_TEAM_IDX>/  -- not merely under /teams/.          #
# #  /teams/arena/probe passes the command post and is DENIED by the robot.   #
# #  /probe, /test, /scan, /map, /tf, /cmd_vel, /odometry/filtered: denied.   #
# #                                                                           #
# #  This script defaults to /teams/team_${MY_TEAM_IDX}/recovery_probe and    #
# #  warns loudly if you override it with anything that cannot cross.         #
# #                                                                           #
# #############################################################################
#
#
#  #####  SAFETY -- READ BEFORE USING --method tailscale / iptables / link  ####
#
#  Three of the four disconnect methods cut real network paths on THIS machine.
#  If you are connected over that same path -- SSH'd into the robot over
#  Tailscale, which is the normal way to work on it -- YOU WILL CUT YOUR OWN
#  SESSION and lose control of the machine mid-test.
#
#      container   SAFE (default). Only stops/starts the bridge container.
#                  Your SSH session is untouched.
#      tailscale    DANGEROUS. `tailscale down` kills every tailnet connection,
#                  including your SSH session.
#      iptables    DANGEROUS. DROPs tcp/7447 both ways. Safe for SSH *unless*
#                  your session rides that port, but a botched rule or a
#                  Ctrl-C at the wrong moment can strand the host.
#      link        DANGEROUS. `ip link set tailscale0 down` takes the whole
#                  VPN interface down, killing SSH over Tailscale.
#
#  Because of that, tailscale / iptables / link all REQUIRE the explicit
#  `--i-understand` flag. Run them from a LOCAL console, a serial console, or
#  an SSH session over a DIFFERENT path (LAN IP, not the tailnet).
#
#  The link is ALWAYS restored from an EXIT trap, so Ctrl-C, a failure, or a
#  killed terminal will not leave the network broken. (A hard `kill -9` of
#  this script still can -- restore manually with the commands the script
#  prints at startup.)
#
#
#  PREREQUISITES
#  -------------
#  * bash 4+, coreutils, awk. `date +%s%N` for nanosecond timestamps.
#  * ROS 2 Humble sourced (or --ros-container to run ros2 inside a container).
#  * ROS_DOMAIN_ID=0 -- the domain the bridge attaches to.
#  * The zenoh bridge container running on BOTH machines:
#        robot        -> etz_zenoh_bridge   (AutonomousWarfare compose)
#        command post -> zenoh_bridge       (Forward_Command_Post compose)
#  * A steady publisher on the FAR machine (command given below).
#  * method=container : docker installed, the container present.
#  * method=tailscale : tailscale CLI, already authenticated.
#  * method=iptables  : iptables, root or PASSWORDLESS sudo.
#  * method=link      : iproute2, root or PASSWORDLESS sudo, the iface exists.
#
#    Passwordless sudo is REQUIRED (checked up front) for the privileged
#    methods: a sudo password prompt in the middle of a timed loop would sit
#    there blocking and corrupt the measurement.
#
#
# =============================================================================
#  EXACT RUN COMMANDS -- THIS TEST NEEDS TWO MACHINES
# =============================================================================
#
#  Machine A = the FAR end. It just publishes, continuously, and is never cut.
#  Machine B = the machine whose link gets cut. THIS SCRIPT RUNS ON MACHINE B.
#
#  --- ON MACHINE A (say the COMMAND POST / overhead-cam PC) -----------------
#
#      source /opt/ros/humble/setup.bash
#      export ROS_DOMAIN_ID=0
#      ros2 topic pub -r 10 /teams/team_0/recovery_probe std_msgs/msg/String \
#          "{data: 'etz-recovery-probe'}"
#
#      Leave that running for the whole test. 10 Hz matches the real tactical
#      cadence and is comfortably under the bridge's 40 Hz /teams/* cap
#      (pub_max_frequencies in Forward_Command_Post/zenoh/config.json5).
#
#  --- ON MACHINE B (say the ROBOT / Raspberry Pi 5) -------------------------
#
#      # ROS 2 on the robot lives inside the `ros2_humble` container, so tell
#      # the script to run its `ros2 topic echo` in there with --ros-container.
#
#      # 1) SAFE default -- stop/start the bridge container:
#      bash /home/yahav/Eye-To-Zion---Autonomous-Robot-Combat/measurement_scripts/measure_zenoh_recovery.sh \
#          --method container \
#          --container etz_zenoh_bridge \
#          --ros-container ros2_humble \
#          --team-idx 0 --repeats 5 --outage-seconds 10 \
#          --csv /tmp/etz_recovery_container.csv
#
#      # 2) Real network cut (DANGEROUS -- local console only):
#      bash .../measure_zenoh_recovery.sh --method iptables --i-understand \
#          --ros-container ros2_humble --repeats 5 --csv /tmp/etz_rec_ipt.csv
#
#      # 3) Full VPN drop (DANGEROUS -- local console only):
#      bash .../measure_zenoh_recovery.sh --method tailscale --i-understand \
#          --ros-container ros2_humble --repeats 5 --csv /tmp/etz_rec_ts.csv
#
#  If MACHINE B is the command post instead, drop --ros-container (ROS 2 is on
#  the host there) and use `--container zenoh_bridge` -- the command post's
#  bridge container has a different name than the robot's.
#
#  The four methods answer different questions and are worth running all four:
#  `container` = how fast does the bridge process itself re-peer; `iptables` =
#  how fast does a black-holed TCP session recover; `tailscale`/`link` = how
#  fast does the whole VPN plus the bridge on top of it come back.
#
#
# =============================================================================
#  EXPECTED OUTPUT FORMAT
# =============================================================================
#  The block below is an ILLUSTRATIVE SAMPLE showing the FORMAT ONLY.
#  Every number in it is an INVENTED PLACEHOLDER, not a measurement from this
#  system -- nothing here has ever been measured, which is why this script
#  exists. Do not copy these numbers into any report.
#
#    ==============================================================
#     ZENOH LINK RECOVERY   (ILLUSTRATIVE SAMPLE -- FAKE DATA)
#    ==============================================================
#     method .............. container (docker stop/start etz_zenoh_bridge)
#     probe topic ......... /teams/team_0/recovery_probe
#     outage hold ......... 10 s
#     trials .............. 5
#    --------------------------------------------------------------
#      trial | cut ok | recovery (s) | blackout (s) | note
#      ------+--------+--------------+--------------+---------------
#          1 |  yes   |        3.114 |       13.286 | -
#          2 |  yes   |        2.902 |       13.041 | -
#          3 |  yes   |        4.517 |       14.660 | -
#          4 |  yes   |        3.008 |       13.155 | -
#          5 |  yes   |        3.221 |       13.370 | -
#    --------------------------------------------------------------
#     RECOVERY (restore -> first message), 5/5 trials:
#       min ....  2.902 s
#       mean ...  3.352 s
#       max ....  4.517 s
#     BLACKOUT (cut -> first message), includes the 10 s hold:
#       min .... 13.041 s
#       mean ... 13.502 s
#       max .... 14.660 s
#    --------------------------------------------------------------
#     BASELINE: the superseded Redis bridge retried every 2.000 s
#     (hardcoded loop, redis_bridge.py @ fce9305) -- but required a
#     central Redis server as a single point of failure. Zenoh peer
#     mode has none. Compare the mean above against 2.000 s.
#    --------------------------------------------------------------
#     CSV written ......... /tmp/etz_recovery_container.csv
#    ==============================================================
#
#  Exit status: 0 on a completed run, 1 on usage/prerequisite failure,
#  2 if traffic never established at all (almost always the allowlist gotcha).
#
# =============================================================================

set -euo pipefail

# -----------------------------------------------------------------------------
# Defaults
# -----------------------------------------------------------------------------
METHOD="container"
REPEATS=5
OUTAGE_SECONDS=10
TEAM_IDX="${MY_TEAM_IDX:-0}"
TOPIC=""
CONTAINER="etz_zenoh_bridge"
ROS_CONTAINER=""
IFACE="tailscale0"
PORT=7447
SETTLE_TIMEOUT=60          # max wait for traffic to appear before a trial
RECOVERY_TIMEOUT=180       # max wait for the first message after restore
CUT_VERIFY_SECONDS=5       # how long to watch for traffic to actually stop
SETTLE_BETWEEN=5           # pause between trials
POLL_INTERVAL=0.05         # how often to check for new messages
CSV_PATH=""
I_UNDERSTAND=0
DRY_RUN=0
TS_UP_ARGS=""

WORKDIR=""
ECHO_PID=""
LINK_IS_CUT=0
IPT_RULES_ADDED=0

RED=""; YEL=""; GRN=""; BLD=""; RST=""
if [[ -t 1 ]]; then
  RED=$'\033[31m'; YEL=$'\033[33m'; GRN=$'\033[32m'; BLD=$'\033[1m'; RST=$'\033[0m'
fi

usage() {
  cat <<'EOF'
measure_zenoh_recovery.sh -- measure zenoh/Tailscale link recovery time.

Runs on the machine whose link gets cut. A steady publisher must already be
running on the OTHER machine (see the header of this file for the exact
`ros2 topic pub` command).

OPTIONS
  --method M          Disconnect method (default: container)
                        container  SAFE. docker stop/start the bridge container.
                        tailscale  DANGEROUS. `tailscale down` / `tailscale up`.
                        iptables   DANGEROUS. DROP tcp/<port> in+out, then flush.
                        link       DANGEROUS. `ip link set <iface> down/up`.
                      The three dangerous methods can cut your own SSH session
                      and REQUIRE --i-understand.
  --i-understand      Acknowledge that the chosen method can cut your own
                      connection to this machine. Required for tailscale,
                      iptables and link.
  --repeats N         Number of disconnect trials (default: 5).
  --outage-seconds N  How long to hold the link down each trial (default: 10).
  --team-idx N        Team index for the default probe topic. Defaults to the
                      MY_TEAM_IDX env var, else 0.
  --topic T           Override the probe topic. MUST be under
                      /teams/team_<idx>/ or the bridge allowlist drops it.
                      (default: /teams/team_<idx>/recovery_probe)
  --container NAME    Bridge container for --method container.
                      robot: etz_zenoh_bridge (default)
                      command post: zenoh_bridge
  --ros-container N   Run the `ros2 topic echo` inside this docker container
                      instead of on the host. Needed on the robot, where ROS 2
                      lives in the `ros2_humble` container.
  --iface NAME        Interface for --method link (default: tailscale0).
  --port N            TCP port for --method iptables (default: 7447).
  --tailscale-up-args "..."   Extra args appended to `tailscale up` on restore.
  --settle-timeout N  Max seconds to wait for traffic before a trial (default: 60).
  --recovery-timeout N  Max seconds to wait for recovery (default: 180).
  --cut-verify-seconds N  Seconds to confirm traffic really stopped (default: 5).
  --settle-between N  Seconds to pause between trials (default: 5).
  --csv PATH          Write per-trial results to this CSV.
  --dry-run           Print the cut/restore commands without running them.
  -h, --help          This help.

EXAMPLES
  # safe default, on the robot
  ./measure_zenoh_recovery.sh --method container --container etz_zenoh_bridge \
      --ros-container ros2_humble --repeats 5 --csv /tmp/rec.csv

  # real network cut -- LOCAL CONSOLE ONLY
  ./measure_zenoh_recovery.sh --method iptables --i-understand --repeats 5
EOF
}

log()  { printf '%s\n' "$*"; }
info() { printf '  %s\n' "$*"; }
warn() { printf '%s\n' "${YEL}WARNING: $*${RST}" >&2; }
die()  { printf '%s\n' "${RED}ERROR: $*${RST}" >&2; exit 1; }

# Nanosecond epoch. Integer arithmetic only -- no bc dependency, and no
# locale-dependent decimal separator to trip over.
now_ns() { date +%s%N; }

# Format an integer nanosecond delta as seconds with 3 decimals.
ns_to_s() {
  local ns="$1"
  local ms=$(( ns / 1000000 ))
  printf '%d.%03d' $(( ms / 1000 )) $(( ms % 1000 ))
}

# -----------------------------------------------------------------------------
# Arguments
# -----------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --method)               METHOD="${2:?--method needs a value}"; shift 2 ;;
    --i-understand)         I_UNDERSTAND=1; shift ;;
    --repeats)              REPEATS="${2:?}"; shift 2 ;;
    --outage-seconds)       OUTAGE_SECONDS="${2:?}"; shift 2 ;;
    --team-idx)             TEAM_IDX="${2:?}"; shift 2 ;;
    --topic)                TOPIC="${2:?}"; shift 2 ;;
    --container)            CONTAINER="${2:?}"; shift 2 ;;
    --ros-container)        ROS_CONTAINER="${2:?}"; shift 2 ;;
    --iface)                IFACE="${2:?}"; shift 2 ;;
    --port)                 PORT="${2:?}"; shift 2 ;;
    --tailscale-up-args)    TS_UP_ARGS="${2:?}"; shift 2 ;;
    --settle-timeout)       SETTLE_TIMEOUT="${2:?}"; shift 2 ;;
    --recovery-timeout)     RECOVERY_TIMEOUT="${2:?}"; shift 2 ;;
    --cut-verify-seconds)   CUT_VERIFY_SECONDS="${2:?}"; shift 2 ;;
    --settle-between)       SETTLE_BETWEEN="${2:?}"; shift 2 ;;
    --csv)                  CSV_PATH="${2:?}"; shift 2 ;;
    --dry-run)              DRY_RUN=1; shift ;;
    -h|--help)              usage; exit 0 ;;
    *) die "unknown option '$1' (try --help)" ;;
  esac
done

case "$METHOD" in
  container|tailscale|iptables|link) ;;
  *) die "--method must be one of: container, tailscale, iptables, link" ;;
esac

[[ "$REPEATS" =~ ^[0-9]+$ && "$REPEATS" -ge 1 ]] || die "--repeats must be a positive integer"
[[ "$OUTAGE_SECONDS" =~ ^[0-9]+$ ]] || die "--outage-seconds must be an integer"

if [[ -z "$TOPIC" ]]; then
  TOPIC="/teams/team_${TEAM_IDX}/recovery_probe"
fi
[[ "$TOPIC" == /* ]] || TOPIC="/$TOPIC"

# -----------------------------------------------------------------------------
# The allowlist warning -- the single most common way this test "fails"
# -----------------------------------------------------------------------------
STRICT_PREFIX="/teams/team_${TEAM_IDX}/"
if [[ "$TOPIC" != /teams/* ]]; then
  cat >&2 <<EOF
${RED}
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!! ZENOH BRIDGE ALLOWLIST WARNING -- THIS TEST WILL MEASURE NOTHING
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!  probe topic : ${TOPIC}
!!  This does NOT start with /teams/ , so it is DENIED by the bridge
!!  allowlist on BOTH machines. It is never declared to Zenoh, in either
!!  direction -- silently, with no error and no log line. It will never
!!  cross the link, so recovery can never be detected.
!!
!!  Use a topic under ${STRICT_PREFIX} , e.g. ${STRICT_PREFIX}recovery_probe
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
${RST}
EOF
  sleep 3
elif [[ "$TOPIC" != ${STRICT_PREFIX}* ]]; then
  cat >&2 <<EOF
${YEL}
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!! ZENOH BRIDGE ALLOWLIST WARNING
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!  probe topic : ${TOPIC}
!!  This matches the COMMAND POST allowlist (^/teams/.*) but NOT the ROBOT
!!  allowlist (^/teams/team_${TEAM_IDX}/.*\$), which is the narrower of the two.
!!  If either end of this test is the robot, the topic will silently never
!!  cross the bridge.
!!
!!  Prefer ${STRICT_PREFIX}recovery_probe
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
${RST}
EOF
  sleep 3
fi

# -----------------------------------------------------------------------------
# Safety gate for the dangerous methods
# -----------------------------------------------------------------------------
# NOTE: `link` is gated here too. It is not one of the two methods the brief
# named, but `ip link set tailscale0 down` is every bit as capable of killing
# an SSH session over the tailnet as `tailscale down` is, so it gets the same
# guard rather than a weaker one.
if [[ "$METHOD" != "container" ]]; then
  if [[ "$I_UNDERSTAND" -ne 1 ]]; then
    cat >&2 <<EOF
${RED}${BLD}
################################################################################
#  REFUSING TO RUN --method ${METHOD} WITHOUT --i-understand
################################################################################
#
#  This method cuts a REAL NETWORK PATH on THIS machine:
#
#    tailscale : \`tailscale down\` drops every tailnet connection.
#    iptables  : DROPs all tcp/${PORT} traffic in and out.
#    link      : \`ip link set dev ${IFACE} down\` downs the whole VPN interface.
#
#  IF YOU ARE SSH'd INTO THIS MACHINE OVER TAILSCALE, THE tailscale AND link
#  METHODS WILL CUT YOUR OWN SESSION AND YOU WILL LOSE CONTROL OF THE HOST
#  MID-TEST. The iptables method will do the same if your session rides
#  tcp/${PORT}.
#
#  Run these from a local console, a serial console, or an SSH session over a
#  DIFFERENT path (the LAN IP, not the 100.x.y.z tailnet address).
#
#  The link is always restored from an EXIT trap, so Ctrl-C or a crash will
#  not leave the network broken. A \`kill -9\` of this script still could;
#  the manual restore command is printed at startup.
#
#  If you have read all of that and still want to proceed, re-run with
#  --i-understand appended.
#
#  Or just use the safe default:  --method container
################################################################################
${RST}
EOF
    exit 1
  fi
  printf '%s\n' "${YEL}${BLD}"
  printf '%s\n' "################################################################"
  printf '%s\n' "#  RUNNING DANGEROUS METHOD: ${METHOD}"
  printf '%s\n' "#  This can cut your own connection to this machine."
  printf '%s\n' "#  Manual restore if this script is killed with -9:"
  case "$METHOD" in
    tailscale) printf '%s\n' "#      sudo tailscale up ${TS_UP_ARGS}" ;;
    iptables)  printf '%s\n' "#      sudo iptables -D INPUT  -p tcp --dport ${PORT} -j DROP   (x4, see below)" ;;
    link)      printf '%s\n' "#      sudo ip link set dev ${IFACE} up" ;;
  esac
  printf '%s\n' "#  Starting in 5 seconds -- Ctrl-C now to abort."
  printf '%s\n' "################################################################"
  printf '%s\n' "${RST}"
  sleep 5
fi

# -----------------------------------------------------------------------------
# Privilege helper. Passwordless sudo is mandatory for the privileged methods:
# a password prompt landing in the middle of a timed loop would block and
# silently corrupt the measurement.
# -----------------------------------------------------------------------------
SUDO=""
need_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    SUDO=""
    return
  fi
  command -v sudo >/dev/null 2>&1 || die "not root and sudo is not installed; --method ${METHOD} needs privileges"
  if ! sudo -n true 2>/dev/null; then
    die "--method ${METHOD} needs PASSWORDLESS sudo (a password prompt mid-run would block the timing loop and corrupt the measurement).
       Either run this script as root, or configure NOPASSWD sudo for the commands it uses,
       or use the safe default: --method container"
  fi
  SUDO="sudo"
}

# -----------------------------------------------------------------------------
# Prerequisite checks -- fail fast, before anything is cut
# -----------------------------------------------------------------------------
check_prereqs() {
  command -v awk  >/dev/null 2>&1 || die "awk not found"
  command -v date >/dev/null 2>&1 || die "date not found"

  if [[ -n "$ROS_CONTAINER" ]]; then
    command -v docker >/dev/null 2>&1 || die "--ros-container given but docker is not installed"
    docker inspect "$ROS_CONTAINER" >/dev/null 2>&1 \
      || die "--ros-container '$ROS_CONTAINER' does not exist. Running containers: $(docker ps --format '{{.Names}}' | tr '\n' ' ')"
    [[ "$(docker inspect -f '{{.State.Running}}' "$ROS_CONTAINER")" == "true" ]] \
      || die "--ros-container '$ROS_CONTAINER' exists but is not running. Start it: docker start $ROS_CONTAINER"
  else
    command -v ros2 >/dev/null 2>&1 || die "\`ros2\` not on PATH. Either source the ROS environment:
           source /opt/ros/humble/setup.bash
           export ROS_DOMAIN_ID=0
       or, if ROS 2 lives in a container on this machine (it does on the robot),
       pass:  --ros-container ros2_humble"
  fi

  case "$METHOD" in
    container)
      command -v docker >/dev/null 2>&1 || die "docker not found (needed for --method container)"
      docker inspect "$CONTAINER" >/dev/null 2>&1 || die "container '$CONTAINER' does not exist.
       Robot uses        : etz_zenoh_bridge
       Command post uses : zenoh_bridge
       Currently running : $(docker ps --format '{{.Names}}' | tr '\n' ' ')"
      [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" == "true" ]] \
        || die "container '$CONTAINER' exists but is not running. Start it first: docker start $CONTAINER"
      # restart:always in both compose files means docker will not fight us --
      # `docker stop` is respected until an explicit `docker start`.
      ;;
    tailscale)
      command -v tailscale >/dev/null 2>&1 || die "tailscale CLI not found (needed for --method tailscale)"
      need_root
      $SUDO tailscale status >/dev/null 2>&1 \
        || warn "\`tailscale status\` did not report cleanly. If tailscaled is not up and authenticated, \`tailscale up\` may not restore the link and the EXIT trap cannot help you."
      ;;
    iptables)
      command -v iptables >/dev/null 2>&1 || die "iptables not found (needed for --method iptables)"
      need_root
      $SUDO iptables -L INPUT -n >/dev/null 2>&1 || die "cannot read the iptables INPUT chain (permissions? nftables-only host?)"
      ;;
    link)
      command -v ip >/dev/null 2>&1 || die "iproute2 (\`ip\`) not found (needed for --method link)"
      need_root
      ip link show "$IFACE" >/dev/null 2>&1 || die "interface '$IFACE' does not exist. Available: $(ip -o link show | awk -F': ' '{print $2}' | tr '\n' ' ')"
      ;;
  esac
}

# -----------------------------------------------------------------------------
# Cut / restore. Both must be IDEMPOTENT -- restore() runs from the EXIT trap
# and may be called when nothing is actually cut.
# -----------------------------------------------------------------------------
IPT_COMMENT="etz-recovery-test"

cut_link() {
  if [[ "$DRY_RUN" -eq 1 ]]; then
    log "  [dry-run] would CUT via ${METHOD}"
    LINK_IS_CUT=1
    return
  fi
  case "$METHOD" in
    container)
      # -t 2 bounds the SIGTERM grace period so the actual cut happens close
      # to the moment we record, instead of up to docker's 10 s default later.
      docker stop -t 2 "$CONTAINER" >/dev/null
      ;;
    tailscale)
      $SUDO tailscale down >/dev/null 2>&1 || true
      ;;
    iptables)
      # DROP, not REJECT: a REJECT sends an RST and lets TCP fail immediately,
      # which would measure "how fast does a cleanly refused connection come
      # back" rather than a real link failure. DROP black-holes the traffic
      # the way a dead link does, so the peer has to time out.
      local chain spec
      for chain in INPUT OUTPUT; do
        for spec in "--dport" "--sport"; do
          $SUDO iptables -I "$chain" -p tcp "$spec" "$PORT" \
                -m comment --comment "$IPT_COMMENT" -j DROP
        done
      done
      IPT_RULES_ADDED=1
      ;;
    link)
      $SUDO ip link set dev "$IFACE" down
      ;;
  esac
  LINK_IS_CUT=1
}

restore_link() {
  if [[ "$DRY_RUN" -eq 1 ]]; then
    [[ "$LINK_IS_CUT" -eq 1 ]] && log "  [dry-run] would RESTORE via ${METHOD}"
    LINK_IS_CUT=0
    return
  fi
  case "$METHOD" in
    container)
      docker start "$CONTAINER" >/dev/null 2>&1 || true
      ;;
    tailscale)
      # shellcheck disable=SC2086
      $SUDO tailscale up ${TS_UP_ARGS} >/dev/null 2>&1 || true
      ;;
    iptables)
      if [[ "$IPT_RULES_ADDED" -eq 1 ]]; then
        local chain spec
        for chain in INPUT OUTPUT; do
          for spec in "--dport" "--sport"; do
            # Loop: delete every copy of our rule, in case a previous aborted
            # run left duplicates behind. Stops when -D fails (none left).
            while $SUDO iptables -D "$chain" -p tcp "$spec" "$PORT" \
                        -m comment --comment "$IPT_COMMENT" -j DROP 2>/dev/null; do
              :
            done
          done
        done
        IPT_RULES_ADDED=0
      fi
      ;;
    link)
      $SUDO ip link set dev "$IFACE" up 2>/dev/null || true
      ;;
  esac
  LINK_IS_CUT=0
}

cleanup() {
  local rc=$?
  set +e
  if [[ -n "$ECHO_PID" ]] && kill -0 "$ECHO_PID" 2>/dev/null; then
    kill "$ECHO_PID" 2>/dev/null
    wait "$ECHO_PID" 2>/dev/null
  fi
  if [[ "$LINK_IS_CUT" -eq 1 || "$IPT_RULES_ADDED" -eq 1 ]]; then
    printf '%s\n' "${YEL}restoring the link (EXIT trap) ...${RST}" >&2
    restore_link
    printf '%s\n' "${GRN}link restored.${RST}" >&2
  fi
  [[ -n "$WORKDIR" && -d "$WORKDIR" ]] && rm -rf "$WORKDIR"
  exit $rc
}
# Restore on ANY exit path -- clean finish, error under `set -e`, Ctrl-C, or
# SIGTERM. This is what makes it safe to run a method that cuts the network.
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# -----------------------------------------------------------------------------
# The probe subscriber: `ros2 topic echo` into a file we watch for growth.
# Byte count is the signal -- simple, and it does not care about message shape.
# -----------------------------------------------------------------------------
start_echo() {
  WORKDIR="$(mktemp -d -t etz-recovery-XXXXXX)"
  ECHO_LOG="$WORKDIR/echo.log"
  : > "$ECHO_LOG"

  if [[ -n "$ROS_CONTAINER" ]]; then
    docker exec "$ROS_CONTAINER" bash -lc \
      "source /opt/ros/humble/setup.bash >/dev/null 2>&1; export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}; exec stdbuf -oL ros2 topic echo '$TOPIC' std_msgs/msg/String" \
      >"$ECHO_LOG" 2>"$WORKDIR/echo.err" &
  else
    stdbuf -oL ros2 topic echo "$TOPIC" std_msgs/msg/String \
      >"$ECHO_LOG" 2>"$WORKDIR/echo.err" &
  fi
  ECHO_PID=$!
  sleep 1
  if ! kill -0 "$ECHO_PID" 2>/dev/null; then
    log "--- ros2 topic echo stderr ---"
    cat "$WORKDIR/echo.err" >&2 || true
    die "the \`ros2 topic echo\` subscriber died immediately (see stderr above)"
  fi
}

bytes_seen() { wc -c < "$ECHO_LOG" 2>/dev/null || echo 0; }

# wait_for_traffic <timeout_seconds> -> echoes the ns timestamp of the first
# growth observed, or returns 1 on timeout.
wait_for_traffic() {
  local timeout="$1"
  local baseline; baseline="$(bytes_seen)"
  local deadline=$(( $(now_ns) + timeout * 1000000000 ))
  while [[ "$(now_ns)" -lt "$deadline" ]]; do
    if [[ "$(bytes_seen)" -gt "$baseline" ]]; then
      now_ns
      return 0
    fi
    sleep "$POLL_INTERVAL"
  done
  return 1
}

# traffic_stopped <window_seconds> -> 0 if NO growth for the whole window.
traffic_stopped() {
  local window="$1"
  local baseline; baseline="$(bytes_seen)"
  local deadline=$(( $(now_ns) + window * 1000000000 ))
  while [[ "$(now_ns)" -lt "$deadline" ]]; do
    if [[ "$(bytes_seen)" -gt "$baseline" ]]; then
      return 1
    fi
    sleep "$POLL_INTERVAL"
  done
  return 0
}

# -----------------------------------------------------------------------------
# Run
# -----------------------------------------------------------------------------
check_prereqs

METHOD_DESC=""
case "$METHOD" in
  container) METHOD_DESC="container (docker stop/start ${CONTAINER})" ;;
  tailscale) METHOD_DESC="tailscale (tailscale down/up)" ;;
  iptables)  METHOD_DESC="iptables (DROP tcp/${PORT} in+out, then flush)" ;;
  link)      METHOD_DESC="link (ip link set ${IFACE} down/up)" ;;
esac

log ""
log "=============================================================="
log " ZENOH LINK RECOVERY MEASUREMENT"
log "=============================================================="
info "method .............. ${METHOD_DESC}"
info "probe topic ......... ${TOPIC}"
info "ros2 runs ........... $([[ -n "$ROS_CONTAINER" ]] && echo "inside container '${ROS_CONTAINER}'" || echo "on this host")"
info "ROS_DOMAIN_ID ....... ${ROS_DOMAIN_ID:-(unset -> 0)}"
info "outage hold ......... ${OUTAGE_SECONDS} s"
info "trials .............. ${REPEATS}"
info "recovery timeout .... ${RECOVERY_TIMEOUT} s"
[[ "$DRY_RUN" -eq 1 ]] && info "DRY RUN ............. no cuts will actually be made"
log ""
info "Reminder: a publisher must be running on the OTHER machine:"
info "    ros2 topic pub -r 10 ${TOPIC} std_msgs/msg/String \"{data: 'etz-recovery-probe'}\""
log "--------------------------------------------------------------"

start_echo
log "  subscriber started, waiting for traffic (up to ${SETTLE_TIMEOUT}s) ..."

if ! wait_for_traffic "$SETTLE_TIMEOUT" >/dev/null; then
  cat >&2 <<EOF
${RED}
No messages EVER arrived on ${TOPIC}. Nothing was cut; nothing was measured.

Check, in this order:
  1. THE ALLOWLIST. Is the topic under /teams/team_${TEAM_IDX}/ ? The bridge
     silently drops everything else, in both directions, with no error.
  2. Is the publisher actually running on the OTHER machine?
         ros2 topic pub -r 10 ${TOPIC} std_msgs/msg/String "{data: 'etz-recovery-probe'}"
  3. Are both bridge containers up?    docker ps --filter name=zenoh
  4. Same ROS_DOMAIN_ID (0) on both ends and in both bridge configs?
  5. Can the two peers reach each other?   nc -vz <peer-tailscale-name> ${PORT}
  6. QoS: \`ros2 topic pub\` and \`ros2 topic echo\` both default to RELIABLE,
     so they match -- but a custom publisher on BEST_EFFORT would not.
${RST}
EOF
  exit 2
fi
log "  ${GRN}traffic confirmed flowing.${RST}"
log "--------------------------------------------------------------"

declare -a T_RECOVERY_NS=()
declare -a T_BLACKOUT_NS=()
declare -a T_CUTOK=()
declare -a T_NOTE=()

if [[ -n "$CSV_PATH" ]]; then
  printf 'trial,method,container,iface,port,topic,outage_seconds,cut_confirmed,recovery_seconds,blackout_seconds,t_cut_ns,t_restore_ns,t_first_msg_ns,note\n' > "$CSV_PATH"
fi

for (( trial = 1; trial <= REPEATS; trial++ )); do
  log ""
  log "  --- trial ${trial}/${REPEATS} ---"

  # Re-confirm the link is healthy before each trial, otherwise a trial that
  # starts already-broken would report a nonsense recovery time.
  if ! wait_for_traffic "$SETTLE_TIMEOUT" >/dev/null; then
    warn "trial ${trial}: traffic was not flowing before the cut -- skipping this trial."
    T_RECOVERY_NS+=(-1); T_BLACKOUT_NS+=(-1); T_CUTOK+=("n/a"); T_NOTE+=("no traffic pre-cut")
    continue
  fi

  T_CUT_NS="$(now_ns)"
  log "      cutting link (${METHOD}) ..."
  cut_link

  # Confirm the cut actually took effect. If traffic keeps flowing, the path
  # under test is NOT the path that was cut -- classically, both machines sit
  # on the same LAN subnet and plain DDS discovery
  # (ros_automatic_discovery_range: "SUBNET") is delivering the messages
  # directly, bypassing Zenoh entirely. That invalidates the trial.
  CUT_OK="yes"; NOTE="-"
  if [[ "$DRY_RUN" -eq 1 ]]; then
    CUT_OK="dry"
  elif traffic_stopped "$CUT_VERIFY_SECONDS"; then
    log "      ${GRN}traffic stopped -- cut confirmed.${RST}"
  else
    CUT_OK="NO"
    NOTE="traffic continued after cut"
    warn "trial ${trial}: traffic KEPT FLOWING after the cut. The path you measured is not the path you cut.
         Most likely both machines are on the same LAN subnet and plain DDS is
         delivering directly (ros_automatic_discovery_range: \"SUBNET\"), bypassing
         the Zenoh bridge. Put the machines on different subnets, or verify by
         stopping the bridge container and checking traffic stops."
  fi

  # Hold the outage for the remainder of --outage-seconds, measured from the
  # cut, so cut-command latency does not extend the hold.
  ELAPSED_NS=$(( $(now_ns) - T_CUT_NS ))
  HOLD_REMAIN_NS=$(( OUTAGE_SECONDS * 1000000000 - ELAPSED_NS ))
  if [[ "$HOLD_REMAIN_NS" -gt 0 ]]; then
    log "      holding outage for $(ns_to_s "$HOLD_REMAIN_NS") s more ..."
    sleep "$(ns_to_s "$HOLD_REMAIN_NS")"
  fi

  # T_RESTORE is stamped BEFORE the restore command is issued, consistently
  # for every method, so "recovery" always means "wall time from the operator
  # action to the first message back". `tailscale up` in particular blocks
  # until the tunnel is up -- that wait is part of recovery and is counted.
  BASELINE_BYTES="$(bytes_seen)"
  T_RESTORE_NS="$(now_ns)"
  log "      restoring link ..."
  restore_link

  log "      polling for the first message back (up to ${RECOVERY_TIMEOUT}s) ..."
  T_FIRST_NS=""
  DEADLINE_NS=$(( T_RESTORE_NS + RECOVERY_TIMEOUT * 1000000000 ))
  while [[ "$(now_ns)" -lt "$DEADLINE_NS" ]]; do
    if [[ "$(bytes_seen)" -gt "$BASELINE_BYTES" ]]; then
      T_FIRST_NS="$(now_ns)"
      break
    fi
    sleep "$POLL_INTERVAL"
  done

  if [[ -z "$T_FIRST_NS" ]]; then
    warn "trial ${trial}: NO recovery within ${RECOVERY_TIMEOUT}s."
    T_RECOVERY_NS+=(-1); T_BLACKOUT_NS+=(-1); T_CUTOK+=("$CUT_OK")
    T_NOTE+=("no recovery in ${RECOVERY_TIMEOUT}s")
    if [[ -n "$CSV_PATH" ]]; then
      printf '%d,%s,%s,%s,%s,%s,%d,%s,,,%s,%s,,%s\n' \
        "$trial" "$METHOD" "$CONTAINER" "$IFACE" "$PORT" "$TOPIC" \
        "$OUTAGE_SECONDS" "$CUT_OK" "$T_CUT_NS" "$T_RESTORE_NS" \
        "no recovery in ${RECOVERY_TIMEOUT}s" >> "$CSV_PATH"
    fi
  else
    REC_NS=$(( T_FIRST_NS - T_RESTORE_NS ))
    BLK_NS=$(( T_FIRST_NS - T_CUT_NS ))
    T_RECOVERY_NS+=("$REC_NS"); T_BLACKOUT_NS+=("$BLK_NS")
    T_CUTOK+=("$CUT_OK"); T_NOTE+=("$NOTE")
    log "      ${GRN}recovered in $(ns_to_s "$REC_NS") s  (total blackout $(ns_to_s "$BLK_NS") s)${RST}"
    if [[ -n "$CSV_PATH" ]]; then
      printf '%d,%s,%s,%s,%s,%s,%d,%s,%s,%s,%s,%s,%s,%s\n' \
        "$trial" "$METHOD" "$CONTAINER" "$IFACE" "$PORT" "$TOPIC" \
        "$OUTAGE_SECONDS" "$CUT_OK" "$(ns_to_s "$REC_NS")" "$(ns_to_s "$BLK_NS")" \
        "$T_CUT_NS" "$T_RESTORE_NS" "$T_FIRST_NS" "$NOTE" >> "$CSV_PATH"
    fi
  fi

  if [[ "$trial" -lt "$REPEATS" ]]; then
    log "      settling ${SETTLE_BETWEEN}s before the next trial ..."
    sleep "$SETTLE_BETWEEN"
  fi
done

# -----------------------------------------------------------------------------
# Report
# -----------------------------------------------------------------------------
log ""
log "=============================================================="
log " ZENOH LINK RECOVERY -- RESULTS"
log "=============================================================="
info "method .............. ${METHOD_DESC}"
info "probe topic ......... ${TOPIC}"
info "outage hold ......... ${OUTAGE_SECONDS} s"
info "trials .............. ${REPEATS}"
log "--------------------------------------------------------------"
printf '   trial | cut ok | recovery (s) | blackout (s) | note\n'
printf '   ------+--------+--------------+--------------+---------------\n'
for (( i = 0; i < ${#T_RECOVERY_NS[@]}; i++ )); do
  if [[ "${T_RECOVERY_NS[$i]}" -lt 0 ]]; then
    printf '   %5d | %-6s | %12s | %12s | %s\n' \
      "$(( i + 1 ))" "${T_CUTOK[$i]}" "--" "--" "${T_NOTE[$i]}"
  else
    printf '   %5d | %-6s | %12s | %12s | %s\n' \
      "$(( i + 1 ))" "${T_CUTOK[$i]}" \
      "$(ns_to_s "${T_RECOVERY_NS[$i]}")" \
      "$(ns_to_s "${T_BLACKOUT_NS[$i]}")" "${T_NOTE[$i]}"
  fi
done
log "--------------------------------------------------------------"

summarise() {
  local label="$1"; shift
  local vals=("$@")
  local good=()
  local v
  for v in "${vals[@]}"; do
    [[ "$v" -ge 0 ]] && good+=("$v")
  done
  if [[ "${#good[@]}" -eq 0 ]]; then
    printf '   %s: no successful trials.\n' "$label"
    return
  fi
  local min=${good[0]} max=${good[0]} sum=0
  for v in "${good[@]}"; do
    (( v < min )) && min=$v
    (( v > max )) && max=$v
    sum=$(( sum + v ))
  done
  local mean=$(( sum / ${#good[@]} ))
  printf '   %s, %d/%d trials:\n' "$label" "${#good[@]}" "${#vals[@]}"
  printf '     min .... %8s s\n' "$(ns_to_s "$min")"
  printf '     mean ... %8s s\n' "$(ns_to_s "$mean")"
  printf '     max .... %8s s\n' "$(ns_to_s "$max")"
}

summarise "RECOVERY (restore -> first message)" "${T_RECOVERY_NS[@]}"
log ""
summarise "BLACKOUT (cut -> first message, includes the ${OUTAGE_SECONDS}s hold)" "${T_BLACKOUT_NS[@]}"

log "--------------------------------------------------------------"
log "   BASELINE: the superseded Redis bridge retried every 2.000 s"
log "   (hardcoded loop, ai_vision/redis_bridge/redis_bridge.py @ fce9305)"
log "   -- but required a central Redis server as a single point of"
log "   failure. Zenoh peer mode has none. Compare the mean above"
log "   against 2.000 s, and note that the Redis figure is the OLD"
log "   system's documented retry interval, not a measurement of it."
log "--------------------------------------------------------------"
if [[ -n "$CSV_PATH" ]]; then
  info "CSV written ......... ${CSV_PATH}"
fi
log "=============================================================="
log ""

exit 0
