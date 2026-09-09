#!/bin/bash
# Pre-flight check: are the fixes we think we are testing actually in the SOURCE and in the
# BINARY that roslaunch will run?
#
# This exists because on 2026-09-09 entry_detection_module.py and fast_exploration_fsm.cpp were
# both rewritten at 09:29:36 back to a state without the /mission/stop_exploration handshake.
# Three runs (093043, 094450, 101738) were then flown and analysed as if the fix were present.
# Run 101738 cost 136 s of its 164 s stall to the exact race the missing code prevents. Nothing
# in the launch, the build, or the flight log said the code was gone -- only the wording of one
# log line did. Run this before every test.
cd "$(dirname "$0")/.." || exit 1
WS=catkin_ws
NODE=$WS/devel/.private/exploration_manager/lib/exploration_manager/exploration_node
AP=$WS/devel/lib/libactive_perception.so
PM=$WS/devel/lib/libplan_manage.so
EDM=$WS/src/nidar_mission/scripts/entry_detection_module.py
FSM=$WS/src/fuel/fuel_planner/exploration_manager/src/fast_exploration_fsm.cpp
fail=0
chk() {  # chk <label> <needle> <file...>
    local label=$1 needle=$2; shift 2
    for f in "$@"; do
        if [ ! -e "$f" ]; then echo "MISSING FILE  $label: $f"; fail=1; continue; fi
        local n
        case "$f" in
            *.so|*/exploration_node) n=$(strings "$f" | grep -c -- "$needle") ;;
            *)                       n=$(grep -c -- "$needle" "$f") ;;
        esac
        if [ "$n" -lt 1 ]; then echo "ABSENT        $label  in  $f"; fail=1
        else                    echo "ok            $label  in  $f"; fi
    done
}
echo "--- launch XML well-formedness ---"
# A malformed <!-- --> block (e.g. a bare "--" inside a comment, which XML forbids) makes
# roslaunch refuse the WHOLE file with an RLException. exploration_node then never starts at
# all, but nothing else in the stack notices: PX4/gzserver/FAST-LIO are fine, the vehicle enters
# the arena and hovers, and the EDM just logs "FUEL has not subscribed" once a minute. From the
# outside that is indistinguishable from a stall -- run 20260909_120021 (2026-09-09) burned
# several minutes of wall clock looking like one before the actual cause (a "--" introduced by
# an edit to algorithm.xml) was found. Catch it before launch, not by staring at telemetry.
for xml in "$WS"/src/fuel/fuel_planner/exploration_manager/launch/algorithm.xml \
           "$WS"/src/fuel/fuel_planner/exploration_manager/launch/exploration.launch; do
    if [ -f "$xml" ]; then
        if python3 -c "import xml.dom.minidom as m; m.parse('$xml')" 2>/tmp/xmlcheck_err; then
            echo "ok            well-formed  $xml"
        else
            echo "MALFORMED     $xml"
            sed 's/^/              /' /tmp/xmlcheck_err
            fail=1
        fi
    fi
done

echo "--- source ---"
chk "stop-handshake publisher"  "pub_stop_exploration"                 "$EDM"
chk "stop-handshake helper"     "request_end_of_exploration"           "$EDM"
chk "stop-handshake subscriber" "stopExplorationCallback"              "$FSM"
echo "--- binaries ---"
chk "stop-handshake (FSM)"      "stop requested by mission layer"      "$NODE"
chk "trigger re-arm guard"      "ignoring trigger: exploration was stopped" "$NODE"
chk "degenerate-path retire"    "already standing on viewpoint"        "$NODE"
chk "viewpoint standoff"        "Standoff"                             "$AP"
chk "planExploreTraj guard"     "refusing a"                           "$PM"
chk "gain-weighted ATSP"        "FUEL GAIN"                            "$AP"
echo "--- staleness (binary older than source is a stale build) ---"
for pair in "$NODE:$FSM"; do
    b=${pair%%:*}; s=${pair##*:}
    if [ "$s" -nt "$b" ]; then echo "STALE         $b is older than $s -- rebuild"; fail=1
    else echo "ok            $b newer than $s"; fi
done
[ $fail -eq 0 ] && echo "PARITY OK" || echo "PARITY FAILED -- do not trust this run"
exit $fail
