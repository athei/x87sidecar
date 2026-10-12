# Sourced by run_tests.sh, test_x87_trace.sh and run_benchmarks.sh.
#
# run_bounded SECONDS COMMAND...: run COMMAND in a process group of its own
# and kill the whole group if anything in it is still running after SECONDS,
# exiting 124 like timeout(1), which macOS does not ship.  After COMMAND
# exits, the rest of its group (the sidecar, which exits once it sees the
# target exit) gets the remaining time to finish its output; a member still
# there at the deadline is killed, so nothing left behind can hold the
# output pipe open and keep the caller's $(...) waiting.  Otherwise exits as
# COMMAND did, 128 + the signal number when a signal ended it.
#
# Killing the group, not just the target, is what ends a hung run of the
# loader: a target that is still traced can survive SIGALRM and even
# SIGKILL until its tracer, the sidecar, is gone.  For the same reason a
# run_bounded inside another one is not reached by the outer kill, since it
# moves its command to a group of its own; bound each loader run once.
run_bounded() {
    perl -e '
        my $secs = shift;
        my $pid = fork() // die "fork: $!";
        if ($pid == 0) {
            setpgrp(0, 0);
            exec @ARGV or exit 127;
        }
        # Both sides set the group, so a kill below cannot race the child'"'"'s
        # own setpgrp; the parent'"'"'s call fails harmlessly after the exec.
        setpgrp($pid, $pid);
        $SIG{ALRM} = sub { kill "KILL", -$pid; waitpid($pid, 0); exit 124; };
        alarm $secs;
        waitpid($pid, 0);
        my $status = $?;
        select(undef, undef, undef, 0.02) while kill(0, -$pid);
        alarm 0;
        exit($status & 127 ? 128 + ($status & 127) : $status >> 8);
    ' "$@"
}
