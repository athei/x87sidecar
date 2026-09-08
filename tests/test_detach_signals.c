#include <signal.h>
#include <stdio.h>

static volatile sig_atomic_t handled;

static void handle_signal(int signal) {
    if (signal == SIGUSR1) {
        ++handled;
    }
}

int main(void) {
    struct sigaction action = {0};
    action.sa_handler = handle_signal;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGUSR1, &action, NULL) != 0) {
        puts("FAIL install SIGUSR1 handler");
        return 1;
    }
    // The default loader must release the target without retaining a debugger
    // stop or changing the disposition of later signals. No x87 arithmetic is
    // involved, so a failure here isolates process control from translation.
    for (int i = 1; i <= 100; ++i) {
        if (raise(SIGUSR1) != 0 || handled != i) {
            puts("FAIL SIGUSR1 delivery after detach");
            return 1;
        }
    }
    puts("PASS SIGUSR1 delivery after detach");
    return 0;
}
