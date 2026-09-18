// TODO(Part 5.4): Host application entry point.
//
// Loads config, opens the EventLog, then starts one thread per subsystem through SystemManager and
// blocks in runUntilShutdown(). Ctrl+C or window close triggers an ordered shutdown which must send
// one final zeroed ActuationCommand before the serial port closes (Part 5.4).
//
// Filled in during Phase 3.

int main(int /*argc*/, char** /*argv*/) {
    return 0;
}
