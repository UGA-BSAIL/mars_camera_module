# Message containing info about a single process.

# Process pid
uint32 pid
# Process name
string name
# Process command line invocation
string[] command_line

# Current CPU usage percentage
float32 cpu_usage
# Current memory usage percentage
float32 memory_usage