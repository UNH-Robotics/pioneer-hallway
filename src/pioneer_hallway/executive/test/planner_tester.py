from subprocess import Popen, PIPE, check_call
import time
import sys

async def readline(p):
    return p.stdout.readline()

def communicate(p, message, num_lines=1):
    p.stdin.write(str.encode(message))
    p.stdin.flush()
    out = p.stdout.readline()
    for i in range(num_lines - 1):
        out += p.stdout.readline()
    return out

if __name__ == '__main__':
    path = "/home/hcgs/Dropbox/school/planner/out/artifacts/planner_jar/planner.jar"
    if len(sys.argv) > 1:
        path = sys.argv[1]

    p = Popen(["java", "-jar", path], stdin=PIPE, stdout=PIPE)

    # communicate goal state to planner
    # result = communicate(p, "GOAL\n1932 2846\n")
    result = communicate(p, "GOAL\n2253 2109\n")

    state_template = "STATE\n{} {}\nEND\n"

    # initial state
    cur_state = "1026.0 2754.0 0.0 0.0 0.0"

    for i in range(1000):
        timestamp = int(time.time() * 1000)

        # ask planner for next action result
        result = communicate(p, state_template.format(timestamp, cur_state), 2)

        # print result and show how long it took for planner to respond
        print("Response Time: {}".format(int(time.time() * 1000) - timestamp))
        print(result, "\n")

        result = result.decode('utf-8').split()[2:]

        # if this is the goal, we're done. Make sure planner dies
        if -1 <= round(float(result[0])) - 2253 <= 1 and -1 <= round(float(result[1])) - 2109 <= 1:
            print("GOAL!")
            p.stdin.write(str.encode("DIE"))
            p.stdin.flush()
            break

        # otherwise, set current state to whatever planner projected that we would be in next time
        cur_state = " ".join(result)
