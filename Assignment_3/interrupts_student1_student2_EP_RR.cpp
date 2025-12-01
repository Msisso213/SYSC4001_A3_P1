/**
 * @file interrupts.cpp
 * @author Sasisekhar Govind
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 * 
 */

#include<interrupts_student1_student2.hpp>

// Enum to select scheduling algorithm
enum SchedulingAlgorithm {
    FCFS_SCHEDULING,
    RR_SCHEDULING
};

void FCFS(std::vector<PCB> &ready_queue) {
    std::sort( 
                ready_queue.begin(),
                ready_queue.end(),
                []( const PCB &first, const PCB &second ){
                    return (first.arrival_time > second.arrival_time); 
                } 
            );
}

void RR(std::vector<PCB> &ready_queue) {
    // Round-robin doesn't need sorting
    // Processes are dispatched in FIFO order
}

std::tuple<std::string> run_simulation(std::vector<PCB> list_processes, SchedulingAlgorithm algorithm) {

    std::vector<PCB> ready_queue;   //The ready queue of processes
    std::vector<PCB> wait_queue;    //The wait queue of processes
    std::vector<PCB> job_list;      //A list to keep track of all the processes. This is similar
                                    //to the "Process, Arrival time, Burst time" table that you
                                    //see in questions. You don't need to use it, I put it here
                                    //to make the code easier :).

    unsigned int current_time = 0;
    unsigned int time_quantum = 100;  // 100ms time quantum for Round-Robin
    unsigned int quantum_used = 0;     // Track how long current process has run
    PCB running;

    //Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;

    //make the output table (the header row)
    execution_status = print_exec_header();

    //Loop while till there are no ready or waiting processes.
    //This is the main reason I have job_list, you don't have to use it.
    while(!all_process_terminated(job_list) || job_list.empty()) {

        //Inside this loop, there are three things you must do:
        // 1) Populate the ready queue with processes as they arrive
        // 2) Manage the wait queue
        // 3) Schedule processes from the ready queue

        //Population of ready queue is given to you as an example.
        //Go through the list of proceeses
        for(auto &process : list_processes) {
            if(process.arrival_time == current_time) {//check if the AT = current time
                //if so, assign memory and put the process into the ready queue
                if(assign_memory(process)){
                    states old_state = NEW;
                    process.state = READY;
                    ready_queue.push_back(process);
                    job_list.push_back(process);
                    execution_status += print_exec_status(current_time, process.PID, old_state, READY);
                }
            }
        }

        ///////////////////////MANAGE WAIT QUEUE/////////////////////////
        // Manage processes currently in wait queue (doing I/O)
        for(auto it = wait_queue.begin(); it != wait_queue.end(); ) {
            it->io_time_elapsed++;

            // If I/O is complete, move back to ready queue
            if(it->io_time_elapsed >= it->io_duration) {
                enum states old_state = it->state;
                it->state = READY;
                it->io_time_elapsed = 0;  // Reset for next I/O
                it->cpu_time_since_io = 0; // Reset CPU time counter
                ready_queue.push_back(*it);
                sync_queue(job_list, *it);
                
                execution_status += print_exec_status(current_time, it->PID, old_state, READY);
                
                // Remove from wait queue
                it = wait_queue.erase(it);
            } else {
                ++it;
            }
        }
        /////////////////////////////////////////////////////////////////

        //////////////////////////SCHEDULER//////////////////////////////
        if(running.state == NOT_ASSIGNED && !ready_queue.empty()) {
            // Apply the selected scheduling algorithm
            if(algorithm == FCFS_SCHEDULING) {
                FCFS(ready_queue);
            } else {
                RR(ready_queue);
            }
            
            run_process(running, job_list, ready_queue, current_time);
            quantum_used = 0;  // Reset quantum counter for new process
            execution_status += print_exec_status(current_time, running.PID, READY, RUNNING);
        }

        // Execute the running process (decrement remaining time)
        if(running.state == RUNNING) {
            running.remaining_time--;
            running.cpu_time_since_io++;  // Increment CPU time counter
            quantum_used++;  // Increment quantum counter
            sync_queue(job_list, running);
            
            // Check if process has finished
            if(running.remaining_time == 0) {
                enum states old_state = running.state;
                terminate_process(running, job_list);
                execution_status += print_exec_status(current_time, running.PID, old_state, TERMINATED);
                idle_CPU(running);
                quantum_used = 0;  // Reset quantum counter
            }
            // Check if running process needs I/O
            else if(running.io_freq > 0 && running.cpu_time_since_io >= running.io_freq) {
                enum states old_state = running.state;
                running.state = WAITING;
                running.io_time_elapsed = 0;
                wait_queue.push_back(running);
                sync_queue(job_list, running);
                
                execution_status += print_exec_status(current_time, running.PID, old_state, WAITING);
                
                // CPU becomes idle
                idle_CPU(running);
                quantum_used = 0;  // Reset quantum counter
            }
            // Check if time quantum has expired (preemption) - ONLY for Round-Robin
            else if(algorithm == RR_SCHEDULING && quantum_used >= time_quantum && !ready_queue.empty()) {
                enum states old_state = running.state;
                running.state = READY;
                ready_queue.insert(ready_queue.begin(), running);  // Add to front of ready queue
                sync_queue(job_list, running);
                
                execution_status += print_exec_status(current_time, running.PID, old_state, READY);
                
                // CPU becomes idle, will pick next process in next iteration
                idle_CPU(running);
                quantum_used = 0;  // Reset quantum counter
            }
        }
        /////////////////////////////////////////////////////////////////

        current_time++;  // Don't forget to increment time!
    }
    
    //Close the output table
    execution_status += print_exec_footer();

    return std::make_tuple(execution_status);
}

int main(int argc, char** argv) {

    //Get the input file from the user
    if(argc != 3) {
        std::cout << "ERROR!\nExpected 2 arguments, received " << argc - 1 << std::endl;
        std::cout << "To run the program, do: ./interrupts <your_input_file.txt> <scheduling_algorithm>" << std::endl;
        std::cout << "Scheduling algorithms: FCFS or RR" << std::endl;
        return -1;
    }

    //Open the input file
    auto file_name = argv[1];
    std::ifstream input_file;
    input_file.open(file_name);

    //Ensure that the file actually opens
    if (!input_file.is_open()) {
        std::cerr << "Error: Unable to open file: " << file_name << std::endl;
        return -1;
    }

    // Determine scheduling algorithm from command line argument
    std::string algorithm_str = argv[2];
    SchedulingAlgorithm algorithm;
    
    if(algorithm_str == "FCFS" || algorithm_str == "fcfs") {
        algorithm = FCFS_SCHEDULING;
        std::cout << "Using FCFS scheduling algorithm" << std::endl;
    } else if(algorithm_str == "RR" || algorithm_str == "rr") {
        algorithm = RR_SCHEDULING;
        std::cout << "Using Round-Robin scheduling algorithm with time quantum = 100ms" << std::endl;
    } else {
        std::cerr << "Error: Invalid scheduling algorithm. Use 'FCFS' or 'RR'" << std::endl;
        return -1;
    }

    //Parse the entire input file and populate a vector of PCBs.
    //To do so, the add_process() helper function is used (see include file).
    std::string line;
    std::vector<PCB> list_process;
    while(std::getline(input_file, line)) {
        auto input_tokens = split_delim(line, ", ");
        auto new_process = add_process(input_tokens);
        list_process.push_back(new_process);
    }
    input_file.close();

    //With the list of processes, run the simulation
    auto [exec] = run_simulation(list_process, algorithm);

    write_output(exec, "execution.txt");

    return 0;
}