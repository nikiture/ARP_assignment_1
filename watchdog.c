#include <stdio.h> 
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <sys/time.h>
#include <sys/select.h>
#include <unistd.h> 
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include "shmem_info.h"
#include <semaphore.h>


const char * logres = "log_results.txt";

//watchdog version 1: it sends a signal to every process it controls the checks the log
int main (int argc, char ** argv) {

    int feedback_fd = open (logres, O_CREAT | O_RDWR, 0666);
    if (feedback_fd < 0) {
        perror ("log_res open");
        exit (EXIT_FAILURE);
    }

    fd_set log_check;
    const int proc_numb = 2;
    pid_t ID_monitored [proc_numb], proc_id;
    time_t last_resp [proc_numb], resp_time;
    struct timeval wait_time;

    int log_fd [proc_numb];
    
    for (int i = 0; i < proc_numb; i++){
        log_fd [i] = open (log_file [i], O_RDWR);
        if (log_fd [i] < 0) {
            perror ("log opening");
            exit (EXIT_FAILURE);
        }
    }
    sleep (4);
    int select_res, read_res, count;
    char proc_info [proc_numb] [10];
    char log_str [10];
    char log_feedback [80];
    int proc_resp [proc_numb];
    const double max_time = 4;

    time_t curr_time;
    if (time (&curr_time) < 0) {
        perror ("time measure");
        exit (EXIT_FAILURE);
    }
    

    for (int i = 0; i < proc_numb; i++) {
        wait_time.tv_sec = 1;
        wait_time.tv_usec = 0;

        FD_ZERO (&log_check);
        FD_SET (log_fd [i], &log_check);

        select_res = select (log_fd [i] + 1, &log_check, NULL, NULL, &wait_time);
        if (select_res < 0) {
            perror ("select");
            exit (EXIT_FAILURE);
        }

        if (select_res == 0) { //less processes than the expected number has responded, the watchdog terminates immediately to make the game closed by the master process
            printf ("watchdog here!\nsome process (the %dth one) did not identify itself\n", i);
            write (feedback_fd, "some process did not identify itself", 37);
            exit (EXIT_FAILURE);
        }
        if (read (log_fd [i], log_str, 10) < 0) {
            perror ("pid read");
            exit (EXIT_FAILURE);
        }
        if (sscanf (log_str, "%d", &ID_monitored [i]) < 0) {
            perror ("pid deformatting");
            exit (EXIT_FAILURE);
        }
        last_resp [i] = curr_time;
    }

    int syscall_res;

    while (1) {

        for (int i = 0; i < proc_numb; i++) {
            if (kill (ID_monitored [i], SIGUSR1) < 0) {
                perror ("kill");
                exit (EXIT_FAILURE);
            }
        }
        usleep (5000); //waits a bit to make sure that all processes have written
       
        wait_time.tv_sec = 0;
        wait_time.tv_usec = 5000;
        for (count = 0; count < proc_numb; count ++) { 
            //reset respense checker and FD_SET for select
            proc_resp [count] = 1;
            FD_ZERO (&log_check);
            FD_SET (log_fd [count], &log_check);

            wait_time.tv_sec = 0;
            wait_time.tv_usec = 5000;

            select_res = select (log_fd [count] + 1, &log_check, NULL, NULL, &wait_time);

            if (select_res < 0) {
                perror ("select");
                for (int i = 0; i < proc_numb; i++) {
                    close (log_fd [i]);
                }
                for (int i = 0; i < proc_numb; i++) {
                    kill (ID_monitored [i], SIGKILL);
                }
                exit (EXIT_FAILURE);
            }
            if (select_res == 0) { //setting up such that last_resp is not updated to now if no response was received
                proc_resp [count] = -1;
                continue;
            }
            if (read_res = read (log_fd [count], proc_info [count], 10) < 0) {
                perror ("log read");
                for (int i = 0; i < proc_numb; i++) {
                    close (log_fd [i]);
                }
                for (int i = 0; i < proc_numb; i++) {
                    if (kill (ID_monitored [i], SIGKILL) < 0) {
                        perror ("kill");
                    }
                }
                exit (EXIT_FAILURE);
            }
        }
        
        for (int i = 0; i < proc_numb; i++) {

            if (time (&curr_time) < 0) {
                perror ("time measure");
                for (int j = 0; j < proc_numb; j++) {
                    if (kill (ID_monitored [i], SIGKILL) < 0) {
                        perror ("kill");
                    }
                }
                exit (EXIT_FAILURE);
            }
            if (proc_resp [i] > 0) {
                last_resp [i] = curr_time;
            }
            else {
                printf ("Watchdog here!\nprocess %d has missed a call\n", ID_monitored [i]);
                if (sprintf (log_feedback, "%s %d %s", "process", ID_monitored [i], "has missed a call\n") < 0) {
                    perror ("sprintf");
                    for (int j = 0; j < proc_numb; j++) {
                        if (kill (ID_monitored [i], SIGKILL) < 0) {
                            perror ("kill");
                        }
                    }
                    exit (EXIT_FAILURE);
                }
                if (write (feedback_fd, log_feedback, sizeof (log_feedback)) < 0) {
                    perror ("write");
                    for (int j = 0; j < proc_numb; j++) {
                        if (kill (ID_monitored [i], SIGKILL) < 0) {
                            perror ("kill");
                        }
                    }
                    exit (EXIT_FAILURE);
                }
            }
        }

        if (time(&curr_time) < 0) {
            perror ("time measure");
            for (int i = 0; i < proc_numb; i++) {
                kill (ID_monitored [i], SIGKILL);
            }
            exit (EXIT_FAILURE);
        }

        for (int i = 0; i < proc_numb; i++) {
            if (curr_time - last_resp [i] > max_time) {
               
                printf ("watchdog here!\nno response from process %d for more than %lf seconds\n", ID_monitored [i], max_time);
                printf ("killing all processes\n");
                sprintf (log_feedback, "%s %d %s %lf %s", "no response from process", ID_monitored [i], "for more than", max_time, "seconds\nkilling all processes\n");
                for (int i = 0; i < proc_numb; i++) {
                    kill (ID_monitored [i], SIGKILL);
                }
                for (int i = 0; i < proc_numb; i++) {
                    close (log_fd [i]);
                }
                exit (EXIT_FAILURE);
            }
        }
        sleep (2);
    }
    return 0;
}