#include <stdio.h>
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/time.h>
#include <sys/types.h> 
#include <signal.h>
#include <unistd.h> 
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ncurses.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include "shmem_info.h"

double PROPULSION_STEP = 1;
const double root2 = 1.41421;
const int frequency = 100;               //frequency of dynamycs computation in hertz
#define SEC_TO_USEC 1000000

const int log_id = 0; //position in log_file where pid is written

void watchdog_req (int signumb) {
    if (signumb == SIGUSR1) {
        int fd = open (log_file [log_id], O_WRONLY);
        if (fd < 0) {
            perror ("log_file open");
            printf ("issues opening file\n");
        }

        int pid = getpid ();
        char logdata [10];
        
        if (sprintf (logdata, "%d", pid) < 0) {
            perror ("sprintf");
            printf ("issues formatting\n");
        }

        if (write (fd, logdata, sizeof (logdata)) < 0) {
            perror ("write");
            printf ("issues writing to file\n");
        }

        close (fd);
    }
}

void Close_all (int fd_1, int fd_2, int fd_3, int fd_4, void * pos_writer, sem_t * map_semaph, sem_t * kb_sem, void * kb_shm, int exit_status) {
    if (fd_1 > 0) close (fd_1);
    if (fd_2 > 0) close (fd_2);
    if (fd_3 > 0) close (fd_3);
    if (fd_4 > 0) close (fd_4);

    munmap (pos_writer, 2 * sizeof (double));

    munmap (kb_shm, sizeof (int));

    sem_close (map_semaph);

    sem_close (kb_sem);

    sem_unlink (KB_SEM);

    sem_unlink (MAP_SEM);

    exit (exit_status);
}
int Get_Kb_In (int kb_in, int * quit, int * reset, double * kb_forces_x, double * kb_forces_y) {
    
    *quit = 0;
    *reset = 0;
    
    if (kb_in > 0) {
        switch (kb_in) {
            case 'w': //upper left
                *kb_forces_x -=  PROPULSION_STEP / root2;
                *kb_forces_y += PROPULSION_STEP / root2;
                break;
            case 's': //only left
                *kb_forces_x -= PROPULSION_STEP;
                break;
            case 'x': //down left
                *kb_forces_x -= PROPULSION_STEP / root2;
                *kb_forces_y -= PROPULSION_STEP / root2;
                break;
            case 'c': //only down
                *kb_forces_y -= PROPULSION_STEP;
                break;
            case 'v': //down right
                *kb_forces_x += PROPULSION_STEP / root2;
                *kb_forces_y -= PROPULSION_STEP / root2;
                break;
            case 'f': //only right
                *kb_forces_x += PROPULSION_STEP;
                break;
            case 'r': //up right
                *kb_forces_x += PROPULSION_STEP / root2;
                *kb_forces_y += PROPULSION_STEP / root2;
                break;
            case 'e': //only up
                *kb_forces_y += PROPULSION_STEP;
                break;
            case 'd': //brake
                //set all forces to 0
                *kb_forces_x = 0;
                *kb_forces_y = 0;
                break;
            case 'q'://exit game
                *quit = 1;
                *kb_forces_x = 0;
                *kb_forces_y = 0;
                break;
            case 'z': //restart
                *reset = 1;
                *kb_forces_x = 0;
                *kb_forces_y = 0;
                break;
            default: //anything different from w, s, x, c, v, f, r, e, d, q or z is considered as no press
                break;
        } 
    }
    return kb_in;
}
int update_BB (double * pos, sem_t * map_semaph, void * pos_writer) {
    if ((sem_wait (map_semaph)) < 0) {
        perror ("semaph taking");
        Close_all (-1, -1, -1, -1, pos_writer, map_semaph, NULL, NULL, EXIT_FAILURE);
    }

    void * syscall_res = memcpy (pos_writer, pos, 2 * sizeof (double));
    if (syscall_res == NULL) {
        perror ("shared memory writing");
        Close_all (-1, -1, -1, -1, pos_writer, map_semaph, NULL, NULL, EXIT_FAILURE);
    }
    if (sem_post (map_semaph) < 0) {
        perror ("semaph release");
        Close_all (-1, -1, -1, -1, pos_writer, map_semaph, NULL, NULL, EXIT_FAILURE);
    }
}

int main (int argc, char ** argv) {
    
    signal (SIGUSR1, watchdog_req);

    sem_t * map_semaph = sem_open (MAP_SEM, O_CREAT, 0666, 1);
    if (map_semaph == SEM_FAILED) {
        perror ("semaphore opening");
        exit (EXIT_FAILURE);
    }

    if (sem_init (map_semaph, 0, 1) < 0) {
        perror ("semaphore initialisation");
        sem_close (map_semaph); 
        exit (EXIT_FAILURE);
    }

    sem_t * kb_sem = sem_open (KB_SEM, O_CREAT, 0666, 1);
    if (kb_sem == SEM_FAILED) {
        perror ("semaphore creation kb");
        sem_close (map_semaph);
        exit (EXIT_FAILURE);
    }

    if (sem_init (kb_sem, 0, 1) < 0) {
        perror ("semaphore kb creation");
        sem_close (map_semaph);
        sem_close (kb_sem);
    }

    struct timespec start_time;
    struct timespec end_time;

    //0 is position of previous iteration, 1 is position of 2 iterations ago
    double drone_x [2], drone_y [2];

    //position in current iteration of the simulation
    double drone_act [2], v_x, v_y;
    double x_start = MAP_X_SIZE / 2;
    double y_start = MAP_Y_SIZE / 2;

    //initialisation of drone position on the map (roughly at the center); 
    drone_act [0] = x_start;
    drone_act [1] = y_start;

    drone_x [0] = drone_x [1] = drone_act [0];
    drone_y [0] = drone_y [1] = drone_act [1];

    
    int count, syscall_res;
    int fd_out = 0, fd_wtch = 0, fd_param = 0; 

    

    fd_wtch = open (log_file [log_id], O_WRONLY, 0666);
    if (fd_wtch < 0) {
        perror ("open");
        Close_all (-1, -1, -1, fd_wtch, NULL, map_semaph, kb_sem, NULL, EXIT_FAILURE); //null because shared memories not yet initialised
    }

    char logdata [10];
    int pid = getpid ();

    if (sprintf (logdata, "%d", pid) < 0) {
        perror ("pid formatting");
        Close_all (-1, fd_wtch, -1, -1, NULL, map_semaph, kb_sem, NULL, EXIT_FAILURE);
    }

    if (write (fd_wtch, logdata, sizeof (logdata)) < 0) {
        perror ("watchdog write");

        Close_all (-1, fd_wtch, -1, -1, NULL, map_semaph, kb_sem, NULL, EXIT_FAILURE);
    }

    char * param_file = "parameters.txt";

    //string in which parameters of simulation are read;
    // 1 is force from keyboard, 2 is mass, 3 is friction coefficient, 
    //4 is obstacle / wall force intensity and 5 is max distance from obstacle to perceive force
    char param_str [5 * sizeof(double)]; 

    
    
    double ro_max;                      //maximum distance to perceive force from walls/obstacles
    double wall_dist_x, wall_dist_y;    //distance of drone from walls (used for right wall and low wall)

    double M, K, Eta; 

    if ((fd_param = open (param_file, O_RDONLY)) < 0) {
        perror ("open: ");
        Close_all (-1, -1, fd_wtch, -1, NULL, map_semaph, kb_sem, NULL, EXIT_FAILURE);
    }

    int kb_fd = shm_open (KB_ADDR, O_RDWR, 0666);
    if (kb_fd < 0) {
        perror ("shared memory opening kb");
        Close_all (-1, fd_param, fd_wtch, -1, NULL, map_semaph, kb_sem, NULL, EXIT_FAILURE);
    }

    void * kb_ptr = mmap (0, sizeof (int), PROT_WRITE | PROT_READ, MAP_SHARED, kb_fd, 0);
    if (kb_ptr == MAP_FAILED) {
        perror ("shared memory kb mapping");
        Close_all (-1, fd_wtch, fd_param, kb_fd, NULL, map_semaph, kb_sem, NULL, EXIT_FAILURE);
    }
    
    fd_out = shm_open (MAP_ADDR, O_CREAT | O_RDWR, 0666);
    if (fd_out < 0) { 
        perror ("open");
        Close_all (fd_out, fd_wtch, fd_param, kb_fd, NULL, map_semaph, kb_sem, kb_ptr, EXIT_FAILURE);
    }

    char pos_str [2 * sizeof (double)];

    void * pos_writer = mmap (0, 2 * sizeof (double), PROT_WRITE | PROT_READ, MAP_SHARED, fd_out, 0);
    if (pos_writer == MAP_FAILED) {
        perror ("shm mapping");
        Close_all (fd_out, fd_param, fd_wtch, kb_fd, NULL, map_semaph, kb_sem, kb_ptr, EXIT_FAILURE);
    }
    
    
    update_BB (drone_act, map_semaph, pos_writer);

    int reset, quit;

    double Force_x, Force_y, kb_force_x, kb_force_y, v_ipot;

    double obs_force_x, obs_force_y, targ_force_x, targ_force_y; 

    double wall_force_x, wall_force_y;

    long int nsec_diff;

    long int time_to_sleep;

    const int wxsize = MAP_X_SIZE, wysize = MAP_Y_SIZE;

    syscall_res = lseek (fd_param, 0, SEEK_SET);
    if (syscall_res < 0) {
        perror ("lseek");
        Close_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, map_semaph, kb_sem, kb_ptr, EXIT_FAILURE);
    }
    syscall_res = read (fd_param, param_str, 5 * sizeof (double));
    if (syscall_res < 0) {
        perror ("param_read 1:");
        Close_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, map_semaph, kb_sem, kb_ptr, EXIT_FAILURE);
    }

    syscall_res = sscanf (param_str, "%lf %lf %lf %lf %lf", &PROPULSION_STEP, &M, &K, &Eta, &ro_max);
    if (syscall_res < 0) {
        perror ("param_scan 1:");
        Close_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, map_semaph, kb_sem, kb_ptr, EXIT_FAILURE);
    }

    //values used in differential equation of drone
    //multiplying by frequency instead of dividing by dt to avoid getting nan as result

    double K_1 = M * frequency * frequency;
    double K_2 = K * frequency;
    int kb_in;
    void * memcopy_res;

    while (1) {

        if (sem_wait (kb_sem) < 0) {
            perror ("kb semaphore taking");
            Close_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, map_semaph, kb_sem, kb_ptr, EXIT_FAILURE);
        }

        memcopy_res = memcpy (&kb_in, kb_ptr, sizeof (int));
        if (memcopy_res == NULL) {
            perror ("shared memory kb write");
            Close_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, map_semaph, kb_sem, kb_ptr, EXIT_FAILURE);
        }

        if (sem_post (kb_sem) < 0) {
            perror ("kb semaphore release");
            Close_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, map_semaph, kb_sem, kb_ptr, EXIT_FAILURE);
        }

                                /*forces acting on drone*/
        //forces for keyboard input (received from the process displaying the map)
        Get_Kb_In (kb_in, &quit, &reset, &kb_force_x, &kb_force_y);

        //commands related to keyboard input although not related to  forces

        if (reset == 1) {//reset drone to starting position
            drone_act [0] = drone_x [0] = drone_x [1] = x_start;
            drone_act [1] = drone_y [0] = drone_y [1] = y_start;
        }

        if (quit == 1) {
            Close_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, map_semaph, kb_sem, kb_ptr, EXIT_SUCCESS);
        }

        //obstacles
        obs_force_x = 0;
        obs_force_y = 0;

        //targets
        targ_force_x = 0;
        targ_force_y = 0;

        //walls 
        wall_force_x = 0;
        wall_force_y = 0;

        wall_dist_x = MAP_X_SIZE - drone_act [0];
        wall_dist_y = MAP_Y_SIZE - drone_act [1];
        if (drone_act [0] < ro_max) { //left end, force to the right
            wall_force_x = Eta * (1/drone_act [0] - 1/ro_max) /(drone_act [0] * drone_act [0]);
        }
        if (drone_act [1] < ro_max) { //lower end, force upward
            wall_force_y = Eta * (1/drone_act [1] - 1/ro_max) /(drone_act [1] * drone_act [1]);
        }
        if (wall_dist_x < ro_max) {//right edge, force towards the right
            wall_force_x = - Eta * (1/wall_dist_x - 1/ro_max) /(wall_dist_x * wall_dist_x);
        }
        if (wall_dist_y < ro_max) {//upper edge, force downward
            wall_force_y = - Eta * (1/wall_dist_y - 1/ro_max) /(wall_dist_y * wall_dist_y);
        }

        //total
        Force_x = kb_force_x + obs_force_x + targ_force_x + wall_force_x;
        Force_y = kb_force_y + obs_force_y + targ_force_y + wall_force_y;


                                    /*physics step simulation*/
        //euler method: F = M* (xi-2 + xi - 2*xi-1)/dt^2 + K(xi - xi-1)/dt  
        //xi-2 is drone_x [0], xi-1 is drone_x [1], xi is drone_act [0];  
        //similar format for y position 
        //using euler formula for current x position

        drone_act [0] = (Force_x + (2 * K_1 + K_2) * drone_x [0] - K_1 * drone_x [1]) / (K_1 + K_2);

        //update x positions of previous iterations

        drone_x [1] = drone_x [0];
        drone_x [0] = drone_act [0];
        
        //euler formula for current y position

        drone_act [1] = (Force_y + (2 * K_1 + K_2) * drone_y [0] - K_1 * drone_y [1]) / (K_1 + K_2);

        //update positions of previous iterations

        drone_y [1] = drone_y [0];
        drone_y [0] = drone_act [1];

        //send to blackboard / map displayer the just computed location of the drone
        update_BB (drone_act, map_semaph, pos_writer);
        
        //wait for beginning of next period
        usleep (SEC_TO_USEC / frequency);
    }

    Close_all (fd_out, fd_wtch, fd_param, kb_fd, pos_writer, map_semaph, kb_sem, kb_ptr, EXIT_FAILURE); //failure because program should never end here
    return 0;
}