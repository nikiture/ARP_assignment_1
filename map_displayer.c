#include <stdio.h>
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/time.h>
#include <sys/types.h> 
#include <unistd.h> 
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <ncurses.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include "shmem_info.h"

const char DRONE_MARKER = 'X';
#define SEC_TO_USEC 1000000
const double framerate = 50;
int logfd = 0;


void printerror (const char * errmsg) {
    perror (errmsg);
    printw ("issues in %s\n\r", errmsg);
    refresh ();
    if (logfd > 0) close (logfd);
    exit (EXIT_FAILURE);
}
const int log_id = 1; //position in log_file where pid is written
        
void watchdog_req (int signumb) {

    if (signumb == SIGUSR1) {
        logfd = open (log_file [log_id], O_WRONLY, 0666);
        if (logfd < 0) {
            printerror ("log file open");
        }

        int pid = getpid ();
        char logdata [10];

        if (sprintf (logdata, "%d", pid) < 0) {
            printerror ("log formatting");
        }
        if (write (logfd, logdata, sizeof (logdata)) < 0) {
            printerror ("log write");
        }
        close (logfd);
        
    }
}

int main (int argc, char ** argv) {
    
    signal (SIGUSR1, watchdog_req);

    sem_t * map_semaph = sem_open (MAP_SEM, O_CREAT, 0666, 1);
    if (map_semaph == SEM_FAILED) {
        printerror ("semaphore creation");
    }

    if (sem_init (map_semaph, 0, 1) < 0) printerror ("semaphore initialisation");

    sem_t * kb_sem = sem_open (KB_SEM, O_CREAT, 0666, 1);
    if (kb_sem == SEM_FAILED) {
        printerror ("semaphore creation kb");
    }

    if (sem_init (kb_sem, 0, 1) < 0) printerror ("semaphore kb creation");

    int fd_drone = shm_open (MAP_ADDR, O_RDWR, 0666);
    if (fd_drone < 0) printerror ("shared memory opening");

    void * drone_ptr = mmap (0, 2 * sizeof (double), PROT_READ | PROT_WRITE, MAP_SHARED, fd_drone, 0);
    if (drone_ptr == MAP_FAILED) printerror ("shared memory mapping");

    int kb_fd = shm_open (KB_ADDR, O_RDWR, 0666);
    if (kb_fd < 0) printerror ("shared memory opening kb");

    void * kb_ptr = mmap (0, sizeof (int), PROT_WRITE | PROT_READ, MAP_SHARED, kb_fd, 0);
    if (kb_ptr == MAP_FAILED) printerror ("shared memory kb mapping");

    int logfd = open (log_file [log_id], O_WRONLY, 0666);
    if (logfd < 0) printerror ("log file open");

    char logdata [10];
    int syscall_res;

    int pid = getpid ();

    if (sprintf (logdata, "%d", pid) < 0) printerror ("pid log formatting");

    if (write (logfd, logdata, sizeof (logdata)) < 0) {
        printerror ("watchdog write");
    }

    double drone_pos [2], old_pos [2];

    int mapsize [2];
    int kb_res;
    initscr();

    printf ("welcome to the game! here are the commands for the game\n\r");
    printf ("use the w, e, r, s, d, f, x, c and v keys to control the drone\n\r");
    printf ("press q to quit the game, z to restart the game\n\r");

    usleep (50000); //waits a bit of time (50 milliseconds) in order to let the drone dynamics process write the first position on the shared memory

    struct timespec start_time;
    struct timespec end_time;
    struct timespec delta_time, rem_time;
    long int time_to_sleep;
    long int nsec_diff;
    void * memcopy_res = NULL;
    
    wresize (stdscr, MAP_Y_SIZE, MAP_X_SIZE);
    resizeterm (MAP_Y_SIZE, MAP_X_SIZE);
    cbreak();
    noecho();
    curs_set (0);
    nodelay (stdscr, 0);
    wtimeout (stdscr, 5);
    start_color ();
    //int colorable = has_colors ();
    if (has_colors () == 1) {
        int color_change = can_change_color ();
        /*printf ("%d\n", color_change);
        sleep (4);*/   
        init_pair (1, COLOR_GREEN, COLOR_BLACK); //first is characters, second is background
        init_pair (2, COLOR_WHITE, COLOR_BLACK);
        //attron (COLOR_PAIR (1));
        bkgd (COLOR_PAIR (1));
    }
    /*printw ("%d", has_colors ());
    refresh ();*/
    sleep (20);

    while (1) {

        //read kb presses and send them to dynamics process
        kb_res = getch ();
        if (sem_wait (kb_sem) < 0) printerror ("kb semaphore taking");

        memcopy_res = memcpy (kb_ptr, &kb_res, sizeof (int));
        if (memcopy_res == NULL) printerror ("shared memory kb write");

        if (sem_post (kb_sem) < 0) printerror ("kb semaphore release");

        if (kb_res == 'q') {
            close (logfd);
            exit (EXIT_SUCCESS);
        }

                            /*receive from shared memory drone position*/
        if (sem_wait (map_semaph) < 0) printerror ("semaphore taking");

        memcopy_res = memcpy (drone_pos, drone_ptr, 2 * sizeof (double));

        if (memcopy_res == NULL) printerror ("shared memory reading");
        
        if (sem_post (map_semaph) < 0) printerror ("semaphore releasing");
        //adjustemnts to drone position to match with ncurses: positive going downward, maximum at 0
        drone_pos [1] *= -1; 
        drone_pos [1] += MAP_Y_SIZE;
        //limitations to positions put just in case to still visualize the drone, but they should not be necessary with the wall forces implemented
        if (drone_pos [0] < 0) drone_pos [0] = 0;
        if (drone_pos [0] > MAP_X_SIZE) drone_pos [0] = MAP_X_SIZE;
        if (drone_pos [1] < 0) drone_pos [1] = 0;
        if (drone_pos [1] > MAP_Y_SIZE) drone_pos [1] = MAP_Y_SIZE;

        wresize (stdscr, MAP_Y_SIZE, MAP_X_SIZE); //makes sure the playground (visualized using box () below) is of the expected size
        syscall_res = clear ();
        if (syscall_res == ERR) {
            printf ("issues clearing\n\r");
            printerror ("addch clear");
        }
        //set character color to white for border, the re-set it back to color assigned to drone's character

        attroff (COLOR_PAIR (1));
        attron (COLOR_PAIR (2));
        box (stdscr, '|', '-');
        attroff (COLOR_PAIR (2));
        attron (COLOR_PAIR (1));

        //update drone position on the map displayed
        syscall_res = mvprintw ((int) round (drone_pos [1]), (int) round (drone_pos [0]), "%c", DRONE_MARKER);
        if (syscall_res == ERR) {
            printf ("issues placing drone\n\r");
            printerror ("addch");
        }
        refresh ();
        
        usleep (SEC_TO_USEC / framerate);


    }
    return 0;
}