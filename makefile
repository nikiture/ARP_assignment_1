all: Master drone_dyn watchdog map_displayer
	
clean-logs:
	rm log_results.txt
clean:
	rm Master drone_dyn map_displayer watchdog 
Master: Master.c
	gcc Master.c -o Master -l rt -lpthread
drone_dyn: drone_dyn.c
	gcc drone_dyn.c -o drone_dyn -lm -lrt -lpthread
watchdog: watchdog.c 
	gcc watchdog.c -o watchdog
map_displayer: map_displayer.c
	gcc map_displayer.c -o map_displayer -lm -lrt -lpthread -lncurses
